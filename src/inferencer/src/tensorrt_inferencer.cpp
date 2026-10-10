#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <cuda_runtime_api.h>
#include <inferencer/inferencer.h>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <boost/filesystem.hpp>
#include <boost/dll/alias.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace inferencer
{

class RTInferencer: public Inferencer
{
    public:
    RTInferencer(): buffers(1, nullptr)
    {
        cudaError_t error = cudaStreamCreate(&stream);

        if(error != cudaSuccess)
        {
            throw std::runtime_error("Failed to create cuda stream: " + std::string(cudaGetErrorString(error)));
        }

    }

    ~RTInferencer()
    {
        cudaStreamDestroy(stream);

        for(const auto& buffer:buffers)
        {
            if(buffer != nullptr)
                cudaFree(buffer);
        }
    }

    bool loadModel(const std::string& modelName) final
    {

        bool needSerialize = false;
        boost::filesystem::path filePath(modelName);

        std::string fileExt = filePath.extension().string();

        if(fileExt == ".onnx")
        {
            boost::filesystem::path engineFilePath = filePath;
            engineFilePath.replace_extension("engine");

            if(boost::filesystem::exists(engineFilePath))
            {
                engine = loadEngine(engineFilePath.c_str());
            }


            if(engine == nullptr)
            {
                engine = loadOnnx(filePath.string());
                if(engine != nullptr)
                    saveEngine(engineFilePath.string());
            }
        }
        else if(fileExt == ".engine")
        {
            engine = loadEngine(filePath.string());
        }
        else
        {
            errorString = "Unsupported model file format " + fileExt;
            return false;
        }


        if (engine == nullptr)
        {
            return false;
        }

        context.reset(engine->createExecutionContext());

        if(context == nullptr)
        {
            throw std::runtime_error("Failed to create tensorrt execution context");
        }
        return true;
    }

    unsigned getInputBuffer(const std::string& inputName, void** buffer) final
    {
        int inputIndex = engine->getBindingIndex(inputName.c_str());

        if(inputIndex == -1)
        {
            errorString = "Invaild input tensor name " + std::string(inputName);
            return 0;
        }

        nvinfer1::Dims inputDims = engine->getBindingDimensions(inputIndex);
        nvinfer1::DataType inputDataType = engine->getBindingDataType(inputIndex);

        size_t inputSize = getVolume(inputDims) * getDataTypeSize(inputDataType);

        *buffer = allocBuffer(inputIndex, inputSize);

        if(*buffer == nullptr)
            return 0;

        return inputSize;
    } 

    unsigned getOutputBuffer(const std::string& outputName, void** buffer) final
    {
        int outputIndex = engine->getBindingIndex(outputName.c_str());
        if(outputIndex == -1)
        {
            errorString = "Invaild output tensor name " + std::string(outputName);
            return false;
        }

        nvinfer1::Dims outputDims = engine->getBindingDimensions(outputIndex);
        nvinfer1::DataType outputDataType = engine->getBindingDataType(outputIndex); 

        size_t outputSize = getVolume(outputDims) * getDataTypeSize(outputDataType);

        *buffer = allocBuffer(outputIndex, outputSize);

        if(*buffer == nullptr)
            return 0;

        return outputSize;
    } 

    bool infer() final
    {
        if(!context->enqueueV2(buffers.data(), stream, nullptr))
        {
            errorString = "Failed to enqueue the stream";
            return false;
        }

        cudaStreamSynchronize(stream);
        return true;
    }

    const std::string& getErrorString() const final 
    {
        return errorString;
    }

    static std::shared_ptr<RTInferencer> create()
    {
        return std::make_shared<RTInferencer>();
    }

    private:
    struct NvInferDeleter 
    {
        template <typename T>
        void operator()(T* obj) const 
        {
            if (obj) 
            {
                obj->destroy();
            }
        }
    };

    class Logger : public nvinfer1::ILogger           
    {
        void log(Severity severity, const char* msg) noexcept override
        {
            // suppress info-level messages
            if (severity == Severity::kINFO)
                RCLCPP_INFO(rclcpp::get_logger("tensorrt_inferencer"), "%s", msg);
            else if (severity == Severity::kVERBOSE)
                RCLCPP_DEBUG(rclcpp::get_logger("tensorrt_inferencer"), "%s", msg);
            else if (severity == Severity::kWARNING)
                RCLCPP_WARN(rclcpp::get_logger("tensorrt_inferencer"), "%s", msg);
            else
                RCLCPP_ERROR(rclcpp::get_logger("tensorrt_inferencer"), "%s", msg);
        }
    } logger;

    std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> engine;
    std::unique_ptr<nvinfer1::IExecutionContext, NvInferDeleter> context;

    std::vector<void*> buffers;
    cudaStream_t stream;
    std::string errorString;

    std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> loadOnnx(const std:: string& fileName)
    {
        std::unique_ptr<nvinfer1::IBuilder, NvInferDeleter> builder{nvinfer1::createInferBuilder(logger)};
        if (builder == nullptr)
        {
            throw std::runtime_error("Failed to create tensorrt network builder");
        }

        const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        std::unique_ptr<nvinfer1::INetworkDefinition, NvInferDeleter> network{builder->createNetworkV2(explicitBatch)};
        if (network == nullptr)
        {
            throw std::runtime_error("Failed to build tensorrt network");
        }
        
        std::unique_ptr<nvonnxparser::IParser, NvInferDeleter> parser{nvonnxparser::createParser(*network, logger)};
        if (parser == nullptr)
        {
            throw std::runtime_error("Failed to create onnx model parser");
        }

        if(!parser->parseFromFile(fileName.c_str(), static_cast<int32_t>(nvinfer1::ILogger::Severity::kVERBOSE)))
        {
            errorString = "Failed to parse onnx model file " + fileName;
            return nullptr;
        }


        std::unique_ptr<nvinfer1::IBuilderConfig, NvInferDeleter> config{builder->createBuilderConfig()};
        if (config == nullptr)
        {
            throw std::runtime_error("Failed to create tensorrt builder configuration");
        }

        size_t totalMemory;
        cudaError_t error = cudaMemGetInfo(nullptr, &totalMemory);
        if (error != cudaSuccess) 
        {
            throw std::runtime_error("Failed to get cuda memory info: " + std::string(cudaGetErrorString(error)));
        }

        config->setMaxWorkspaceSize(totalMemory/4);

        nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
        if (profile == nullptr)
        {
            throw std::runtime_error("Failed to create tensorrt optimization profile");
        }

        nvinfer1::ITensor* input = network->getInput(0);
        const char* inputName = input->getName();

        nvinfer1::Dims dims = input->getDimensions();
        dims.d[0] = 1;

        profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMIN, dims);
        profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kOPT, dims);
        profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMAX, dims);
        config->addOptimizationProfile(profile);

        std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> engine{builder->buildEngineWithConfig(*network, *config)};
        if(engine == nullptr)
        {
            errorString = "Failed to build tensorrt engine from onnx model " + fileName;
            return nullptr;
        }

        return std::move(engine);
    }

    std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> loadEngine(const std::string& fileName)
    {
        std::ifstream file(fileName, std::ios::binary);
        if (!file) 
        {
            errorString = "Model file " + fileName + " does not exist";
            return nullptr;
        }

        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> engineData(fileSize);
        file.read(engineData.data(), fileSize);
        if (!file) 
        {
            errorString = "Failed to read the model file " + fileName;
            return nullptr;
        } 

        file.close();

        std::unique_ptr<nvinfer1::IRuntime, NvInferDeleter> runtime{nvinfer1::createInferRuntime(logger)};
        if (runtime == nullptr)
        {
            throw std::runtime_error("Failed to create tensorrt runtime");
        }

        std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> engine{runtime->deserializeCudaEngine(engineData.data(), engineData.size())};
        if (engine == nullptr)
        {
            errorString = "Failed to deserialize tensorrt engine file " + fileName;
            return nullptr;
        }

        return std::move(engine);
    }

    bool saveEngine(const std::string& fileName)
    {
        //Serialize and save the model
        std::unique_ptr<nvinfer1::IHostMemory, NvInferDeleter> serializedModel{engine->serialize()};

        std::ofstream file(fileName, std::ios::binary);
        if (!file) 
        {
            errorString = "Failed to create the engine file " + fileName;
            return false;
        }

        // Write the serialized model to the file
        file.write(static_cast<const char*>(serializedModel->data()), serializedModel->size());
        if (!file) 
        {
            errorString = "Failed to write to the engine file" + fileName;
            return false;
        }   

        file.close();
        return true;
    }

    size_t getDataTypeSize(nvinfer1::DataType type) 
    {
        switch (type) 
        {
            case nvinfer1::DataType::kFLOAT: return 4;   // 4 bytes for float
            case nvinfer1::DataType::kHALF:  return 2;   // 2 bytes for half-precision float
            case nvinfer1::DataType::kINT8:  return 1;   // 1 byte for int8
            case nvinfer1::DataType::kINT32: return 4;   // 4 bytes for int32
            case nvinfer1::DataType::kBOOL:  return 1;   // 1 byte for bool
            default: return 0;
        }
    }   

    size_t getVolume(const nvinfer1::Dims& dims) 
    {
        size_t volume = 1; // Start with a volume of 1
        for (int i = 0; i < dims.nbDims; ++i) 
        {
            if (dims.d[i] == -1) 
            {
                return 0;
            }
            volume *= dims.d[i];
        }
        return volume;
    }

    void* allocBuffer(int index, size_t size)
    {
        if(index + 1 <= buffers.size())
        {
            if(buffers[index] != nullptr)
            {
                return buffers[index];
            }
        }
        else
        {
            buffers.resize(index + 1, nullptr);
        }

        void* buffer;
        cudaError_t error = cudaMallocManaged(&buffer, size);
        if(error != cudaSuccess)
        {
            throw std::runtime_error("Failed to allocate memory for the input tensor: " + std::string(cudaGetErrorString(error)));
        }

        buffers[index] = buffer;
        return buffer;
    }

};
}

BOOST_DLL_ALIAS(inferencer::RTInferencer::create, tensorrt_inferencer);
