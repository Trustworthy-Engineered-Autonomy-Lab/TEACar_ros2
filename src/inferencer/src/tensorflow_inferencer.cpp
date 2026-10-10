#include <stdexcept>
#include <cstdlib>
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <tensorflow/core/platform/status.h>
#include <tensorflow/core/platform/errors.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/framework/tensor_shape.h>
#include <tensorflow/core/platform/logging.h>
#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/cc/saved_model/loader.h>
#include <inferencer/inferencer.h>

#include <boost/process/environment.hpp>
#include <boost/filesystem.hpp>
#include <boost/dll/alias.hpp>

#ifdef __cplusplus
extern "C" {
#endif

extern const char* TF_Version(void);

#ifdef __cplusplus
}
#endif

namespace inferencer
{
class TFInferencer : public Inferencer
{
    
private:
    tensorflow::SavedModelBundle modelBundle;
    tensorflow::SignatureDef signature;
    std::vector<std::pair<std::string,tensorflow::Tensor>> inputTensorPairs;
    std::vector<std::string> outputTensorNames;
    std::vector<tensorflow::Tensor> outputTensors;

    std::string errorString;

    void** outputBuffer;
    public:
    TFInferencer()
    {
        // boost::process::environment env = boost::this_process::environment();
        // env["TF_CPP_MIN_LOG_LEVEL"] = "1";
        
        TF_Version();

    }

    ~TFInferencer()
    {

    }
    
static std::shared_ptr<TFInferencer> create(){
    return std::make_shared<TFInferencer>();
}

bool loadModel(const std::string& modelName) final
{
    tensorflow::SessionOptions sessionOptions;
    tensorflow::RunOptions runOptions;
    
    tensorflow::Status status = tensorflow::LoadSavedModel(sessionOptions, runOptions, {modelName}, {"serve"}, &this->modelBundle);

    if (!status.ok()) 
    {
        this->errorString = status.ToString();
        return false;
    }

    const tensorflow::MetaGraphDef& graphDef = this->modelBundle.meta_graph_def;

    const auto& signatureDefMap = graphDef.signature_def();
    
    auto it = signatureDefMap.find("serving_default");
    if(it == signatureDefMap.end())
    {
        this->errorString = "Failed to find the default signature";
        return false;
    }

    this->signature = it->second;
    return true;
}

unsigned getInputBuffer(const std::string& inputName, void** buffer) final
{

    auto it = this->signature.inputs().find(inputName);
    if(it == this->signature.inputs().end())
    {
        std::ostringstream errorStream;
        errorStream << "Invalid input tensor name: " << inputName << ". Valid names are: ";

        // Iterate over all valid input tensor names
        for (const auto& input : this->signature.inputs())
        {
            errorStream << input.first << " ";
        }

        this->errorString = errorStream.str();
        return 0;
    }

    const tensorflow::TensorInfo& tensorInfo = it->second;
    const std::string& tensorName = tensorInfo.name();

    auto it1 = std::find_if(
        this->inputTensorPairs.begin(),
        this->inputTensorPairs.end(),
        [&tensorName](const std::pair<std::string, tensorflow::Tensor>& pair) {
            return pair.first == tensorName; // Compare the tensor name
        });
    
    if(it1 != this->inputTensorPairs.end())
    {
        int index = std::distance(this->inputTensorPairs.begin(), it1);
        *buffer = this->inputTensorPairs[index].second.data();
        return this->inputTensorPairs[index].second.tensor_data().size();
    }

    tensorflow::TensorShapeProto shapeProto = tensorInfo.tensor_shape();
    if(shapeProto.dim(0).size() == -1)
    {
        // ROS_INFO("Dynamic batch size detected. Will set to one");
        shapeProto.mutable_dim(0)->set_size(1);
    }

    // ROS_INFO_STREAM("Input tensor shape " + shapeProto.DebugString());

    tensorflow::TensorShape shape(shapeProto);

    tensorflow::Tensor inputTensor = tensorflow::Tensor(tensorInfo.dtype(), shape);

    if (!inputTensor.IsInitialized())
    {
        this->errorString = "Tensor is not initialized";
        return 0;
    }

    size_t inputByteSize = inputTensor.tensor_data().size();

    this->inputTensorPairs.emplace_back(tensorName, std::move(inputTensor));

    *buffer = this->inputTensorPairs.back().second.data();

    return inputByteSize;
} 

unsigned getOutputBuffer(const std::string& outputName, void** buffer) final
{    

    auto it = this->signature.outputs().find(outputName);
    if(it == this->signature.outputs().end())
    {
        std::ostringstream errorStream;
        errorStream << "Invalid output tensor name: " << outputName << ". Valid names are: ";

        // Iterate over all valid input tensor names
        for (const auto& output : this->signature.outputs())
        {
            errorStream << output.first << " ";
        }
        this->errorString = errorStream.str();
        return 0;
    }

    const tensorflow::TensorInfo& tensorInfo = it->second;
    const std::string& tensorName = tensorInfo.name();

    auto it1 = std::find(this->outputTensorNames.begin(), this->outputTensorNames.end(), tensorName);
    if(it1 != this->outputTensorNames.end())
    {
        int index = std::distance(this->outputTensorNames.begin(), it1);
        return this->outputTensors[index].tensor_data().size();
    }

    tensorflow::TensorShapeProto shapeProto = tensorInfo.tensor_shape();
    if(shapeProto.dim(0).size() == -1)
    {
        // ROS_INFO("Dynamic batch size detected. Will set to one");
        shapeProto.mutable_dim(0)->set_size(1);
    }

    // ROS_INFO_STREAM("Output tensor shape " + shapeProto.DebugString());
    tensorflow::TensorShape shape(shapeProto);

    tensorflow::Tensor outputTensor = tensorflow::Tensor(tensorInfo.dtype(), shape);

    if (!outputTensor.IsInitialized())
    {
        this->errorString = "Tensor is not initialized";
        return 0;
    }

    size_t outputByteSize = outputTensor.tensor_data().size();

    this->outputBuffer = buffer;

    this->outputTensorNames.emplace_back(tensorName);
    this->outputTensors.emplace_back(std::move(outputTensor));

    return outputByteSize;
} 

    bool infer() final
{
    
    tensorflow::Status status = this->modelBundle.GetSession()->Run(this->inputTensorPairs, this->outputTensorNames, {}, &this->outputTensors);
    if (!status.ok())
    {
        this->errorString = "Failed to run inference " + status.ToString();
        return false;
    }

    *this->outputBuffer = const_cast<void*>(reinterpret_cast<const void*>(this->outputTensors[0].tensor_data().data()));

    return true;
}

    const std::string& getErrorString() const final
{
    return this->errorString;
}
};
}

BOOST_DLL_ALIAS(inferencer::TFInferencer::create, tensorflow_inferencer);