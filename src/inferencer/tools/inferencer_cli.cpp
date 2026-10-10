// Run an inferencer backend without ROS, e.g. to check a model or to compare two builds of a backend.
//
// Usage: inferencer_cli <backend> <model_file> <input_name> <output_name> [image_file roi_x roi_y roi_width roi_height]
//
// The backend library lib<backend>_inferencer.so is searched in LD_LIBRARY_PATH, same as nn_controller.
// Without an image the float input tensor is filled with a fixed pattern (i % 256). With an image, it is
// converted to RGB, cropped to the roi and converted to float the same way nn_controller does.

#include <inferencer/inferencer.h>

#include <boost/dll/import.hpp>
#include <boost/function.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>

int main(int argc, char** argv)
{
    if (argc != 5 && argc != 10)
    {
        std::fprintf(stderr, "Usage: %s <backend> <model_file> <input_name> <output_name> "
                             "[image_file roi_x roi_y roi_width roi_height]\n", argv[0]);
        return 1;
    }

    const std::string backend = std::string(argv[1]) + "_inferencer";
    const std::string modelName = argv[2];
    const std::string inputName = argv[3];
    const std::string outputName = argv[4];

    // The creator keeps the backend library loaded, so it must outlive the inferencer
    using Creator = std::shared_ptr<inferencer::Inferencer>();
    boost::function<Creator> creator;
    std::shared_ptr<inferencer::Inferencer> inferencer;

    void* outputBuffer = nullptr;
    void* inputBuffer = nullptr;

    try
    {
        creator = boost::dll::import_alias<Creator>(backend, backend,
                                                    boost::dll::load_mode::append_decorations |
                                                    boost::dll::load_mode::search_system_folders);
        inferencer = creator();
        std::printf("Loaded backend %s\n", backend.c_str());

        if (!inferencer->loadModel(modelName))
        {
            std::fprintf(stderr, "Failed to load model file %s: %s\n", modelName.c_str(), inferencer->getErrorString().c_str());
            return 1;
        }
        std::printf("Loaded model %s\n", modelName.c_str());

        // Same order as nn_controller: output tensor first, then input tensor
        size_t outputBufferSize = inferencer->getOutputBuffer(outputName, &outputBuffer);
        if (outputBufferSize == 0)
        {
            std::fprintf(stderr, "Failed to allocate output tensor %s: %s\n", outputName.c_str(), inferencer->getErrorString().c_str());
            return 1;
        }

        size_t inputBufferSize = inferencer->getInputBuffer(inputName, &inputBuffer);
        if (inputBufferSize == 0)
        {
            std::fprintf(stderr, "Failed to allocate input tensor %s: %s\n", inputName.c_str(), inferencer->getErrorString().c_str());
            return 1;
        }
        std::printf("Input tensor %s: %zu bytes, output tensor %s: %zu bytes\n",
                    inputName.c_str(), inputBufferSize, outputName.c_str(), outputBufferSize);

        if (argc == 5)
        {
            float* input = static_cast<float*>(inputBuffer);
            for (size_t i = 0; i < inputBufferSize / sizeof(float); ++i)
                input[i] = static_cast<float>(i % 256);
            std::printf("Input: pattern i %% 256\n");
        }
        else
        {
            cv::Mat bgr = cv::imread(argv[5], cv::IMREAD_COLOR);
            if (bgr.empty())
            {
                std::fprintf(stderr, "Failed to read image %s\n", argv[5]);
                return 1;
            }
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

            cv::Rect roi = cv::Rect(std::atoi(argv[6]), std::atoi(argv[7]), std::atoi(argv[8]), std::atoi(argv[9])) &
                           cv::Rect(0, 0, rgb.cols, rgb.rows);
            size_t roiByteSize = roi.area() * 4 * rgb.channels();
            if (roiByteSize != inputBufferSize)
            {
                std::fprintf(stderr, "Image roi byte size %zu does not match the input buffer size %zu\n", roiByteSize, inputBufferSize);
                return 1;
            }

            cv::Mat imageMat(roi.height, roi.width, CV_32FC3, inputBuffer);
            rgb(roi).convertTo(imageMat, CV_32FC3);
            if (imageMat.data != inputBuffer)
            {
                std::fprintf(stderr, "Input image was not written to the input buffer\n");
                return 1;
            }
            std::printf("Input: %s (%dx%d) roi x: %d, y: %d, width: %d, height: %d\n",
                        argv[5], rgb.cols, rgb.rows, roi.x, roi.y, roi.width, roi.height);
        }

        if (!inferencer->infer())
        {
            std::fprintf(stderr, "Failed to run inference: %s\n", inferencer->getErrorString().c_str());
            return 1;
        }

        const float* output = static_cast<const float*>(outputBuffer);
        std::printf("Output:");
        for (size_t i = 0; i < outputBufferSize / sizeof(float); ++i)
            std::printf(" %.6f", output[i]);
        std::printf("\n");

        const int runs = 50;
        auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < runs; ++i)
        {
            if (!inferencer->infer())
            {
                std::fprintf(stderr, "Failed to run inference: %s\n", inferencer->getErrorString().c_str());
                return 1;
            }
        }
        auto end = std::chrono::steady_clock::now();
        std::printf("Average inference time over %d runs: %.3f ms\n", runs,
                    std::chrono::duration<double, std::milli>(end - start).count() / runs);
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    return 0;
}
