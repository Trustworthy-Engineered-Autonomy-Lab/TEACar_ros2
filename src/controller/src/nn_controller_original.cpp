#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core.hpp>

#include <boost/dll/import.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "controller/controller.hpp"
#include <inferencer/inferencer.h>

// Steers the car with a neural network. The throttle is left to the other controllers (e.g. the joystick).
//
// The inferencer backend lib<backend>_inferencer.so is loaded at runtime, then the model is loaded and the
// tensors are allocated, retrying every second until each step succeeds. After that the first image
// configures the roi and every following image is fed to the model.
class NNController : public controller::Controller
{
public:
    NNController() : controller::Controller("nn_controller_node"), status_(Status::INIT_BACKEND)
    {
        this->declare_parameter<std::string>("backend", "tensorflow");
        // Empty means <teacar_bringup share>/models/model
        this->declare_parameter<std::string>("model_file", "");
        this->declare_parameter<std::string>("input_name", "input");
        this->declare_parameter<std::string>("output_name", "outputs");
        // Region of the image fed to the model. A width/height <= 0 means up to the image border
        this->declare_parameter<int>("roi.x", 0);
        this->declare_parameter<int>("roi.y", 0);
        this->declare_parameter<int>("roi.width", 0);
        this->declare_parameter<int>("roi.height", 0);

        interval_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/interval", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NNController::timer_callback, this));
    }

private:
    enum class Status
    {
        INIT_BACKEND,
        LOAD_MODEL,
        ALLOC_OUTPUT,
        ALLOC_INPUT,
        WAIT_IMAGE,
        RUNNING,
    };

    using Creator = std::shared_ptr<inferencer::Inferencer>();

    Status status_;
    // Keeps the backend library loaded, so it is declared before (and destroyed after) the inferencer
    boost::function<Creator> creator_;
    std::shared_ptr<inferencer::Inferencer> inferencer_;

    void* outputBuffer_ = nullptr;
    void* inputBuffer_ = nullptr;
    size_t inputBufferSize_ = 0;
    size_t outputBufferSize_ = 0;

    cv::Mat imageMat_;
    cv::Rect roi_;
    int channels_ = 0;
    uint32_t imageWidth_ = 0;
    uint32_t imageHeight_ = 0;
    std::string imageEncoding_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr interval_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        if (status_ == Status::INIT_BACKEND)
        {
            std::string backend = this->get_parameter("backend").as_string() + "_inferencer";

            try
            {
                creator_ = boost::dll::import_alias<Creator>(backend, backend,
                                                             boost::dll::load_mode::append_decorations |
                                                             boost::dll::load_mode::search_system_folders);
                inferencer_ = creator_();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to initialize the %s backend: %s. Will retry", backend.c_str(), e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully initialize %s backend", backend.c_str());
            status_ = Status::LOAD_MODEL;
        }
        else if (status_ == Status::LOAD_MODEL)
        {
            std::string modelName = this->get_parameter("model_file").as_string();
            if (modelName.empty())
            {
                std::string defaultFolder;
                try
                {
                    defaultFolder = ament_index_cpp::get_package_share_directory("teacar_bringup");
                }
                catch (const std::exception &e)
                {
                    RCLCPP_FATAL(this->get_logger(), "Could not find package teacar_bringup: %s. Exiting", e.what());
                    rclcpp::shutdown();
                    return;
                }
                modelName = (boost::filesystem::path(defaultFolder) / "models" / "model").string();
            }

            bool result;

            try
            {
                result = inferencer_->loadModel(modelName);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to load model file %s: %s. Exiting", modelName.c_str(), e.what());
                rclcpp::shutdown();
                return;
            }

            if (!result)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to load model file %s: %s. Will retry", modelName.c_str(), inferencer_->getErrorString().c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully load model file %s", modelName.c_str());
            status_ = Status::ALLOC_OUTPUT;
        }
        else if (status_ == Status::ALLOC_OUTPUT)
        {
            std::string outputName = this->get_parameter("output_name").as_string();

            try
            {
                outputBufferSize_ = inferencer_->getOutputBuffer(outputName, &outputBuffer_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to allocate output tensor: %s. Exiting", e.what());
                rclcpp::shutdown();
                return;
            }

            if (outputBufferSize_ == 0)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to allocate output tensor: %s. Will retry", inferencer_->getErrorString().c_str());
                return;
            }

            if (outputBufferSize_ != sizeof(float))
            {
                RCLCPP_FATAL(this->get_logger(), "Invalid byte size of output tensor. Need 4 but get %zu. Exiting", outputBufferSize_);
                rclcpp::shutdown();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully allocate output tensor %s", outputName.c_str());
            status_ = Status::ALLOC_INPUT;
        }
        else if (status_ == Status::ALLOC_INPUT)
        {
            std::string inputName = this->get_parameter("input_name").as_string();

            try
            {
                inputBufferSize_ = inferencer_->getInputBuffer(inputName, &inputBuffer_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to allocate input tensor %s: %s. Exiting", inputName.c_str(), e.what());
                rclcpp::shutdown();
                return;
            }

            if (inputBufferSize_ == 0)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to allocate input tensor %s: %s. Will retry", inputName.c_str(), inferencer_->getErrorString().c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully allocate input tensor %s", inputName.c_str());

            // Best effort is compatible with both reliable and best effort camera publishers
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "~/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                std::bind(&NNController::image_callback, this, std::placeholders::_1));

            status_ = Status::WAIT_IMAGE;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (status_ == Status::WAIT_IMAGE)
        {
            configure_input(msg);
        }
        else if (status_ == Status::RUNNING)
        {
            run_inference(msg);
        }
    }

    // Checks the first image against the input tensor and maps the roi of the images onto the input buffer
    void configure_input(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        int imageWidth = static_cast<int>(msg->width);
        int imageHeight = static_cast<int>(msg->height);

        int x = this->get_parameter("roi.x").as_int();
        int y = this->get_parameter("roi.y").as_int();
        int roiWidth = this->get_parameter("roi.width").as_int();
        int roiHeight = this->get_parameter("roi.height").as_int();
        if (roiWidth <= 0)
            roiWidth = imageWidth;
        if (roiHeight <= 0)
            roiHeight = imageHeight;

        cv::Rect validRegion(0, 0, imageWidth, imageHeight);
        roi_ = cv::Rect(x, y, roiWidth, roiHeight) & validRegion;

        int channels = 0;
        if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        {
            channels = 1;
        }
        else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            channels = 3;
        }
        else
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Unsupported image encoding %s. Will retry", msg->encoding.c_str());
            return;
        }

        size_t roiByteSize = static_cast<size_t>(roi_.area()) * 4 * channels;

        if (roiByteSize != inputBufferSize_)
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Image roi byte size %zu does not match the input buffer size %zu. Will retry", roiByteSize, inputBufferSize_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Valid roi region is x: %d, y: %d, width: %d, height: %d", roi_.x, roi_.y, roi_.width, roi_.height);

        imageMat_ = cv::Mat(roi_.height, roi_.width, channels == 1 ? CV_32FC1 : CV_32FC3, inputBuffer_);
        channels_ = channels;
        imageWidth_ = msg->width;
        imageHeight_ = msg->height;
        imageEncoding_ = msg->encoding;

        status_ = Status::RUNNING;
    }

    void run_inference(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        if (msg->width != imageWidth_ || msg->height != imageHeight_ || msg->encoding != imageEncoding_)
        {
            RCLCPP_WARN(this->get_logger(), "Image changed to %ux%u %s. Will configure the input again",
                        msg->width, msg->height, msg->encoding.c_str());
            status_ = Status::WAIT_IMAGE;
            return;
        }

        if (msg->step < msg->width * channels_ || msg->data.size() < static_cast<size_t>(msg->step) * msg->height)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid image: step %u, data size %zu for %ux%u %s",
                         msg->step, msg->data.size(), msg->width, msg->height, msg->encoding.c_str());
            return;
        }

        // Wrap the image data without copying, same as cv_bridge::toCvShare with the source encoding.
        // cv_bridge is not used because it is built against another OpenCV version on the Jetson
        cv::Mat image(static_cast<int>(msg->height), static_cast<int>(msg->width), channels_ == 1 ? CV_8UC1 : CV_8UC3,
                      const_cast<uint8_t *>(msg->data.data()), msg->step);

        // imageMat_ has the size and type of the roi, so the pixels are written into the input buffer
        image(roi_).convertTo(imageMat_, channels_ == 1 ? CV_32FC1 : CV_32FC3);

        auto start = std::chrono::steady_clock::now();
        bool success = inferencer_->infer();
        auto end = std::chrono::steady_clock::now();

        std_msgs::msg::Float64 dt_msg;
        dt_msg.data = std::chrono::duration<double>(end - start).count();
        interval_pub_->publish(dt_msg);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to run inference %s", inferencer_->getErrorString().c_str());
            image_sub_.reset();
            status_ = Status::INIT_BACKEND;
            return;
        }

        float steer = *reinterpret_cast<float *>(outputBuffer_);

        // Only steer, the throttle is controlled by the other controllers
        this->control(0.0f, steer);

        RCLCPP_DEBUG(this->get_logger(), "Run inference successfully %f", steer);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NNController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
