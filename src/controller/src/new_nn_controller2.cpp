#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include <inferencer/inferencer.h>
#include <boost/dll/import.hpp>
#include <boost/function.hpp>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>


class NNController : public rclcpp::Node
{
public:
  NNController()
  : Node("nn_controller_node")
  {
    // --------------------------------------------------
    // ROS parameters
    // --------------------------------------------------

    const auto camera_topic =
        declare_parameter<std::string>(
            "camera_topic",
            "/image_raw");

    enable_inference_ =
        declare_parameter<bool>(
            "enable_inference",
            false);

    const auto backend =
        declare_parameter<std::string>(
            "backend",
            "tensorrt");

    const auto model_file =
        declare_parameter<std::string>(
            "model_file",
            "/home/nvidia/donkeycar_ros/cnn_controller.onnx");

    const auto input_name =
        declare_parameter<std::string>(
            "input_name",
            "image");

    const auto output_name =
        declare_parameter<std::string>(
            "output_name",
            "steer");


    // --------------------------------------------------
    // Image configuration
    // --------------------------------------------------

    input_encoding_ =
        declare_parameter<std::string>(
            "input_encoding",
            "rgb8");

    roi_x_ =
        declare_parameter<int>(
            "roi.x",
            0);

    roi_y_ =
        declare_parameter<int>(
            "roi.y",
            0);

    roi_width_ =
        declare_parameter<int>(
            "roi.width",
            0);

    roi_height_ =
        declare_parameter<int>(
            "roi.height",
            0);


    if (input_encoding_ != "rgb8" &&
        input_encoding_ != "mono8")
    {
      throw std::runtime_error(
          "input_encoding must be rgb8 or mono8");
    }

    if (roi_x_ < 0 ||
        roi_y_ < 0 ||
        roi_width_ < 0 ||
        roi_height_ < 0)
    {
      throw std::runtime_error(
          "ROI parameters must be nonnegative");
    }


    // --------------------------------------------------
    // Initialize neural-network backend
    // --------------------------------------------------

    if (enable_inference_)
    {
      const std::string backend_library =
          backend + "_inferencer";

      creator_ =
          boost::dll::import_alias<Creator>(
              backend_library,
              backend_library,
              boost::dll::load_mode::append_decorations |
              boost::dll::load_mode::search_system_folders);

      inferencer_ = creator_();

      RCLCPP_INFO(
          get_logger(),
          "Loaded inferencer backend: %s",
          backend_library.c_str());


      // Load cnn_controller.onnx
      if (!inferencer_->loadModel(model_file))
      {
        throw std::runtime_error(
            "Failed to load model: " +
            inferencer_->getErrorString());
      }

      RCLCPP_INFO(
          get_logger(),
          "Loaded model: %s",
          model_file.c_str());


      // ROS1 allocated output first, then input.
      output_buffer_size_ =
          inferencer_->getOutputBuffer(
              output_name,
              &output_buffer_);

      if (output_buffer_size_ == 0)
      {
        throw std::runtime_error(
            "Failed to allocate output tensor: " +
            inferencer_->getErrorString());
      }


      input_buffer_size_ =
          inferencer_->getInputBuffer(
              input_name,
              &input_buffer_);

      if (input_buffer_size_ == 0)
      {
        throw std::runtime_error(
            "Failed to allocate input tensor: " +
            inferencer_->getErrorString());
      }


      // Steering should be exactly one float.
      if (output_buffer_size_ != sizeof(float))
      {
        throw std::runtime_error(
            "Expected steering output to contain one float");
      }


      RCLCPP_INFO(
          get_logger(),
          "Inferencer ready. Input=%zu bytes, output=%zu bytes",
          input_buffer_size_,
          output_buffer_size_);
    }


    // --------------------------------------------------
    // Camera subscriber
    // --------------------------------------------------

    image_sub_ =
        create_subscription<sensor_msgs::msg::Image>(
            camera_topic,
            rclcpp::SensorDataQoS(),
            std::bind(
                &NNController::imageCallback,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Subscribed to camera topic: %s",
        camera_topic.c_str());
  }


private:

  // Same creator type used by inferencer_cli.cpp.
  using Creator =
      std::shared_ptr<inferencer::Inferencer>();

  boost::function<Creator> creator_;

  std::shared_ptr<inferencer::Inferencer>
      inferencer_;


  // Tensor buffers
  void* input_buffer_ = nullptr;
  void* output_buffer_ = nullptr;

  size_t input_buffer_size_ = 0;
  size_t output_buffer_size_ = 0;


  // ROS
  rclcpp::Subscription<
      sensor_msgs::msg::Image>::SharedPtr image_sub_;


  bool enable_inference_ = false;

  std::string input_encoding_;

  int roi_x_ = 0;
  int roi_y_ = 0;
  int roi_width_ = 0;
  int roi_height_ = 0;

  uint64_t received_images_ = 0;


  // --------------------------------------------------
  // Neural-network inference
  // --------------------------------------------------

  bool inferSteering(
      const cv::Mat &float_image,
      float &steering)
  {
    if (!inferencer_ ||
        !input_buffer_ ||
        !output_buffer_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Inferencer is not initialized");

      return false;
    }


    // Calculate size of prepared image.
    const size_t image_bytes =
        float_image.total() *
        float_image.elemSize();


    // This reproduces the ROS1 size check.
    if (image_bytes != input_buffer_size_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Image byte size %zu does not match "
          "input tensor size %zu",
          image_bytes,
          input_buffer_size_);

      return false;
    }


    // Wrap TensorRT's input buffer with an OpenCV Mat.
    cv::Mat input_mat(
        float_image.rows,
        float_image.cols,
        float_image.type(),
        input_buffer_);


    // Copy prepared camera image directly into
    // TensorRT input memory.
    float_image.copyTo(input_mat);


    // Run the network.
    if (!inferencer_->infer())
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to run inference: %s",
          inferencer_->getErrorString().c_str());

      return false;
    }


    // cnn_controller.onnx produces one float:
    // output tensor "steer".
    steering =
        *static_cast<const float*>(
            output_buffer_);

    return true;
  }


  // --------------------------------------------------
  // Camera callback
  // --------------------------------------------------

  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    ++received_images_;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Image received: %u x %u, encoding=%s, count=%llu",
        msg->width,
        msg->height,
        msg->encoding.c_str(),
        static_cast<unsigned long long>(
            received_images_));


    try
    {
      const auto image =
          cv_bridge::toCvShare(
              msg,
              input_encoding_);


      const int remaining_width =
          image->image.cols - roi_x_;

      const int remaining_height =
          image->image.rows - roi_y_;


      if (remaining_width <= 0 ||
          remaining_height <= 0)
      {
        throw std::runtime_error(
            "ROI origin is outside the image");
      }


      const int width =
          roi_width_ == 0
              ? remaining_width
              : roi_width_;

      const int height =
          roi_height_ == 0
              ? remaining_height
              : roi_height_;


      if (width > remaining_width ||
          height > remaining_height)
      {
        throw std::runtime_error(
            "ROI extends outside the image");
      }


      const cv::Rect roi(
          roi_x_,
          roi_y_,
          width,
          height);


      // ------------------------------------------------
      // Same preparation as ROS1:
      //
      // uint8 RGB → float32 RGB
      //
      // Pixel values stay approximately 0..255.
      // ------------------------------------------------

      cv::Mat float_image;

      image->image(roi).convertTo(
          float_image,
          input_encoding_ == "mono8"
              ? CV_32FC1
              : CV_32FC3);

      double pixel_min, pixel_max;
      cv::minMaxLoc(
          float_image.reshape(1),
          &pixel_min,
          &pixel_max);

      const cv::Scalar avg = cv::mean(float_image);

      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "NN input: encoding=%s, crop=(%d,%d,%d,%d), "
          "channels=%d, min=%.1f, max=%.1f, "
          "mean=(%.1f,%.1f,%.1f)",
          input_encoding_.c_str(),
          roi_x_, roi_y_, width, height,
          float_image.channels(),
          pixel_min, pixel_max,
          avg[0], avg[1], avg[2]);

      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Image prepared: %d x %d, channels=%d, float32",
          float_image.cols,
          float_image.rows,
          float_image.channels());


      // Camera-only mode
      if (!enable_inference_)
      {
        return;
      }


      // ------------------------------------------------
      // Run neural network
      // ------------------------------------------------

      float steering = 0.0f;

      if (inferSteering(
              float_image,
              steering))
      {
        if (!std::isfinite(steering))
        {
          throw std::runtime_error(
              "Inferencer returned nonfinite steering");
        }


        RCLCPP_INFO(
            get_logger(),
            "Predicted steering: %.6f",
            steering);
      }
    }

    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Image processing failed: %s",
          e.what());
    }
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  int result = 0;

  try
  {
    rclcpp::spin(
        std::make_shared<NNController>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(
        rclcpp::get_logger("nn_controller_node"),
        "%s",
        e.what());

    result = 1;
  }

  rclcpp::shutdown();
  return result;
}
