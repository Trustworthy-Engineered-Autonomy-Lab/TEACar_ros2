#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>


class NNController : public rclcpp::Node {
public:
  NNController() : Node("nn_controller_node") {
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/image_raw");
    enable_inference_ = declare_parameter<bool>("enable_inference", false);


    // Future inferencer configuration. These values are NOT loaded or used yet.
    declare_parameter<std::string>("backend", "");
    declare_parameter<std::string>("model_file", "");
    declare_parameter<std::string>("input_name", "input");
    declare_parameter<std::string>("output_name", "outputs");


    input_encoding_ = declare_parameter<std::string>("input_encoding", "rgb8");
    roi_x_ = declare_parameter<int>("roi.x", 0);
    roi_y_ = declare_parameter<int>("roi.y", 0);
    roi_width_ = declare_parameter<int>("roi.width", 0);
    roi_height_ = declare_parameter<int>("roi.height", 0);
    if (input_encoding_ != "rgb8" && input_encoding_ != "mono8")
      throw std::runtime_error("input_encoding must be rgb8 or mono8");
    if (roi_x_ < 0 || roi_y_ < 0 || roi_width_ < 0 || roi_height_ < 0)
      throw std::runtime_error("ROI parameters must be nonnegative");


    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, rclcpp::SensorDataQoS(),
      std::bind(&NNController::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to camera topic: %s", camera_topic.c_str());
    if (enable_inference_)
      RCLCPP_WARN(get_logger(),
        "ROS2 inferencer is not implemented. Camera processing will run, but no steering prediction will be produced.");
  }


private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  bool enable_inference_ = false;
  bool logged_inferencer_pending_ = false;
  std::string input_encoding_;
  int roi_x_, roi_y_, roi_width_, roi_height_;
  uint64_t received_images_ = 0;


  bool inferSteering(const cv::Mat &float_image, float &steering) {
    // TODO: Connect the real ROS2 inferencer here when it is available.
    // 1. Load backend/model once during initialization, not on every image.
    // 2. Validate tensor names, shapes, layout, float32 type, and buffer sizes.
    // 3. Copy float_image into the input tensor using the model's required layout.
    // 4. Call the real inferencer and check success.
    // 5. Read its single float steering output into steering and return true.
    // Return false until a real prediction can be obtained. No dummy steering.
    (void)float_image;
    (void)steering;
    if (!logged_inferencer_pending_) {
      RCLCPP_WARN(get_logger(), "Inferencer connection pending; no steering output.");
      logged_inferencer_pending_ = true;
    }
    return false;
  }


  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    ++received_images_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Image received: %u x %u, encoding=%s, count=%llu",
      msg->width, msg->height, msg->encoding.c_str(),
      static_cast<unsigned long long>(received_images_));


    try {
      const auto image = cv_bridge::toCvShare(msg, input_encoding_);
      const int remaining_width = image->image.cols - roi_x_;
      const int remaining_height = image->image.rows - roi_y_;
      if (remaining_width <= 0 || remaining_height <= 0)
        throw std::runtime_error("ROI origin is outside the image");
      const int width = roi_width_ == 0 ? remaining_width : roi_width_;
      const int height = roi_height_ == 0 ? remaining_height : roi_height_;
      if (width > remaining_width || height > remaining_height)
        throw std::runtime_error("ROI extends outside the image");
      const cv::Rect roi(roi_x_, roi_y_, width, height);
      cv::Mat float_image;
      image->image(roi).convertTo(float_image,
        input_encoding_ == "mono8" ? CV_32FC1 : CV_32FC3);
      // ROS1 reference: float32, interleaved channels, pixels 0..255.
      // There is no resizing or normalization. Confirm with the future model.
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Image prepared: %d x %d, channels=%d, float32",
        float_image.cols, float_image.rows, float_image.channels());


      if (!enable_inference_) return;
      float steering = 0.0f; // Not a prediction; used only if inferSteering succeeds.
      if (inferSteering(float_image, steering)) {
        if (!std::isfinite(steering))
          throw std::runtime_error("Inferencer returned nonfinite steering");
        RCLCPP_INFO(get_logger(), "Predicted steering: %.6f", steering);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Image processing failed: %s", e.what());
    }
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<NNController>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("nn_controller_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
