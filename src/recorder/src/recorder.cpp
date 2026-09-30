#include <rclcpp/rclcpp.hpp>
#include <teacar_msgs/msg/motioncmd.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <filesystem>
#include <fstream>
#include <ctime>

class RecorderNode : public rclcpp::Node
{
public:
    RecorderNode() : rclcpp::Node("recorder_node"),
                     image_sub_(dynamic_cast<rclcpp::Node *>(this), "/camera/image_raw"),
                     motion_cmd_sub_(dynamic_cast<rclcpp::Node *>(this), "/combined_motion_cmd"),
                     sync_sub_(ApproxSyncPolicy(10), image_sub_, motion_cmd_sub_)
    {
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RecorderNode::on_parameter_change, this, std::placeholders::_1));

        std::filesystem::path default_data_path = std::filesystem::temp_directory_path() / "collect_%Y_%m_%d_%H_%M_%S";

        this->declare_parameter<bool>("enable", false);
        this->declare_parameter<std::string>("data_folder", default_data_path.string());
        this->declare_parameter<int>("downsample_rate", 1);
        this->declare_parameter<int>("record_button", 5);
        this->declare_parameter<bool>("compress_on_exit", true);
        this->declare_parameter<std::string>("data_folder_resolved", "");

        sync_sub_.registerCallback(std::bind(&RecorderNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&RecorderNode::joy_callback, this, std::placeholders::_1));
        saved_count_pub_ = this->create_publisher<std_msgs::msg::UInt64>("/recorder/saved_count", 10);
    }

    ~RecorderNode()
    {
        std::filesystem::path data_folder = image_folder_.parent_path();
        if (std::filesystem::exists(data_folder))
        {
            label_file_.close();
            compress_data_file(data_folder);
        }
    }

    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                             teacar_msgs::msg::Motioncmd>;

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<teacar_msgs::msg::Motioncmd> motion_cmd_sub_;

    message_filters::Synchronizer<ApproxSyncPolicy> sync_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr saved_count_pub_;

    std::filesystem::path image_folder_;
    std::filesystem::path label_file_path_;
    std::ofstream label_file_;

    uint64_t image_count_ = 0;
    uint64_t saved_image_count_ = 0;
    uint64_t last_saved_image_count_ = 0;

    bool enable_from_js_ = false;
    bool enable_from_param_ = false;

    int record_button_ = 5;
    int record_button_state_ = 0;
    int downsample_rate_ = 1;

    bool datafolder_created_ = false;
    bool compress_on_exit_ = false;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        try
        {
            if (msg->buttons[record_button_])
            {
                if (msg->buttons[record_button_] != record_button_state_)
                    RCLCPP_INFO(this->get_logger(), "Start recording");

                enable_from_js_ = true;
            }
            else
            {
                if (msg->buttons[record_button_] != record_button_state_)
                {
                    RCLCPP_INFO(this->get_logger(), "Stop recording, saved %ld images, there are %ld images in total", saved_image_count_ - last_saved_image_count_, saved_image_count_);
                    last_saved_image_count_ = saved_image_count_;
                }

                enable_from_js_ = false;
            }

            record_button_state_ = msg->buttons[record_button_];
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invaild button number: %s", e.what());
        }
    }

    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr image,
                       const teacar_msgs::msg::Motioncmd::ConstSharedPtr motion_cmd_msg)
    {
        bool enable = enable_from_param_ || enable_from_js_;

        if (!enable)
            return;

        if (image_count_ % downsample_rate_ == 0)
        {

            cv_bridge::CvImageConstPtr cvImage;
            try
            {
                cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
                return;
            }
            if (!std::filesystem::exists(image_folder_))
            {
                std::error_code ec;
                std::filesystem::create_directories(image_folder_, ec);
                if (ec)
                {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to create %s: %s", image_folder_.c_str(), ec.message().c_str());
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Created folder %s", std::filesystem::absolute(image_folder_).c_str());
            }
            if (!std::filesystem::exists(label_file_path_))
            {
                label_file_ = std::ofstream(label_file_path_.string());
                RCLCPP_INFO(this->get_logger(), "Opened label file %s", label_file_path_.c_str());
            }

            std::string image_name = std::to_string(saved_image_count_ + 1) + ".jpg";
            std::filesystem::path image_path = image_folder_ / image_name;

            cv::imwrite(image_path.string(), cvImage->image);
            label_file_ << image_name << "," << motion_cmd_msg->steer << "," << motion_cmd_msg->throttle << std::endl;
            RCLCPP_DEBUG(this->get_logger(), "Image %s saved!", image_path.c_str());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Saved %d images", saved_image_count_);

            saved_image_count_ += 1;
            std_msgs::msg::UInt64 msg;
            msg.data = saved_image_count_;
            saved_count_pub_->publish(msg);
        }

        image_count_ += 1;
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            const std::string &param_name = param.get_name();
            if (param_name == "enable")
            {
                enable_from_param_ = param.as_bool();
            }
            else if (param_name == "data_folder")
            {
                if (enable_from_js_ || enable_from_param_)
                {
                    RCLCPP_ERROR(this->get_logger(), "Changing the data folder is not possible while recording");
                }
                else
                {
                    std::filesystem::path data_folder = make_file_name(param.as_string());
                    if (data_folder != image_folder_.parent_path())
                    {
                        rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
                            std::chrono::milliseconds(1),
                            [this,timer=timer, data_folder=data_folder.string()]() {
                                this->set_parameter(rclcpp::Parameter("data_folder_resolved", data_folder));;
                                timer->cancel();  // run once
                            }
                        );
                        label_file_.close();
                        image_folder_ = data_folder / "images";
                        label_file_path_ = data_folder / "labels.csv";
                        image_count_ = 0;
                        saved_image_count_ = 0;
                    }
                }
            }
            else if (param_name == "downsample_rate")
            {
                int new_downsample_rate = param.as_int();
                if (new_downsample_rate <= 0)
                    RCLCPP_ERROR(this->get_logger(), "Invalid downsampling rate %d", new_downsample_rate);
                else
                    downsample_rate_ = new_downsample_rate;
            }
            else if (param_name == "record_button")
            {
                if (enable_from_js_ || enable_from_param_)
                    RCLCPP_ERROR(this->get_logger(), "Changing the record button is not possible while recording");
                else
                    record_button_ = param.as_int();
            }
            else if (param_name == "compress_on_exit")
            {
                compress_on_exit_ = param.as_bool();
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Joystick axis parameters updated";
        return result;
    }

    std::filesystem::path make_file_name(const std::string &pattern)
    {
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);

        char buffer[256];
        std::strftime(buffer, sizeof(buffer), pattern.c_str(), &tm);

        return std::filesystem::path(buffer);
    }

    void compress_data_file(const std::filesystem::path &data_folder)
    {
        std::filesystem::path tar_folder = data_folder.parent_path();
        std::filesystem::path tar_name = data_folder.filename().replace_extension("tar.gz");
        std::filesystem::path tar_path = tar_folder / tar_name;

        std::string cmd = "tar -czf '" + tar_path.string() + "' -C '" + data_folder.string() + "' .";
        if (!std::system(cmd.c_str()))
            RCLCPP_ERROR(this->get_logger(), "Failed to compress the data folder");
        else
            RCLCPP_INFO(this->get_logger(), "Saved compressed data folder to " + tar_path.string());
    }
};

// Entry point
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
