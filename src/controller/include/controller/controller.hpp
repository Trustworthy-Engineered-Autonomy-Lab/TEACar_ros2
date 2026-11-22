#ifndef CONTROLLER__CONTROLLER_HPP_
#define CONTROLLER__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <teacar_msgs/msg/motioncmd.hpp>

#include <cmath>
#include <string>

namespace controller
{
    class Controller : public rclcpp::Node
    {
    public:
        Controller(const std::string &node_name) : rclcpp::Node(node_name)
        {
            throttle_publisher_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/throttle", rclcpp::SensorDataQoS());
            steer_publisher_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/steer", rclcpp::SensorDataQoS());
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Controller::parameterCallback,
                                                                              this, std::placeholders::_1));

            this->declare_parameter<double>("throttle_ratio", 1.0);
            this->declare_parameter<double>("steer_ratio", 1.0);

            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
            RCLCPP_INFO(this->get_logger(), " Controller %s Configuration", this->get_name());
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "%-20s | %-10s", "Parameter", "Value");
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "%-20s | %-10f", "throttle ratio", throttle_ratio_);
            RCLCPP_INFO(this->get_logger(), "%-20s | %-10f", "Steer ratio", steer_ratio_);
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        }

        void control(float throttle, float steer)
        {
            auto throttle_msg = teacar_msgs::msg::Motioncmd();
            throttle_msg.header.stamp = this->get_clock()->now();
            throttle_msg.header.frame_id = this->get_name();

            if (throttle > 0)
            {
                throttle_msg.command = throttle_ratio_ * std::log(1 + throttle * 1.71828);
            }
            else
            {
                throttle_msg.command = -throttle_ratio_ * std::log(1 - throttle * 1.71828);
            }

            throttle_publisher_->publish(throttle_msg);

            RCLCPP_DEBUG(this->get_logger(), "Sent throttle command %f from node %s", throttle_msg.command, this->get_name());

            auto steer_msg = teacar_msgs::msg::Motioncmd();
            steer_msg.header.stamp = this->get_clock()->now();
            steer_msg.header.frame_id = this->get_name();
            steer_msg.command = steer * steer_ratio_;

            steer_publisher_->publish(steer_msg);

            RCLCPP_DEBUG(this->get_logger(), "Sent steer command %f from node %s", steer_msg.command, this->get_name());
        }

    private:
        float throttle_ratio_;
        float steer_ratio_;

        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr throttle_publisher_;
        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr steer_publisher_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
        {
            for (const auto &param : parameters)
            {
                if (param.get_name() == "throttle_ratio")
                {
                    throttle_ratio_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated throttle_ratio: %f", throttle_ratio_);
                }
                else if (param.get_name() == "steer_ratio")
                {
                    steer_ratio_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated steer_ratio: %f", steer_ratio_);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "Parameters updated successfully";
            return result;
        }
    };

}

#endif // CONTROLLER__CONTROLLER_HPP_