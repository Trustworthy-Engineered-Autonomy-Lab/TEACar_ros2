// actuator_node.hpp
#ifndef ACTUATOR__ACTUATOR_NODE_HPP_
#define ACTUATOR__ACTUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "teacar_msgs/msg/motioncmd.hpp"
#include <unordered_map>
#include <array>
#include <string>
#include <algorithm>

namespace actuator
{
    class Actuator : public rclcpp::Node
    {

    public:
        Actuator() : Node("actuator_node")
        {
            // Parameter update callback
            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&Actuator::on_parameter_change, this, std::placeholders::_1));

            // Declare and get parameter
            this->declare_parameter<int>("control_frequency", 50);

            // Create publisher and subscriber
            combined_throttle_pub_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/combined_throttle", rclcpp::SensorDataQoS());
            combined_steer_pub_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/combined_steer", rclcpp::SensorDataQoS());

            throttle_sub_ = this->create_subscription<teacar_msgs::msg::Motioncmd>(
                "/throttle", rclcpp::SensorDataQoS(),
                std::bind(&Actuator::throttle_callback, this, std::placeholders::_1));

            steer_sub_ = this->create_subscription<teacar_msgs::msg::Motioncmd>(
                "/steer", rclcpp::SensorDataQoS(),
                std::bind(&Actuator::steer_callback, this, std::placeholders::_1));
        }

    protected:
        virtual void actuate(float throttle, float steer) {}

    private:
        int control_frequency_ = 50;
        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr combined_throttle_pub_;
        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr combined_steer_pub_;
        rclcpp::Subscription<teacar_msgs::msg::Motioncmd>::SharedPtr throttle_sub_;
        rclcpp::Subscription<teacar_msgs::msg::Motioncmd>::SharedPtr steer_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        std::unordered_map<std::string, float> throttle_cmds_;
        std::unordered_map<std::string, float> steer_cmds_;

        void timer_callback()
        {
            float combined_throttle = 0.0f;

            for (const auto &pair : throttle_cmds_)
            {
                combined_throttle += pair.second;
            }

            // Use manual clamping instead of std::clamp for compatibility
            combined_throttle = std::max(-1.0f, std::min(1.0f, combined_throttle));

            teacar_msgs::msg::Motioncmd throttle_msg;
            throttle_msg.header.stamp = this->get_clock()->now();
            throttle_msg.header.frame_id = "actuator";
            throttle_msg.command = combined_throttle;

            if (combined_throttle_pub_)
            {
                combined_throttle_pub_->publish(throttle_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published combined throttle %f", combined_throttle);
            }

            float combined_steer = 0.0f;

            for (const auto &pair : steer_cmds_)
            {
                combined_steer += pair.second;
            }

            combined_steer = std::max(-1.0f, std::min(1.0f, combined_steer));

            teacar_msgs::msg::Motioncmd steer_msg;
            steer_msg.header.stamp = this->get_clock()->now();
            steer_msg.header.frame_id = "actuator";
            steer_msg.command = combined_steer;

            if (combined_steer_pub_)
            {
                combined_steer_pub_->publish(steer_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published combined steer %f", combined_steer);
            }

            actuate(combined_throttle, combined_steer);
        }

        void throttle_callback(const teacar_msgs::msg::Motioncmd::SharedPtr msg)
        {
            RCLCPP_DEBUG(this->get_logger(), "Received throttle command %f from node: %s",
                         msg->command, msg->header.frame_id.c_str());

            throttle_cmds_[msg->header.frame_id] = msg->command;
        }

        void steer_callback(const teacar_msgs::msg::Motioncmd::SharedPtr msg)
        {
            RCLCPP_DEBUG(this->get_logger(), "Received steer command %f from node: %s",
                         msg->command, msg->header.frame_id.c_str());
            
            steer_cmds_[msg->header.frame_id] = msg->command;
        }

        rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
        {
            for (const auto &param : parameters)
            {
                if (param.get_name() == "control_frequency")
                {
                    set_control_frequency(param.as_int());
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "Updated parameters successfully";
            return result;
        }

        void set_control_frequency(int new_freq)
        {
            // Check if the control frequency is valid
            if (new_freq <= 0)
            {
                RCLCPP_WARN(this->get_logger(), "Attempted to set invalid control_frequency %d. Ignored", new_freq);
                return;
            }

            // If the frequency is changed, stop and remove the old timer
            if ((new_freq != control_frequency_) && timer_)
            {
                timer_->reset();
                timer_->cancel();
                timer_.reset();
            }

            control_frequency_ = new_freq;

            // Create the timer
            if (!timer_)
            {
                timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(1.0 / control_frequency_),
                    std::bind(&Actuator::timer_callback, this));
            }

            RCLCPP_INFO(this->get_logger(), "Updated control_frequency to %d Hz", control_frequency_);
        }
    };
}

#endif // ACTUATOR__ACTUATOR_NODE_HPP_