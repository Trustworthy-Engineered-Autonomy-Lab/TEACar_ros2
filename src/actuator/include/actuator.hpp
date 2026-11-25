// actuator.hpp
#ifndef ACTUATOR__ACTUATOR_HPP_
#define ACTUATOR__ACTUATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "teacar_msgs/msg/motioncmd.hpp"
#include <unordered_map>
#include <string>
#include <vector>
#include <algorithm>

namespace actuator
{
    // Subscribes to "/<name>", collects the latest command from each sender
    // (keyed by header.frame_id), and lets a subclass combine them.
    class ChannelBase
    {
    public:
        ChannelBase(rclcpp::Node *node, const std::string &name) : name(name), node(node)
        {
            callback_handle_ = node->add_on_set_parameters_callback(
                std::bind(&ChannelBase::handle_set_param, this, std::placeholders::_1));

            sub_ = node->create_subscription<teacar_msgs::msg::Motioncmd>(
                name, rclcpp::SensorDataQoS(),
                std::bind(&ChannelBase::on_receive_command, this, std::placeholders::_1));
        }

        virtual ~ChannelBase() = default;

        // Combine the current commands, publish the result and return it.
        virtual float get_control() = 0;

    protected:
        std::string name;
        rclcpp::Node *node; // non-owning: the node owns its channels
        std::unordered_map<std::string, float> cmds;

        // Subclasses handle their own parameters here. Return false with a reason to reject.
        virtual bool on_set_param(const rclcpp::Parameter &, std::string &) { return true; }

    private:
        rclcpp::Subscription<teacar_msgs::msg::Motioncmd>::SharedPtr sub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        void on_receive_command(const teacar_msgs::msg::Motioncmd::SharedPtr msg)
        {
            RCLCPP_DEBUG(node->get_logger(), "Received command %f on channel %s from node: %s",
                         msg->command, name.c_str(), msg->header.frame_id.c_str());

            cmds[msg->header.frame_id] = msg->command;
        }

        rcl_interfaces::msg::SetParametersResult handle_set_param(const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : parameters)
            {
                if (!on_set_param(param, result.reason))
                {
                    result.successful = false;
                    return result;
                }
            }
            return result;
        }
    };

    // Sums all current commands and clamps the result to [-1, 1].
    class SumChannel : public ChannelBase
    {
    public:
        SumChannel(rclcpp::Node *node, const std::string &name) : ChannelBase(node, name)
        {
            pub_ = node->create_publisher<teacar_msgs::msg::Motioncmd>("/combined_" + name, rclcpp::SensorDataQoS());
        }

        float get_control() override
        {
            float combined = 0.0f;
            for (const auto &pair : cmds)
            {
                combined += pair.second;
            }

            // Use manual clamping instead of std::clamp for compatibility
            combined = std::max(-1.0f, std::min(1.0f, combined));

            teacar_msgs::msg::Motioncmd msg;
            msg.header.stamp = node->get_clock()->now();
            msg.header.frame_id = "actuator";
            msg.command = combined;

            pub_->publish(msg);
            RCLCPP_DEBUG(node->get_logger(), "Published combined command %f on channel %s", combined, name.c_str());

            return combined;
        }

    private:
        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr pub_;
    };
}

#endif // ACTUATOR__ACTUATOR_HPP_
