#ifndef CONTROLLER__CONTROLLER_HPP_
#define CONTROLLER__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <teacar_msgs/msg/motioncmd.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <memory>

namespace controller
{
    class ChannelBase
    {
    public:
        ChannelBase(rclcpp::Node *node, const std::string& name):name(name), node(node)
        {
            pub_ = node->create_publisher<teacar_msgs::msg::Motioncmd>("/" + name, rclcpp::SensorDataQoS());
            callback_handle_ = node->add_on_set_parameters_callback(std::bind(&ChannelBase::on_set_param,
                                                                              this, std::placeholders::_1));
        }

        virtual void update_control(float value) = 0;

    protected:
        std::string name;
        rclcpp::Node *node;  // non-owning: the node owns its channels

        void send_command(float command)
        {
            auto msg = teacar_msgs::msg::Motioncmd();
            msg.header.stamp = node->get_clock()->now();
            msg.header.frame_id = node->get_name();
            msg.command = command;

            pub_->publish(msg);

            RCLCPP_DEBUG(node->get_logger(), "Sent command %f from channel %s of node %s", msg.command, name.c_str(), node->get_name());
        }

        virtual rcl_interfaces::msg::SetParametersResult on_set_param(const std::vector<rclcpp::Parameter> &parameters) = 0;

    private:
        rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr pub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };

    class LinearChannel: public ChannelBase
    {
    public:
        LinearChannel(rclcpp::Node *node, const std::string& name): ChannelBase(node, name)
        {
            ratio_ = node->declare_parameter<double>(name + ".ratio", 1.0);
        }

        virtual void update_control(float value) override
        {
            send_command(value * ratio_);
        }

    private:
        float ratio_ = 1.0f;

        virtual rcl_interfaces::msg::SetParametersResult on_set_param(const std::vector<rclcpp::Parameter> &parameters) override
        {
            for (const auto &param : parameters)
            {
                if (param.get_name() == name + ".ratio")
                {
                    ratio_ = param.as_double();
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