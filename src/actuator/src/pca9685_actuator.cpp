// pca9685_actuator_node.cpp
#include "actuator.hpp" // Base class
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class PCA9685
{
public:
    PCA9685() = default;
    ~PCA9685() { close(); }

    bool open(const std::string &device_name, float freq = 60, int address = 0x40)
    {
        fd_ = ::open(device_name.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            error_ = "Failed to open I2C device: " + device_name;
            return false;
        }
        if (ioctl(fd_, I2C_SLAVE, address) < 0)
        {
            error_ = "Failed to get I2C access for: " + device_name;
            goto clean_up;
        }

        if (!(reset() && set_pwm_freq(freq)))
            goto clean_up;

        return true;
    clean_up:
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    bool is_opened()
    {
        return fd_ >= 0;
    }

    void close()
    {
        if (fd_ >= 0)
        {
            sleep();
            ::close(fd_);
            fd_ = -1;
        }
    }

    float get_pwm_freq()
    {
        uint8_t prescale;
        if (!read_reg(PRESCALE, &prescale))
            return 0;

        return oscillator_freq / (4096.0 * (prescale + 1));
    }

    bool set_pwm_freq(float freq)
    {
        // Range output modulation frequency is dependant on oscillator
        if (freq < 1)
            freq = 1;
        if (freq > 3500)
            freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

        float prescaleval = ((oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
        if (prescaleval < PCA9685_PRESCALE_MIN)
            prescaleval = PCA9685_PRESCALE_MIN;
        if (prescaleval > PCA9685_PRESCALE_MAX)
            prescaleval = PCA9685_PRESCALE_MAX;
        uint8_t prescale = static_cast<uint8_t>(prescaleval);

        uint8_t oldmode;
        if (!read_reg(MODE1, &oldmode))
            return false;

        // sleep and set prescaler
        uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
        bool success = write_reg(MODE1, newmode) && write_reg(PRESCALE, prescale) && write_reg(MODE1, oldmode);
        if (!success)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // This sets the MODE1 register to turn on auto increment.
        if (!write_reg(MODE1, oldmode | MODE1_RESTART | MODE1_AI))
            return false;

        return true;
    }

    bool set_pwm_dutycycle(int ch, float duty_cycle)
    {
        // OFF count is 12 bits (0-4095). 4096 would set the "full OFF" bit and turn the output off.
        duty_cycle = std::max(0.0f, std::min(1.0f, duty_cycle));
        uint16_t off = std::min<uint16_t>(static_cast<uint16_t>(4096 * duty_cycle), 4095);
        return write_reg(LED0_ON_L + 4 * ch, 0) &&
               write_reg(LED0_ON_H + 4 * ch, 0) &&
               write_reg(LED0_OFF_L + 4 * ch, off & 0xFF) &&
               write_reg(LED0_OFF_H + 4 * ch, off >> 8);
    }

    std::string get_error()
    {
        return std::strerror(errno);
    }

private:
    int fd_ = -1;
    std::string error_;
    static constexpr uint8_t MODE1 = 0x00, PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06, LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08, LED0_OFF_H = 0x09;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;

    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_SLEEP = 0x10;
    static constexpr uint8_t MODE1_AI = 0x20;

    static constexpr float oscillator_freq = 25000000.0f;
    static constexpr uint8_t PCA9685_PRESCALE_MIN = 3;
    static constexpr uint8_t PCA9685_PRESCALE_MAX = 255;

    bool reset()
    {
        if (!write_reg(MODE1, MODE1_RESTART))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return true;
    }

    bool wakeup()
    {
        uint8_t sleep;
        if (!read_reg(MODE1, &sleep))
            return false;
        uint8_t wakeup = sleep & ~MODE1_SLEEP;
        return write_reg(MODE1, wakeup);
    }

    bool sleep()
    {
        uint8_t awake;
        if (!read_reg(MODE1, &awake))
            return false;
        uint8_t sleep = awake | MODE1_SLEEP;
        if (!write_reg(MODE1, sleep))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return true;
    }

    bool write_reg(uint8_t reg, uint8_t val)
    {
        uint8_t buf[2] = {reg, val};
        return write(fd_, buf, 2) == 2;
    }

    bool read_reg(uint8_t reg, uint8_t *val)
    {
        return write(fd_, &reg, 1) == 1 && read(fd_, val, 1) == 1;
    }
};

class PCA9685Channel : actuator::SumChannel
{
public:
    PCA9685Channel(rclcpp::Node *node, const std::string &name) : SumChannel(node, name)
    {
        // Declare before registering the callback: the callback validates each value against
        // the other two, which are not known until all three have been declared.
        min_pw_ = node->declare_parameter(name + ".min_pulsewidth", 1000);
        mid_pw_ = node->declare_parameter(name + ".mid_pulsewidth", 1500);
        max_pw_ = node->declare_parameter(name + ".max_pulsewidth", 2000);
        if (!check_pwm_pulsewidth(min_pw_, max_pw_, mid_pw_))
            throw std::invalid_argument("Invalid pulse widths for channel " + name +
                                        ": require 0 < min < mid < max");

        param_callback_handle_ = node->add_on_set_parameters_callback(
                std::bind(&PCA9685Channel::on_set_param, this, std::placeholders::_1));
    }

    float get_pulsewidth()
    {
        float command = get_control();
        float pw = command >= 0 ? command * (max_pw_ - mid_pw_) + mid_pw_
                                    : command * (mid_pw_ - min_pw_) + mid_pw_;

        return pw;
    }

private:
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int min_pw_;
    int mid_pw_;
    int max_pw_;

    rcl_interfaces::msg::SetParametersResult on_set_param(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Updated parameters successfully";
        for (const auto &param : parameters)
        {
            const std::string &param_name = param.get_name();
            if (param_name == name + ".min_pulsewidth")
            {
                int new_min_pw = param.as_int();
                if(!check_pwm_pulsewidth(new_min_pw, max_pw_, mid_pw_))
                {
                    result.successful = false;
                    result.reason = "Invalid minimum pulse width";
                    break;
                }
                min_pw_ = new_min_pw;
            }
            else if (param_name == name + ".mid_pulsewidth")
            {
                int new_mid_pw = param.as_int();
                if(!check_pwm_pulsewidth(min_pw_, max_pw_, new_mid_pw))
                {
                    result.successful = false;
                    result.reason = "Invalid midpoint pulse width";
                    break;
                }
                mid_pw_ = new_mid_pw;
            }
            else if (param_name == name + ".max_pulsewidth")
            {
                int new_max_pw = param.as_int();
                if(!check_pwm_pulsewidth(min_pw_, new_max_pw, mid_pw_))
                {
                    result.successful = false;
                    result.reason = "Invalid maximum pulse width";
                    break;
                }
                max_pw_ = new_max_pw;
            }
        }

        return result;
    }

    bool check_pwm_pulsewidth(int min, int max, int mid)
    {
        if(min > 0 && mid > min && max > mid)
            return true;
        return false;
    }

};

class PCA9685ActuatorNode : public rclcpp::Node
{
public:
    PCA9685ActuatorNode() : rclcpp::Node("pca9685_actuator_node"),
                            throttle_channel_(this, "throttle"),
                            steer_channel_(this, "steer")
    {
        pca_ = std::make_unique<PCA9685>();
        // Parameter update callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PCA9685ActuatorNode::on_parameter_change, this, std::placeholders::_1));

        this->declare_parameter("bus_device", std::string("/dev/i2c-1"));
        this->declare_parameter("pwm_frequency", 60);
        this->declare_parameter("steer_pwm_channel", 1);
        this->declare_parameter("throttle_pwm_channel", 0);

        this->declare_parameter("control_frequency", 50);

        pca9685_monitor_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PCA9685ActuatorNode::pca9685_monitor_callback, this));
    }

private:
    std::unique_ptr<PCA9685> pca_;
    std::string bus_device_;
    int pwm_freq_;
    int steer_ch_, throttle_ch_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::TimerBase::SharedPtr pca9685_monitor_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    PCA9685Channel throttle_channel_;
    PCA9685Channel steer_channel_;

    void control_loop()
    {
        if (!pca_->is_opened())
            return;

        float steer_pw = steer_channel_.get_pulsewidth();
        float throttle_pw = throttle_channel_.get_pulsewidth();

        float cycle = 1000000.0f / pwm_freq_;

        float steer_duty = steer_pw / cycle;
        float throttle_duty = throttle_pw / cycle;

        RCLCPP_DEBUG(this->get_logger(), "Calculated PWM -> Throttle: %.2f%%, Steer: %.2f%%", throttle_duty * 100, steer_duty * 100);

        if (!pca_->set_pwm_dutycycle(throttle_ch_, throttle_duty))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write throttle PWM: %s", pca_->get_error().c_str());
            pca_->close();
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "✅ Throttle PWM successfully written: duty = %.3f", throttle_duty);
        }

        if (!pca_->set_pwm_dutycycle(steer_ch_, steer_duty))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write steer PWM: %s", pca_->get_error().c_str());
            pca_->close();
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "✅ Steer PWM successfully written: duty = %.3f", steer_duty);
        }
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            const std::string &param_name = param.get_name();
            if (param_name == "bus_device")
            {
                std::string new_bus_device = param.as_string();
                if (new_bus_device != bus_device_)
                {
                    if (!pca_->is_opened())
                    {
                        // If pca9685 is not opened, update the parameter than wait for the monitor to open it.
                        bus_device_ = new_bus_device;
                        RCLCPP_INFO(this->get_logger(), "Bus device is set to %s", bus_device_.c_str());
                    }
                    else
                    {
                        // Otherwise create a new instance and try opening it.
                        std::unique_ptr<PCA9685> new_pca = std::make_unique<PCA9685>();
                        if (!new_pca->open(new_bus_device, pwm_freq_))
                        {
                            RCLCPP_ERROR(this->get_logger(), "Could not open pca9685 on %s. Ignored", new_bus_device.c_str());
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(), "Bus device is set to %s. Opened pca9685 on it.", new_bus_device.c_str());
                            bus_device_ = new_bus_device;
                            pca_ = std::move(new_pca);
                        }
                    }
                }
            }
            else if (param_name == "pwm_frequency")
            {
                pwm_freq_ = param.as_int();
                if (!pca_->is_opened())
                {
                    RCLCPP_ERROR(this->get_logger(), "Parameter value updated to %d. But failed to apply pwm frequency setting: pca9685 is not opened yet.", pwm_freq_);
                }
                else if (!pca_->set_pwm_freq(pwm_freq_))
                {
                    RCLCPP_ERROR(this->get_logger(), "Parameter value updated to %d. But failed to apply pwm frequency setting: %s", pwm_freq_, pca_->get_error().c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "PWM frequencty is set to %d", pwm_freq_);
                }
            }
            else if (param_name == "steer_pwm_channel")
            {
                steer_ch_ = check_pwm_channel(param.as_int(), steer_ch_, "steering");
            }
            else if (param_name == "throttle_pwm_channel")
            {
                throttle_ch_ = check_pwm_channel(param.as_int(), throttle_ch_, "throttle");
            }
            else if (param_name == "control_frequency")
            {
                int freq = param.as_int();
                if (freq <= 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid control_frequency %d. Rejected", freq);
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = false;
                    result.reason = "control_frequency must be positive";
                    return result;
                }
                // Assigning a new timer destroys the old one, which removes it from the executor.
                control_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq),
                                                         std::bind(&PCA9685ActuatorNode::control_loop, this));
                RCLCPP_INFO(this->get_logger(), "Control frequency is set to %d Hz", freq);
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Updated parameters successfully";
        return result;
    }

    int check_pwm_channel(int new_value, int old_value, const std::string &name)
    {
        if (new_value < 0 || new_value > 15)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid pwm channel %d for %s", new_value, name.c_str());
            return old_value;
        }

        RCLCPP_INFO(this->get_logger(), "pwm channel for %s is set to %d", name.c_str(), new_value);
        return new_value;
    }

    void pca9685_monitor_callback()
    {
        if (!pca_->is_opened())
        {
            if (pca_->open(bus_device_, pwm_freq_))
                RCLCPP_INFO(this->get_logger(), "Opened pca9685 on %s", bus_device_.c_str());
            else
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to open pca9685 on %s. Will retry", bus_device_.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCA9685ActuatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}