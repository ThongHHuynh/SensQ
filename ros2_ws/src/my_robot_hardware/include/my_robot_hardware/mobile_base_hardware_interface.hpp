#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

// hardware interface syntax
#include "hardware_interface/system_interface.hpp"

#include "my_robot_hardware/serial_port.hpp"
#include <string>
#include <vector>

namespace mobile_base_hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface //system.hpp
{
public: 
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    //System interface override

    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
    std::string port_={"/dev/ttyUSB0"};
    int baud_{115200};
    int timeout_ms_{5};


    //double counts_per_rev_{1440.0};        // encoder counts per wheel revolution (output shaft)
    double counts_per_rev_{449}; 
    // bool invert_left_{false};
    // bool invert_right_{false};
    bool invert_left_cmd_{false};
    bool invert_right_cmd_{false};
    bool invert_left_enc_{false};
    bool invert_right_enc_{false};

    double max_rad_s_{20.0};               // safety clamp

    // Serial
    SerialPort serial_;
    std::string rx_buffer_;

    // State/command
    std::vector<double> hw_pos_;           // rad
    std::vector<double> hw_vel_;           // rad/s
    std::vector<double> hw_cmd_;           // rad/s

    // Encoder tracking
    std::vector<long long> last_counts_;
    bool have_counts_{false};

}; //class MobileBaseHWI 

} //namespace mobile_base_hardware
#endif