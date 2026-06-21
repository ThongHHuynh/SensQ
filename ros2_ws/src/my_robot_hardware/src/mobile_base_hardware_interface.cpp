#include "my_robot_hardware/mobile_base_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <sstream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace mobile_base_hardware
{
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  rx_buffer_.clear();
  have_counts_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Expect exactly 2 joints (left/right)
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Mobile Base Hardware"),
                 "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read params from <ros2_control><hardware><param .../>
  auto get = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  //stoi: string to int
  //stod: string to double
  port_ = get("port", port_); 
  baud_ = std::stoi(get("baud", std::to_string(baud_)));
  timeout_ms_ = std::stoi(get("timeout_ms", std::to_string(timeout_ms_)));
  counts_per_rev_ = std::stod(get("counts_per_rev", std::to_string(counts_per_rev_)));
  // invert_left_ = (get("invert_left", "false") == "true");
  // invert_right_ = (get("invert_right", "false") == "true");

  invert_left_cmd_ = (get("invert_left_cmd", "false") == "true");
  invert_right_cmd_ = (get("invert_right_cmd", "false") == "true");
  invert_left_enc_ = (get("invert_left_enc", "false") == "true");
  invert_right_enc_ = (get("invert_right_enc", "false") == "true");

  max_rad_s_ = std::stod(get("max_rad_s", std::to_string(max_rad_s_)));

  hw_pos_.assign(2, 0.0); //assign array [0,0]
  hw_vel_.assign(2, 0.0);
  hw_cmd_.assign(2, 0.0);
  last_counts_.assign(2, 0);

  rx_buffer_.clear();
  have_counts_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_pos_[0]);
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[0]);
  si.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_pos_[1]);
  si.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[1]);
  return si;
}

std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[0]);
  ci.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[1]);
  return ci;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("------HARDWARE INTERFACE INFO------"),
              "Opening serial %s @ %d", port_.c_str(), baud_);
  RCLCPP_INFO(rclcpp::get_logger("Mobile Base HW Interface"),
              "Opening serial %s @ %d", port_.c_str(), baud_);

  if (!serial_.open(port_, baud_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveSerialSystem"),
                 "FAILED TO OPEN SERIAL PORT %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset runtime state
  rx_buffer_.clear();
  have_counts_ = false;
  std::fill(hw_pos_.begin(), hw_pos_.end(), 0.0);
  std::fill(hw_vel_.begin(), hw_vel_.end(), 0.0);
  std::fill(hw_cmd_.begin(), hw_cmd_.end(), 0.0);
  std::fill(last_counts_.begin(), last_counts_.end(), 0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Optionally stop motors
  serial_.write_string("V 0 0\n");
  serial_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!serial_.is_open())
    return hardware_interface::return_type::ERROR;

  // Pull available bytes
  char tmp[256];
  int n = serial_.read_bytes(tmp, sizeof(tmp), timeout_ms_);
  if (n > 0)
    rx_buffer_.append(tmp, tmp + n);

  // Parse complete lines
  std::size_t pos_nl;
  while ((pos_nl = rx_buffer_.find('\n')) != std::string::npos)
  {
    std::string line = rx_buffer_.substr(0, pos_nl);
    rx_buffer_.erase(0, pos_nl + 1);

    // Expected: "C left right"
    if (line.size() < 1) continue;
    if (line[0] != 'C') continue;

    std::istringstream iss(line); //create string stream
    char tag;
    long long lc, rc;
    if (!(iss >> tag >> rc >> lc)) continue; //read tag -> left commanf -> right command

    if (invert_left_enc_)  lc = -lc;
    if (invert_right_enc_) rc = -rc;

    const double dt = std::max(1e-6, period.seconds());
    const double rad_per_count = 2.0 * M_PI / std::max(1.0, counts_per_rev_);

    if (!have_counts_)
    {
      last_counts_[0] = lc;
      last_counts_[1] = rc;
      have_counts_ = true;
      continue;
    }

    const long long dl = lc - last_counts_[0];
    const long long dr = rc - last_counts_[1];
    last_counts_[0] = lc;
    last_counts_[1] = rc;

    const double dtheta_l = static_cast<double>(dl) * rad_per_count;
    const double dtheta_r = static_cast<double>(dr) * rad_per_count;

    hw_pos_[0] += dtheta_l;
    hw_pos_[1] += dtheta_r;

    hw_vel_[0] = dtheta_l / dt;
    hw_vel_[1] = dtheta_r / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!serial_.is_open())
    return hardware_interface::return_type::ERROR;

  auto clamp = [&](double v) {
    return std::max(-max_rad_s_, std::min(max_rad_s_, v));
  };

  double wl = clamp(hw_cmd_[0]);
  double wr = clamp(hw_cmd_[1]);

  if (invert_left_cmd_)  wl = -wl;
  if (invert_right_cmd_) wr = -wr;

  std::ostringstream oss; //create string builder
  oss.setf(std::ios::fixed); //set fixed format
  oss.precision(4); //4 dec
  oss << "V " << wr << " " << wl << "\n";

  if (!serial_.write_string(oss.str()))
    return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}

}  // namespace mobile_base_hardware

//MAKE CLASS A PLUGIN namespace/classname
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
