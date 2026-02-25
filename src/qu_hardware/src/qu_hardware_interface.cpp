#include "qu_hardware/qu_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cmath>

namespace qu_hardware
{

QuHardwareInterface::QuHardwareInterface()
  : arduino_driver_(nullptr),
    serial_port_(""),
    baud_rate_(115200),
    num_joints_(0)
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), "Constructor called");
}

QuHardwareInterface::~QuHardwareInterface()
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), "Destructor called");
  
  if (arduino_driver_ && arduino_driver_->isConnected()) {
    arduino_driver_->disconnect();
  }
}

hardware_interface::CallbackReturn QuHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Initializing QuHardwareInterface...");

  if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end()) {
    serial_port_ = info_.hardware_parameters["serial_port"];
    RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
                "Serial port: %s", serial_port_.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("QuHardwareInterface"), 
                 "Parameter 'serial_port' not found!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
                "Baud rate: %d", baud_rate_);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("QuHardwareInterface"), 
                "Parameter 'baud_rate' not found, using default: 115200");
    baud_rate_ = 115200;
  }

  num_joints_ = info_.joints.size();
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Number of joints: %zu", num_joints_);

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  hw_positions_.resize(num_joints_, 1.5708);
  hw_commands_.resize(num_joints_, 1.5708);
  last_commands_.resize(num_joints_, 1.5708);

  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "QuHardwareInterface initialized successfully!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Configuring hardware...");

  try {
    arduino_driver_ = std::make_shared<ArduinoDriver>(serial_port_, baud_rate_);
    
    RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
                "Connecting to Arduino on %s...", serial_port_.c_str());
    
    if (!arduino_driver_->connect()) {
      RCLCPP_ERROR(rclcpp::get_logger("QuHardwareInterface"), 
                   "Failed to connect to Arduino!");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
                "Arduino connected successfully!");
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("QuHardwareInterface"), 
                 "Exception during configuration: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Activating hardware...");

  double safe_position_rad = 1.5708;
  
  for (size_t i = 0; i < num_joints_; ++i) {
    hw_positions_[i] = safe_position_rad;
    hw_commands_[i] = safe_position_rad;
    last_commands_[i] = safe_position_rad;
  }

  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Moving all servos to 90° (standing position)...");
  
  int channels[] = {0, 1, 2, 4, 5, 6, 9, 10, 11, 13, 14, 15};
  for (int i = 0; i < 12; i++) {
    arduino_driver_->setServoAngle(channels[i], 90.0);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Hardware activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Deactivating hardware...");

  if (arduino_driver_ && arduino_driver_->isConnected()) {
    int channels[] = {0, 1, 2, 4, 5, 6, 9, 10, 11, 13, 14, 15};
    for (int i = 0; i < 12; i++) {
      arduino_driver_->setServoAngle(channels[i], 90.0);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("QuHardwareInterface"), 
              "Cleaning up hardware...");

  if (arduino_driver_ && arduino_driver_->isConnected()) {
    arduino_driver_->disconnect();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type QuHardwareInterface::read(
  const rclcpp::Time & /*time*/, 
  const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    hw_positions_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuHardwareInterface::write(
  const rclcpp::Time & /*time*/, 
  const rclcpp::Duration & /*period*/)
{
  struct ServoMapping {
    std::string joint_name;
    int servo_id;
  };
  
  std::vector<ServoMapping> mappings = {
    {"hip_back_right", 0},
    {"lower_leg_back_right", 1},
    {"upper_leg_back_right", 2},
    {"hip_back_left", 4},
    {"upper_leg_back_left", 5},
    {"lower_leg_back_left", 6},
    {"upper_leg_front_right", 9},
    {"lower_leg_front_right", 10},
    {"hip_front_right", 11},
    {"hip_front_left", 13},
    {"upper_leg_front_left", 14},
    {"lower_leg_front_left", 15}
  };

  const double THRESHOLD = 0.001;
  
  for (const auto& mapping : mappings) {
    int joint_idx = -1;
    for (size_t i = 0; i < num_joints_; ++i) {
      if (joint_names_[i] == mapping.joint_name) {
        joint_idx = i;
        break;
      }
    }

    if (joint_idx >= 0) {
      double target_rad = hw_commands_[joint_idx];
      
      if (std::abs(target_rad - last_commands_[joint_idx]) > THRESHOLD) {
        // Convert radians to degrees: 1.5708 rad = 90°
        double target_deg = (target_rad - 1.5708) * 180.0 / M_PI + 90.0;
        
        if (target_deg < 0.0) target_deg = 0.0;
        if (target_deg > 180.0) target_deg = 180.0;
        
        arduino_driver_->setServoAngle(mapping.servo_id, target_deg);
        
        last_commands_[joint_idx] = target_rad;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> 
QuHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]
      )
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
QuHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], 
        hardware_interface::HW_IF_POSITION, 
        &hw_commands_[i]
      )
    );
  }

  return command_interfaces;
}

} // namespace qu_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(qu_hardware::QuHardwareInterface, hardware_interface::SystemInterface)