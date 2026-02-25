#ifndef QU_HARDWARE__QU_HARDWARE_INTERFACE_HPP_
#define QU_HARDWARE__QU_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "qu_hardware/arduino_driver.hpp"

namespace qu_hardware
{

class QuHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(QuHardwareInterface)

  QuHardwareInterface();
  virtual ~QuHardwareInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  std::shared_ptr<ArduinoDriver> arduino_driver_;
  std::string serial_port_;
  int baud_rate_;
  
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> last_commands_;  // Track last sent commands
};

}  // namespace qu_hardware

#endif  // QU_HARDWARE__QU_HARDWARE_INTERFACE_HPP_