#include "aubo_hardware_interface.h"

#include <ctime>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
namespace aubo_driver {

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface
    // on each joint
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' has %zu state interface. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  joint_position_state.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());
  joint_velocity_state.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  joint_position_command.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
  joint_velocity_command.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());

  const std::string robot_ip_ = info_.hardware_parameters["robot_ip"];
  aubo_control = std::make_shared<aubo_sdk_control::AuboSdkControl>(robot_ip_);

  if (!aubo_control->powerOnRobot()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
              "Starting ...please wait...");
  if (!aubo_control->startupRobot()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t joint = 0; joint < info_.joints.size(); joint++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[joint].name, hardware_interface::HW_IF_POSITION,
        &joint_position_state[joint]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[joint].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_state[joint]));
  }

  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t index = 0; index < info_.joints.size(); index++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[index].name, hardware_interface::HW_IF_POSITION,
        &joint_position_command[index]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_command[index]));
  }

  return command_interfaces;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  auto position = aubo_control->getJointState().position;

  auto velocity = aubo_control->getJointState().velocity;
  if (!position.empty()) {
    for (size_t index = 0; index < joint_position_state.size(); index++) {
      joint_position_state[index] = position[index];
    }
  }
  if (!velocity.empty()) {
    for (size_t index = 0; index < joint_velocity_state.size(); index++) {
      joint_velocity_state[index] = velocity[index];
    }
  }

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  aubo_control->servoJoint(joint_position_command);
  return hardware_interface::return_type::OK;
}

}  // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
