#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

// System
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/visibility_control.h"

// ROS
#include <aubo/robot/robot_state.h>
#include <aubo_dashboard_msgs/msg/robot_mode.h>

#include <aubo_sdk_control/aubo_sdk_control.hpp>

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serviceinterface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "thread"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
namespace aubo_driver {

class AuboHardwareInterface : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state);
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &system_info) final;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      final;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) final;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) final;

 private:
  std::shared_ptr<aubo_sdk_control::AuboSdkControl> aubo_control;

  std::vector<double> joint_position_state;
  std::vector<double> joint_velocity_state;
  std::vector<double> joint_position_command;
  std::vector<double> joint_velocity_command;
};
}  // namespace aubo_driver

#endif
