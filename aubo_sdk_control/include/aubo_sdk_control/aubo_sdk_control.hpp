

#pragma once
#include <aubo_msgs/msg/robot_mode.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <aubo_sdk_client/aubo_imitator.hpp>
#include <aubo_sdk_client/aubo_sdk_client.hpp>
#include <aubo_sdk_client/interface/aubo_sdk_interface.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <deque>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
namespace aubo_sdk_control {
class AuboSdkControl {
 public:
  AuboSdkControl(std::string robot_ip);
  ~AuboSdkControl();

  using JointTrajectoryMsg = control_msgs::action::JointTrajectory;
  using JointTrajectoryGoal =
      rclcpp_action::ServerGoalHandle<JointTrajectoryMsg>;

  bool powerOnRobot();
  bool powerOffRobot();
  bool startupRobot();
  void moveJoint(std::vector<double> target_position);
  void servoJoint(std::vector<double> target_position);
  aubo_sdk_client::RobotJointsStates getJointState();

 private:
  std::shared_ptr<std::thread> async_thread_;

  void spinThread();
  void createRosInterface();
  void publisherRobotMode();

  rclcpp::Publisher<aubo_msgs::msg::RobotMode>::SharedPtr robot_mode_pub;
  rclcpp::TimerBase::SharedPtr robot_mode_timer_pub;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr robot_power_on_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr robot_startup_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr robot_power_off_service;
  rclcpp::Service<aubo_msgs::srv::SetIO>::SharedPtr set_io_service;

  void robotPowerOnCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void robotPowerOffCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void robotPowerStartupCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void setIOStateCallback(
      const std::shared_ptr<aubo_msgs::srv::SetIO::Request> request,
      std::shared_ptr<aubo_msgs::srv::SetIO::Response> response);

  std::shared_ptr<rclcpp::Node> aubo_control_node;
  std::shared_ptr<aubo_sdk_client::AuboSdkInterface> aubo_interface;
};
}  // namespace aubo_sdk_control