
#include <aubo_sdk_control/aubo_sdk_control.hpp>
namespace aubo_sdk_control {

AuboSdkControl::AuboSdkControl(std::string robot_ip)
    : aubo_control_node(std::make_shared<rclcpp::Node>("aubo_control_node")) {
  RCLCPP_INFO_STREAM(aubo_control_node->get_logger(), "AuboSdkControl : ready");
  createRosInterface();

  aubo_interface = std::make_shared<aubo_sdk_client::AuboImitator>(robot_ip);

  async_thread_ =
      std::make_shared<std::thread>(&AuboSdkControl::spinThread, this);
}

AuboSdkControl::~AuboSdkControl() {
  aubo_interface->stopServoMode();
  // aubo_interface->powerOff();
  async_thread_->join();
}

void AuboSdkControl::spinThread() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(aubo_control_node);
  executor.spin();
}

void AuboSdkControl::createRosInterface() {
  robot_mode_pub =
      aubo_control_node->create_publisher<aubo_msgs::msg::RobotMode>(
          "/robot_mode", 5);
  robot_mode_timer_pub = aubo_control_node->create_wall_timer(
      500ms, std::bind(&AuboSdkControl::publisherRobotMode, this));

  robot_power_on_service =
      aubo_control_node->create_service<std_srvs::srv::Trigger>(
          "robot/power_on",
          std::bind(&AuboSdkControl::robotPowerOnCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  robot_power_off_service =
      aubo_control_node->create_service<std_srvs::srv::Trigger>(
          "robot/power_off",
          std::bind(&AuboSdkControl::robotPowerOffCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  robot_startup_service =
      aubo_control_node->create_service<std_srvs::srv::Trigger>(
          "robot/startup",
          std::bind(&AuboSdkControl::robotPowerStartupCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  set_io_service = aubo_control_node->create_service<aubo_msgs::srv::SetIO>(
      "robot/set_ip", std::bind(&AuboSdkControl::setIOStateCallback, this,
                                std::placeholders::_1, std::placeholders::_2));
}

void AuboSdkControl::servoJoint(std::vector<double> target_position) {
  try {
    aubo_interface->servoJoint(target_position);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(aubo_control_node->get_logger(), "Error servoJoint: %s",
                 e.what());
  }
}

void AuboSdkControl::publisherRobotMode() {
  aubo_msgs::msg::RobotMode msg;
  msg.robot_mode = aubo_interface->getModeRobot();
  robot_mode_pub->publish(msg);
}

void AuboSdkControl::robotPowerOnCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    aubo_interface->powerOn();
    response->success = true;
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  }
}
void AuboSdkControl::robotPowerOffCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    aubo_interface->powerOff();
    response->success = true;
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  }
}
void AuboSdkControl::robotPowerStartupCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    aubo_interface->startup();
    response->success = true;
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  }
}

void AuboSdkControl::setIOStateCallback(
    const std::shared_ptr<aubo_msgs::srv::SetIO::Request> request,
    std::shared_ptr<aubo_msgs::srv::SetIO::Response> response) {
  // request->
}
aubo_sdk_client::RobotJointsStates AuboSdkControl::getJointState() {
  return aubo_interface->getJointStates();
}

bool AuboSdkControl::powerOnRobot() {
  try {
    aubo_interface->powerOn();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(aubo_control_node->get_logger(), "Error powerOnRobot: %s",
                 e.what());
    return false;
  }
}
bool AuboSdkControl::powerOffRobot() {
  try {
    aubo_interface->powerOff();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(aubo_control_node->get_logger(), "Error powerOffRobot: %s",
                 e.what());
    return false;
  }
}
bool AuboSdkControl::startupRobot() {
  try {
    aubo_interface->startup();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(aubo_control_node->get_logger(), "Error startupRobot: %s",
                 e.what());
    return false;
  }
}
}  // namespace aubo_sdk_control
