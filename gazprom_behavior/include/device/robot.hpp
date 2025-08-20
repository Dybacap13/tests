#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gazprom::device {
using Response = std_srvs::srv::Trigger::Response;
class Robot {
 public:
  Robot(std::shared_ptr<rclcpp::Node> node);

  std::shared_future<std::shared_ptr<Response>> stop();
  std::shared_future<std::shared_ptr<Response>> startup();
  std::shared_future<std::shared_ptr<Response>> powerOff();
  std::shared_future<std::shared_ptr<Response>> powerOn();

  std::shared_future<std::shared_ptr<Response>> moveJoint(
      std::vector<double> target_joint, double acceleration = 0.05,
      double velocity = 0.5, double blend_radius = 0.1, double duration);

  std::shared_future<std::shared_ptr<Response>> servoJoint(
      std::vector<double> target_joint, double acceleration = 0.005,
      double velocity = 0.005, double time = 0.01, double lookahead_time = 0.1,
      double gain = 200);

  std::shared_future<std::shared_ptr<Response>> stopServoMode();
  std::shared_future<std::shared_ptr<Response>> startServoMode();
  std::shared_future<std::shared_ptr<Response>> setInputState(int8_t pin,
                                                              double state);
  std::shared_future<std::shared_ptr<Response>> setOutputState(int8_t pin,
                                                               double state);

 private:
  void createService();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr startup_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_input_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_output_client;
};
}  // namespace gazprom::device