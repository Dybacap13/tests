#pragma once
// System

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "thread"

namespace aubo_sdk_client {

struct RobotJointsStates {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> current;
  std::vector<double> temperature;
  std::vector<double> voltage;
};

class AuboSdkInterface {
 public:
  virtual RobotJointsStates getJointStates() = 0;
  virtual void stop() = 0;
  virtual void startup() = 0;
  virtual void powerOff() = 0;
  virtual void powerOn() = 0;
  virtual int getModeRobot() = 0;
  virtual int moveJoint(std::vector<double> target_joint,
                        double acceleration = 0.05, double velocity = 0.5,
                        double blend_radius = 0.1, double duration = 0) = 0;

  virtual int servoJoint(std::vector<double> target_joint,
                         double acceleration = 0.005, double velocity = 0.005,
                         double time = 0.01, double lookahead_time = 0.1,
                         double gain = 200) = 0;

  virtual void stopServoMode() = 0;
  virtual void startServoMode() = 0;
  virtual void setInputState(int8_t pin, double state) = 0;
  virtual void setOutputState(int8_t pin, double state) = 0;
  virtual ~AuboSdkInterface() {}
};
}  // namespace aubo_sdk_client