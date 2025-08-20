#include <aubo_sdk_client/aubo_imitator.hpp>
using namespace std::chrono_literals;
namespace aubo_sdk_client {
AuboImitator::AuboImitator(std::string robot_ip_) : terminated(false) {
  mode = arcs::aubo_sdk::RobotModeType::PowerOff;
  robot_state.position.resize(6, 0.0);
  robot_state.velocity.resize(6, 0.0);
  robot_state.current.resize(6, 0.0);
  ctime = 1.0 / rate * 1000 * 1000;  // microseconds
  cycle_thread = std::thread(&AuboImitator::cycle, this);
}

void AuboImitator::cycle() {
  auto cycletime = std::chrono::microseconds(this->ctime);
  while (!terminated.load()) {
    auto start = std::chrono::high_resolution_clock::now();
    update();
    auto end = std::chrono::high_resolution_clock::now();
    const auto delta = end - start;
    if (delta > cycletime) {
      // std::cout << "System too slow for cycle time " << cycletime.count()
      //           << "ms sending takes " << delta.count() << "ns" << std::endl;
    } else {
      std::this_thread::sleep_for(cycletime - delta);
    }
  }
}

AuboImitator::~AuboImitator() {
  terminated = true;
  cycle_thread.join();
}

void AuboImitator::update() {
  if (mode != arcs::aubo_sdk::RobotModeType::Running) {
    return;
  }
  moveByTrajectory();
}

//*************************************
//      powerOn powerOff startup      *
//*************************************

void AuboImitator::startup() {
  if (mode != arcs::aubo_sdk::RobotModeType::Idle) {
    std::cout << "Robot is not turned on" << std::endl;
    return;
  }
  if (mode == arcs::aubo_sdk::RobotModeType::Running) {
    return;
  }

  mode = arcs::aubo_sdk::RobotModeType::Running;
}

void AuboImitator::powerOn() {
  if (mode == arcs::aubo_sdk::RobotModeType::Idle) {
    return;
  }
  if (mode == arcs::aubo_sdk::RobotModeType::Running) {
    return;
  }
  if (mode != arcs::aubo_sdk::RobotModeType::PowerOff) {
    std::cout << "The robot is in the mode: " +
                     std::to_string(static_cast<int>(mode)) +
                     " power on cancelled"
              << std::endl;
    return;
  }
  mode = arcs::aubo_sdk::RobotModeType::Idle;
}

void AuboImitator::powerOff() {
  mode = arcs::aubo_sdk::RobotModeType::PowerOff;
}

//******************
//      get's      *
//******************

int AuboImitator::getModeRobot() { return static_cast<int>(mode); }

RobotJointsStates AuboImitator::getJointStates() {
  const std::lock_guard<std::mutex> lock(target_position_mutex);
  return robot_state;
}

void AuboImitator::stop() {}

void AuboImitator::stopServoMode() {
  mode = arcs::aubo_sdk::RobotModeType::Idle;
}

void AuboImitator::startServoMode() {
  mode = arcs::aubo_sdk::RobotModeType::Running;
}
//*******************
//      move        *
//*******************
void AuboImitator::moveByTrajectory() {
  const std::lock_guard<std::mutex> lock(target_position_mutex);
  if (comparetePosition(target_position, robot_state.position)) {
    return;
  }
  for (size_t index = 0; index < robot_state.position.size(); index++) {
    if (robot_state.position[index] < target_position[index]) {
      robot_state.position[index] +=
          std::min(profile_velocity / rate,
                   target_position[index] - robot_state.position[index]);
    } else {
      robot_state.position[index] = target_position[index];
    }
  }
}

bool AuboImitator::comparetePosition(std::vector<double> current_position,
                                     std::vector<double> target_position) {
  for (size_t index = 0; index < current_position.size(); index++) {
    if (current_position[index] != target_position[index]) {
      return false;
    }
  }
  return true;
}
int AuboImitator::moveJoint(std::vector<double> target_joint,
                            double acceleration, double velocity,
                            double blend_radius, double duration) {
  if (!checkVectorNan(target_joint)) {
    return 0;
  }
  const std::lock_guard<std::mutex> lock(target_position_mutex);
  target_position = target_joint;
  return 1;
}

int AuboImitator::servoJoint(std::vector<double> target_joint,
                             double acceleration, double velocity, double time,
                             double lookahead_time, double gain) {
  if (!checkVectorNan(target_joint)) {
    return 0;
  }
  const std::lock_guard<std::mutex> lock(target_position_mutex);
  target_position = target_joint;
  return 1;
}

bool AuboImitator::checkVectorNan(std::vector<double> target) {
  for (const auto& value : target) {
    if (std::isnan(value)) {
      return false;
    }
  }
  return true;
}

void AuboImitator::setInputState(int8_t pin, double state) {}
void AuboImitator::setOutputState(int8_t pin, double state) {}
//*******************
//      dop         *
//*******************

}  // namespace aubo_sdk_client
