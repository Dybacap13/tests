#include <aubo_sdk_client/aubo_sdk_client.hpp>
using namespace std::chrono_literals;
namespace aubo_sdk_client {
AuboSdkClient::AuboSdkClient(std::string robot_ip_) {
  std::cout << 33333 << std::endl;
  rpc_client_ = std::make_shared<arcs::aubo_sdk::RpcClient>();

  rpc_client_->setRequestTimeout(1000);
  std::cout << 1 << std::endl;
  rpc_client_->connect(robot_ip_, 30004);
  std::cout << 2 << std::endl;
  rpc_client_->login("aubo", "123456");
  std::cout << 88888 << std::endl;
  rtde_client_ = std::make_shared<arcs::aubo_sdk::RtdeClient>();
  rtde_client_->connect(robot_ip_, 30010);
  rtde_client_->login("aubo", "123456");

  robot_name_ = rpc_client_->getRobotNames().front();
  std::cout << 99999 << std::endl;
  configSubscribe();
}

//*************************************
//      powerOn powerOff startup      *
//*************************************

void AuboSdkClient::startup() {
  auto robot_interface = rpc_client_->getRobotInterface(robot_name_);
  auto robot_mode = rpc_client_->getRobotInterface(robot_name_)
                        ->getRobotState()
                        ->getRobotModeType();
  if (robot_mode != arcs::aubo_sdk::RobotModeType::Idle) {
    std::cout << "Robot is not turned on" << std::endl;
    return;
  }
  if (robot_mode == arcs::aubo_sdk::RobotModeType::Running) {
    return;
  }
  if (!proverochkaModeRobot()) {
    throw std::runtime_error("Strange mode, nichego delat' ne bydy");
  }

  // TODO Переделать учет нагрузки
  double mass = 0.0;
  std::vector<double> cog(3, 0.0);
  std::vector<double> aom(3, 0.0);
  std::vector<double> inertia(6, 0.0);
  robot_interface->getRobotConfig()->setPayload(mass, cog, aom, inertia);

  rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->startup();
  waitForRobotMode(arcs::aubo_sdk::RobotModeType::Running);
}

void AuboSdkClient::powerOn() {
  auto mode = rpc_client_->getRobotInterface(robot_name_)
                  ->getRobotState()
                  ->getRobotModeType();

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
  rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->poweron();
  waitForRobotMode(arcs::aubo_sdk::RobotModeType::Idle);
}

void AuboSdkClient::powerOff() {
  rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->poweroff();
  waitForRobotMode(arcs::aubo_sdk::RobotModeType::PowerOff);
}

//******************
//      get's      *
//******************

int AuboSdkClient::getModeRobot() {
  std::lock_guard<std::mutex> lck(rtde_mtx_);
  return static_cast<int>(robot_mode_);
}

RobotJointsStates AuboSdkClient::getJointStates() {
  std::lock_guard<std::mutex> lck(rtde_mtx_);
  return robot_state;
}

void AuboSdkClient::stop() {}

void AuboSdkClient::stopServoMode() {
  if (!rpc_client_->getRobotInterface(robot_name_)
           ->getMotionControl()
           ->isServoModeEnabled()) {
    return;
  }

  rpc_client_->getRobotInterface(robot_name_)
      ->getMotionControl()
      ->setServoMode(false);

  auto start = std::chrono::high_resolution_clock::now();

  while (rpc_client_->getRobotInterface(robot_name_)
             ->getMotionControl()
             ->isServoModeEnabled()) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start)
            .count() > 15 * 1000) {
      throw std::runtime_error("timeout");
    }
    std::this_thread::sleep_for(5ms);
  }
}

void AuboSdkClient::startServoMode() {
  if (rpc_client_->getRobotInterface(robot_name_)
          ->getMotionControl()
          ->isServoModeEnabled()) {
    return;
  }

  rpc_client_->getRobotInterface(robot_name_)
      ->getMotionControl()
      ->setServoMode(true);
  auto start = std::chrono::high_resolution_clock::now();
  while (!rpc_client_->getRobotInterface(robot_name_)
              ->getMotionControl()
              ->isServoModeEnabled()) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start)
            .count() > 15 * 1000) {
      throw std::runtime_error("timeout");
    }
    std::this_thread::sleep_for(5ms);
  }
}
//*******************
//      move        *
//*******************

int AuboSdkClient::moveJoint(std::vector<double> target_joint,
                             double acceleration, double velocity,
                             double blend_radius, double duration) {
  if (!checkVectorNan(target_joint)) {
    return 0;
  }
  return rpc_client_->getRobotInterface(robot_name_)
      ->getMotionControl()
      ->moveJoint(target_joint, acceleration, velocity, blend_radius, duration);
}

int AuboSdkClient::servoJoint(std::vector<double> target_joint,
                              double acceleration, double velocity, double time,
                              double lookahead_time, double gain) {
  if (!checkVectorNan(target_joint)) {
    return 0;
  }
  if (!rpc_client_->getRobotInterface(robot_name_)
           ->getMotionControl()
           ->isServoModeEnabled()) {
    rpc_client_->getRobotInterface(robot_name_)
        ->getMotionControl()
        ->setServoMode(true);
  }

  return rpc_client_->getRobotInterface(robot_name_)
      ->getMotionControl()
      ->servoJoint(target_joint, acceleration, velocity, time, lookahead_time,
                   gain);
}

bool AuboSdkClient::checkVectorNan(std::vector<double> target) {
  for (const auto& value : target) {
    if (std::isnan(value)) {
      return false;
    }
  }
  return true;
}
//*******************
//      dop         *
//*******************

bool AuboSdkClient::proverochkaModeRobot() {
  auto robot_mode = rpc_client_->getRobotInterface(robot_name_)
                        ->getRobotState()
                        ->getRobotModeType();
  if (robot_mode == arcs::aubo_sdk::RobotModeType::Error) {
    return false;
  }

  if (robot_mode == arcs::aubo_sdk::RobotModeType::Maintaince) {
    return false;
  }

  if (robot_mode == arcs::aubo_sdk::RobotModeType::NoController) {
    return false;
  }
  if (robot_mode == arcs::aubo_sdk::RobotModeType::ConfirmSafety) {
    return false;
  }
  if (robot_mode == arcs::aubo_sdk::RobotModeType::BackDrive) {
    return false;
  }

  if (robot_mode == arcs::aubo_sdk::RobotModeType::PowerOffing) {
    return false;
  }
  if (robot_mode == arcs::aubo_sdk::RobotModeType::Booting) {
    return false;
  }
  return true;
}

void AuboSdkClient::waitForRobotMode(
    arcs::aubo_sdk::RobotModeType target_mode) {
  auto current_mode = rpc_client_->getRobotInterface(robot_name_)
                          ->getRobotState()
                          ->getRobotModeType();
  auto start = std::chrono::high_resolution_clock::now();
  while (current_mode != target_mode) {
    std::this_thread::sleep_for(100ms);
    current_mode = rpc_client_->getRobotInterface(robot_name_)
                       ->getRobotState()
                       ->getRobotModeType();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start)
            .count() > 15 * 1000) {
      throw std::runtime_error("Robot not change mode, timeout");
    }
  }
}

void AuboSdkClient::configSubscribe() {
  int topic1 =
      rtde_client_->setTopic(false,
                             {"R1_actual_q", "R1_actual_qd", "R1_robot_mode",
                              "R1_safety_mode", "runtime_state"},
                             500, 0);

  rtde_client_->subscribe(topic1, [this](arcs::aubo_sdk::InputParser& parser) {
    std::unique_lock<std::mutex> lck(rtde_mtx_);
    robot_state.position = parser.popVectorDouble();
    robot_state.velocity = parser.popVectorDouble();
    robot_mode_ = parser.popRobotModeType();
    safety_mode_ = parser.popSafetyModeType();
    runtime_state_ = parser.popRuntimeState();
    read_state_robot = true;
  });
}

void AuboSdkClient::setInputState(int8_t pin, double state) {
  // rpc_client_->getRobotInterface(robot_name_)
  //     ->getIoControl()
  //     ->setStandardDigitalInputAction(pin, state);
}

void AuboSdkClient::setOutputState(int8_t pin, double state) {
  // arcs::common_interface::StandardInputAction::

  // rpc_client_->getRobotInterface(robot_name_)
  //     ->getIoControl()
  //     ->setStandardDigitalInputAction(pin, state);
}

}  // namespace aubo_sdk_client
