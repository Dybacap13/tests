
#pragma once
#include <aubo/robot/robot_state.h>
#include <aubo_dashboard_msgs/msg/robot_mode.h>

#include <atomic>
#include <aubo_sdk_client/interface/aubo_sdk_interface.hpp>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include "serviceinterface.h"

namespace aubo_sdk_client {
class AuboImitator : public AuboSdkInterface {
 public:
  AuboImitator(std::string ip_robot);
  ~AuboImitator();
  RobotJointsStates getJointStates() override;

  /**
   * Подразумевается остановка робота во время выполнения траектории - не
   * реализовано
   */
  void stop() override;

  /**
   * Перевод робота в рабочий режим: снятие с тормозов, подразумевается, что
   * питание на робота уже подано
   */
  void startup() override;

  /**
   * Выключение питания робота
   */
  void powerOff() override;

  /**
   * Включение питания робота
   */
  void powerOn() override;
  /**
   * Получить текущий режим робота
   * @return режим робота в соответсвии с
   * ros2_aubo_ws/src/aubo_msgs/msg/RobotMode.msg
   */

  int getModeRobot() override;

  /**
   * Делает moveJoint
   *
   * @param target_joint - целевая позиция
   * @param acceleration - ускорение
   * @param velocity - скорость
   * @param blend_radius - радиус сглаживания или что-то в этом роде
   * @param duration - продолжительность движения
   * Когда продолжительность = 0, время выполнения
   * будет рассчитано на основе скорости и ускорения.
   * Если продолжительность не
   * равна 0, скорость и ускорение будут проигнорированы.
   * @return статус выполнения команды, в случае успеха 0
   */

  int moveJoint(std::vector<double> target_joint, double acceleration = 0.05,
                double velocity = 0.5, double blend_radius = 0.1,
                double duration = 0) override;

  /**
   * Делает servoJoint
   *
   * @param target_joint - целевая позиция
   * @param acceleration - ускорение
   * @param velocity - скорость
   * @param time - время работы
   * @param lookahead_time - не понятно
   * @param gain - пропорциональный коэффициент усиления Пропорциональный
   * коэффициент усиления для отслеживания целевого положения [100, 200],
   * используется для управления плавностью и точностью движения. Чем больше
   * пропорциональный коэффициент усиления, тем больше времени требуется для
   * достижения целевого положения и тем меньше перерегулирование.
   * @return статус выполнения команды, в случае успеха 0
   */

  int servoJoint(std::vector<double> target_joint, double acceleration = 0.005,
                 double velocity = 0.005, double time = 0.01,
                 double lookahead_time = 0.1, double gain = 200) override;
  /**
   * Выключить серво режим
   */
  void stopServoMode() override;

  /**
   * Включить серво режим
   */
  void startServoMode() override;

  void setInputState(int8_t pin, double state) override;
  void setOutputState(int8_t pin, double state) override;

 private:
  arcs::aubo_sdk::RobotModeType mode;
  std::vector<std::string> joint_names_;
  std::string robot_ip_;
  std::string robot_name_;
  RobotJointsStates robot_state;

  std::vector<double> target_position;

  std::atomic<bool> terminated;
  uint32_t ctime;
  double rate = 5.0;
  double profile_velocity = 0.75;
  std::thread cycle_thread;
  std::mutex target_position_mutex;
  bool checkVectorNan(std::vector<double> target);
  void moveByTrajectory();
  void cycle();
  void update();
  bool comparetePosition(std::vector<double> current_position,
                         std::vector<double> target_position);
};

}  // namespace aubo_sdk_client