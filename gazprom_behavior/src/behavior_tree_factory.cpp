#include "behavior_tree_factory.hpp"

#include <behaviortree_cpp/xml_parsing.h>

#include <enums/enum_list.hpp>

#include "behavior_tree_node/action/choose_mark.hpp"
#include "behavior_tree_node/action/robot_action/get_io_robot.hpp"
#include "behavior_tree_node/action/robot_action/servo_move.hpp"
#include "behavior_tree_node/action/robot_action/set_io_robot.hpp"
#include "behavior_tree_node/action/robot_action/stop_robot.hpp"
//
#include "behavior_tree_node/action/error_handler.hpp"
#include "behavior_tree_node/action/get_number_cycle_paint.hpp"
#include "behavior_tree_node/action/paint_counter_increment.hpp"
#include "behavior_tree_node/action/set_mode.hpp"

//
#include "behavior_tree_node/check/check_connection_robot.hpp"
//
#include "behavior_tree_node/is/is_mark_selected.hpp"
#include "behavior_tree_node/is/is_mode.hpp"

namespace gazprom_behavior {

BehaviorTreeFactory::BehaviorTreeFactory(
    std::shared_ptr<DeviceList> device_list)
    : device_list_(device_list) {
  init();
}

void BehaviorTreeFactory::initEnums() {
  BT::BehaviorTreeFactory::registerScriptingEnums<StateIO>();
  BT::BehaviorTreeFactory::registerScriptingEnums<ModeProgramm>();
}

void BehaviorTreeFactory::initNodes() {
  // action
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::ServoMoveRobotAction>(
      "ServoMoveRobotAction", device_list_->robot, device_list_->dashboard);
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::SetIORobotAction>(
      "SetIORobotAction", device_list_->robot, device_list_->dashboard);
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::IsIORobotAction>(
      "IsIORobotAction", device_list_->robot, device_list_->dashboard);
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::StopRobotAction>(
      "StopRobotAction", device_list_->robot, device_list_->dashboard);

  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::ErrorHandler>(
      "ErrorHandler");
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::GetNumberCyclePaint>(
      "GetNumberCyclePaint");
  BT::BehaviorTreeFactory::registerNodeType<
      gazprom_node::PaintCounterIncrement>("PaintCounterIncrement");
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::SetMode>("SetMode");

  //

  //
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::IsIORobotAction>(
      "IsMode");
  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::IsIORobotAction>(
      "IsMarkSelected");

  BT::BehaviorTreeFactory::registerNodeType<gazprom_node::CheckConnectionRobot>(
      "CheckConnectionRobot");
}
void BehaviorTreeFactory::init() {
  initModel();
  initTrees();
}
void BehaviorTreeFactory::initModel() {
  initNodes();
  initEnums();
}
void BehaviorTreeFactory::initTrees() {
  registerBehaviorTreeFromFile("./xml/main_tree.btproj");
}
std::string BehaviorTreeFactory::getXmlModel() {
  return writeTreeNodesModelXML(*this, false);
}
}  // namespace gazprom_behavior