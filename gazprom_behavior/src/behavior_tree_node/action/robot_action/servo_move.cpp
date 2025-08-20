

#include <behavior_tree_node/action/robot_action/servo_move.hpp>
namespace gazprom_node {

ServoMoveRobotAction::ServoMoveRobotAction(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<gazprom::device::Robot> robot,
    std::shared_ptr<gazprom::device::Dashboard> dashboard)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus ServoMoveRobotAction::onStart() {}
BT::NodeStatus ServoMoveRobotAction::onRunning() {}
void ServoMoveRobotAction::onHalted() {}

}  // namespace gazprom_node
