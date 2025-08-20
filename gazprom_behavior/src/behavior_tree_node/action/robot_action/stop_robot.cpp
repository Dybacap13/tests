

#include <behavior_tree_node/action/robot_action/stop_robot.hpp>
namespace gazprom_node {

StopRobotAction::StopRobotAction(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<gazprom::device::Robot> robot,
    std::shared_ptr<gazprom::device::Dashboard> dashboard)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus StopRobotAction::onStart() {}
BT::NodeStatus StopRobotAction::onRunning() {}
void StopRobotAction::onHalted() {}

}  // namespace gazprom_node
