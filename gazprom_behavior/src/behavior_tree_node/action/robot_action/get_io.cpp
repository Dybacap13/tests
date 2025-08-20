#include <behavior_tree_node/action/robot_action/get_io.hpp>
namespace gazprom_node {

IsIORobotAction::IsIORobotAction(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<gazprom::device::Robot> robot,
    std::shared_ptr<gazprom::device::Dashboard> dashboard)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus IsIORobotAction::onStart() {}
BT::NodeStatus IsIORobotAction::onRunning() {}
void IsIORobotAction::onHalted() {}

}  // namespace gazprom_node
