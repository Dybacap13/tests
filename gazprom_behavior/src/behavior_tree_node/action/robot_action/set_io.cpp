#include <behavior_tree_node/action/robot_action/set_io.hpp>
namespace gazprom_node {

SetIORobotAction::SetIORobotAction(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<gazprom::device::Robot> robot,
    std::shared_ptr<gazprom::device::Dashboard> dashboard)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus SetIORobotAction::onStart() {}
BT::NodeStatus SetIORobotAction::onRunning() {}
void SetIORobotAction::onHalted() {}

}  // namespace gazprom_node
