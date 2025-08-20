#include "behavior_tree_node/check/check_connection_robot.hpp"

namespace gazprom_node {
CheckConnectionRobot::CheckConnectionRobot(const std::string& name,
                                           const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus CheckConnectionRobot::tick() { return BT::NodeStatus::SUCCESS; }
}  // namespace gazprom_node