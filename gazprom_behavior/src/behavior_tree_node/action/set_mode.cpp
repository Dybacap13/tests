#include <behavior_tree_node/action/set_mode.hpp>
namespace gazprom_node {
SetMode::SetMode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus SetMode::tick() { return BT::NodeStatus::SUCCESS; }
}  // namespace gazprom_node