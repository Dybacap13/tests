#include "behavior_tree_node/is/is_mark_selected.hpp"

namespace gazprom_node {
IsMode::IsMode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus IsMode::tick() { return BT::NodeStatus::SUCCESS; }
}  // namespace gazprom_node