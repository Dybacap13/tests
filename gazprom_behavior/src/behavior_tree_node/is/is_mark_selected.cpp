#include "behavior_tree_node/is/is_mark_selected.hpp"

namespace gazprom_node {
IsMarkSelected::IsMarkSelected(const std::string& name,
                                       const BT::NodeConfig& config
                                       )
    : BT::SyncActionNode(name, config){}

BT::NodeStatus IsMarkSelected::tick() {
  return BT::NodeStatus::SUCCESS;
}
}  