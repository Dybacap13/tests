#include <behavior_tree_node/action/get_number_cycle_paint.hpp>
namespace gazprom_node {
GetNumberCyclePaint::GetNumberCyclePaint(const std::string& name,
                                         const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus GetNumberCyclePaint::tick() { return BT::NodeStatus::SUCCESS; }
}  // namespace gazprom_node