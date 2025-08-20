#include <behavior_tree_node/action/paint_counter_increment.hpp>
namespace gazprom_node {
PaintCounterIncrement::PaintCounterIncrement(const std::string& name,
                                             const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus PaintCounterIncrement::tick() { return BT::NodeStatus::SUCCESS; }
}  // namespace gazprom_node