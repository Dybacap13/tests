#pragma once
#include <behaviortree_cpp/action_node.h>

namespace gazprom_node {
class PaintCounterIncrement : public BT::SyncActionNode {
 public:
  PaintCounterIncrement(const std::string &, const BT::NodeConfig &);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }

 private:
};
}  // namespace gazprom_node
