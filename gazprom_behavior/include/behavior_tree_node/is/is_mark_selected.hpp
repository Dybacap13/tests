#pragma once
#include <behaviortree_cpp/action_node.h>

namespace gazprom_node {
class IsMarkSelected : public BT::SyncActionNode {
 public:
  IsMarkSelected(const std::string &, const BT::NodeConfig &);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }

 private:
};

}  // namespace gazprom_node