#pragma once
#include <behaviortree_cpp/action_node.h>

namespace gazprom_node {
class GetNumberCyclePaint : public BT::SyncActionNode {
 public:
  GetNumberCyclePaint(const std::string &, const BT::NodeConfig &);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<int>("number_cycle")};
  }

 private:
};

}  // namespace gazprom_node