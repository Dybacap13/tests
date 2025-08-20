#pragma once
#include <behaviortree_cpp/action_node.h>

#include <behavior_tree_node/enums/enum_list.hpp>
namespace gazprom_node {
class SetMode : public BT::SyncActionNode {
 public:
  SetMode(const std::string &, const BT::NodeConfig &);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<ModeProgramm>("mode")};
  }

 private:
};
}  // namespace gazprom_node
