#pragma once

#include <behaviortree_cpp/action_node.h>

#include <device/dashboard.hpp>
#include <device/robot.hpp>

namespace gazprom_node {

class SetIORobotAction : public BT::StatefulActionNode {
 public:
  SetIORobotAction(const std::string&, const BT::NodeConfig&,
                       std::shared_ptr<gazprom::device::Robot> robot,
                       std::shared_ptr<gazprom::device::Dashboard> dashboard);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("pin"),BT::InputPort<bool>("state")};
  }

 private:
  std::shared_future<std::shared_ptr<gazprom::device::Response>> future_;
  std::shared_ptr<gazprom::device::Robot> robot_;
  std::shared_ptr<gazprom::device::Robot> dashboard_;
};
}  // namespace gazprom_node
