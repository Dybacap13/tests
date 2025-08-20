#pragma once

#include <behavior_tree_factory.hpp>
#include <device/device_list.hpp>
#include <rclcpp/rclcpp.hpp>
namespace gazprom_behavior {
class BehaviorTree {
 public:
  BehaviorTree(std::shared_ptr<DeviceList> device_list, double rate);
  ~BehaviorTree();

  void tick();

  // void stop();

 private:
  std::shared_ptr<DeviceList> device_list_;
  std::shared_ptr<BehaviorTreeFactory> factory_;
  std::shared_ptr<BT::Tree> tree_;

  double rate_;
};
}  // namespace gazprom_behavior