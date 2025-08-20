#ifndef BEHAVIOR_TREE_FACTORY_WITH_ROS_H_
#define BEHAVIOR_TREE_FACTORY_WITH_ROS_H_

#include <behaviortree_cpp/xml_parsing.h>

#include <behavior_tree.hpp>
#include <behavior_tree_factory.hpp>
#include <device/device_list.hpp>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("behavior_tree_node");
  auto device = std::make_shared<DeviceList>(node);
  auto behavior = std::make_shared<gazprom_behavior::BehaviorTree>(device, 1.0);

  std::thread node_spin_thread = std::thread([&]() { rclcpp::spin(node); });
  behavior->tick();
  rclcpp::shutdown();
  node_spin_thread.join();
  return 0;
}
#endif