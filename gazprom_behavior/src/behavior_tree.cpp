#include <behavior_tree.hpp>
using namespace std::chrono_literals;
namespace gazprom_behavior {

BehaviorTree::BehaviorTree(std::shared_ptr<DeviceList> device_list, double rate)
    : device_list_(device_list),
      rate_(rate)

{
  factory_ = std::make_shared<BehaviorTreeFactory>(device_list_);

  tree_ = std::make_shared<BT::Tree>(factory_->createTree("main"));
}

void BehaviorTree::tick() {
  auto cycletime = std::chrono::microseconds(1.0 / this->rate_ * 1000 *
                                             1000);  // microseconds);
  while (rclcpp::ok()) {
    auto start = std::chrono::high_resolution_clock::now();
    auto status = tree_->tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      std::cout << "--- status: " << BT::toStr(status) << "\n\n";
      break;
    }
    auto end = std::chrono::high_resolution_clock::now();
    const auto delta = end - start;
    if (delta > cycletime) {
      std::cout << "System too slow for cycle time " << cycletime.count()
                << "ms sending takes " << delta.count() << "ns" << std::endl;
    } else {
      std::this_thread::sleep_for(cycletime - delta);
    }
  }
}

}  // namespace gazprom_behavior