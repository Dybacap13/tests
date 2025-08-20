#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <device/device_list.hpp>
#include <memory>
namespace gazprom_behavior {

class BehaviorTreeFactory : public BT::BehaviorTreeFactory {
 public:
  BehaviorTreeFactory(std::shared_ptr<DeviceList> device_list);

  std::string getXmlModel();

 private:
  void init();
  void initModel();
  void initTrees();
  void initNodes();
  void initEnums();

  std::shared_ptr<DeviceList> device_list_;
};
}  // namespace gazprom_behavior