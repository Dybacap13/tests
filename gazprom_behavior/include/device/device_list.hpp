#pragma once

#include <device/dashboard.hpp>
#include <device/robot.hpp>

struct DeviceList {
  DeviceList(std::shared_ptr<rclcpp::Node> node);
  std::shared_ptr<gazprom::device::Robot> robot;
  std::shared_ptr<gazprom::device::Dashboard> dashboard;
};

DeviceList::DeviceList(std::shared_ptr<rclcpp::Node> node) {
  robot = std::make_shared<gazprom::device::Robot>(node);
  dashboard = std::make_shared<gazprom::device::Dashboard>(node);
}