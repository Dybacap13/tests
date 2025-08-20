
#include <behavior_tree_node/action/error_handler.hpp>
namespace gazprom_node {

ErrorHandler::ErrorHandler(
    const std::string&, const BT::NodeConfig&,
    std::shared_ptr<gazprom::device::Robot> robot,
    std::shared_ptr<gazprom::device::Dashboard> dashboard) {}

BT::NodeStatus ErrorHandler::onStart() {}
BT::NodeStatus ErrorHandler::onRunning() {}
void ErrorHandler::onHalted() {}

}  // namespace gazprom_node
