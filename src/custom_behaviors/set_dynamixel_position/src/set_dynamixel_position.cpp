#include <set_dynamixel_position/set_dynamixel_position.hpp>

namespace set_dynamixel_position
{
SetDynamixelPosition::SetDynamixelPosition(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetDynamixelPosition::providedPorts()
{
  // TODO(...)
  return BT::PortsList({});
}

BT::KeyValueVector SetDynamixelPosition::metadata()
{
  // TODO(...)
  return { { "description", "Call service to set the position of a dynamixel motor" } };
}

BT::NodeStatus SetDynamixelPosition::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace set_dynamixel_position
