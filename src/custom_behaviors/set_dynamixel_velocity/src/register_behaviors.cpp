#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <set_dynamixel_velocity/set_dynamixel_velocity.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace set_dynamixel_velocity
{
class SetDynamixelVelocityBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<SetDynamixelVelocity>(factory, "SetDynamixelVelocity", shared_resources);
  }
};
}  // namespace set_dynamixel_velocity

PLUGINLIB_EXPORT_CLASS(set_dynamixel_velocity::SetDynamixelVelocityBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
