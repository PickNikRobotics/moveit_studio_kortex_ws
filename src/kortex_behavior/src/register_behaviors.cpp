#include <behaviortree_cpp/bt_factory.h>
#include <kortex_behavior/deactivate_controllers.hpp>
#include <kortex_behavior/teleoperate_non_servo.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

namespace kortex_behavior
{
class KortexBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    using namespace moveit_studio::behaviors;

    registerBehavior<DeactivateControllers>(factory, "DeactivateControllers", shared_resources);
    registerBehavior<TeleoperateNonServo>(factory, "TeleoperateNonServo", shared_resources);
  }
};
}  // namespace kortex_behavior

PLUGINLIB_EXPORT_CLASS(kortex_behavior::KortexBehaviorsLoader, moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
