#pragma once

#include <moveit_studio_behavior_interface/impl/action_client_behavior_base_impl.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <dynamixel_control_msgs/action/set_velocity.hpp>

namespace set_dynamixel_velocity
{
using SetVelocity = dynamixel_control_msgs::action::SetVelocity;
/**
 * @brief TODO(...)
 */
class SetDynamixelVelocity : public moveit_studio::behaviors::ActionClientBehaviorBase<SetVelocity>
{
public:
  /**
   * @brief Constructor for the set_dynamixel_velocity behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when
   * this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in
   * the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the
   * Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this
   * Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after
   * the initialize() function is called, so these classes should not be used within the constructor.
   */
  SetDynamixelVelocity(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the set_dynamixel_velocity Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return set_dynamixel_velocity does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getActionName() override;

  /**
   * @brief Creates a service request message.
   * @return Returns an instance of SetVelocity::Request. Since the request message is empty, this always succeeds.
   */
  tl::expected<SetVelocity::Goal, std::string> createGoal() override;

  /**
   * @brief Returns a string with information about why the action was aborted.
   */
  std::string getAbortedMessage(const std::shared_ptr<const SetVelocity::Result> result) const override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /**
   * @brief Holds the result of calling the service asynchronously.
   */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace set_dynamixel_velocity
