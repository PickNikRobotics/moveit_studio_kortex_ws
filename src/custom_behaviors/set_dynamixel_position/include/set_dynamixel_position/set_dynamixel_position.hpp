#pragma once

#include <behaviortree_cpp/action_node.h>

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
// Just try to do a trigger node first
#include <std_srvs/srv/trigger.hpp>

namespace set_dynamixel_position
{
using Trigger = std_srvs::srv::Trigger;
/**
 * @brief TODO(...)
 */
class SetDynamixelPosition : public moveit_studio::behaviors::ServiceClientBehaviorBase<Trigger>
{
public:
  /**
   * @brief Constructor for the set_dynamixel_position behavior.
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
  SetDynamixelPosition(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);
  SetDynamixelPosition(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
                       std::unique_ptr<moveit_studio::behaviors::ClientInterfaceBase<Trigger>> client_interface);

  /**
   * @brief Implementation of the required providedPorts() function for the set_dynamixel_position Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return set_dynamixel_position does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  /**
   * @brief Creates a service request message.
   * @return Returns an instance of Trigger::Request. Since the request message is empty, this always succeeds.
   */
  tl::expected<Trigger::Request, std::string> createRequest() override;

  /**
   * @brief Determines if the service request succeeded or failed based on the contents of the response message's success field.
   * @param response Response message received from the service server.
   * @return The return value matches the value of response.success.
   */
  tl::expected<bool, std::string> processResponse(const Trigger::Response& response) override;

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
}  // namespace set_dynamixel_position
