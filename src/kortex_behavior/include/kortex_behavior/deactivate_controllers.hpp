// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_agent_msgs/srv/set_string_array.hpp>

namespace moveit_studio::behaviors
{
using SetStringArray = moveit_studio_agent_msgs::srv::SetStringArray;

/**
 * @brief Deactivate controllers, whose names are set by the "controller_names" parameter.
 *
 * @details
 * | Data Port Name   | Port Type | Object Type |
 * | ---------------- | --------- | ----------- |
 * | controller_names | input     | std::string |
 */
class DeactivateControllers final : public ServiceClientBehaviorBase<SetStringArray>
{
public:
  DeactivateControllers(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<BehaviorContext>& shared_resources);

  /**
   * @brief Required implementation of the static providedPorts function.
   * @return The list of input and output ports used by this behavior.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Get the name of the service.
   * @details For this behavior the service name is always "/ensure_controller_is_active".
   * @return Returns the service name. Since the service name is set to a constant value, this always succeeds.
   */
  fp::Result<std::string> getServiceName() override;

  /**
   * @brief Create the service request message for the "/ensure_controller_is_active" service.
   * @details This reads the controller names from the controller_names input data port.
   * @return a SetStringArray request message containing the controller names.
   */
  fp::Result<SetStringArray::Request> createRequest() override;

  /**
   * @brief Handles the service response message once the service finishes.
   * @return Returns an error result if the service finished but did not succeed, since this is treated as an unexpected
   * error state. Otherwise, returns true.
   */
  fp::Result<bool> processResponse(const SetStringArray::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return response_future_;
  }

  /** @brief Holds the result of calling the service asynchronously. */
  std::shared_future<fp::Result<bool>> response_future_;
};
}  // namespace moveit_studio::behaviors
