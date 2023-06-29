// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace kortex_behavior
{
using SwitchController = controller_manager_msgs::srv::SwitchController;

/**
 * @brief Call the /controller_manager/switch_controller service to deactivate and activate controllers.
 *
 * Ingests a list of controllers to deactivate, and a list of controllers to activate.
 * 
 * @details
 * | Data Port Name         | Port Type | Object Type |
 * | ---------------------- | --------- | ----------- |
 * | deactivate_controllers | input     | std::string |
 * | activate_controllers   | input     | std::string |
 */
class SwitchROS2Controllers final : public ServiceClientBehaviorBase<SwitchController>
{
public:
  SwitchROS2Controllers(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<BehaviorContext>& shared_resources);

  /**
   * @brief Required implementation of the static providedPorts function.
   * @return The list of input and output ports used by this behavior.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Get the name of the service.
   * @details For this behavior the service name is always "/controller_manager/switch_controller".
   * @return Returns the service name. Since the service name is set to a constant value, this always succeeds.
   */
  fp::Result<std::string> getServiceName() override;

  /**
   * @brief Create the service request message for the "/controller_manager/switch_controller" service.
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
}  // namespace kortex_behavior
