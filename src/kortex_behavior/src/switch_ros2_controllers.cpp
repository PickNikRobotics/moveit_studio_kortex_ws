// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kortex_behavior/switch_ros2_controllers.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace
{
constexpr auto kPortDeactivateControllerNames = "deactivate_controllers";
constexpr auto kPortActivateControllerNames = "activate_controllers";
// Service name is hardcoded for now
constexpr auto kServiceName = "/controller_manager/switch_controllers";
}  // namespace

namespace kortex_behavior
{
SwitchROS2Controllers::SwitchROS2Controllers(const std::string& name, const BT::NodeConfiguration& config,
                                             const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<controller_manager_msgs::srv::SwitchController>(name, config, shared_resources)
{
}

BT::PortsList SwitchROS2Controllers::providedPorts()
{
  return { BT::InputPort<std::string>(kPortDeactivateControllerNames) };
  return { BT::InputPort<std::string>(kPortActivateControllerNames) };
}

fp::Result<std::string> SwitchROS2Controllers::getServiceName()
{
  return kServiceName;
}

fp::Result<controller_manager_msgs::srv::SwitchController::Request> SwitchROS2Controllers::createRequest()
{
  // Get required values from input ports.
  const auto deactivate_controller_names = getInput<std::string>(kPortDeactivateControllerNames);
  const auto activate_controller_names = getInput<std::string>(kPortActivateControllerNames);

  if (const auto error = maybe_error(deactivate_controller_names, activate_controller_names); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required value from input data port: " + error.value()));
  }

  SwitchCcontroller_manager_msgs::srv::SwitchControllerontroller::Request request;
  {
  }

  return request;
}

fp::Result<bool> SwitchROS2Controllers::processResponse(const controller_manager_msgs::srv::SwitchController::Response& response)
{
  if (!response.status.success)
  {
    return tl::make_unexpected(
        fp::Internal(std::string("Service request failed: ").append(response.status.error_message)));
  }

  return true;
}
}  // namespace kortex_behavior

template class moveit_studio::behaviors::ServiceClientBehaviorBase<controller_manager_msgs::srv::SwitchController>;
