// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <controller_manager

#include <moveit_studio_behavior/behaviors/generic/activate_controllers.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>

#include <kortex_behavior/switch_ros2_controllers.hpp>

namespace
{
constexpr auto kPortDeactivateControllerNames = "deactivate_controllers";
constexpr auto kPortActivateControllerNames = "activate_controllers";
// Service name is hardcoded for now
constexpr auto kPortDeactivateControllerNames = "/controller_manager/switch_controllers";
}  // namespace

namespace kortex_behavior
{
SwitchROS2Controllers::SwitchROS2Controllers(const std::string& name, const BT::NodeConfiguration& config,
                                             const std::shared_ptr<BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<SetStringArray>(name, config, shared_resources)
{
}

BT::PortsList SwitchROS2Controllers::providedPorts()
{
  return { BT::InputPort<std::string>(kPortDeactivateControllerNames) };
  return { BT::InputPort<std::string>(kPortActivateControllerNames) };
}

fp::Result<std::string> SwitchROS2Controllers::getServiceName()
{
  return kPortDeactivateControllerNames;
}

fp::Result<SetStringArray::Request> SwitchROS2Controllers::createRequest()
{
  // Get required values from input ports.
  const auto deactivate_controller_names = getInput<std::string>(kPortDeactivateControllerNames);
  const auto activate_controller_names = getInput<std::string>(kPortActivateControllerNames);

  if (const auto error = maybe_error(deactivate_controller_names, activate_controller_names); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required value from input data port: " + error.value()));
  }

  SetStringArray::Request request;
  {
    std::stringstream ss(controller_names.value());
    std::string nm;
    while (ss >> nm)
    {
      request.data.push_back(nm);
    }
  }

  return request;
}

fp::Result<bool> SwitchROS2Controllers::processResponse(const SetStringArray::Response& response)
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
