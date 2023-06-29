// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kortex_behavior/deactivate_controllers.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>

namespace
{
constexpr auto kPortControllerNames = "controller_names";
constexpr auto kDeactivateControllerService = "/ensure_controller_is_active";
}  // namespace

namespace moveit_studio::behaviors
{
DeactivateControllers::DeactivateControllers(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<SetStringArray>(name, config, shared_resources)
{
}

BT::PortsList DeactivateControllers::providedPorts()
{
  return { BT::InputPort<std::string>(kPortControllerNames) };
}

fp::Result<std::string> DeactivateControllers::getServiceName()
{
  return kDeactivateControllerService;
}

fp::Result<SetStringArray::Request> DeactivateControllers::createRequest()
{
  // Get required values from input ports.
  const auto controller_names = getInput<std::string>(kPortControllerNames);

  if (const auto error = maybe_error(controller_names); error)
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

fp::Result<bool> DeactivateControllers::processResponse(const SetStringArray::Response& response)
{
  if (!response.status.success)
  {
    return tl::make_unexpected(
        fp::Internal(std::string("Service request failed: ").append(response.status.error_message)));
  }

  return true;
}
}  // namespace moveit_studio::behaviors

template class moveit_studio::behaviors::ServiceClientBehaviorBase<moveit_studio_agent_msgs::srv::SetStringArray>;
