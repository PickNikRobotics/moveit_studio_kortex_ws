// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kortex_behavior/switch_ros2_controllers.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>

namespace
{
constexpr auto kPortStartControllerNames = "start_controllers";
constexpr auto kPortStopControllerNames = "stop_controllers";
constexpr auto kPortDeactivateControllerNames = "deactivate_controllers";
constexpr auto kPortActivateControllerNames = "activate_controllers";
// Service name is hardcoded for now
constexpr auto kServiceName = "/controller_manager/switch_controllers";
}  // namespace

void add_controllers_to_list(const std::string& controllers, std::vector<std::string> list)
{
    std::stringstream ss(controllers);
    std::string nm;
    while (ss >> nm)
    {
      list.push_back(nm);
    }
}

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
  return { BT::InputPort<std::string>(kPortStopControllerNames) };
  return { BT::InputPort<std::string>(kPortStartControllerNames) };
}

fp::Result<std::string> SwitchROS2Controllers::getServiceName()
{
  return kServiceName;
}

fp::Result<controller_manager_msgs::srv::SwitchController::Request> SwitchROS2Controllers::createRequest()
{
  // Get required values from input ports.
  const auto start_controller_names = getInput<std::string>(kPortStartControllerNames);
  const auto stop_controller_names = getInput<std::string>(kPortStopControllerNames);
  const auto deactivate_controller_names = getInput<std::string>(kPortDeactivateControllerNames);
  const auto activate_controller_names = getInput<std::string>(kPortActivateControllerNames);

  if (const auto error = moveit_studio::behaviors::maybe_error(start_controller_names, stop_controller_names, deactivate_controller_names, activate_controller_names); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required value from input data port: " + error.value()));
  }

  controller_manager_msgs::srv::SwitchController::Request request;
  request.strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  add_controllers_to_list(start_controller_names.value(), request.start_controllers);
  add_controllers_to_list(stop_controller_names.value(), request.stop_controllers);
  add_controllers_to_list(deactivate_controller_names.value(), request.deactivate_controllers);
  add_controllers_to_list(activate_controller_names.value(), request.activate_controllers);

  return request;
}

fp::Result<bool> SwitchROS2Controllers::processResponse(const controller_manager_msgs::srv::SwitchController::Response& response)
{
  if (!response.ok)
  {
    return tl::make_unexpected(fp::Internal("Service request failed!"));
  }

  return true;
}
}  // namespace kortex_behavior

template class moveit_studio::behaviors::ServiceClientBehaviorBase<controller_manager_msgs::srv::SwitchController>;
