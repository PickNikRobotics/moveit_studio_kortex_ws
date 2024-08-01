#include <set_dynamixel_position/set_dynamixel_position.hpp>

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("SetDynamixelPosition");
constexpr std::chrono::seconds kServiceResponseTimeoutSeconds{ 3 };
constexpr auto kPortServiceName = "service_name";
}  // namespace

namespace set_dynamixel_position
{
SetDynamixelPosition::SetDynamixelPosition(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::ServiceClientBehaviorBase<Trigger>(name, config, shared_resources)
{
}
SetDynamixelPosition::SetDynamixelPosition(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_studio::behaviors::ClientInterfaceBase<Trigger>> client_interface)
  : moveit_studio::behaviors::ServiceClientBehaviorBase<Trigger>(name, config, shared_resources,
                                                                 std::move(client_interface))
{
}

BT::PortsList SetDynamixelPosition::providedPorts()
{
  // TODO(...)
  return { BT::InputPort<std::string>(kPortServiceName, "/set_dynamixel_position",
                                      "Name of the service to send a request to.") };
}

BT::KeyValueVector SetDynamixelPosition::metadata()
{
  // TODO(...)
  return { { "description", "Call service to set the position of a dynamixel motor" } };
}

tl::expected<std::string, std::string> SetDynamixelPosition::getServiceName()
{
  const auto service_name = getInput<std::string>(kPortServiceName);

  if (const auto error = moveit_studio::behaviors::maybe_error(service_name))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  return service_name.value();
}

tl::expected<std::chrono::duration<double>, std::string> SetDynamixelPosition::getResponseTimeout()
{
  return kServiceResponseTimeoutSeconds;
}

tl::expected<Trigger::Request, std::string> SetDynamixelPosition::createRequest()
{
  // Create request message.
  return std_srvs::srv::Trigger::Request{};
}

tl::expected<bool, std::string> SetDynamixelPosition::processResponse(const Trigger::Response& response)
{
  return response.success;
}

}  // namespace set_dynamixel_position
