#include <boost/geometry/util/range.hpp>
#include <set_dynamixel_position/set_dynamixel_position.hpp>

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("SetDynamixelPosition");
constexpr std::chrono::seconds kServiceResponseTimeoutSeconds{ 3 };
constexpr auto kPortServiceName = "service_name";
constexpr auto kPortMotorType = "motor_type";
constexpr auto kPortPosition = "position";
}  // namespace

namespace set_dynamixel_position
{
SetDynamixelPosition::SetDynamixelPosition(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::ServiceClientBehaviorBase<SetPosition>(name, config, shared_resources)
{
}
SetDynamixelPosition::SetDynamixelPosition(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_studio::behaviors::ClientInterfaceBase<SetPosition>> client_interface)
  : moveit_studio::behaviors::ServiceClientBehaviorBase<SetPosition>(name, config, shared_resources,
                                                                     std::move(client_interface))
{
}

BT::PortsList SetDynamixelPosition::providedPorts()
{
  return { BT::InputPort<std::string>(kPortServiceName, "/set_dynamixel_position",
                                      "Name of the service to send a request to."),
           BT::InputPort<std::string>(kPortMotorType, "XL430-W250-T", "Model of the motor."),
           BT::InputPort<int>(kPortPosition, 0, "Position value to set the motor to.") };
}

BT::KeyValueVector SetDynamixelPosition::metadata()
{
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

tl::expected<SetPosition::Request, std::string> SetDynamixelPosition::createRequest()
{
  const auto motor_type = getInput<std::string>(kPortMotorType);
  const auto position = getInput<int>(kPortPosition);

  if (const auto error = moveit_studio::behaviors::maybe_error(motor_type, position))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  // Create request message.
  SetPosition::Request request;
  request.motor_type = motor_type.value();
  request.position = position.value();
  return request;
}

tl::expected<bool, std::string> SetDynamixelPosition::processResponse(const SetPosition::Response& response)
{
  return response.success;
}

}  // namespace set_dynamixel_position
