#include <boost/geometry/util/range.hpp>
#include <set_dynamixel_velocity/set_dynamixel_velocity.hpp>

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("SetDynamixelVelocity");
constexpr auto kPortActionName = "action_name";
constexpr auto kPortMotorType = "motor_type";
constexpr auto kPortNormalizedVelocity = "normalized_velocity";
constexpr auto kPortAbsNormalizedLoadThreshold = "absolute_normalized_load_threshold";
constexpr auto kPortMonitorLoad = "monitor_load";
}  // namespace

namespace set_dynamixel_velocity
{
SetDynamixelVelocity::SetDynamixelVelocity(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::ActionClientBehaviorBase<SetVelocity>(name, config, shared_resources)
{
}

BT::PortsList SetDynamixelVelocity::providedPorts()
{
  return { BT::InputPort<std::string>(kPortActionName, "/set_dynamixel_velocity",
                                      "Name of the service to send a request to."),
           BT::InputPort<std::string>(kPortMotorType, "XL430-W250-T", "Model of the motor."),
           BT::InputPort<double>(kPortNormalizedVelocity, "0.0",
                                 "Normalized velocity to set the motor to in the range [-1.0, 1.0]."),
           BT::InputPort<double>(kPortAbsNormalizedLoadThreshold, "0.0",
                                 "Absolute normalized load threshold in the range [0.0, 1.0]. When achieved, the "
                                 "action server will return success."),
           BT::InputPort<bool>(kPortMonitorLoad, false,
                               "If true, the load will be monitored according to the provided absolute normalized load "
                               "threshold.") };
}

BT::KeyValueVector SetDynamixelVelocity::metadata()
{
  return { { "description", "Call action server to set the velocity of a dynamixel motor" } };
}

tl::expected<std::string, std::string> SetDynamixelVelocity::getActionName()
{
  const auto action_server_name = getInput<std::string>(kPortActionName);

  if (const auto error = moveit_studio::behaviors::maybe_error(action_server_name))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  return action_server_name.value();
}

tl::expected<SetVelocity::Goal, std::string> SetDynamixelVelocity::createGoal()
{
  const auto motor_type = getInput<std::string>(kPortMotorType);
  const auto normalized_velocity = getInput<double>(kPortNormalizedVelocity);
  const auto abs_normalized_load_threshold = getInput<double>(kPortAbsNormalizedLoadThreshold);
  const auto monitor_load = getInput<bool>(kPortMonitorLoad);

  if (const auto error = moveit_studio::behaviors::maybe_error(motor_type, normalized_velocity,
                                                               abs_normalized_load_threshold, monitor_load))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  // Create goal message.
  SetVelocity::Goal goal;
  goal.motor_type = motor_type.value();
  goal.normalized_velocity = normalized_velocity.value();
  goal.abs_normalized_load_threshold = abs_normalized_load_threshold.value();
  goal.monitor_load = monitor_load.value();
  return goal;
}

std::string SetDynamixelVelocity::getAbortedMessage(const std::shared_ptr<const SetVelocity::Result> result) const
{
  return "Action goal was aborted: " + result->error_string;
}

}  // namespace set_dynamixel_velocity
