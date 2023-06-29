// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kortex_behavior/teleoperate_non_servo.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace
{
using namespace std::chrono_literals;

constexpr auto kPortIDControllerName = "controller_name";

constexpr auto kMoveJointSubTopic = "/agent_interface/move_joint";
constexpr auto kMoveTwistSubTopic = "/agent_interface/move_end_effector";
constexpr auto kMoveTwistPubTopic = "/twist_controller/commands";

constexpr auto kActivateControllerService = "/ensure_controller_is_active";

constexpr auto kServiceResponseMaxWait = 5s;

/**
 * @brief Check if a ROS 2 service client is initialized and ready to communicate with its server
 * @param client Service client to check
 * @return Error result of type fp::FailedPrecondition if @ref client is a nullptr. Error result of type fp::Unavailable
 * if the client's service server is not ready.
 */
fp::Result<void> validateClient(std::shared_ptr<rclcpp::ClientBase> client)
{
  if (client == nullptr)
  {
    return tl::make_unexpected(fp::FailedPrecondition("Service client was not initialized"));
  }

  if (!client->service_is_ready())
  {
    return tl::make_unexpected(
        fp::Unavailable(std::string("Service ").append(client->get_service_name()).append(" is not ready. ")));
  }

  return {};
}

/**
 * @brief Send a request through a service client for a std_srvs::srv::Trigger service and block until a response is
 * received or a timeout limit is reached.
 * @param client Service client to use
 * @return fp::Result containing void if the service succeeds. Returns an error result of type fp::Timeout if the wait
 * duration exceeded @ref kServiceResponseMaxWait. Returns an error result of type fp::Aborted if the server reports
 * that the service request did not succeed.
 */
fp::Result<void> sendTriggerRequestBlocking(const std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>>& client)
{
  auto result_future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (result_future.wait_for(kServiceResponseMaxWait) == std::future_status::timeout)
  {
    return tl::make_unexpected(fp::Timeout(
        std::string("Timed out waiting for result from service ").append(client->get_service_name()).append(".")));
  }

  const auto result = result_future.get();
  if (!result->success)
  {
    return tl::make_unexpected(fp::Aborted(std::string("Service ")
                                               .append(client->get_service_name())
                                               .append(" failed with error message: ")
                                               .append(result->message)));
  }

  return {};
}

/**
 * @brief Send a request through a service client for a moveit_studio_agent_msgs::srv::SetStringArray service and block
 * until a response is received or a timeout limit is reached.
 * @param client Service client to use
 * @param request Service request message to send
 * @return fp::Result containing void if the service succeeds. Returns an error result of type fp::Timeout if the wait
 * duration exceeded @ref kServiceResponseMaxWait. Returns an error result of type fp::Aborted if the server reports that
 * the service request did not succeed, and an error message containing the failure message returned in the service response.
 */
fp::Result<void>
requestSwitchController(const std::shared_ptr<rclcpp::Client<moveit_studio_agent_msgs::srv::SetStringArray>>& client,
                        const std::shared_ptr<moveit_studio_agent_msgs::srv::SetStringArray::Request>& request)
{
  auto result_future = client->async_send_request(request);
  if (result_future.wait_for(kServiceResponseMaxWait) == std::future_status::timeout)
  {
    return tl::make_unexpected(fp::Timeout(
        std::string("Timed out waiting for result from service ").append(client->get_service_name()).append(".")));
  }

  const auto result = result_future.get();
  if (!result->status.success)
  {
    return tl::make_unexpected(fp::Aborted(std::string("Service ")
                                               .append(client->get_service_name())
                                               .append(" failed with message: ")
                                               .append(result->status.error_message)));
  }
  return {};
}

}  // namespace

namespace moveit_studio::behaviors
{
TeleoperateNonServo::TeleoperateNonServo(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
  , move_end_effector_subscriber_{ shared_resources_->node->create_subscription<TwistStamped>(
        kMoveTwistSubTopic, 1, [this](const TwistStamped::SharedPtr msg) { onCartesianJog(msg); }) }
  , switch_controller_client_{ shared_resources_->node->create_client<moveit_studio_agent_msgs::srv::SetStringArray>(
        kActivateControllerService) }
{
}

TeleoperateNonServo::~TeleoperateNonServo()
{
  // If this thread is joinable, join it here.
  if (setup_teleop_thread_.joinable())
  {
    setup_teleop_thread_.join();
  }
}

BT::PortsList TeleoperateNonServo::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDControllerName),
  };
}

BT::NodeStatus TeleoperateNonServo::onStart()
{
  // If the thread is joinable, join it here before we replace it with a new thread.
  if (setup_teleop_thread_.joinable())
  {
    setup_teleop_thread_.join();
  }

  process_status_ = BT::NodeStatus::RUNNING;

  setup_teleop_thread_ = std::thread{ [this]() { doTeleoperation(); } };

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TeleoperateNonServo::onRunning()
{
  return process_status_;
}

void TeleoperateNonServo::onHalted()
{
  {
    std::scoped_lock lock(publisher_mutex_);
    twist_publisher_.reset();
  }
}

void TeleoperateNonServo::doTeleoperation()
{
  // Validate required service clients
  if (const auto result = validateClients(); !result)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Failed to begin teleoperation: required client not available: " + result.error().what);
    process_status_ = BT::NodeStatus::FAILURE;
    return;
  }

  // Get required inputs from Behavior data ports
  const auto controller_name = getInput<std::string>(kPortIDControllerName);

  if (const auto error = maybe_error(controller_name); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    process_status_ = BT::NodeStatus::FAILURE;
    return;
  }

  // TODO(abake48): Once Support for ros2_control::SwitchControllers has been implemented, manage that here.

  // Make sure that the controllers required by MoveIt Servo are active before performing servo motion
  auto req = std::make_shared<moveit_studio_agent_msgs::srv::SetStringArray::Request>();
  req->data.push_back(controller_name.value());
  
  if (const auto result = requestSwitchController(switch_controller_client_, req); !result)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to enable required controller " +
                                                                 controller_name.value() + ". " + result.error().what);
    process_status_ = BT::NodeStatus::FAILURE;
    return;
  }

  // Initialize the publishers to begin forwarding command messages to necessary topics.
  {
    std::scoped_lock lock(publisher_mutex_);

    // TODO(abake48): Add Joint Jog Support for Kinova
    twist_publisher_ = shared_resources_->node->create_publisher<TwistStamped>(kMoveTwistPubTopic, 1);
  }
}

void TeleoperateNonServo::onJointJog(const JointJog::SharedPtr msg)
{
  // TODO(abake48): add Joint Jog support for Kinova.
  return;
  
}

void TeleoperateNonServo::onCartesianJog(const TwistStamped::SharedPtr msg)
{
  std::scoped_lock lock(publisher_mutex_);
  if (twist_publisher_ == nullptr)
  {
    return;
  }

  twist_publisher_->publish(*msg);
}

fp::Result<void> TeleoperateNonServo::validateClients() const
{
  if (const auto error = fp::maybe_error(validateClient(switch_controller_client_));
      error)
  {
    return tl::make_unexpected(error.value());
  }

  return {};
}

}  // namespace moveit_studio::behaviors
