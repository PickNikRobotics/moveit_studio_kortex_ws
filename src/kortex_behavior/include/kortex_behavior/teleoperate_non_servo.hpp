// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <chrono>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <fp/all.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_studio_agent_msgs/srv/set_string_array.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace moveit_studio::behaviors
{
using TwistStamped = geometry_msgs::msg::TwistStamped;
using JointJog = control_msgs::msg::JointJog;

/**
 * @brief This is a special Behavior to enable human-in-the-loop teleoperation by forwarding commands through the Objective
 * Server.
 *
 * @details When started, this Behavior will run INDEFINITELY until it is halted. This will happen either when the root
 * node of the behavior tree is halted as the Objective is canceled, or when this Behavior's parent control node halts
 * it. When this Behavior first transitions from IDLE to RUNNING, it starts Servo control using the services advertised
 * by the Servo server node. While this Behavior is RUNNING, it subscribes to JointJog and TwistStamped command messages
 * that originate in the user interface, and republishes these messages to the command topics advertised by the Servo
 * server node. When this Behavior is halted, it pauses Servo control using the server's services.
 *
 * | Data Port Name       | Port Type | Object Type                  |
 * | -------------------- |-----------|------------------------------|
 * | controller_name      | Input     | std::string                  |
 */
class TeleoperateNonServo final : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  /**
   * @brief Constructor for the Teleoperate Behavior.
   * @param name Name of the Behavior. Used internally by BehaviorTree.Cpp.
   * @param config BehaviorTree.Cpp NodeConfiguration. Used internally by BehaviorTree.Cpp.
   * @param shared_resources Shared pointer to the ObjectiveServerNode's BehaviorContext. Passed to the constructor of
   * SharedResourcesNode, where it is used as a member variable.
   */
  TeleoperateNonServo(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<BehaviorContext>& shared_resources);

  ~TeleoperateNonServo() final;

  /**
   * @brief Required BehaviorTree.Cpp function to define data ports for this Behavior type.
   * @return BT::PortsList describing Behavior data poirts
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of onStart(). Runs when the Behavior is ticked for the first time.
   * @return Always returns BT::NodeStatus::RUNNING, since the success of Behavior's initialization is checked in @ref
   * onRunning().
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Implementation of onRunning(). Checks the status of the Behavior when it is ticked after it starts running.
   * @return BT::NodeStatus::FAILURE if we fail to validate the service clients, fail to retrieve required values from
   * the Behavior data ports, or fail to start the MoveIt Servo server.
   */
  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  /**
   * @brief create subscribers for teleop commands.
   * @details Run asynchronously in @ref setup_teleop_thread_
   */
  void doTeleoperation();

  /**
   * @brief Currently does nothing as JointJogging is not supported by Kinova drivers at the moment
   * @param msg JointJog command
   */
  void onJointJog(const JointJog::SharedPtr msg);

  /**
   * @brief Subscription callback for MoveIt Servo TwistStamped command messages.
   * @details Publishes the received command messages through @ref servo_twist_publisher_ if the publisher exists
   * @param msg Servo command
   */
  void onCartesianJog(const TwistStamped::SharedPtr msg);

  /**
   * @brief Check that the Behavior's service clients were all initialized, and that all the corresponding service
   * servers exist.
   * @return fp::Result containing void if validation succeeds. Returns error code of type fp::FailedPrecondition if a
   * service client was not initialized. Returns error code of type fp::Unavailable if the service server required by
   * one of the service clients was not ready.
   */
  fp::Result<void> validateClients() const;

  std::atomic<BT::NodeStatus> process_status_;

  /**
   * @brief Subscriber for TwistStamped command messages.
   * @details Messages will be published to this topic by the part of the system responsible for handling user input,
   * such as the MoveIt Studio Web UI.
   */
  rclcpp::Subscription<TwistStamped>::SharedPtr move_end_effector_subscriber_;

  /**
   * @brief Controls access to @ref twist_publisher_.
   * @details Locked within the following scopes:
   * - In @ref doTeleoperation while the publishers are being initialized
   * - In @ref onJointJog and onCartesianJog while command messages are being published
   * - In @ref onHalted while the publishers are being reset
   */
  std::mutex publisher_mutex_;

  /**
   * @brief Thread to handle executing @ref doTeleoperation asynchronously.
   */
  std::thread setup_teleop_thread_;

  /**
   * @brief Publisher for MoveIt Servo TwistStamped command messages.
   * @details The subscriber for the advertised topic is part of the MoveIt Servo server node.
   */
  rclcpp::Publisher<TwistStamped>::SharedPtr twist_publisher_;

  /**
   * @brief Service client to activate a set of ros2_control controllers.
   * @details This client sends its requests to the service server advertised by the SetActiveControllerService
   * MoveGroup capability plugin provided in the moveit_studio_plugins package.
   */
  std::shared_ptr<rclcpp::Client<moveit_studio_agent_msgs::srv::SetStringArray>> switch_controller_client_;
};
}  // namespace moveit_studio::behaviors
