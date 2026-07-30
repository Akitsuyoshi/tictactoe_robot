#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_manipulator/motion_controller.hpp"
#include "tictactoe_interfaces/action/execute_motion.hpp"

class MotionActionServer : public rclcpp::Node {
public:
  using ExecuteMotion = tictactoe_interfaces::action::ExecuteMotion;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

  MotionActionServer();
  void initialize();

private:
  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;

  std::shared_ptr<MotionController> controller_;

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const ExecuteMotion::Goal> goal);

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::TimerBase::SharedPtr init_timer_;
};