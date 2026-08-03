#include "arm_manipulator/motion_action_server.hpp"

using namespace std::placeholders;

MotionActionServer::MotionActionServer() : Node("motion_action_server") {
  action_server_ = rclcpp_action::create_server<ExecuteMotion>(
      this, "execute_motion",
      std::bind(&MotionActionServer::handleGoal, this, _1, _2),
      std::bind(&MotionActionServer::handleCancel, this, _1),
      std::bind(&MotionActionServer::handleAccepted, this, _1));
  //   init_timer_ =
  //       create_wall_timer(std::chrono::seconds(1),
  //                         std::bind(&MotionActionServer::initialize, this));
}

void MotionActionServer::initialize() {
  //   init_timer_->cancel();
  controller_ = std::make_shared<MotionController>(shared_from_this());
  controller_->initialize();

  RCLCPP_INFO(get_logger(), "Motion Action Server ready.");
}

rclcpp_action::GoalResponse MotionActionServer::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMotion::Goal> goal) {
  RCLCPP_INFO(get_logger(), "Received goal: command=%d cell=%d", goal->command,
              goal->cell);

  if ((goal->command == 1 || goal->command == 2) &&
      (goal->cell < 1 || goal->cell > 9)) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MotionActionServer::handleCancel(const std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(get_logger(), "Cancel requested.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionActionServer::handleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  std::thread(std::bind(&MotionActionServer::execute, this, goal_handle))
      .detach();
}

void MotionActionServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle) {
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ExecuteMotion::Feedback>();
  auto result = std::make_shared<ExecuteMotion::Result>();

  feedback->current_step = "Planning";
  feedback->pose = controller_->currentPose();
  goal_handle->publish_feedback(feedback);

  bool success = false;

  switch (goal->command) {
  case ExecuteMotion::Goal::DRAW_GRID:
    feedback->current_step = "Drawing Grid";
    goal_handle->publish_feedback(feedback);

    success = controller_->drawLines(goal->cell);
    break;
  case ExecuteMotion::Goal::DRAW_CIRCLE:
    feedback->current_step = "Drawing Circle";
    goal_handle->publish_feedback(feedback);

    success = controller_->drawCircle(goal->cell);
    break;
  case ExecuteMotion::Goal::DRAW_CROSS:
    feedback->current_step = "Drawing Cross";
    goal_handle->publish_feedback(feedback);

    success = controller_->drawCross(goal->cell);
    break;
  case ExecuteMotion::Goal::MOVE_ZERO:
    feedback->current_step = "Moving to Zero";
    goal_handle->publish_feedback(feedback);

    success = controller_->moveToNamedPose("zero");
    break;
  case ExecuteMotion::Goal::MOVE_HOME:
    feedback->current_step = "Moving to Home";
    goal_handle->publish_feedback(feedback);

    success = controller_->moveToNamedPose("home");
    break;
  case ExecuteMotion::Goal::MOVE_CLEAR:
    feedback->current_step = "Moving to Clear";
    goal_handle->publish_feedback(feedback);

    success = controller_->moveToNamedPose("clear_real");
    break;
  default:
    RCLCPP_ERROR(get_logger(), "Unknown command: %u", goal->command);
    success = false;
    break;
  }

  feedback->pose = controller_->currentPose();

  if (goal_handle->is_canceling()) {
    result->success = false;
    goal_handle->canceled(result);
    return;
  }

  if (success) {
    feedback->current_step = "Completed";
    goal_handle->publish_feedback(feedback);

    result->success = true;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    goal_handle->abort(result);
  }
}