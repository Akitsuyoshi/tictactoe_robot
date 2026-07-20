#include "arm_manipulator/motion_controller_sim.hpp"
#include "arm_manipulator/path_generator.hpp"

#include <chrono>

using namespace std::chrono_literals;

const rclcpp::Logger MotionControllerSim::LOGGER =
    rclcpp::get_logger("motion_controller_sim");
const std::string MotionControllerSim::PLANNING_GROUP_ARM = "arm";

MotionControllerSim::MotionControllerSim(const rclcpp::Node::SharedPtr &node)
    : node_(node) {
  RCLCPP_INFO(LOGGER, "Initializing MotionControllerSim...");

  // Spin node
  executor_.add_node(node_);
  std::thread([this]() { this->executor_.spin(); }).detach();

  move_group_arm_ =
      std::make_shared<MoveGroupInterface>(node_, PLANNING_GROUP_ARM);
  joint_model_group_arm_ =
      move_group_arm_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_ARM);

  current_state_ = move_group_arm_->getCurrentState(10);
  current_state_->copyJointGroupPositions(joint_model_group_arm_,
                                          joint_group_positions_);

  move_group_arm_->setStartStateToCurrentState();

  RCLCPP_INFO(LOGGER, "Planning Frame: %s",
              move_group_arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End Effector Link: %s",
              move_group_arm_->getEndEffectorLink().c_str());

  getParameters();

  RCLCPP_INFO(LOGGER, "MotionControllerSim initialized.");
}

MotionControllerSim::~MotionControllerSim() {
  RCLCPP_INFO(LOGGER, "MotionControllerSim terminated.");
}

void MotionControllerSim::executeTrajectory() {
  RCLCPP_INFO(LOGGER, "Executing trajectory");

  if (!approach())
    return;
  rclcpp::sleep_for(1s);

  if (!retreat())
    return;
  rclcpp::sleep_for(1s);

  // drawCircle(2.0);

  drawLines(0.01);

  RCLCPP_INFO(LOGGER, "Trajectory complete");
}

bool MotionControllerSim::approach() {
  Pose start = currentPose();
  Pose end = start;
  end.position.z -= approach_distance_;

  std::vector<Pose> waypoints{start, end};

  return executeCartesian(waypoints, "Approach");
}

bool MotionControllerSim::retreat() {
  Pose start = currentPose();
  Pose end = start;
  end.position.z += retreat_distance_;

  std::vector<Pose> waypoints{start, end};

  return executeCartesian(waypoints, "Retreat");
}

bool MotionControllerSim::drawCircle(double radius) {
  auto waypoints = PathGenerator::generateCircle(currentPose(), radius);

  return executeCartesian(waypoints, "Draw Circle");
}

bool MotionControllerSim::drawCross(double size) {
  auto waypoints = PathGenerator::generateCross(currentPose(), size);

  return executeCartesian(waypoints, "Draw Cross");
}

bool MotionControllerSim::drawLines(double cell_size) {
  auto waypoints = PathGenerator::generateGrid(currentPose(), cell_size);

  return executeCartesian(waypoints, "Draw Lines");
}

MotionControllerSim::Pose MotionControllerSim::currentPose() const {
  return move_group_arm_->getCurrentPose().pose;
}

bool MotionControllerSim::executeCartesian(const std::vector<Pose> &waypoints,
                                           const std::string &plan_name) {
  RCLCPP_INFO(LOGGER, "Planning %s", plan_name.c_str());

  cartesian_plan_.joint_trajectory.points.clear();

  move_group_arm_->setStartStateToCurrentState();

  double fraction = move_group_arm_->computeCartesianPath(
      waypoints, END_EFFECTOR_STEP, JUMP_THRESHOLD, cartesian_plan_, true);
  if (fraction < MIN_CARTESIAN_FRACTION) {
    RCLCPP_ERROR(LOGGER, "Cartesian path fraction %.3f", fraction);
    return false;
  }

  auto result = move_group_arm_->execute(cartesian_plan_);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "%s complete.", plan_name.c_str());

  return true;
}

void MotionControllerSim::setupNamedPose(const std::string &pose_name) {
  move_group_arm_->setNamedTarget(pose_name);
}

void MotionControllerSim::getParameters() {
  approach_distance_ = node_->declare_parameter("approach_distance", 0.05);
  retreat_distance_ = node_->declare_parameter("retreat_distance", 0.05);

  RCLCPP_INFO(LOGGER, "approach_distance = %.3f", approach_distance_);
  RCLCPP_INFO(LOGGER, "retreat_distance = %.3f", retreat_distance_);
}