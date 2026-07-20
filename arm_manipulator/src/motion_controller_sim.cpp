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
  executor_thread_ = std::thread([this]() { executor_.spin(); });

  move_group_arm_ =
      std::make_shared<MoveGroupInterface>(node_, PLANNING_GROUP_ARM);
  joint_model_group_arm_ =
      move_group_arm_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_ARM);

  current_state_ = move_group_arm_->getCurrentState(10);
  current_state_->copyJointGroupPositions(joint_model_group_arm_,
                                          joint_group_positions_);

  move_group_arm_->setStartStateToCurrentState();
  current_pose_ = move_group_arm_->getCurrentPose().pose;

  RCLCPP_INFO(LOGGER, "Planning Frame: %s",
              move_group_arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End Effector Link: %s",
              move_group_arm_->getEndEffectorLink().c_str());

  getParameters();

  RCLCPP_INFO(LOGGER, "MotionControllerSim initialized.");
}

MotionControllerSim::~MotionControllerSim() {
  executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  RCLCPP_INFO(LOGGER, "MotionControllerSim terminated.");
}

void MotionControllerSim::executeTrajectory() {
  RCLCPP_INFO(LOGGER, "Executing trajectory");

  if (!moveToCell(5))
    return;

  if (!drawLines(0.05))
    return;

  if (!moveToCell(3))
    return;

  if (!drawCircle(0.02))
    return;

  if (!moveToCell(9))
    return;

  if (!drawCross(0.02))
    return;

  RCLCPP_INFO(LOGGER, "Trajectory complete");
}

bool MotionControllerSim::moveToCell(int cell) {
  Pose target = getCellPose(cell);
  target.position.z += approach_distance_;
  move_group_arm_->setPoseTarget(target);

  Plan plan;
  auto result = move_group_arm_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  return move_group_arm_->execute(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS;
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
  auto stroke = PathGenerator::generateCircle(currentPose(), radius);

  return executeCartesian(stroke, "Draw Circle");
}

bool MotionControllerSim::drawCross(double size) {
  auto strokes = PathGenerator::generateCross(currentPose(), size);

  return executeStrokes(strokes, "Draw Cross");
}

bool MotionControllerSim::drawLines(double cell_size) {
  auto strokes = PathGenerator::generateGrid(currentPose(), cell_size);

  return executeStrokes(strokes, "Draw Lines");
}

bool MotionControllerSim::executeStrokes(
    const std::vector<PathGenerator::Stroke> &strokes,
    const std::string &name) {
  for (const auto &stroke : strokes) {
    if (!approach())
      return false;
    rclcpp::sleep_for(1s);

    if (!executeCartesian(stroke, name))
      return false;
    rclcpp::sleep_for(1s);

    if (!retreat())
      return false;
    rclcpp::sleep_for(1s);
  }

  return true;
}

MotionControllerSim::Pose MotionControllerSim::currentPose() const {
  return move_group_arm_->getCurrentPose().pose;
}

bool MotionControllerSim::executeCartesian(const std::vector<Pose> &waypoints,
                                           const std::string &plan_name) {
  RCLCPP_INFO(LOGGER, "Planning %s", plan_name.c_str());

  move_group_arm_->setStartStateToCurrentState();
  RobotTrajectory trajectory;

  double fraction = move_group_arm_->computeCartesianPath(
      waypoints, END_EFFECTOR_STEP, JUMP_THRESHOLD, trajectory, true);
  if (fraction < MIN_CARTESIAN_FRACTION) {
    RCLCPP_ERROR(LOGGER, "Cartesian path fraction %.3f", fraction);
    return false;
  }

  auto result = move_group_arm_->execute(trajectory);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_arm_->stop();
    RCLCPP_ERROR(LOGGER, "Execution failed. Error code: %d", result.val);
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

  for (int i = 1; i <= 9; ++i) {
    auto values = node_->declare_parameter<std::vector<double>>(
        "board.cells." + std::to_string(i));
    if (values.size() != 7) {
      throw std::runtime_error("Cell " + std::to_string(i) +
                               " must contain [x, y, z, x, y, z, w]");
    }

    Pose pose;
    pose.position.x = values[0];
    pose.position.y = values[1];
    pose.position.z = values[2];

    pose.orientation.x = values[3];
    pose.orientation.y = values[4];
    pose.orientation.z = values[5];
    pose.orientation.w = values[6];

    cell_poses_[i - 1] = pose;
  }
}

MotionControllerSim::Pose MotionControllerSim::getCellPose(int cell) const {
  return cell_poses_.at(cell - 1);
}