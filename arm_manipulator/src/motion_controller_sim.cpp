#include "arm_manipulator/motion_controller_sim.hpp"
#include "arm_manipulator/path_generator.hpp"

#include <chrono>

using namespace std::chrono_literals;

const std::string MotionControllerSim::PLANNING_GROUP_ARM = "arm";

MotionControllerSim::MotionControllerSim() : Node("motion_controller_sim") {
  getParameters();
}

MotionControllerSim::~MotionControllerSim() {
  RCLCPP_INFO(get_logger(), "MotionControllerSim terminated.");
}

void MotionControllerSim::initialize() {
  RCLCPP_INFO(get_logger(), "Initializing MotionControllerSim...");
  move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(),
                                                         PLANNING_GROUP_ARM);

  // Wait to get new states
  move_group_arm_->startStateMonitor();
  rclcpp::sleep_for(1s);

  current_state_ = move_group_arm_->getCurrentState(10.0);
  if (!current_state_) {
    throw std::runtime_error("Failed to receive current robot state.");
  }
  joint_model_group_arm_ =
      current_state_->getJointModelGroup(PLANNING_GROUP_ARM);

  // Print out system info
  RCLCPP_INFO(get_logger(), "Planning Frame: %s",
              move_group_arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(get_logger(), "End Effector Link: %s",
              move_group_arm_->getEndEffectorLink().c_str());
  RCLCPP_INFO(get_logger(), "Available Planning Groups:");
  std::vector<std::string> group_names =
      move_group_arm_->getJointModelGroupNames();
  for (long unsigned int i = 0; i < group_names.size(); i++) {
    RCLCPP_INFO(get_logger(), "Group %ld: %s", i, group_names[i].c_str());
  }
  logCurrentPose();

  // Set start state of robot to current state
  move_group_arm_->setStartStateToCurrentState();

  RCLCPP_INFO(get_logger(), "MotionControllerSim initialized.");
}

void MotionControllerSim::executeTrajectory() {
  RCLCPP_INFO(get_logger(), "Executing trajectory");

  if (!drawLines()) {
    return;
  }

  //   if (!drawCross(5)) {
  //     return;
  //   }

  RCLCPP_INFO(get_logger(), "Trajectory complete");
}

bool MotionControllerSim::moveAbove(const Pose &pose) {
  Pose start = currentPose();
  Pose end = pose;
  end.position.z += offset_distance_;

  std::vector<Pose> waypoints{start, end};

  return executeCartesian(waypoints, "Move Above Stroke");
}

bool MotionControllerSim::approach() {
  Pose start = currentPose();
  Pose end = start;
  end.position.z -= offset_distance_;

  std::vector<Pose> waypoints{start, end};

  return executeCartesian(waypoints, "Approach");
}

bool MotionControllerSim::retreat() {
  Pose start = currentPose();
  Pose end = start;
  end.position.z += offset_distance_;

  std::vector<Pose> waypoints{start, end};

  return executeCartesian(waypoints, "Retreat");
}

bool MotionControllerSim::drawCircle(int cell) {
  auto strokes =
      PathGenerator::generateCircle(getCellPose(cell), circle_radius_);

  return executeStrokes(strokes, "Draw Circle");
}

bool MotionControllerSim::drawCross(int cell) {
  auto strokes = PathGenerator::generateCross(getCellPose(cell), cross_size_);

  return executeStrokes(strokes, "Draw Cross");
}

bool MotionControllerSim::drawLines(int cell) {
  auto strokes =
      PathGenerator::generateGrid(getCellPose(cell), grid_cell_size_);

  return executeStrokes(strokes, "Draw Lines");
}

bool MotionControllerSim::executeStrokes(
    const std::vector<PathGenerator::Stroke> &strokes,
    const std::string &name) {
  for (const auto &stroke : strokes) {
    if (stroke.empty()) {
      continue;
    }
    // Move above the first waypoint
    if (!moveAbove(stroke.front())) {
      return false;
    }
    rclcpp::sleep_for(1s);

    // Lower pen
    if (!approach()) {
      return false;
    }
    rclcpp::sleep_for(1s);

    // Draw
    if (!executeCartesian(stroke, name)) {
      return false;
    }
    rclcpp::sleep_for(1s);

    // Lift pen
    if (!retreat()) {
      return false;
    }
    rclcpp::sleep_for(1s);
  }

  return true;
}

MotionControllerSim::Pose MotionControllerSim::currentPose() const {
  return move_group_arm_->getCurrentPose().pose;
}

void MotionControllerSim::logCurrentPose() const {
  auto pose = currentPose();

  RCLCPP_INFO(get_logger(),
              "Position:"
              "\nx %.4f"
              "\ny %.4f"
              "\nz %.4f",
              pose.position.x, pose.position.y, pose.position.z);
  RCLCPP_INFO(get_logger(),
              "Orientation:"
              "\nqx %.4f"
              "\nqy %.4f"
              "\nqz %.4f"
              "\nqw %.4f",
              pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w);
}

bool MotionControllerSim::executeCartesian(const std::vector<Pose> &waypoints,
                                           const std::string &plan_name) {
  RCLCPP_INFO(get_logger(), "Planning %s", plan_name.c_str());

  move_group_arm_->setStartStateToCurrentState();
  RobotTrajectory trajectory;

  double fraction = move_group_arm_->computeCartesianPath(
      waypoints, END_EFFECTOR_STEP, JUMP_THRESHOLD, trajectory);
  if (fraction < MIN_CARTESIAN_FRACTION) {
    RCLCPP_ERROR(get_logger(), "Cartesian path fraction %.3f", fraction);
    return false;
  }

  auto result = move_group_arm_->execute(trajectory);
  move_group_arm_->stop();
  move_group_arm_->clearPoseTargets();

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "%s planning failed (fraction %.2f)",
                 plan_name.c_str(), fraction);
    return false;
  }

  RCLCPP_INFO(get_logger(), "%s complete.", plan_name.c_str());

  return true;
}

void MotionControllerSim::setupNamedPose(const std::string &pose_name) {
  move_group_arm_->setNamedTarget(pose_name);
}

void MotionControllerSim::getParameters() {
  offset_distance_ = declare_parameter("offset_distance", 0.00);
  circle_radius_ = declare_parameter("circle_radius", 0.00);
  cross_size_ = declare_parameter("cross_size", 0.00);
  grid_cell_size_ = declare_parameter("grid_cell_size", 0.00);
  RCLCPP_INFO(get_logger(), "===== Motion Parameters =====");
  RCLCPP_INFO(get_logger(), "offset_distance : %.3f m", offset_distance_);
  RCLCPP_INFO(get_logger(), "circle_radius   : %.3f m", circle_radius_);
  RCLCPP_INFO(get_logger(), "cross_size      : %.3f m", cross_size_);
  RCLCPP_INFO(get_logger(), "grid_cell_size  : %.3f m", grid_cell_size_);

  for (int i = 1; i <= 9; ++i) {
    auto values = declare_parameter<std::vector<double>>("board.cells." +
                                                         std::to_string(i));
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