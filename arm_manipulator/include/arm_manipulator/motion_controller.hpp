#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include "rclcpp/node.hpp"
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "arm_manipulator/path_generator.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class MotionController {
  using Pose = geometry_msgs::msg::Pose;

public:
  explicit MotionController(const rclcpp::Node::SharedPtr &node);
  ~MotionController();

  void initialize();

  bool moveToNamedPose(const std::string &pose_name);

  /// Motion primitives
  bool moveAbove(const Pose &pose);
  bool approach();
  bool retreat();

  /// Drawing primitives
  bool drawCircle(int cell);
  bool drawCross(int cell);
  bool drawLines(int cell = 5);

  bool executeStrokes(const std::vector<PathGenerator::Stroke> &strokes,
                      const std::string &name);

  Pose currentPose() const;

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  //-----------------------
  // Initialization
  //-----------------------
  void getParameters();

  //-----------------------
  // Motion helpers
  //-----------------------
  void setupNamedPose(const std::string &pose_name);

  bool executeCartesian(const std::vector<Pose> &waypoints,
                        const std::string &plan_name);

  Pose getCellPose(int cell) const;
  void logCurrentPose() const;

  //-----------------------
  // Members
  //-----------------------
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  const JointModelGroup *joint_model_group_arm_;

  RobotStatePtr current_state_;

  //-----------------------
  // Parameters
  //-----------------------
  double offset_distance_;
  double circle_radius_;
  double cross_size_;
  double grid_cell_size_;
  std::array<Pose, 9> cell_poses_;

  //-----------------------
  // Planning constants
  //-----------------------
  static constexpr double JUMP_THRESHOLD = 0.0;
  static constexpr double END_EFFECTOR_STEP = 0.003;
  static constexpr double MIN_CARTESIAN_FRACTION = 0.99;

  static const std::string PLANNING_GROUP_ARM;
};

#endif