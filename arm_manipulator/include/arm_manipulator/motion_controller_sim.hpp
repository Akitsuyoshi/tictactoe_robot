#ifndef MOTION_CONTROLLER_SIM_HPP
#define MOTION_CONTROLLER_SIM_HPP

#include "rclcpp/node.hpp"
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class MotionControllerSim {
public:
  explicit MotionControllerSim(const rclcpp::Node::SharedPtr &node);
  ~MotionControllerSim();

  /// Execute a demo trajectory
  void executeTrajectory();

  /// Motion primitives
  bool approach();
  bool retreat();

  /// Drawing primitives
  bool drawCircle(double radius);
  bool drawCross(double size);
  bool drawLines(double cell_size);

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
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

  Pose currentPose() const;

  //-----------------------
  // Members
  //-----------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  const JointModelGroup *joint_model_group_arm_;

  RobotStatePtr current_state_;

  std::vector<double> joint_group_positions_;

  RobotTrajectory cartesian_plan_;

  //-----------------------
  // Parameters
  //-----------------------
  double approach_distance_;
  double retreat_distance_;

  //-----------------------
  // Planning constants
  //-----------------------
  static constexpr double JUMP_THRESHOLD = 0.0;
  static constexpr double END_EFFECTOR_STEP = 0.01;
  static constexpr double MIN_CARTESIAN_FRACTION = 0.99;

  static const rclcpp::Logger LOGGER;
  static const std::string PLANNING_GROUP_ARM;
};

#endif