#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("motion_controller_sim");
static const std::string PLANNING_GROUP_ARM = "arm";

class MotionControllerSim {
public:
  MotionControllerSim(rclcpp::Node::SharedPtr node) : node_(node) {
    RCLCPP_INFO(LOGGER, "Initializing Class: MotionControllerSim...");

    // Spin node
    executor_.add_node(node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // Init move group interfaces
    move_group_arm_ =
        std::make_shared<MoveGroupInterface>(node_, PLANNING_GROUP_ARM);
    // Get init state
    joint_model_group_arm_ =
        move_group_arm_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ARM);
    // Print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_arm_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_arm_->getJointModelGroupNames();
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // Get current state of robot
    current_state_arm_ = move_group_arm_->getCurrentState(10);
    current_state_arm_->copyJointGroupPositions(joint_model_group_arm_,
                                                joint_group_positions_arm_);
    current_state_arm_->copyJointGroupPositions(
        joint_model_group_arm_, init_joint_group_positions_arm_);
    // Set start state of robot to current state
    move_group_arm_->setStartStateToCurrentState();

    get_parameters();

    RCLCPP_INFO(LOGGER, "Class Initialized: MotionControllerSim");
  };

  ~MotionControllerSim() { RCLCPP_INFO(LOGGER, "Class Terminated"); }

  void execute_trajectory() {
    RCLCPP_INFO(LOGGER, "Executing Trajectory");

    setup_waypoints_target(+0.000, +0.000, -approach_distance_);
    if (!execute_cartesian("Approaching")) {
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    setup_waypoints_target(+0.000, +0.000, +retreat_distance_);
    if (!execute_cartesian("Retreating")) {
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "Completed Executing Trajectory");
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  const JointModelGroup *joint_model_group_arm_;

  // trajectory planning variables for arm
  std::vector<double> joint_group_positions_arm_;
  std::vector<double> init_joint_group_positions_arm_;
  RobotStatePtr current_state_arm_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_arm_;

  // cartesian trajectory variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  const double MIN_CARTESIAN_FRACTION = 0.99;

  // variables from ROS params
  double approach_distance_;
  double retreat_distance_;

  void setup_named_pose_arm(std::string pose_name) {
    move_group_arm_->setNamedTarget(pose_name);
  }

  void setup_waypoints_target(double x_delta, double y_delta, double z_delta) {
    // initially set target pose to current pose of the robot
    cartesian_waypoints_.clear();
    target_pose_arm_ = move_group_arm_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_arm_);
    target_pose_arm_.position.x += x_delta;
    target_pose_arm_.position.y += y_delta;
    target_pose_arm_.position.z += z_delta;
    cartesian_waypoints_.push_back(target_pose_arm_);
  }

  bool execute_cartesian(std::string plan_type) {
    // execute the planned trajectory to target using cartesian path
    RCLCPP_INFO(LOGGER, "Planning %s", plan_type.c_str());
    // clear prev plan
    cartesian_trajectory_plan_.joint_trajectory.points.clear();

    move_group_arm_->setStartStateToCurrentState();

    double plan_fraction_arm = move_group_arm_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_, true);

    if (plan_fraction_arm < MIN_CARTESIAN_FRACTION) {
      RCLCPP_ERROR(LOGGER, "Failed planning %s", plan_type.c_str());
      RCLCPP_INFO(LOGGER, "Cartesian path fraction: %.3f", plan_fraction_arm);
      cartesian_waypoints_.clear();
      return false;
    }

    RCLCPP_INFO(LOGGER, "Executing %s", plan_type.c_str());
    bool result = (move_group_arm_->execute(cartesian_trajectory_plan_) ==
                   moveit::core::MoveItErrorCode::SUCCESS);
    if (!result) {
      RCLCPP_ERROR(LOGGER, "Failed executing %s", plan_type.c_str());
      cartesian_waypoints_.clear();
      return false;
    }

    RCLCPP_INFO(LOGGER, "Succeeded %s", plan_type.c_str());
    cartesian_waypoints_.clear();
    return true;
  }

  void get_parameters() {
    approach_distance_ = node_->get_parameter("approach_distance").as_double();
    retreat_distance_ = node_->get_parameter("retreat_distance").as_double();

    RCLCPP_INFO(LOGGER, "Parameters: approach=%.3f retreat=%.3f",
                approach_distance_, retreat_distance_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("motion_controller_sim", options);
  MotionControllerSim motion_controller_sim(node);
  motion_controller_sim.execute_trajectory();

  rclcpp::shutdown();
  return 0;
}