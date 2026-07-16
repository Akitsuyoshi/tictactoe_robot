#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
class PlanAndExecute {
public:
  PlanAndExecute(rclcpp::Node::SharedPtr node) : node_(node) {
    RCLCPP_INFO(LOGGER, "Initializing Class: PlanAndExecute...");
    RCLCPP_INFO(LOGGER, "Class Initialized: PlanAndExecute");
  };
  ~PlanAndExecute() { RCLCPP_INFO(LOGGER, "Class Terminated"); }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("plan_and_execute");
  PlanAndExecute plan_and_execute(node);
  //   plan_and_execute.execute_trajectory();

  rclcpp::shutdown();
  return 0;
}