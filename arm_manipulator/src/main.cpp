#include "arm_manipulator/motion_controller_sim.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("motion_controller_sim");
  auto controller = std::make_shared<MotionControllerSim>(node);
  controller->executeTrajectory();

  rclcpp::shutdown();
  return 0;
}