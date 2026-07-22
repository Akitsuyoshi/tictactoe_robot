#include "arm_manipulator/motion_controller_sim.hpp"
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotionControllerSim>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  std::thread spin_thread([&]() { executor.spin(); });

  controller->initialize();
  controller->executeTrajectory();

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}