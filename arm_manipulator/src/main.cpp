#include "arm_manipulator/motion_action_server.hpp"
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<MotionActionServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  std::thread spin_thread([&]() { executor.spin(); });
  server->initialize();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}