#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tictactoe_interfaces/action/execute_motion.hpp"

class MotionActionClient : public rclcpp::Node {
public:
  using ExecuteMotion = tictactoe_interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion =
      rclcpp_action::ClientGoalHandle<ExecuteMotion>;

  explicit MotionActionClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("motion_action_client", options) {

    action_client_ =
        rclcpp_action::create_client<ExecuteMotion>(this, "execute_motion");

    timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MotionActionClient::generate_sequence_and_start, this));
  }

private:
  rclcpp_action::ClientGoalHandle<ExecuteMotion>::SharedPtr goal_handle_;
  rclcpp_action::Client<ExecuteMotion>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<uint8_t> command_sequence_;
  size_t current_step_index_ = 0;
  const size_t TOTAL_STEPS = 14;
  int current_cell_index_ = 1;

  void trigger_capture_image() {
    RCLCPP_INFO(get_logger(), "Spawning image capture thread...");

    std::string command =
        "ros2 run dataset_scripts save_image_node --ros-args -p split:=test";

    int return_code = std::system(command.c_str());
    if (return_code != 0) {
      RCLCPP_WARN(get_logger(),
                  "Image capture process returned a non-zero exit code: %d",
                  return_code);
    } else {
      RCLCPP_INFO(get_logger(),
                  "Image successfully captured by background process!");
    }
  }

  void generate_sequence_and_start() {
    timer_->cancel();

    command_sequence_.push_back(ExecuteMotion::Goal::MOVE_HOME); // 1
    command_sequence_.push_back(ExecuteMotion::Goal::DRAW_GRID); // 2

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(ExecuteMotion::Goal::DRAW_CIRCLE,
                                          ExecuteMotion::Goal::DRAW_CROSS);

    for (size_t i = 2; i < 11; ++i) { // 3 to 11 (Random draws)
      command_sequence_.push_back(static_cast<uint8_t>(distr(gen)));
    }

    command_sequence_.push_back(ExecuteMotion::Goal::MOVE_HOME);  // 12
    command_sequence_.push_back(ExecuteMotion::Goal::MOVE_CLEAR); // 13
    command_sequence_.push_back(ExecuteMotion::Goal::MOVE_HOME);  // 14

    send_next_goal();
  }

  void send_next_goal() {
    if (current_step_index_ >= TOTAL_STEPS) {
      RCLCPP_INFO(get_logger(), "All 14 steps completed.");
      rclcpp::shutdown();
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not found.");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = ExecuteMotion::Goal();
    goal_msg.command = command_sequence_[current_step_index_];
    if (goal_msg.command == ExecuteMotion::Goal::DRAW_CIRCLE ||
        goal_msg.command == ExecuteMotion::Goal::DRAW_CROSS) {
      goal_msg.cell = static_cast<int32_t>(current_cell_index_);
      current_cell_index_++;
    } else {
      goal_msg.cell = 5;
    }

    auto send_goal_options =
        rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MotionActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&MotionActionClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &MotionActionClient::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(
      const GoalHandleExecuteMotion::SharedPtr &goal_handle) {
    if (!goal_handle)
      rclcpp::shutdown();
  }

  void feedback_callback(GoalHandleExecuteMotion::SharedPtr,
                         const std::shared_ptr<const ExecuteMotion::Feedback>) {
  }

  void result_callback(const GoalHandleExecuteMotion::WrappedResult &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Goal failed. Aborting.");
      rclcpp::shutdown();
      return;
    }

    // Capture the image
    if (current_step_index_ == 11) {
      trigger_capture_image();
    }

    current_step_index_++;
    send_next_goal();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionActionClient>());
  rclcpp::shutdown();
  return 0;
}