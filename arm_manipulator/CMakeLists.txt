cmake_minimum_required(VERSION 3.8)
project(arm_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tictactoe_interfaces REQUIRED)

# Executables
add_executable(motion_action_server 
	src/main.cpp
	src/motion_action_server.cpp
	src/motion_controller.cpp
	src/path_generator.cpp
)

target_include_directories(motion_action_server PUBLIC include)
ament_target_dependencies(motion_action_server
    rclcpp
    rclcpp_action
	tictactoe_interfaces
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    interactive_markers
    control_msgs
    moveit_msgs
)

install(TARGETS 
    motion_action_server
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
	config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
