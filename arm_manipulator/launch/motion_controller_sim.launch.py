import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    config_file = os.path.join(get_package_share_directory("arm_manipulator"), "config", "motion_params_sim.yaml")

    moveit_cpp_node = Node(
        name="motion_controller",
        package="arm_manipulator",
        executable="motion_controller",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
            config_file,
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )