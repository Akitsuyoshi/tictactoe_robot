# Tic-Tac-Toe Robot

A ROS2-based project that enables a robotic arm to autonomously play tic-tac-toe using computer vision, motion planning, and the Minimax algorithm.

## Overview

This project integrates robotic manipulation, computer vision, and game AI to play tic-tac-toe against a human opponent.

The system performs the following tasks:

* Detects the tic-tac-toe board using a camera
* Recognizes player symbols (X and O) using deep learning YOLO model
* Computes the optimal move using the Minimax algorithm
* Plans and executes robot arm motions with MoveIt 2

## Helper scripts for developing

### For arm manipulator pkg

Lanuch the simulatin:

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

To kill properly:

```
ps aux | grep gz

pkill -9 gzserver
pkill -9 gzclient
pkill -9 gazebo
```

Laucn moveit2:

```bash
ros2 launch piper_with_gripper_moveit move_group_sim.launch.py
```

Launch rviz2:

```bash
ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py
```

To build and launch motion controller in sim:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select arm_manipulator && source install/setup.bash
ros2 launch arm_manipulator motion_controller_sim.launch.py
```

To build and launch motion controller in real:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select arm_manipulator && source install/setup.bash
ros2 launch arm_manipulator motion_controller_real.launch.py
```