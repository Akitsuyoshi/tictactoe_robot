# Tic-Tac-Toe Robot

A ROS 2 project that enables a robotic arm to autonomously play tic-tac-toe using computer vision, motion planning, and the Minimax algorithm.

## Overview

This project integrates robotic manipulation, computer vision, and game AI to play tic-tac-toe against a human opponent.

The system performs the following tasks:

* Detects the tic-tac-toe board using a camera
* Recognizes player symbols (X and O) using deep learning YOLO model
* Computes the optimal move using the Minimax algorithm
* Plans and executes robot arm motions with MoveIt 2