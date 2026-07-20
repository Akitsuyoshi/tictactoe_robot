#include "arm_manipulator/path_generator.hpp"

std::vector<PathGenerator::Pose>
PathGenerator::generateCircle(const Pose &center, double radius, int points) {
  std::vector<Pose> waypoints;

  for (int i = 0; i <= points; i++) {
    double theta = 2.0 * M_PI * i / points;
    double x = radius * cos(theta);
    double y = radius * sin(theta);
    waypoints.push_back(offsetPose(center, x, y));
  }

  return waypoints;
}

std::vector<PathGenerator::Pose>
PathGenerator::generateCross(const Pose &center, double size) {
  std::vector<Pose> waypoints;

  double h = size / 2.0;

  // First diagonal
  waypoints.push_back(offsetPose(center, -h, -h));
  waypoints.push_back(offsetPose(center, h, h));

  // Lift is handled by robot controller

  // Second diagonal
  waypoints.push_back(offsetPose(center, -h, h));
  waypoints.push_back(offsetPose(center, h, -h));

  return waypoints;
}

std::vector<PathGenerator::Pose> PathGenerator::generateGrid(const Pose &center,
                                                             double cell_size) {
  std::vector<Pose> waypoints;

  double half = cell_size * 1.5;

  // Vertical line 1
  waypoints.push_back(offsetPose(center, -cell_size / 2, -half));
  waypoints.push_back(offsetPose(center, -cell_size / 2, half));

  // Vertical line 2
  waypoints.push_back(offsetPose(center, cell_size / 2, -half));
  waypoints.push_back(offsetPose(center, cell_size / 2, half));

  // Horizontal line 1
  waypoints.push_back(offsetPose(center, -half, -cell_size / 2));
  waypoints.push_back(offsetPose(center, half, -cell_size / 2));

  // Horizontal line 2
  waypoints.push_back(offsetPose(center, -half, cell_size / 2));
  waypoints.push_back(offsetPose(center, half, cell_size / 2));

  return waypoints;
}

PathGenerator::Pose PathGenerator::offsetPose(const Pose &pose, double x,
                                              double y) {
  Pose result = pose;
  result.position.x += x;
  result.position.y += y;
  return result;
}