#include "arm_manipulator/path_generator.hpp"

PathGenerator::Stroke PathGenerator::generateCircle(const Pose &center,
                                                    double radius, int points) {
  Stroke stroke;

  for (int i = 0; i <= points; i++) {
    double theta = 2.0 * M_PI * i / points;
    double x = radius * cos(theta);
    double y = radius * sin(theta);
    stroke.push_back(offsetPose(center, x, y));
  }

  return stroke;
}

std::vector<PathGenerator::Stroke>
PathGenerator::generateCross(const Pose &center, double size) {
  std::vector<Stroke> strokes;

  double h = size / 2.0;

  // First diagonal
  strokes.push_back({offsetPose(center, -h, -h), offsetPose(center, h, h)});
  // Second diagonal
  strokes.push_back({offsetPose(center, -h, h), offsetPose(center, h, -h)});

  return strokes;
}

std::vector<PathGenerator::Stroke>
PathGenerator::generateGrid(const Pose &center, double cell_size) {
  std::vector<Stroke> strokes;

  double half = cell_size * 1.5;

  strokes.push_back({offsetPose(center, -cell_size / 2, -half),
                     offsetPose(center, -cell_size / 2, half)});

  strokes.push_back({offsetPose(center, cell_size / 2, -half),
                     offsetPose(center, cell_size / 2, half)});

  strokes.push_back({offsetPose(center, -half, -cell_size / 2),
                     offsetPose(center, half, -cell_size / 2)});

  strokes.push_back({offsetPose(center, -half, cell_size / 2),
                     offsetPose(center, half, cell_size / 2)});

  return strokes;
}

PathGenerator::Pose PathGenerator::offsetPose(const Pose &pose, double x,
                                              double y) {
  Pose result = pose;
  result.position.x += x;
  result.position.y += y;
  return result;
}