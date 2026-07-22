#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

class PathGenerator {
public:
  using Pose = geometry_msgs::msg::Pose;
  using Stroke = std::vector<Pose>;

  static std::vector<Stroke> generateCircle(const Pose &center, double radius,
                                            int points = 30);

  static std::vector<Stroke> generateCross(const Pose &center, double size);

  static std::vector<Stroke> generateGrid(const Pose &center, double cell_size);

private:
  static Pose offsetPose(const Pose &pose, double x, double y);
};

#endif