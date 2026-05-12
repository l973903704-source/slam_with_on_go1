#ifndef SLAM_V44_LOCAL_CONTROLLER_H
#define SLAM_V44_LOCAL_CONTROLLER_H

#include <geometry_msgs/Twist.h>

#include <vector>

#include "slam_v44/types.h"

namespace slam_v44
{

struct LocalControllerConfig
{
  double goal_tolerance = 0.20;
  double yaw_tolerance = 0.25;
  double lookahead_distance = 0.60;
  double max_linear_speed = 0.25;
  double max_angular_speed = 0.60;
  double linear_gain = 0.70;
  double angular_gain = 1.40;
  double min_front_obstacle_distance = 0.45;
  double slow_front_obstacle_distance = 0.90;
};

struct ControlResult
{
  geometry_msgs::Twist cmd;
  bool reached = false;
  bool blocked = false;
};

class LocalController
{
public:
  explicit LocalController(const LocalControllerConfig& config);

  ControlResult computeCommand(const Pose2D& robot_pose,
                               const std::vector<Pose2D>& path,
                               double front_obstacle_distance) const;

private:
  int findLookaheadIndex(const Pose2D& robot_pose, const std::vector<Pose2D>& path) const;
  double clamp(double value, double min_value, double max_value) const;

  LocalControllerConfig config_;
};

}  // namespace slam_v44

#endif  // SLAM_V44_LOCAL_CONTROLLER_H

