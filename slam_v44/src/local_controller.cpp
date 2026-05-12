#include "slam_v44/local_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace slam_v44
{

LocalController::LocalController(const LocalControllerConfig& config)
  : config_(config)
{
}

ControlResult LocalController::computeCommand(const Pose2D& robot_pose,
                                              const std::vector<Pose2D>& path,
                                              double front_obstacle_distance) const
{
  ControlResult result;
  if (path.empty())
  {
    result.blocked = true;
    return result;
  }

  const Pose2D& goal = path.back();
  const double goal_dx = goal.x - robot_pose.x;
  const double goal_dy = goal.y - robot_pose.y;
  const double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);

  if (goal_distance <= config_.goal_tolerance)
  {
    result.reached = true;
    return result;
  }

  if (front_obstacle_distance < config_.min_front_obstacle_distance)
  {
    result.blocked = true;
    return result;
  }

  const int target_index = findLookaheadIndex(robot_pose, path);
  const Pose2D& target = path[static_cast<size_t>(target_index)];
  const double dx = target.x - robot_pose.x;
  const double dy = target.y - robot_pose.y;
  const double target_distance = std::sqrt(dx * dx + dy * dy);
  const double target_yaw = std::atan2(dy, dx);
  const double yaw_error = normalizeAngle(target_yaw - robot_pose.theta);

  double linear_speed = config_.linear_gain * std::min(target_distance, goal_distance);
  if (std::fabs(yaw_error) > 1.0)
  {
    linear_speed = 0.0;
  }
  else
  {
    linear_speed *= std::max(0.0, std::cos(yaw_error));
  }

  if (front_obstacle_distance < config_.slow_front_obstacle_distance)
  {
    const double span = std::max(0.01, config_.slow_front_obstacle_distance - config_.min_front_obstacle_distance);
    const double scale = (front_obstacle_distance - config_.min_front_obstacle_distance) / span;
    linear_speed *= clamp(scale, 0.0, 1.0);
  }

  result.cmd.linear.x = clamp(linear_speed, 0.0, config_.max_linear_speed);
  result.cmd.angular.z = clamp(config_.angular_gain * yaw_error,
                               -config_.max_angular_speed,
                               config_.max_angular_speed);
  return result;
}

int LocalController::findLookaheadIndex(const Pose2D& robot_pose, const std::vector<Pose2D>& path) const
{
  int closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.size(); ++i)
  {
    const double dx = path[i].x - robot_pose.x;
    const double dy = path[i].y - robot_pose.y;
    const double distance = dx * dx + dy * dy;
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_index = static_cast<int>(i);
    }
  }

  for (size_t i = static_cast<size_t>(closest_index); i < path.size(); ++i)
  {
    const double dx = path[i].x - robot_pose.x;
    const double dy = path[i].y - robot_pose.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    if (distance >= config_.lookahead_distance)
    {
      return static_cast<int>(i);
    }
  }

  return static_cast<int>(path.size() - 1);
}

double LocalController::clamp(double value, double min_value, double max_value) const
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

}  // namespace slam_v44

