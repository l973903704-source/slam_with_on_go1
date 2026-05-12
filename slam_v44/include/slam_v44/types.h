#ifndef SLAM_V44_TYPES_H
#define SLAM_V44_TYPES_H

#include <ros/time.h>

#include <cmath>
#include <string>
#include <vector>

namespace slam_v44
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct ProcessedScan
{
  ros::Time stamp;
  std::string frame_id;
  std::vector<Point2D> points;
};

inline double normalizeAngle(double angle)
{
  while (angle > kPi)
  {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi)
  {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double squaredDistance(const Pose2D& a, const Pose2D& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline Point2D transformPoint(const Pose2D& pose, const Point2D& point)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  return Point2D{pose.x + c * point.x - s * point.y,
                 pose.y + s * point.x + c * point.y};
}

}  // namespace slam_v44

#endif  // SLAM_V44_TYPES_H

