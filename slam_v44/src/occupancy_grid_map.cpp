#include "slam_v44/occupancy_grid_map.h"

#include <algorithm>
#include <cmath>

namespace slam_v44
{

namespace
{
double probabilityFromLogOdds(double log_odds)
{
  return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
}
}  // namespace

OccupancyGridMap::OccupancyGridMap(const OccupancyGridMapConfig& config)
  : config_(config)
{
  width_ = static_cast<int>(std::ceil(config_.width_m / config_.resolution));
  height_ = static_cast<int>(std::ceil(config_.height_m / config_.resolution));
  reset();
}

void OccupancyGridMap::reset()
{
  log_odds_.assign(static_cast<size_t>(width_ * height_), 0.0f);
  observed_.assign(static_cast<size_t>(width_ * height_), 0);
}

void OccupancyGridMap::updateByScan(const Pose2D& pose, const ProcessedScan& scan)
{
  int robot_mx = 0;
  int robot_my = 0;
  if (!worldToMap(pose.x, pose.y, robot_mx, robot_my))
  {
    return;
  }

  for (const Point2D& point : scan.points)
  {
    const Point2D world = transformPoint(pose, point);
    int end_mx = 0;
    int end_my = 0;
    if (!worldToMap(world.x, world.y, end_mx, end_my))
    {
      continue;
    }

    traceRay(robot_mx, robot_my, end_mx, end_my);
    markOccupied(end_mx, end_my);
  }
}

nav_msgs::OccupancyGrid OccupancyGridMap::toMessage(const ros::Time& stamp) const
{
  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = config_.map_frame;
  msg.info.resolution = config_.resolution;
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = config_.origin_x;
  msg.info.origin.position.y = config_.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<size_t>(width_ * height_), -1);

  for (int y = 0; y < height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      const int index = toIndex(x, y);
      if (!observed_[index])
      {
        msg.data[index] = -1;
        continue;
      }

      const double probability = probabilityFromLogOdds(log_odds_[index]);
      if (probability >= config_.occupied_threshold)
      {
        msg.data[index] = 100;
      }
      else if (probability <= config_.free_threshold)
      {
        msg.data[index] = 0;
      }
      else
      {
        msg.data[index] = static_cast<int8_t>(std::max(0.0, std::min(100.0, probability * 100.0)));
      }
    }
  }

  return msg;
}

bool OccupancyGridMap::worldToMap(double wx, double wy, int& mx, int& my) const
{
  mx = static_cast<int>(std::floor((wx - config_.origin_x) / config_.resolution));
  my = static_cast<int>(std::floor((wy - config_.origin_y) / config_.resolution));
  return isInside(mx, my);
}

void OccupancyGridMap::mapToWorld(int mx, int my, double& wx, double& wy) const
{
  wx = config_.origin_x + (static_cast<double>(mx) + 0.5) * config_.resolution;
  wy = config_.origin_y + (static_cast<double>(my) + 0.5) * config_.resolution;
}

bool OccupancyGridMap::isInside(int mx, int my) const
{
  return mx >= 0 && my >= 0 && mx < width_ && my < height_;
}

int OccupancyGridMap::toIndex(int mx, int my) const
{
  return my * width_ + mx;
}

double OccupancyGridMap::occupancyProbability(int mx, int my) const
{
  if (!isInside(mx, my))
  {
    return 0.5;
  }
  const int index = toIndex(mx, my);
  if (!observed_[index])
  {
    return 0.5;
  }
  return probabilityFromLogOdds(log_odds_[index]);
}

double OccupancyGridMap::occupiedLikelihoodWorld(double wx, double wy, int radius_cells) const
{
  int mx = 0;
  int my = 0;
  if (!worldToMap(wx, wy, mx, my))
  {
    return 0.0;
  }

  double best = 0.0;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int nx = mx + dx;
      const int ny = my + dy;
      if (!isInside(nx, ny))
      {
        continue;
      }
      const int index = toIndex(nx, ny);
      if (!observed_[index])
      {
        continue;
      }

      const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy));
      const double weight = 1.0 / (1.0 + distance);
      best = std::max(best, occupancyProbability(nx, ny) * weight);
    }
  }
  return best;
}

bool OccupancyGridMap::isObserved(int mx, int my) const
{
  return isInside(mx, my) && observed_[toIndex(mx, my)];
}

int OccupancyGridMap::width() const
{
  return width_;
}

int OccupancyGridMap::height() const
{
  return height_;
}

double OccupancyGridMap::resolution() const
{
  return config_.resolution;
}

double OccupancyGridMap::originX() const
{
  return config_.origin_x;
}

double OccupancyGridMap::originY() const
{
  return config_.origin_y;
}

void OccupancyGridMap::markFree(int mx, int my)
{
  addLogOdds(mx, my, config_.log_odds_miss);
}

void OccupancyGridMap::markOccupied(int mx, int my)
{
  addLogOdds(mx, my, config_.log_odds_hit);
}

void OccupancyGridMap::addLogOdds(int mx, int my, double delta)
{
  if (!isInside(mx, my))
  {
    return;
  }

  const int index = toIndex(mx, my);
  observed_[index] = 1;
  const double value = static_cast<double>(log_odds_[index]) + delta;
  log_odds_[index] = static_cast<float>(
      std::max(config_.log_odds_min, std::min(config_.log_odds_max, value)));
}

void OccupancyGridMap::traceRay(int x0, int y0, int x1, int y1)
{
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;

  while (true)
  {
    if (x == x1 && y == y1)
    {
      break;
    }

    markFree(x, y);
    const int e2 = 2 * err;
    if (e2 >= dy)
    {
      err += dy;
      x += sx;
    }
    if (e2 <= dx)
    {
      err += dx;
      y += sy;
    }

    if (!isInside(x, y))
    {
      break;
    }
  }
}

}  // namespace slam_v44

