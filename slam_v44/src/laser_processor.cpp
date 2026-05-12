#include "slam_v44/laser_processor.h"

#include <algorithm>
#include <cmath>

namespace slam_v44
{

LaserProcessor::LaserProcessor(const LaserProcessorConfig& config)
  : config_(config)
{
}

ProcessedScan LaserProcessor::process(const sensor_msgs::LaserScan& msg) const
{
  ProcessedScan scan;
  scan.stamp = msg.header.stamp;
  scan.frame_id = msg.header.frame_id;

  double min_range = config_.min_range;
  double max_range = config_.max_range;
  if (config_.use_scan_range_limits)
  {
    min_range = std::max(min_range, static_cast<double>(msg.range_min));
    if (msg.range_max > 0.0f)
    {
      max_range = std::min(max_range, static_cast<double>(msg.range_max));
    }
  }

  const int stride = std::max(1, config_.scan_skip);
  scan.points.reserve(msg.ranges.size() / static_cast<size_t>(stride) + 1);

  for (size_t i = 0; i < msg.ranges.size(); i += static_cast<size_t>(stride))
  {
    const double range = msg.ranges[i];
    if (!std::isfinite(range) || range < min_range || range > max_range)
    {
      continue;
    }

    const double angle = static_cast<double>(msg.angle_min) +
                         static_cast<double>(i) * static_cast<double>(msg.angle_increment);
    scan.points.push_back(Point2D{range * std::cos(angle), range * std::sin(angle)});
  }

  return scan;
}

}  // namespace slam_v44

