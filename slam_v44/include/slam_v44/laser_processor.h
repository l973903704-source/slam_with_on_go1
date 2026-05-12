#ifndef SLAM_V44_LASER_PROCESSOR_H
#define SLAM_V44_LASER_PROCESSOR_H

#include <sensor_msgs/LaserScan.h>

#include "slam_v44/types.h"

namespace slam_v44
{

struct LaserProcessorConfig
{
  double min_range = 0.05;
  double max_range = 20.0;
  bool use_scan_range_limits = true;
  int scan_skip = 1;
};

class LaserProcessor
{
public:
  explicit LaserProcessor(const LaserProcessorConfig& config);

  ProcessedScan process(const sensor_msgs::LaserScan& msg) const;

private:
  LaserProcessorConfig config_;
};

}  // namespace slam_v44

#endif  // SLAM_V44_LASER_PROCESSOR_H

