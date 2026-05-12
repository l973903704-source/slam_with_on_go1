#ifndef SLAM_V44_SCAN_MATCHER_H
#define SLAM_V44_SCAN_MATCHER_H

#include "slam_v44/occupancy_grid_map.h"
#include "slam_v44/types.h"

namespace slam_v44
{

struct ScanMatcherConfig
{
  double linear_search_window = 0.25;
  double angular_search_window = 0.17;
  double linear_step = 0.02;
  double angular_step = 0.017;
  int correlation_radius_cells = 2;
  int max_scoring_points = 240;
};

struct ScanMatchResult
{
  Pose2D pose;
  double score = 0.0;
  bool valid = false;
};

class ScanMatcher
{
public:
  explicit ScanMatcher(const ScanMatcherConfig& config);

  void setMap(const OccupancyGridMap* map);
  ScanMatchResult match(const ProcessedScan& scan, const Pose2D& predicted_pose) const;

private:
  double scoreCandidate(const ProcessedScan& scan, const Pose2D& pose, int stride) const;

  ScanMatcherConfig config_;
  const OccupancyGridMap* map_ = nullptr;
};

}  // namespace slam_v44

#endif  // SLAM_V44_SCAN_MATCHER_H

