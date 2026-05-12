#include "slam_v44/scan_matcher.h"

#include <algorithm>
#include <cmath>

namespace slam_v44
{

ScanMatcher::ScanMatcher(const ScanMatcherConfig& config)
  : config_(config)
{
}

void ScanMatcher::setMap(const OccupancyGridMap* map)
{
  map_ = map;
}

ScanMatchResult ScanMatcher::match(const ProcessedScan& scan, const Pose2D& predicted_pose) const
{
  ScanMatchResult result;
  result.pose = predicted_pose;

  if (map_ == nullptr || scan.points.empty())
  {
    return result;
  }

  const int stride = std::max(1, static_cast<int>(scan.points.size()) /
                                    std::max(1, config_.max_scoring_points));
  const double linear_step = std::max(0.001, config_.linear_step);
  const double angular_step = std::max(0.001, config_.angular_step);

  double best_score = -1.0;
  Pose2D best_pose = predicted_pose;

  for (double dx = -config_.linear_search_window; dx <= config_.linear_search_window + 1e-9; dx += linear_step)
  {
    for (double dy = -config_.linear_search_window; dy <= config_.linear_search_window + 1e-9; dy += linear_step)
    {
      for (double da = -config_.angular_search_window; da <= config_.angular_search_window + 1e-9; da += angular_step)
      {
        Pose2D candidate;
        candidate.x = predicted_pose.x + dx;
        candidate.y = predicted_pose.y + dy;
        candidate.theta = normalizeAngle(predicted_pose.theta + da);

        const double score = scoreCandidate(scan, candidate, stride);
        if (score > best_score)
        {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  result.pose = best_pose;
  result.score = std::max(0.0, best_score);
  result.valid = best_score >= 0.0;
  return result;
}

double ScanMatcher::scoreCandidate(const ProcessedScan& scan, const Pose2D& pose, int stride) const
{
  double score = 0.0;
  int count = 0;

  for (size_t i = 0; i < scan.points.size(); i += static_cast<size_t>(stride))
  {
    const Point2D world = transformPoint(pose, scan.points[i]);
    const double likelihood = map_->occupiedLikelihoodWorld(world.x, world.y, config_.correlation_radius_cells);
    if (likelihood > 0.0)
    {
      score += likelihood;
    }
    ++count;
  }

  if (count == 0)
  {
    return 0.0;
  }

  return score / static_cast<double>(count);
}

}  // namespace slam_v44

