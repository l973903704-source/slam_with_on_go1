#ifndef SLAM_V44_OCCUPANCY_GRID_MAP_H
#define SLAM_V44_OCCUPANCY_GRID_MAP_H

#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <string>
#include <vector>

#include "slam_v44/types.h"

namespace slam_v44
{

struct OccupancyGridMapConfig
{
  double resolution = 0.05;
  double width_m = 40.0;
  double height_m = 40.0;
  double origin_x = -20.0;
  double origin_y = -20.0;
  double log_odds_hit = 0.85;
  double log_odds_miss = -0.40;
  double log_odds_min = -5.0;
  double log_odds_max = 5.0;
  double occupied_threshold = 0.65;
  double free_threshold = 0.35;
  std::string map_frame = "map";
};

class OccupancyGridMap
{
public:
  explicit OccupancyGridMap(const OccupancyGridMapConfig& config);

  void reset();
  void updateByScan(const Pose2D& pose, const ProcessedScan& scan);
  nav_msgs::OccupancyGrid toMessage(const ros::Time& stamp) const;

  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  void mapToWorld(int mx, int my, double& wx, double& wy) const;
  bool isInside(int mx, int my) const;
  int toIndex(int mx, int my) const;

  double occupancyProbability(int mx, int my) const;
  double occupiedLikelihoodWorld(double wx, double wy, int radius_cells) const;
  bool isObserved(int mx, int my) const;

  int width() const;
  int height() const;
  double resolution() const;
  double originX() const;
  double originY() const;

private:
  void markFree(int mx, int my);
  void markOccupied(int mx, int my);
  void addLogOdds(int mx, int my, double delta);
  void traceRay(int x0, int y0, int x1, int y1);

  OccupancyGridMapConfig config_;
  int width_ = 0;
  int height_ = 0;
  std::vector<float> log_odds_;
  std::vector<uint8_t> observed_;
};

}  // namespace slam_v44

#endif  // SLAM_V44_OCCUPANCY_GRID_MAP_H

