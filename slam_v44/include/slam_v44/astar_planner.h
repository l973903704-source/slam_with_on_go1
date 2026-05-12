#ifndef SLAM_V44_ASTAR_PLANNER_H
#define SLAM_V44_ASTAR_PLANNER_H

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "slam_v44/types.h"

namespace slam_v44
{

struct AStarPlannerConfig
{
  double robot_radius = 0.25;
  double inflation_radius = 0.35;
  bool allow_unknown = false;
  bool diagonal_move = true;
  int occupied_value = 65;
};

class AStarPlanner
{
public:
  explicit AStarPlanner(const AStarPlannerConfig& config);

  bool plan(const nav_msgs::OccupancyGrid& map,
            const Pose2D& start,
            const Pose2D& goal,
            std::vector<Pose2D>& path) const;

private:
  bool worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, int& mx, int& my) const;
  void mapToWorld(const nav_msgs::OccupancyGrid& map, int mx, int my, double& wx, double& wy) const;
  bool isInside(const nav_msgs::OccupancyGrid& map, int mx, int my) const;
  int toIndex(const nav_msgs::OccupancyGrid& map, int mx, int my) const;
  bool isTraversable(const nav_msgs::OccupancyGrid& map, int mx, int my, int inflation_cells) const;
  std::vector<Pose2D> rebuildPath(const nav_msgs::OccupancyGrid& map,
                                  const std::vector<int>& parent,
                                  int start_index,
                                  int goal_index) const;
  std::vector<Pose2D> simplifyPath(const std::vector<Pose2D>& path) const;

  AStarPlannerConfig config_;
};

}  // namespace slam_v44

#endif  // SLAM_V44_ASTAR_PLANNER_H

