#include "slam_v44/astar_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace slam_v44
{

namespace
{
struct QueueNode
{
  int index = 0;
  double f = 0.0;

  bool operator<(const QueueNode& other) const
  {
    return f > other.f;
  }
};

double heuristic(int ax, int ay, int bx, int by)
{
  const double dx = static_cast<double>(ax - bx);
  const double dy = static_cast<double>(ay - by);
  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

AStarPlanner::AStarPlanner(const AStarPlannerConfig& config)
  : config_(config)
{
}

bool AStarPlanner::plan(const nav_msgs::OccupancyGrid& map,
                        const Pose2D& start,
                        const Pose2D& goal,
                        std::vector<Pose2D>& path) const
{
  path.clear();
  if (map.info.width == 0 || map.info.height == 0 || map.data.empty())
  {
    return false;
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!worldToMap(map, start.x, start.y, start_x, start_y) ||
      !worldToMap(map, goal.x, goal.y, goal_x, goal_y))
  {
    return false;
  }

  const int inflation_cells = static_cast<int>(
      std::ceil((config_.robot_radius + config_.inflation_radius) / map.info.resolution));

  if (!isTraversable(map, start_x, start_y, 0) ||
      !isTraversable(map, goal_x, goal_y, inflation_cells))
  {
    return false;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int cell_count = width * height;
  const int start_index = toIndex(map, start_x, start_y);
  const int goal_index = toIndex(map, goal_x, goal_y);

  std::vector<double> g_score(static_cast<size_t>(cell_count), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<size_t>(cell_count), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(cell_count), 0);
  std::priority_queue<QueueNode> open;

  g_score[start_index] = 0.0;
  open.push(QueueNode{start_index, heuristic(start_x, start_y, goal_x, goal_y)});

  const int neighbor_count = config_.diagonal_move ? 8 : 4;
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty())
  {
    const QueueNode current = open.top();
    open.pop();

    if (closed[current.index])
    {
      continue;
    }
    closed[current.index] = 1;

    if (current.index == goal_index)
    {
      path = simplifyPath(rebuildPath(map, parent, start_index, goal_index));
      return !path.empty();
    }

    const int cx = current.index % width;
    const int cy = current.index / width;
    for (int i = 0; i < neighbor_count; ++i)
    {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!isInside(map, nx, ny) || !isTraversable(map, nx, ny, inflation_cells))
      {
        continue;
      }

      const int neighbor_index = toIndex(map, nx, ny);
      if (closed[neighbor_index])
      {
        continue;
      }

      const double step_cost = (dx[i] != 0 && dy[i] != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g < g_score[neighbor_index])
      {
        parent[neighbor_index] = current.index;
        g_score[neighbor_index] = tentative_g;
        const double f = tentative_g + heuristic(nx, ny, goal_x, goal_y);
        open.push(QueueNode{neighbor_index, f});
      }
    }
  }

  return false;
}

bool AStarPlanner::worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, int& mx, int& my) const
{
  mx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
  my = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
  return isInside(map, mx, my);
}

void AStarPlanner::mapToWorld(const nav_msgs::OccupancyGrid& map, int mx, int my, double& wx, double& wy) const
{
  wx = map.info.origin.position.x + (static_cast<double>(mx) + 0.5) * map.info.resolution;
  wy = map.info.origin.position.y + (static_cast<double>(my) + 0.5) * map.info.resolution;
}

bool AStarPlanner::isInside(const nav_msgs::OccupancyGrid& map, int mx, int my) const
{
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(map.info.width) &&
         my < static_cast<int>(map.info.height);
}

int AStarPlanner::toIndex(const nav_msgs::OccupancyGrid& map, int mx, int my) const
{
  return my * static_cast<int>(map.info.width) + mx;
}

bool AStarPlanner::isTraversable(const nav_msgs::OccupancyGrid& map, int mx, int my, int inflation_cells) const
{
  for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
  {
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
    {
      if (dx * dx + dy * dy > inflation_cells * inflation_cells)
      {
        continue;
      }

      const int nx = mx + dx;
      const int ny = my + dy;
      if (!isInside(map, nx, ny))
      {
        return false;
      }

      const int value = map.data[toIndex(map, nx, ny)];
      if (value < 0)
      {
        if (!config_.allow_unknown)
        {
          return false;
        }
        continue;
      }
      if (value >= config_.occupied_value)
      {
        return false;
      }
    }
  }
  return true;
}

std::vector<Pose2D> AStarPlanner::rebuildPath(const nav_msgs::OccupancyGrid& map,
                                              const std::vector<int>& parent,
                                              int start_index,
                                              int goal_index) const
{
  std::vector<Pose2D> reversed;
  int current = goal_index;
  const int width = static_cast<int>(map.info.width);

  while (current >= 0)
  {
    const int mx = current % width;
    const int my = current / width;
    double wx = 0.0;
    double wy = 0.0;
    mapToWorld(map, mx, my, wx, wy);
    reversed.push_back(Pose2D{wx, wy, 0.0});

    if (current == start_index)
    {
      break;
    }
    current = parent[current];
  }

  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

std::vector<Pose2D> AStarPlanner::simplifyPath(const std::vector<Pose2D>& path) const
{
  if (path.size() <= 2)
  {
    return path;
  }

  std::vector<Pose2D> simplified;
  simplified.push_back(path.front());

  double previous_angle = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);
  for (size_t i = 2; i < path.size(); ++i)
  {
    const double angle = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
    if (std::fabs(normalizeAngle(angle - previous_angle)) > 0.15)
    {
      simplified.push_back(path[i - 1]);
      previous_angle = angle;
    }
  }

  simplified.push_back(path.back());
  return simplified;
}

}  // namespace slam_v44

