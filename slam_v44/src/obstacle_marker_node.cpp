#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>

class PersistentObstacleMarkerNode
{
public:
  PersistentObstacleMarkerNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
  {
    private_nh_.param("map_topic", map_topic_, std::string("/slam_v44/map"));
    private_nh_.param("persistent_obstacle_marker_topic", marker_topic_, std::string("/slam_v44/persistent_obstacle_markers"));
    private_nh_.param("persistent_obstacle_occupied_value", occupied_value_, 65);
    private_nh_.param("persistent_obstacle_free_value", free_value_, 20);
    private_nh_.param("persistent_obstacle_boundary_only", boundary_only_, true);
    private_nh_.param("persistent_obstacle_scale", marker_scale_, 0.08);
    private_nh_.param("persistent_obstacle_stride", stride_, 1);
    private_nh_.param("persistent_obstacle_max_points", max_points_, 0);
    private_nh_.param("persistent_obstacle_resolution", persistent_resolution_, 0.0);
    private_nh_.param("persistent_obstacle_min_confirmations", min_confirmations_, 1);
    private_nh_.param("persistent_obstacle_z", point_z_, 0.06);
    private_nh_.param("map_frame", map_frame_, std::string("map"));

    stride_ = std::max(1, stride_);
    occupied_value_ = std::max(0, std::min(100, occupied_value_));
    free_value_ = std::max(0, std::min(100, free_value_));
    marker_scale_ = std::max(0.005, marker_scale_);
    min_confirmations_ = std::max(1, min_confirmations_);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1, true);
    map_sub_ = nh_.subscribe(map_topic_, 1, &PersistentObstacleMarkerNode::mapCallback, this);

    ROS_INFO_STREAM("slam_v44 persistent obstacle layer listening on " << map_topic_
                    << ", publishing remembered red obstacle points on " << marker_topic_);
  }

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
  {
    if (map_msg->info.width == 0 || map_msg->info.height == 0 || map_msg->data.empty())
    {
      ROS_WARN_THROTTLE(2.0, "slam_v44 persistent obstacle layer: received empty map.");
      return;
    }

    const double map_resolution = static_cast<double>(map_msg->info.resolution);
    if (map_resolution <= 0.0)
    {
      ROS_WARN_THROTTLE(2.0, "slam_v44 persistent obstacle layer: map resolution is invalid.");
      return;
    }

    if (persistent_resolution_ <= 0.0)
    {
      persistent_resolution_ = map_resolution;
    }

    const int width = static_cast<int>(map_msg->info.width);
    const int height = static_cast<int>(map_msg->info.height);
    const double origin_x = map_msg->info.origin.position.x;
    const double origin_y = map_msg->info.origin.position.y;

    int added_or_confirmed = 0;
    const auto index = [width](int x, int y) { return y * width + x; };
    const auto is_occupied = [this](int8_t value) {
      return value >= occupied_value_;
    };
    const auto is_free = [this](int8_t value) {
      return value >= 0 && value <= free_value_;
    };

    for (int y = 0; y < height; y += stride_)
    {
      for (int x = 0; x < width; x += stride_)
      {
        const int idx = index(x, y);
        if (idx < 0 || idx >= static_cast<int>(map_msg->data.size()))
        {
          continue;
        }

        if (!is_occupied(map_msg->data[idx]))
        {
          continue;
        }

        if (boundary_only_ && !isBoundaryCell(*map_msg, x, y, is_free))
        {
          continue;
        }

        const double world_x = origin_x + (static_cast<double>(x) + 0.5) * map_resolution;
        const double world_y = origin_y + (static_cast<double>(y) + 0.5) * map_resolution;
        const int cell_x = static_cast<int>(std::floor(world_x / persistent_resolution_));
        const int cell_y = static_cast<int>(std::floor(world_y / persistent_resolution_));
        const std::uint64_t key = encodeKey(cell_x, cell_y);

        ++persistent_cells_[key];
        ++added_or_confirmed;
      }
    }

    publishPersistentMarker(map_msg->header.stamp);

    ROS_INFO_THROTTLE(3.0,
                      "slam_v44 persistent obstacle layer: current map added/confirmed %d cells, remembered %zu cells.",
                      added_or_confirmed,
                      persistent_cells_.size());
  }

  void publishPersistentMarker(const ros::Time& stamp)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_.empty() ? std::string("map") : map_frame_;
    marker.ns = "slam_v44_persistent_obstacle_boundary";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);

    const std::size_t reserve_count = max_points_ > 0
                                        ? static_cast<std::size_t>(max_points_)
                                        : persistent_cells_.size();
    marker.points.reserve(reserve_count);

    for (const auto& item : persistent_cells_)
    {
      if (item.second < min_confirmations_)
      {
        continue;
      }

      int cell_x = 0;
      int cell_y = 0;
      decodeKey(item.first, cell_x, cell_y);

      geometry_msgs::Point point;
      point.x = (static_cast<double>(cell_x) + 0.5) * persistent_resolution_;
      point.y = (static_cast<double>(cell_y) + 0.5) * persistent_resolution_;
      point.z = point_z_;
      marker.points.push_back(point);

      if (max_points_ > 0 && static_cast<int>(marker.points.size()) >= max_points_)
      {
        break;
      }
    }

    marker_pub_.publish(marker);
  }

  template <typename FreeCellPredicate>
  bool isBoundaryCell(const nav_msgs::OccupancyGrid& map_msg,
                      int x,
                      int y,
                      FreeCellPredicate is_free) const
  {
    const int width = static_cast<int>(map_msg.info.width);
    const int height = static_cast<int>(map_msg.info.height);
    const auto index = [width](int cx, int cy) { return cy * width + cx; };

    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
        {
          continue;
        }

        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
          return true;
        }

        const int nidx = index(nx, ny);
        if (nidx < 0 || nidx >= static_cast<int>(map_msg.data.size()))
        {
          continue;
        }

        if (is_free(map_msg.data[nidx]))
        {
          return true;
        }
      }
    }

    return false;
  }

  static std::uint64_t encodeKey(int x, int y)
  {
    const std::uint64_t ux = static_cast<std::uint32_t>(x);
    const std::uint64_t uy = static_cast<std::uint32_t>(y);
    return (ux << 32U) | uy;
  }

  static void decodeKey(std::uint64_t key, int& x, int& y)
  {
    x = static_cast<int>(static_cast<std::int32_t>((key >> 32U) & 0xffffffffU));
    y = static_cast<int>(static_cast<std::int32_t>(key & 0xffffffffU));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;

  std::string map_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  int occupied_value_ = 65;
  int free_value_ = 20;
  bool boundary_only_ = true;
  double marker_scale_ = 0.08;
  int stride_ = 1;
  int max_points_ = 0;
  double persistent_resolution_ = 0.0;
  int min_confirmations_ = 1;
  double point_z_ = 0.06;
  std::unordered_map<std::uint64_t, int> persistent_cells_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_v44_persistent_obstacle_marker_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  PersistentObstacleMarkerNode node(nh, private_nh);
  ros::spin();
  return 0;
}

