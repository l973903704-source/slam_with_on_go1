#ifndef SLAM_V44_NAVIGATION_SYSTEM_H
#define SLAM_V44_NAVIGATION_SYSTEM_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <string>
#include "slam_v44/astar_planner.h"
#include "slam_v44/local_controller.h"
#include "slam_v44/types.h"

namespace slam_v44
{

class NavigationSystem
{
public:
  NavigationSystem(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  bool lookupRobotPose(Pose2D& pose);
  bool makePlan(const Pose2D& robot_pose);
  void publishPath(const std::vector<Pose2D>& path, const ros::Time& stamp);
  void publishStop();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber external_pose_sub_;
  ros::Subscriber extra_pose_sub_;
  ros::Subscriber external_odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher path_pub_;
  ros::Timer control_timer_;
  tf::TransformListener tf_listener_;

  AStarPlanner planner_;
  LocalController controller_;

  std::string scan_topic_;
  std::string map_topic_;
  std::string pose_topic_;
  std::string external_pose_topic_;
  std::string extra_pose_topic_;
  std::string external_odom_topic_;
  std::string goal_topic_;
  std::string cmd_vel_topic_;
  std::string path_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double front_obstacle_angle_ = 0.70;
  double replan_interval_ = 1.0;

  nav_msgs::OccupancyGrid latest_map_;
  bool have_map_ = false;
  Pose2D fallback_pose_;
  bool have_fallback_pose_ = false;
  Pose2D goal_;
  bool have_goal_ = false;
  std::vector<Pose2D> current_path_;
  ros::Time last_plan_time_;
  double front_obstacle_distance_ = 1000.0;
};

}  // namespace slam_v44

#endif  // SLAM_V44_NAVIGATION_SYSTEM_H

