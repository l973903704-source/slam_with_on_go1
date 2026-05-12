#ifndef SLAM_V44_SLAM_SYSTEM_H
#define SLAM_V44_SLAM_SYSTEM_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "slam_v44/laser_processor.h"
#include "slam_v44/occupancy_grid_map.h"
#include "slam_v44/scan_matcher.h"
#include "slam_v44/types.h"

namespace slam_v44
{

class SlamSystem
{
public:
  SlamSystem(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  bool getOdomPredictedPose(Pose2D& predicted_pose, Pose2D& relative_odom_pose) const;
  void updateOdomCorrection(const Pose2D& relative_odom_pose, const Pose2D& corrected_pose);
  void publishState(const ros::Time& stamp);
  void publishTf(const ros::Time& stamp);
  geometry_msgs::PoseStamped poseToMessage(const ros::Time& stamp) const;
  bool shouldUpdateKeyframe(const Pose2D& pose) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher map_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher path_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  LaserProcessor laser_processor_;
  OccupancyGridMap map_;
  ScanMatcher scan_matcher_;

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string pose_topic_;
  std::string path_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  bool publish_map_to_odom_tf_ = true;
  bool use_odom_prediction_ = true;
  bool require_odom_for_mapping_ = true;
  double min_match_score_ = 0.02;
  double keyframe_min_translation_ = 0.08;
  double keyframe_min_rotation_ = 0.06;
  int publish_map_every_n_scans_ = 1;

  bool initialized_ = false;
  Pose2D pose_;
  Pose2D last_keyframe_pose_;
  Pose2D odom_reference_pose_;
  Pose2D latest_odom_pose_;
  Pose2D odom_map_correction_;
  bool have_odom_reference_ = false;
  bool have_latest_odom_ = false;
  nav_msgs::Path path_msg_;
  int scan_count_ = 0;
};

}  // namespace slam_v44

#endif  // SLAM_V44_SLAM_SYSTEM_H

