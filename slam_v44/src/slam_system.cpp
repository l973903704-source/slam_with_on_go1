#include "slam_v44/slam_system.h"

#include <tf/transform_datatypes.h>

#include <cmath>

namespace slam_v44
{

namespace
{
Pose2D poseFromOdometry(const nav_msgs::Odometry& msg)
{
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = tf::getYaw(msg.pose.pose.orientation);
  return pose;
}

Pose2D inversePose(const Pose2D& pose)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  Pose2D inverse;
  inverse.x = -c * pose.x - s * pose.y;
  inverse.y = s * pose.x - c * pose.y;
  inverse.theta = normalizeAngle(-pose.theta);
  return inverse;
}

Pose2D composePose(const Pose2D& first, const Pose2D& second)
{
  const double c = std::cos(first.theta);
  const double s = std::sin(first.theta);
  Pose2D composed;
  composed.x = first.x + c * second.x - s * second.y;
  composed.y = first.y + s * second.x + c * second.y;
  composed.theta = normalizeAngle(first.theta + second.theta);
  return composed;
}

LaserProcessorConfig loadLaserConfig(ros::NodeHandle& nh)
{
  LaserProcessorConfig config;
  nh.param("min_range", config.min_range, config.min_range);
  nh.param("max_range", config.max_range, config.max_range);
  nh.param("use_scan_range_limits", config.use_scan_range_limits, config.use_scan_range_limits);
  nh.param("scan_skip", config.scan_skip, config.scan_skip);
  return config;
}

OccupancyGridMapConfig loadMapConfig(ros::NodeHandle& nh)
{
  OccupancyGridMapConfig config;
  nh.param("map_resolution", config.resolution, config.resolution);
  nh.param("map_width_m", config.width_m, config.width_m);
  nh.param("map_height_m", config.height_m, config.height_m);
  nh.param("map_origin_x", config.origin_x, config.origin_x);
  nh.param("map_origin_y", config.origin_y, config.origin_y);
  nh.param("log_odds_hit", config.log_odds_hit, config.log_odds_hit);
  nh.param("log_odds_miss", config.log_odds_miss, config.log_odds_miss);
  nh.param("log_odds_min", config.log_odds_min, config.log_odds_min);
  nh.param("log_odds_max", config.log_odds_max, config.log_odds_max);
  nh.param("occupied_threshold", config.occupied_threshold, config.occupied_threshold);
  nh.param("free_threshold", config.free_threshold, config.free_threshold);
  nh.param("map_frame", config.map_frame, config.map_frame);
  return config;
}

ScanMatcherConfig loadMatcherConfig(ros::NodeHandle& nh)
{
  ScanMatcherConfig config;
  nh.param("linear_search_window", config.linear_search_window, config.linear_search_window);
  nh.param("angular_search_window", config.angular_search_window, config.angular_search_window);
  nh.param("linear_step", config.linear_step, config.linear_step);
  nh.param("angular_step", config.angular_step, config.angular_step);
  nh.param("correlation_radius_cells", config.correlation_radius_cells, config.correlation_radius_cells);
  nh.param("max_scoring_points", config.max_scoring_points, config.max_scoring_points);
  return config;
}
}  // namespace

SlamSystem::SlamSystem(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , laser_processor_(loadLaserConfig(private_nh_))
  , map_(loadMapConfig(private_nh_))
  , scan_matcher_(loadMatcherConfig(private_nh_))
{
  private_nh_.param("scan_topic", scan_topic_, std::string("/slamware_ros_sdk_server_node/scan"));
  std::string default_odom_topic;
  private_nh_.param("external_odom_topic", default_odom_topic, std::string("/slamware_ros_sdk_server_node/odom"));
  private_nh_.param("odom_topic", odom_topic_, default_odom_topic);
  private_nh_.param("map_topic", map_topic_, std::string("/slam_v44/map"));
  private_nh_.param("pose_topic", pose_topic_, std::string("/slam_v44/pose"));
  private_nh_.param("slam_path_topic", path_topic_, std::string("/slam_v44/slam_path"));
  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
  private_nh_.param("base_frame", base_frame_, std::string("base_link"));
  private_nh_.param("publish_tf", publish_tf_, publish_tf_);
  private_nh_.param("publish_map_to_odom_tf", publish_map_to_odom_tf_, publish_map_to_odom_tf_);
  private_nh_.param("use_odom_prediction", use_odom_prediction_, use_odom_prediction_);
  private_nh_.param("require_odom_for_mapping", require_odom_for_mapping_, require_odom_for_mapping_);
  private_nh_.param("min_match_score", min_match_score_, min_match_score_);
  private_nh_.param("keyframe_min_translation", keyframe_min_translation_, keyframe_min_translation_);
  private_nh_.param("keyframe_min_rotation", keyframe_min_rotation_, keyframe_min_rotation_);
  private_nh_.param("publish_map_every_n_scans", publish_map_every_n_scans_, publish_map_every_n_scans_);
  private_nh_.param("initial_x", pose_.x, 0.0);
  private_nh_.param("initial_y", pose_.y, 0.0);
  private_nh_.param("initial_theta", pose_.theta, 0.0);
  pose_.theta = normalizeAngle(pose_.theta);
  last_keyframe_pose_ = pose_;
  odom_map_correction_ = pose_;

  scan_matcher_.setMap(&map_);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  if (use_odom_prediction_ && !odom_topic_.empty())
  {
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &SlamSystem::odomCallback, this);
  }
  scan_sub_ = nh_.subscribe(scan_topic_, 5, &SlamSystem::scanCallback, this);

  path_msg_.header.frame_id = map_frame_;

  ROS_INFO_STREAM("slam_v44_node listening on " << scan_topic_
                  << ", odom=" << (use_odom_prediction_ ? odom_topic_ : std::string("<disabled>"))
                  << ", publishing map on " << map_topic_);
}

void SlamSystem::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  latest_odom_pose_ = poseFromOdometry(*msg);
  have_latest_odom_ = true;

  if (!have_odom_reference_)
  {
    if (!msg->header.frame_id.empty())
    {
      odom_frame_ = msg->header.frame_id;
    }
    odom_reference_pose_ = latest_odom_pose_;
    odom_map_correction_ = pose_;
    have_odom_reference_ = true;
    ROS_INFO_STREAM("slam_v44: locked odom startup pose as map origin reference. "
                    << "odom x=" << odom_reference_pose_.x
                    << ", y=" << odom_reference_pose_.y
                    << ", yaw=" << odom_reference_pose_.theta
                    << ", frame=" << odom_frame_);
  }
}

void SlamSystem::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  ProcessedScan scan = laser_processor_.process(*msg);
  if (scan.points.empty())
  {
    ROS_WARN_THROTTLE(2.0, "slam_v44: scan has no usable points after filtering.");
    return;
  }

  ++scan_count_;

  Pose2D odom_relative_pose;
  Pose2D predicted_pose = pose_;
  const bool have_odom_prediction = getOdomPredictedPose(predicted_pose, odom_relative_pose);
  if (use_odom_prediction_ && require_odom_for_mapping_ && !have_odom_prediction)
  {
    ROS_WARN_THROTTLE(2.0, "slam_v44: waiting for odom before inserting scans into fixed map frame.");
    return;
  }

  if (!initialized_)
  {
    pose_ = predicted_pose;
    map_.updateByScan(pose_, scan);
    last_keyframe_pose_ = pose_;
    initialized_ = true;
    publishState(scan.stamp);
    ROS_INFO_STREAM("slam_v44 initialized with " << scan.points.size() << " scan points.");
    return;
  }

  const ScanMatchResult match = scan_matcher_.match(scan, predicted_pose);
  if (match.valid && match.score >= min_match_score_)
  {
    pose_ = match.pose;
    if (have_odom_prediction)
    {
      updateOdomCorrection(odom_relative_pose, pose_);
    }
  }
  else
  {
    pose_ = predicted_pose;
    ROS_WARN_THROTTLE(2.0, "slam_v44: low scan match score, using odom prediction.");
  }

  if (shouldUpdateKeyframe(pose_))
  {
    map_.updateByScan(pose_, scan);
    last_keyframe_pose_ = pose_;
  }

  publishState(scan.stamp);
}

bool SlamSystem::getOdomPredictedPose(Pose2D& predicted_pose, Pose2D& relative_odom_pose) const
{
  if (!use_odom_prediction_ || !have_odom_reference_ || !have_latest_odom_)
  {
    return false;
  }

  relative_odom_pose = composePose(inversePose(odom_reference_pose_), latest_odom_pose_);
  predicted_pose = composePose(odom_map_correction_, relative_odom_pose);
  return true;
}

void SlamSystem::updateOdomCorrection(const Pose2D& relative_odom_pose, const Pose2D& corrected_pose)
{
  odom_map_correction_ = composePose(corrected_pose, inversePose(relative_odom_pose));
}

void SlamSystem::publishState(const ros::Time& stamp)
{
  const geometry_msgs::PoseStamped pose_msg = poseToMessage(stamp);
  pose_pub_.publish(pose_msg);

  path_msg_.header.stamp = stamp;
  path_msg_.poses.push_back(pose_msg);
  path_pub_.publish(path_msg_);

  if (publish_map_every_n_scans_ <= 1 || scan_count_ % publish_map_every_n_scans_ == 0)
  {
    map_pub_.publish(map_.toMessage(stamp));
  }

  if (publish_tf_)
  {
    publishTf(stamp);
  }
}

void SlamSystem::publishTf(const ros::Time& stamp)
{
  Pose2D tf_pose = pose_;
  std::string child_frame = base_frame_;
  if (use_odom_prediction_ && publish_map_to_odom_tf_ && have_odom_reference_)
  {
    tf_pose = composePose(odom_map_correction_, inversePose(odom_reference_pose_));
    child_frame = odom_frame_;
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(tf_pose.x, tf_pose.y, 0.0));
  transform.setRotation(tf::createQuaternionFromYaw(tf_pose.theta));
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, map_frame_, child_frame));
}

geometry_msgs::PoseStamped SlamSystem::poseToMessage(const ros::Time& stamp) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  msg.pose.position.x = pose_.x;
  msg.pose.position.y = pose_.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);
  return msg;
}

bool SlamSystem::shouldUpdateKeyframe(const Pose2D& pose) const
{
  const double translation = std::sqrt(squaredDistance(pose, last_keyframe_pose_));
  const double rotation = std::fabs(normalizeAngle(pose.theta - last_keyframe_pose_.theta));
  return translation >= keyframe_min_translation_ || rotation >= keyframe_min_rotation_;
}

}  // namespace slam_v44

