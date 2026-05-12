#include "slam_v44/navigation_system.h"

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>

namespace slam_v44
{

namespace
{
AStarPlannerConfig loadPlannerConfig(ros::NodeHandle& nh)
{
  AStarPlannerConfig config;
  nh.param("robot_radius", config.robot_radius, config.robot_radius);
  nh.param("inflation_radius", config.inflation_radius, config.inflation_radius);
  nh.param("planner_allow_unknown", config.allow_unknown, config.allow_unknown);
  nh.param("planner_diagonal_move", config.diagonal_move, config.diagonal_move);
  nh.param("planner_occupied_value", config.occupied_value, config.occupied_value);
  return config;
}

LocalControllerConfig loadControllerConfig(ros::NodeHandle& nh)
{
  LocalControllerConfig config;
  nh.param("goal_tolerance", config.goal_tolerance, config.goal_tolerance);
  nh.param("yaw_tolerance", config.yaw_tolerance, config.yaw_tolerance);
  nh.param("lookahead_distance", config.lookahead_distance, config.lookahead_distance);
  nh.param("max_linear_speed", config.max_linear_speed, config.max_linear_speed);
  nh.param("max_angular_speed", config.max_angular_speed, config.max_angular_speed);
  nh.param("linear_gain", config.linear_gain, config.linear_gain);
  nh.param("angular_gain", config.angular_gain, config.angular_gain);
  nh.param("min_front_obstacle_distance", config.min_front_obstacle_distance, config.min_front_obstacle_distance);
  nh.param("slow_front_obstacle_distance", config.slow_front_obstacle_distance, config.slow_front_obstacle_distance);
  return config;
}
}  // namespace

NavigationSystem::NavigationSystem(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , planner_(loadPlannerConfig(private_nh_))
  , controller_(loadControllerConfig(private_nh_))
{
  private_nh_.param("scan_topic", scan_topic_, std::string("/slamware_ros_sdk_server_node/scan"));
  std::string default_map_topic;
  private_nh_.param("map_topic", default_map_topic, std::string("/slam_v44/map"));
  private_nh_.param("navigation_map_topic", map_topic_, default_map_topic);
  private_nh_.param("pose_topic", pose_topic_, std::string("/slam_v44/pose"));
  private_nh_.param("external_pose_topic", external_pose_topic_, std::string("/slamware_ros_sdk_server_node/robot_pose"));
  private_nh_.param("external_odom_topic", external_odom_topic_, std::string("/slamware_ros_sdk_server_node/odom"));
  private_nh_.param("extra_pose_topic", extra_pose_topic_, std::string(""));
  private_nh_.param("goal_topic", goal_topic_, std::string("/move_base_simple/goal"));
  private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  private_nh_.param("global_path_topic", path_topic_, std::string("/slam_v44/global_path"));
  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("base_frame", base_frame_, std::string("base_link"));
  private_nh_.param("front_obstacle_angle", front_obstacle_angle_, front_obstacle_angle_);
  private_nh_.param("replan_interval", replan_interval_, replan_interval_);

  double control_rate = 10.0;
  private_nh_.param("control_rate", control_rate, control_rate);
  control_rate = std::max(1.0, control_rate);

  map_sub_ = nh_.subscribe(map_topic_, 1, &NavigationSystem::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &NavigationSystem::goalCallback, this);
  pose_sub_ = nh_.subscribe(pose_topic_, 5, &NavigationSystem::poseCallback, this);
  if (!external_pose_topic_.empty())
  {
    external_pose_sub_ = nh_.subscribe(external_pose_topic_, 5, &NavigationSystem::poseCallback, this);
  }
  if (!extra_pose_topic_.empty())
  {
    extra_pose_sub_ = nh_.subscribe(extra_pose_topic_, 5, &NavigationSystem::poseCallback, this);
  }
  if (!external_odom_topic_.empty())
  {
    external_odom_sub_ = nh_.subscribe(external_odom_topic_, 5, &NavigationSystem::odomCallback, this);
  }
  scan_sub_ = nh_.subscribe(scan_topic_, 5, &NavigationSystem::scanCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate),
                                  &NavigationSystem::controlTimerCallback,
                                  this);

  ROS_INFO_STREAM("slam_v44_navigation_node map=" << map_topic_
                  << " scan=" << scan_topic_
                  << " goal=" << goal_topic_
                  << " cmd=" << cmd_vel_topic_);
}

void NavigationSystem::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  latest_map_ = *msg;
  have_map_ = true;
}

void NavigationSystem::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped goal_msg = *msg;
  if (!goal_msg.header.frame_id.empty() && goal_msg.header.frame_id != map_frame_)
  {
    try
    {
      tf_listener_.transformPose(map_frame_, *msg, goal_msg);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_WARN_STREAM("slam_v44 navigation: cannot transform goal to " << map_frame_ << ": " << ex.what());
      return;
    }
  }

  goal_.x = goal_msg.pose.position.x;
  goal_.y = goal_msg.pose.position.y;
  goal_.theta = tf::getYaw(goal_msg.pose.orientation);
  have_goal_ = true;
  current_path_.clear();
  last_plan_time_ = ros::Time(0);

  ROS_INFO_STREAM("slam_v44 navigation: received goal x=" << goal_.x << ", y=" << goal_.y);
}

void NavigationSystem::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped pose_msg = *msg;
  if (!pose_msg.header.frame_id.empty() && pose_msg.header.frame_id != map_frame_)
  {
    try
    {
      tf_listener_.transformPose(map_frame_, *msg, pose_msg);
    }
    catch (const tf::TransformException&)
    {
      return;
    }
  }

  fallback_pose_.x = pose_msg.pose.position.x;
  fallback_pose_.y = pose_msg.pose.position.y;
  fallback_pose_.theta = tf::getYaw(pose_msg.pose.orientation);
  have_fallback_pose_ = true;
}

void NavigationSystem::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = msg->header;
  pose_msg.pose = msg->pose.pose;
  geometry_msgs::PoseStampedConstPtr pose_ptr(new geometry_msgs::PoseStamped(pose_msg));
  poseCallback(pose_ptr);
}

void NavigationSystem::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  double best = 1000.0;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    const double angle = static_cast<double>(msg->angle_min) +
                         static_cast<double>(i) * static_cast<double>(msg->angle_increment);
    if (std::fabs(angle) > front_obstacle_angle_)
    {
      continue;
    }

    const double range = msg->ranges[i];
    if (!std::isfinite(range) ||
        range < static_cast<double>(msg->range_min) ||
        range > static_cast<double>(msg->range_max))
    {
      continue;
    }
    best = std::min(best, range);
  }
  front_obstacle_distance_ = best;
}

void NavigationSystem::controlTimerCallback(const ros::TimerEvent&)
{
  if (!have_goal_)
  {
    return;
  }

  Pose2D robot_pose;
  if (!lookupRobotPose(robot_pose))
  {
    ROS_WARN_THROTTLE(2.0, "slam_v44 navigation: no robot pose yet.");
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  const bool need_plan = current_path_.empty() ||
                         last_plan_time_.isZero() ||
                         (now - last_plan_time_).toSec() >= replan_interval_;
  if (need_plan && !makePlan(robot_pose))
  {
    ROS_WARN_THROTTLE(2.0, "slam_v44 navigation: planning failed.");
    publishStop();
    return;
  }

  const ControlResult control = controller_.computeCommand(robot_pose,
                                                           current_path_,
                                                           front_obstacle_distance_);
  if (control.reached)
  {
    publishStop();
    have_goal_ = false;
    current_path_.clear();
    ROS_INFO("slam_v44 navigation: goal reached.");
    return;
  }

  if (control.blocked)
  {
    ROS_WARN_THROTTLE(1.0, "slam_v44 navigation: front obstacle too close, stopping.");
    publishStop();
    return;
  }

  cmd_pub_.publish(control.cmd);
}

bool NavigationSystem::lookupRobotPose(Pose2D& pose)
{
  try
  {
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
    pose.x = transform.getOrigin().x();
    pose.y = transform.getOrigin().y();
    pose.theta = tf::getYaw(transform.getRotation());
    return true;
  }
  catch (const tf::TransformException&)
  {
  }

  if (have_fallback_pose_)
  {
    pose = fallback_pose_;
    return true;
  }
  return false;
}

bool NavigationSystem::makePlan(const Pose2D& robot_pose)
{
  if (!have_map_)
  {
    return false;
  }

  std::vector<Pose2D> path;
  if (!planner_.plan(latest_map_, robot_pose, goal_, path))
  {
    return false;
  }

  current_path_ = path;
  last_plan_time_ = ros::Time::now();
  publishPath(current_path_, last_plan_time_);
  return true;
}

void NavigationSystem::publishPath(const std::vector<Pose2D>& path, const ros::Time& stamp)
{
  nav_msgs::Path msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  msg.poses.reserve(path.size());

  for (size_t i = 0; i < path.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = path[i].x;
    pose.pose.position.y = path[i].y;
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < path.size())
    {
      yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
    }
    else
    {
      yaw = goal_.theta;
    }
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.poses.push_back(pose);
  }

  path_pub_.publish(msg);
}

void NavigationSystem::publishStop()
{
  geometry_msgs::Twist stop;
  cmd_pub_.publish(stop);
}

}  // namespace slam_v44

