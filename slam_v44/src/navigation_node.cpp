#include <ros/ros.h>

#include "slam_v44/navigation_system.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_v44_navigation_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  slam_v44::NavigationSystem system(nh, private_nh);
  ros::spin();
  return 0;
}

