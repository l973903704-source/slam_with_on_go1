#include <ros/ros.h>

#include "slam_v44/slam_system.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_v44_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  slam_v44::SlamSystem system(nh, private_nh);
  ros::spin();
  return 0;
}

