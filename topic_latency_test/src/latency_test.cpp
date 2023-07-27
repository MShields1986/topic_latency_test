
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
// #include "nav_msgs/Odometry.h"

void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
// void topicCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  ros::Time then = (*msg).header.stamp;
  ros::Duration dt = now - then;

  // ROS_INFO_STREAM("Packet Created: " << then << " | Packet Received: " << now << " | dt (sec): " << dt.toSec());
  ROS_INFO_STREAM("Latency (sec): " << dt.toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan_front", 10, topicCallback);
  // ros::Subscriber sub = n.subscribe("/scan_rear", 10, topicCallback);
  // ros::Subscriber sub = n.subscribe("/odom", 10, topicCallback);

  ros::spin();

  return 0;
}
