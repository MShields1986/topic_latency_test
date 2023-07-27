
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"

void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  ros::Time then = (*msg).header.stamp;
  ros::Duration dt = now - then;
  ROS_INFO_STREAM("Packet Created: " << then << " | Packet Received: " << now << " | dt (sec): " << dt.toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 10, topicCallback);

  ros::spin();

  return 0;
}
