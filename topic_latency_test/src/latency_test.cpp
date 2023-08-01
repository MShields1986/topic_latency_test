#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

using namespace std;

std::string path(ros::package::getPath("topic_latency_test"));
ofstream LogFile(path + "/data/scan_internal.txt");
// ofstream LogFile(path + "/data/scan_front.txt");
// ofstream LogFile(path + "/data/scan_rear.txt");
// ofstream LogFile(path + "/data/scan_odom.txt");

void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
// void topicCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  ros::Time then = (*msg).header.stamp;
  ros::Duration dt = now - then;

  // ROS_INFO_STREAM("Packet Created: " << then << " | Packet Received: " << now << " | dt (sec): " << dt.toSec());
  ROS_INFO_STREAM("Latency (sec): " << dt.toSec() << endl);
  LogFile << then << ", " << now << ", " << dt << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "latency_tester");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 10, topicCallback);
  // ros::Subscriber sub = n.subscribe("/scan_front", 10, topicCallback);
  // ros::Subscriber sub = n.subscribe("/scan_rear", 10, topicCallback);
  // ros::Subscriber sub = n.subscribe("/odom", 10, topicCallback);

  LogFile << "Packet Created, Packet Received, Latency (sec)" << endl;

  ros::spin();

  LogFile.close();

  return 0;
}
