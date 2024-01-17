#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iiwa_msgs/CartesianPose.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh;

  // ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 100);
  // ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan_front", 100);
  // ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan_rear", 100);
  // ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
  ros::Publisher pub = nh.advertise<iiwa_msgs::CartesianPose>("/iiwa/state/CartesianPose", 100);

  ros::Rate rate(10);

  while (ros::ok())
  {
    // sensor_msgs::LaserScan msg;
    // nav_msgs::Odometry msg;
    iiwa_msgs::CartesianPose msg;

    // msg.header.stamp = ros::Time::now();
    msg.poseStamped.header.stamp = ros::Time::now();

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
