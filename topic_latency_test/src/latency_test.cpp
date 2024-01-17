#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iiwa_msgs/CartesianPose.h>

using namespace std;

std::string path(ros::package::getPath("topic_latency_test"));

std::vector<ros::Time> thens{};
std::vector<ros::Time> nows{};
std::vector<ros::Duration> dts{};

ros::Time now;
ros::Time then;
ros::Duration dt;

// ofstream LogFile(path + "/data/scan_internal.txt");
// ofstream LogFile(path + "/data/scan_front.txt");
// ofstream LogFile(path + "/data/scan_rear.txt");
// ofstream LogFile(path + "/data/odom.txt");
ofstream LogFile(path + "/data/iiwa_cartesian_pose.txt");


// void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
// void topicCallback(const nav_msgs::Odometry::ConstPtr& msg)
void topicCallback(const iiwa_msgs::CartesianPose::ConstPtr& msg)
{
  now = ros::Time::now();
  // then = (*msg).header.stamp;
  // then = msg->header.stamp;
  then = (*msg).poseStamped.header.stamp;
  dt = now - then;

  ROS_INFO_STREAM("Latency (sec): " << dt.toSec());

  thens.push_back(then);
  nows.push_back(now);
  dts.push_back(dt);
}


int main(int argc, char **argv)
{
  thens.reserve(10000);
  nows.reserve(10000);
  dts.reserve(10000);

  ros::init(argc, argv, "latency_tester");
  ros::NodeHandle nh;
  // ros::Rate rate(10);

  // ros::Subscriber sub = nh.subscribe("/scan", 10, topicCallback);
  // ros::Subscriber sub = nh.subscribe("/scan_front", 10, topicCallback);
  // ros::Subscriber sub = nh.subscribe("/scan_rear", 10, topicCallback);
  // ros::Subscriber sub = nh.subscribe("/odom", 10, topicCallback);
  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianPose", 10, topicCallback);

  while (ros::ok())
  {
    ROS_INFO_STREAM("Topic latency monitor running...");
    // ROS_INFO_THROTTLE(10, "Main Loop Latency (sec): " << std::to_string(dts.back().toSec()));
    // ROS_INFO_STREAM("Main Loop Latency (sec): " << dts.back());
    // ros::spinOnce();
    // rate.sleep();

    // This is an order of magnitude faster to just do this rather than using ros::Rate as above
    ros::spin();
  }

  LogFile << "Packet Created, Packet Received, Latency (sec)" << endl;

  for (int i = 0; i < thens.size(); i++)
  {
    LogFile << thens[i] << ", " << nows[i] << ", " << dts[i] << endl;
  }

  LogFile.close();

  return 0;
}
