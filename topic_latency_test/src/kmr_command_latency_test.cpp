#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


int observation_count = 0;
int observation_goal = 1000;

float vel_lim = 0.1; // m/s
geometry_msgs::Twist msg;

bool topic_latch = false;
bool awaiting_response = false;

std::string path(ros::package::getPath("topic_latency_test"));

ros::Time command_send_time;

ros::Publisher pub;
std::ofstream LogFile;


void publishCommand()
{
  ROS_INFO_STREAM("Publishing " << vel_lim);
  msg.linear.x = vel_lim;
  command_send_time = ros::Time::now();
  pub.publish(msg);
  awaiting_response = true;
}


void recordObservation(const ros::Time& robot_stamp, const ros::Time& callback_stamp)
{
  awaiting_response = false;
  topic_latch = false;

  const ros::Duration dt_robot    = robot_stamp    - command_send_time;
  const ros::Duration dt_callback = callback_stamp - command_send_time;

  ROS_INFO_STREAM("Latency robot (sec): " << dt_robot.toSec()
    << "  callback (sec): " << dt_callback.toSec());

  LogFile << command_send_time << ", "
          << robot_stamp       << ", "
          << callback_stamp    << ", "
          << dt_robot          << ", "
          << dt_callback       << std::endl;

  observation_count++;
  ROS_INFO_STREAM("Observation: " << observation_count << "/" << observation_goal);

  if (observation_count >= observation_goal)
  {
    LogFile.close();
    ros::shutdown();
    return;
  }

  ROS_INFO_STREAM("Switching velocity direction...");
  vel_lim = vel_lim * -1;

  ROS_INFO_STREAM("Blocking for 2 seconds");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  publishCommand();
}


void topicCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  const float curr_vel = msg->twist.twist.linear.x;

  ROS_INFO_STREAM_THROTTLE(1.0, "Callback firing. vel_x=" << curr_vel
    << " latch=" << topic_latch << " awaiting=" << awaiting_response);

  if (!topic_latch)
  {
    topic_latch = true;
    return;
  }

  if (awaiting_response)
  {
    if (curr_vel > 0.01 && vel_lim > 0.0)
    {
      recordObservation(msg->header.stamp, ros::Time::now());
      return;
    }
    else if (curr_vel < -0.01 && vel_lim < 0.0)
    {
      recordObservation(msg->header.stamp, ros::Time::now());
      return;
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kmr_command_latency_tester");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, true);
  ros::Subscriber sub = nh.subscribe("/odom", 10, topicCallback);

  LogFile.open(path + "/data/kmr_command.txt");
  LogFile << "Packet Sent, Robot Stamp, Callback Time, Latency Robot (sec), Latency Callback (sec)" << std::endl;

  ROS_INFO_STREAM("KMR latency monitor running...");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  publishCommand();

  ros::Rate rate(20);
  while (ros::ok())
  {
    if (awaiting_response)
    {
      msg.linear.x = vel_lim;
      pub.publish(msg);
    }
    rate.sleep();
  }

  return 0;
}
