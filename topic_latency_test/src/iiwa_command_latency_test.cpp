#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <iiwa_msgs/CartesianPose.h>


int observation_count = 0;
int observation_goal = 1000;

float vel_lim = 100.0; // mm/s
geometry_msgs::TwistStamped msg;

float curr_pos = 0.0;
float prev_pos = 0.0;
float curr_vel = 0.0;
bool topic_latch = false;
bool awaiting_response = false;

std::string path(ros::package::getPath("topic_latency_test"));

ros::Time command_send_time;

ros::Publisher pub;
std::ofstream LogFile;


void publishCommand()
{
  ROS_INFO_STREAM("Publishing " << vel_lim);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "iiwa_link_0";
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = vel_lim;
  msg.twist.linear.z = 0.0;
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


void topicCallback(const iiwa_msgs::CartesianPose::ConstPtr& msg)
{
  curr_pos = msg->poseStamped.pose.position.y;

  ROS_INFO_STREAM_THROTTLE(1.0, "Callback firing. pos_y=" << curr_pos
    << " latch=" << topic_latch << " awaiting=" << awaiting_response);

  if (topic_latch)
  {
    curr_vel = (curr_pos - prev_pos) / 0.0025;  // fixed dt for iiwa ~400 Hz publish rate
    ROS_INFO_STREAM_THROTTLE(0.5, "Velocity: " << curr_vel);

    if (awaiting_response)
    {
      if (curr_vel > 0.01 && vel_lim > 0.0)
      {
        recordObservation(msg->poseStamped.header.stamp, ros::Time::now());
        return;
      }
      else if (curr_vel < -0.01 && vel_lim < 0.0)
      {
        recordObservation(msg->poseStamped.header.stamp, ros::Time::now());
        return;
      }
    }
  }

  prev_pos = curr_pos;
  topic_latch = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "iiwa_command_latency_tester");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/command/CartesianVelocity", 100, true);
  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianPose", 10, topicCallback);

  LogFile.open(path + "/data/iiwa_command.txt");
  LogFile << "Packet Sent, Robot Stamp, Callback Time, Latency Robot (sec), Latency Callback (sec)" << std::endl;

  ROS_INFO_STREAM("iiwa latency monitor running...");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  publishCommand();

  ros::Rate rate(20);
  while (ros::ok())
  {
    if (awaiting_response)
    {
      msg.header.stamp = ros::Time::now();
      pub.publish(msg);
    }
    rate.sleep();
  }

  return 0;
}
