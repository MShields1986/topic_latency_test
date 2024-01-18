#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/Twist.h>
#include <iiwa_msgs/CartesianPose.h>

int observation_count = 0;
int observation_goal = 200;

float vel_lim = 100.0; // mm/s
geometry_msgs::TwistStamped msg;
//geometry_msgs::Twist msg;

float curr_pos = 0.0;
float prev_pos = 0.0;
float curr_vel = 0.0;
bool topic_latch = false;


std::string path(ros::package::getPath("topic_latency_test"));

std::vector<ros::Time> sends{};
std::vector<ros::Time> observes{};
std::vector<ros::Duration> dts{};

ros::Time command_send_time;
ros::Time action_observed_time;
ros::Duration dt;

std::ofstream LogFile(path + "/data/iiwa_command.txt");

void recordObservation()
{
  action_observed_time = ros::Time::now();
  dt = action_observed_time - command_send_time;

  ROS_INFO_STREAM("Latency (sec): " << dt.toSec());

  sends.push_back(command_send_time);
  observes.push_back(action_observed_time);
  dts.push_back(dt);

  observation_count++;
  ROS_INFO_STREAM("Observation: " << observation_count << "/" << observation_goal);

  ROS_INFO_STREAM("Switching velocity direction...");
  vel_lim = vel_lim * -1;

  ROS_INFO_STREAM("Blocking for 2 seconeds");
  std::this_thread::sleep_for(std::chrono::seconds(2));
}


void topicCallback(const iiwa_msgs::CartesianPose::ConstPtr& msg)
{
  curr_pos = msg->poseStamped.pose.position.y;
  
  if (topic_latch){
    curr_vel = (curr_pos - prev_pos) / 0.0025; // ~400Hz m/s
    ROS_INFO_STREAM("Velocity: " << curr_vel);
  }

  if (curr_vel > 0.01 && vel_lim > 0.0)
  {
    recordObservation();

  } else if (curr_vel < -0.01 && vel_lim < 0.0)
  {
    recordObservation();
  }

  prev_pos = curr_pos;

  if (!topic_latch) {
    topic_latch = true;
  }
}


int main(int argc, char **argv)
{
  sends.reserve(observation_goal);
  observes.reserve(observation_goal);
  dts.reserve(observation_goal);

  ros::init(argc, argv, "iiwa_command_latency_tester");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/command/CartesianVelocity", 100, true);
  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianPose", 10, topicCallback);

  ROS_INFO_STREAM("iiwa latency monitor running...");

  while (ros::ok() && observation_count < observation_goal)
  {
    ROS_INFO_STREAM("Publishing " << vel_lim);    

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "iiwa_link_0";
    msg.twist.linear.x = 0.0; //vel_lim;
    msg.twist.linear.y = vel_lim; //0.0;
    msg.twist.linear.z = 0.0;
    // msg.linear.x = vel_lim;

    command_send_time = ros::Time::now();

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  LogFile << "Packet Sent, Action Observed, Latency (sec)" << std::endl;

  for (int i = 0; i < sends.size(); i++)
  {
    LogFile << sends[i] << ", " << observes[i] << ", " << dts[i] << std::endl;
  }

  LogFile.close();

  return 0;
}
