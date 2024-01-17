#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <iiwa_msgs/CartesianVelocity.h>

using namespace std;

float vel_lim = 0.1;
geometry_msgs::TwistStamped msg;

std::string path(ros::package::getPath("topic_latency_test"));

std::vector<ros::Time> sends{};
std::vector<ros::Time> observes{};
std::vector<ros::Duration> dts{};

ros::Time command_send_time;
ros::Time action_observed_time;
ros::Duration dt;

ofstream LogFile(path + "/data/iiwa_command.txt");

void recordObservation()
{
  action_observed_time = ros::Time::now();
  dt = action_observed_time - command_send_time;

  ROS_INFO_STREAM("Latency (sec): " << dt.toSec());

  sends.push_back(command_send_time);
  observes.push_back(action_observed_time);
  dts.push_back(dt);

  ROS_INFO_STREAM("Switching velocity direction...");
  vel_lim = vel_lim * -1;
}


void topicCallback(const iiwa_msgs::CartesianVelocity::ConstPtr& msg)
{
  if (msg->velocity.x > 0.0 && vel_lim > 0.0)
  {
    recordObservation();

  } else if (msg->velocity.x < 0.0 && vel_lim < 0.0)
  {
    recordObservation();
  }
}


int main(int argc, char **argv)
{
  sends.reserve(10000);
  observes.reserve(10000);
  dts.reserve(10000);

  ros::init(argc, argv, "iiwa_command_latency_tester");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/iiwa/command/CartesianVelocity", 100, true);
  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianVelocity", 10, topicCallback);

  ROS_INFO_STREAM("iiwa latency monitor running...");

  while (ros::ok())
  {
    ROS_INFO_STREAM("Publishing " << vel_lim);    

    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = vel_lim;

    command_send_time = ros::Time::now();

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  LogFile << "Packet Sent, Action Observed, Latency (sec)" << endl;

  for (int i = 0; i < sends.size(); i++)
  {
    LogFile << sends[i] << ", " << observes[i] << ", " << dts[i] << endl;
  }

  LogFile.close();

  return 0;
}
