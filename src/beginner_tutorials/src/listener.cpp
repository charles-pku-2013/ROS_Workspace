#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

using namespace std;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM( "I heared msg: " << *msg );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("voice_control", 1000, chatterCallback);

  ros::spin();

  return 0;
}



















