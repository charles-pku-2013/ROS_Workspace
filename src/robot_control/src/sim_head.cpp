#include <ros/ros.h>
#include <iostream>
#include "common_msgs/ServMotorControl.h"


#define NODE_NAME					"sim_robot_head"
#define DEFAULT_FREQUENCY			50
#define DEFAULT_HEAD_CTRL_TOPIC		"robot_control/head_ctrl"

using namespace std;
using namespace common_msgs;

typedef ServMotorControl			HeadCtrlMsgType;

ros::Subscriber 	sub;

void OnHeadCtrlMsg( HeadCtrlMsgType::ConstPtr pMsg )
{
	ROS_INFO_STREAM( "Move head to angle: " << *pMsg );
}


int main( int argc, char **argv )
{
	string HEAD_CTRL_TOPIC;
	
	ros::init( argc, argv, NODE_NAME );
	
	ros::NodeHandle nh;
	
	nh.param < string > ( "HEAD_CTRL_TOPIC", HEAD_CTRL_TOPIC, DEFAULT_HEAD_CTRL_TOPIC );
	
	// print all params
	cout << "Params on this node:" << endl;
	cout << "HEAD_CTRL_TOPIC = " << HEAD_CTRL_TOPIC << endl;		
	
	sub = nh.subscribe<HeadCtrlMsgType>( HEAD_CTRL_TOPIC, 1, &OnHeadCtrlMsg );
	
	cout << "Hi, I'm robot head!" << endl;
	
	ros::Rate spin_rate(DEFAULT_FREQUENCY);
	while( ros::ok() ) {
		ros::spinOnce();
		spin_rate.sleep();
	} // while
	
	return 0;
}




















