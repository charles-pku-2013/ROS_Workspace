#include <ros/ros.h>
#include <iostream>
#include <string>
#include "common_msgs/StepMotorControl.h"


#define NODE_NAME					"sim_robot_wheel"
#define DEFAULT_FREQUENCY			50
#define DEFAULT_WHEEL_CTRL_TOPIC	"robot_control/wheel_ctrl"

using namespace std;
using namespace common_msgs;

typedef StepMotorControl					WheelCtrlMsgType;

ros::Subscriber 	sub;

void OnWheelCtrlMsg( WheelCtrlMsgType::ConstPtr pMsg )
{
	if( pMsg->updated[WheelCtrlMsgType::LEFT] ) {
		ROS_INFO( "Set velocity of left wheel to %lf", pMsg->velocity[WheelCtrlMsgType::LEFT] );
	} // if
	
	if( pMsg->updated[WheelCtrlMsgType::RIGHT] ) {
		ROS_INFO( "Set velocity of right wheel to %lf", pMsg->velocity[WheelCtrlMsgType::RIGHT] );
	} // if	
}


int main( int argc, char **argv )
{
	string WHEEL_CTRL_TOPIC;
	
	ros::init( argc, argv, NODE_NAME );
	
	ros::NodeHandle nh;
	
	nh.param < string > ( "WHEEL_CTRL_TOPIC", WHEEL_CTRL_TOPIC, DEFAULT_WHEEL_CTRL_TOPIC );
	
	// print all params
	cout << "Params on this node:" << endl;
	cout << "WHEEL_CTRL_TOPIC = " << WHEEL_CTRL_TOPIC << endl;	
	
	sub = nh.subscribe<WheelCtrlMsgType>( WHEEL_CTRL_TOPIC, 1, &OnWheelCtrlMsg );
	
	cout << "Hi, I'm robot wheel!" << endl;
	
	ros::Rate spin_rate(DEFAULT_FREQUENCY);
	
	while( ros::ok() ) {
		ros::spinOnce();
		spin_rate.sleep();
	} // while
	
	return 0;
}




















