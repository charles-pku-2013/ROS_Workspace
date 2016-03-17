#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <unistd.h>
#include "common_msgs/ServMotorCmd.h"


#define NODE_NAME					"sim_robot_commander"

#define DEFAULT_WHEEL_CMD_TOPIC				"robot_control/wheel_cmd"		// send wheel movement cmd
#define DEFAULT_HEAD_CMD_TOPIC				"robot_control/head_cmd"		// send head movement cmd

using namespace std;
using namespace common_msgs;

typedef geometry_msgs::Twist		WheelCmdMsgType;
typedef ServMotorCmd    			HeadCmdMsgType;

string		WHEEL_CMD_TOPIC;
string		HEAD_CMD_TOPIC;

ros::Publisher pubWheel;
ros::Publisher pubHead;


void DealHeadCmd( istream &str )
{
	string cmd;
	HeadCmdMsgType::Ptr pMsg( new HeadCmdMsgType );
	
	str >> cmd;
	if( "up" == cmd ) {
		pMsg->direction[HeadCmdMsgType::UP] = true;
	} else if ( "right" == cmd ) {
		pMsg->direction[HeadCmdMsgType::RIGHT] = true;
	} else if ( "down" == cmd ) {
		pMsg->direction[HeadCmdMsgType::DOWN] = true;
	} else if ( "left" == cmd ) {
		pMsg->direction[HeadCmdMsgType::LEFT] = true;
	}
	
	ROS_INFO_STREAM( "Sending head cmd: " << *pMsg );
	pubHead.publish( pMsg );
}


void DealWheelCmd( istream &str )
{
	string sRho, sTheta;
	WheelCmdMsgType::Ptr pMsg( new WheelCmdMsgType );
	
	str >> sRho >> sTheta;
	pMsg->linear.x = atof(sRho.c_str());
	pMsg->angular.z = atof(sTheta.c_str()) / 180.0 * M_PI;
	
	ROS_INFO_STREAM( "Sending wheel cmd: " << *pMsg );
	pubWheel.publish( pMsg );
}


bool LoadParams( ros::NodeHandle &nh )
{
	nh.param < string > ( "WHEEL_CMD_TOPIC", WHEEL_CMD_TOPIC, DEFAULT_WHEEL_CMD_TOPIC );
	nh.param < string > ( "HEAD_CMD_TOPIC", HEAD_CMD_TOPIC, DEFAULT_HEAD_CMD_TOPIC );
	return true;
}


bool LoadParams( const char *filename )
{
	static const char *SEPERATOR = " :\t\f\r\v\n";
	
	typedef map< string, string >	DictType;
	
	ifstream ifs( filename );
	if( !ifs ) {
		cerr << "Cannot read file: " << filename << endl;
		return false; 
	} // if
	
	DictType dict;
	string line;
	char *key, *value;
	
	while( getline(ifs, line) ) {
		if( line.empty() || line[0] == '#' || isspace(line[0]) )
			continue;
		
		key = strtok( const_cast<char*>(line.data()), SEPERATOR );
		value = strtok( NULL, SEPERATOR );
		if( !key || !value || !(*key) || !(*value) ) { continue; }
		dict[key] = value;
	} // while
	
	DictType::const_iterator it;
	
	if( (it = dict.find("WHEEL_CMD_TOPIC")) == dict.end() ) {
		cerr << "Failed to get param WHEEL_CMD_TOPIC" << endl;
		return false;
	} // if
	WHEEL_CMD_TOPIC = it->second;
	
	if( (it = dict.find("HEAD_CMD_TOPIC")) == dict.end() ) {
		cerr << "Failed to get param HEAD_CMD_TOPIC" << endl;
		return false;
	} // if
	HEAD_CMD_TOPIC = it->second;

	return true;
}


int main( int argc, char **argv )
{	
/* 	cout << "launch by roslaunch:" << endl;
	cout << argc << endl;
	for( int i = 0; i < argc; ++i )
		cout << argv[i] << endl;
	cout << endl; */
	
	ros::init( argc, argv, NODE_NAME );
	
	ros::NodeHandle nh;
	
	const char *paramFileName = 0;
	const char *paramFileCmd = "--paramfile=";
	const size_t lenParamFileCmd = strlen(paramFileCmd);
	if( argc >= 2 && strncmp(argv[1], paramFileCmd, lenParamFileCmd) == 0 ) { 
		paramFileName = argv[1] + lenParamFileCmd;
	} // if
	
	if( paramFileName && *paramFileName ) {
		if( !LoadParams( paramFileName ) )
			return -1;		
	} else {
		if( !LoadParams( nh ) )
			return -1;		
	} // if	
	
	// print all params
	cout << "Params on this node:" << endl;
	cout << "WHEEL_CMD_TOPIC = " << WHEEL_CMD_TOPIC << endl;	
	cout << "HEAD_CMD_TOPIC = " << HEAD_CMD_TOPIC << endl;
	
	pubWheel = nh.advertise<WheelCmdMsgType>(WHEEL_CMD_TOPIC, 1, true);
	pubHead = nh.advertise<HeadCmdMsgType>(HEAD_CMD_TOPIC, 1, true);
	
	cout << "Hi, I'm Robot Commander" << endl;
	
	string line, part;
	while( getline(cin, line) ) {
		if( line.empty() || line[0] == '#' )
			continue;
		stringstream str(line);
		str >> part;
		if( part == "head" ) {
			DealHeadCmd( str );
		} else if ( part == "wheel" ) {
			DealWheelCmd( str );
		} // if
		usleep( 500 * 1000 );
	} // while
	
	return 0;
}




















