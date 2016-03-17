#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <sstream>
#include <string>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");		// node名字"talker"

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("greeting", 1);		// "chatter: topic name"

	//  ros::Rate loop_rate(10);
	// srand( time(0) );
	// int sleep_time;
	// int count = 0;
	
	// while( 0 == chatter_pub.getNumSubscribers() ) {
		// usleep(1);
		// ROS_INFO("%d", ++count);
	// }
	
	string line;
	std_msgs::String msg;
	while (ros::ok())
	{
		// ROS_INFO( "num of subscribers: %u", chatter_pub.getNumSubscribers() );
		// getchar();
		
		getline(cin, line);
		msg.data.swap(line);

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

//		ros::spinOnce();
//		loop_rate.sleep();
		// ++count;
		
		// if( count == 10 )
			// break;
		
		// sleep_time = 1000 + rand() % 1000;
		// usleep( sleep_time * 1000 );
		// usleep( 500*1000 );
	}
	
	// pause();

	return 0;
}













