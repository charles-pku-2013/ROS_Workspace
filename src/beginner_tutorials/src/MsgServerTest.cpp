#include "MsgServerRosNode.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>

// 收到消息的处理回调函数，注意，该函数不在主线程中执行。
// 可对收到的消息交给其他模块处理，比如加入线程池的工作队列
void OnMsgCallback( const std_msgs::String::ConstPtr& msg )
{
	ROS_INFO_STREAM( "MsgSvr received: " << *msg );
}


void ReceiveMsgDemo()
{
	const std::string TOPIC = "chatter";
	gRosMsgSvr->addSubscriber<std_msgs::String>( TOPIC, OnMsgCallback );
}

// 若多个线程都有发送消息操作，建议加锁使用。
void PublishMsgDemo()
{
	const std::string TOPIC = "chatter";
	
	gRosMsgSvr->addPublisher<std_msgs::String>( TOPIC );
	
	int count = 0;
	while( ros::ok() ) {
		sleep(1);
		
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << ++count;
		msg.data = ss.str();

		ROS_INFO_STREAM( "msgsvr publishing msg: " << msg );

		gRosMsgSvr->publishMsg( TOPIC, msg );
	} // while
}


int main( int argc, char **argv )
{
	using namespace std;
	
	// 在main函数开始执行
	InitROS( argc, argv );
	
/*
	How to test:
	在beginner_tutorial里有2个收发消息演示的程序
	talker： 发送消息
	listener： 接受消息
	测试 PublishMsgDemo： 先启动listener
	（启动roscore后，直接运行$workspace/devel/lib/beginner_tutorial/下的listener），再运行本程序
	测试 ReceiveMsgDemo： 先启动本程序，再启动talker
*/	
	PublishMsgDemo();
//	ReceiveMsgDemo();
	
	getchar();
	
	return 0;
}











