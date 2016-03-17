#ifndef _MSG_SERVER_ROS_NODE_H
#define _MSG_SERVER_ROS_NODE_H


#include <ros/ros.h>
#include <string>
#include <map>


// class definition
class MsgServerRosNode {
public:
	typedef std::map<std::string, ros::Publisher>	DbPublisherType;
	typedef std::map<std::string, ros::Subscriber>	DbSubscriberType;
	typedef boost::shared_ptr<MsgServerRosNode>		Ptr;
public:
	static Ptr getInstance();		// singleton

	template< typename MsgType >
	void addPublisher( const std::string &topic, std::size_t queueLen = 1, 
							bool latch = false )
	{
		ros::Publisher pub = nh.advertise<MsgType>( topic, queueLen, latch );
		dbPublisher[topic] = pub;
	}
	
	// for global func or static class mem func as callback
	template< typename MsgType >
	void addSubscriber( const std::string &topic, 
							void(*callback)(const typename MsgType::ConstPtr&), 
							std::size_t queueLen = 1 )
	{
		ros::Subscriber sub = nh.subscribe<MsgType>(topic, queueLen, callback);
		dbSubscriber[topic] = sub;
	}
	
	// for member func of a class as callback
	template< typename MsgType, typename T >
	void addSubscriber( const std::string &topic, 
							void(T::*callback)(const typename MsgType::ConstPtr&), 
							T *obj,
							std::size_t queueLen = 1 )
	{
		ros::Subscriber sub = nh.subscribe<MsgType>(topic, queueLen, callback, obj);
		dbSubscriber[topic] = sub;
	}
	
	template< typename MsgType >
	int publishMsg( const std::string &topic, const MsgType &msg )
	{
		DbPublisherType::iterator it = dbPublisher.find(topic);
		if( dbPublisher.end() == it )
			return -1;
		it->second.publish(msg);
		return 0;
	}
	
	template< typename MsgType >
	int publishMsg( const std::string &topic, const typename MsgType::ConstPtr pMsg )
	{
		DbPublisherType::iterator it = dbPublisher.find(topic);
		if( dbPublisher.end() == it )
			return -1;
		it->second.publish(pMsg);
		return 0;		
	}
	
	// for call services
	template< typename ServiceType >
	int callService( const std::string &serviceName, ServiceType &srv )
	{
		ros::ServiceClient client = nh.serviceClient<ServiceType>( serviceName );
		if( client.call(srv) )
			return 0;
		return -1;
	}
private:
	// constructor is private for single instance
	MsgServerRosNode() {}
	// member data
	DbPublisherType		dbPublisher;	// topic - publisher
	DbSubscriberType	dbSubscriber;	// topic - subscriber
	ros::NodeHandle		nh;
	static Ptr			pInstance;
};

/* declare global instance and funcs */
extern MsgServerRosNode::Ptr		gRosMsgSvr;
extern void InitROS( int argc, char **argv, 
			const std::string &nodeName = "msg_svr_node",
			uint32_t nThread = 0 );

#endif
















