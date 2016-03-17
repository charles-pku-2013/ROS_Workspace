#ifndef _GENERAL_ROS_NODE_H
#define _GENERAL_ROS_NODE_H


#include <ros/ros.h>
#include <string>
#include <map>
#include <utility>


// class definition
class GeneralRosNode {
public:
    typedef std::map<std::string, ros::Publisher>   DbPublisherType;
    typedef std::map<std::string, ros::Subscriber>  DbSubscriberType;
    typedef boost::shared_ptr<GeneralRosNode>       Ptr;
public:
    static Ptr getInstance();       // singleton

    template< typename MsgType >
    ros::Publisher* addPublisher( const std::string &topic, std::size_t queueLen = 1, 
                            bool latch = false )
    {
        ros::Publisher *retval = (ros::Publisher*)(0);
        ros::Publisher pub = nh.advertise<MsgType>( topic, queueLen, latch );
        std::pair<DbPublisherType::iterator, bool> insert_ret = 
                                dbPublisher.insert( std::make_pair(topic, pub) );
        retval = &((insert_ret.first)->second);
        return retval;
    }

    ros::Publisher* getPublisher( const std::string &topic )
    {
        DbPublisherType::iterator it = dbPublisher.find( topic );
        if( it == dbPublisher.end() )
            return (ros::Publisher*)(0);
        return &(it->second);
    }
    
    // for global func or static class mem func as callback
    template< typename MsgType >
    ros::Subscriber* addSubscriber( const std::string &topic, 
                            void(*callback)(const typename MsgType::ConstPtr&), 
                            std::size_t queueLen = 1 )
    {
        ros::Subscriber *retval = (ros::Subscriber*)(0);
        ros::Subscriber sub = nh.subscribe<MsgType>(topic, queueLen, callback);
        std::pair<DbSubscriberType::iterator, bool> insert_ret = 
                            dbSubscriber.insert( std::make_pair(topic, sub) );
        retval = &((insert_ret.first)->second);
        return retval;
    }
    
    // for member func of a class as callback
    template< typename MsgType, typename T >
    ros::Subscriber* addSubscriber( const std::string &topic, 
                            void(T::*callback)(const typename MsgType::ConstPtr&), 
                            T *obj,
                            std::size_t queueLen = 1 )
    {
        ros::Subscriber *retval = (ros::Subscriber*)(0);
        ros::Subscriber sub = nh.subscribe<MsgType>(topic, queueLen, callback, obj);
        std::pair<DbSubscriberType::iterator, bool> insert_ret = 
                            dbSubscriber.insert( std::make_pair(topic, sub) );
        retval = &((insert_ret.first)->second);     
        return retval;
    }
    
    ros::Subscriber* getSubscriber( const std::string &topic )
    {
        DbSubscriberType::iterator it = dbSubscriber.find( topic );
        if( it == dbSubscriber.end() )
            return (ros::Subscriber*)0;
        return &(it->second);
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
    bool callService( const std::string &serviceName, ServiceType &srv )
    {
        ros::ServiceClient client = nh.serviceClient <ServiceType> ( serviceName );
        return client.call(srv);
    }
    
    ros::NodeHandle& getNodeHandle() { return nh; }
    const ros::NodeHandle& getNodeHandle() const { return nh; }
    
    // wait other modules that recv msg from this module.
    int waitForSubscribers( uint32_t loop_count = 1000 );   
private:
    // constructor is private for single instance
    GeneralRosNode() {}
    // member data
    DbPublisherType     dbPublisher;    // topic - publisher
    DbSubscriberType    dbSubscriber;   // topic - subscriber
    ros::NodeHandle     nh;
    static Ptr          pInstance;
};



#endif
















