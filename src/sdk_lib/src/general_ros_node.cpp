#include "general_ros_node.h"


/* definition of static members */
GeneralRosNode::Ptr GeneralRosNode::pInstance;

/* definition of non-template member funcs */
GeneralRosNode::Ptr GeneralRosNode::getInstance()
{
    if( !pInstance )
        pInstance.reset( new GeneralRosNode );
    return pInstance;
}


int GeneralRosNode::waitForSubscribers( uint32_t loop_count )
{
    uint32_t i = 0;
    bool loop;
    
    // ROS_INFO("waitForSubscribers %u", dbPublisher.size());
    
    do {
        loop = false;
        for( DbPublisherType::iterator it = dbPublisher.begin(); it != dbPublisher.end(); ++it ) {
            ros::Publisher &pub = it->second;
            if( pub.getNumSubscribers() == 0 )
            { loop = true; /* ROS_INFO_STREAM("go on waiting... " << i); */ }   // go on waiting
        } // for
        usleep( 1 );
    } while( loop && (++i) < loop_count );
    
    if( loop ) {
        ROS_INFO( "There maybe some subscribes not started." );
        return -1;
    } // if 
    
    return 0;
    // return (loop ? -1 : 0);
}
















