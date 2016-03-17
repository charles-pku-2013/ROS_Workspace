#include "MsgServerRosNode.h"

/* definition of static members */
MsgServerRosNode::Ptr	MsgServerRosNode::pInstance;

/* definition of non-template member funcs */
MsgServerRosNode::Ptr MsgServerRosNode::getInstance()
{
	if( !pInstance )
		pInstance.reset( new MsgServerRosNode );
	return pInstance;
}

/* global instance and funcs */
MsgServerRosNode::Ptr		gRosMsgSvr;


void InitROS( int argc, char **argv, 
			const std::string &nodeName,
			uint32_t nThread )			
{
	ros::init(argc, argv, nodeName);
	gRosMsgSvr = MsgServerRosNode::getInstance();
	
	static ros::AsyncSpinner spinner(nThread);
	spinner.start();
}














