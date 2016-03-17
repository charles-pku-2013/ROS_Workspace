#ifndef _ROBOT_SDK_IMPL_H
#define _ROBOT_SDK_IMPL_H

#include "general_ros_node.h"
#include <geometry_msgs/Twist.h>
#include "common_msgs/ServMotorCmd.h"
#include "common_msgs/ServMotorControl.h"
#include "common_msgs/ir_tv.h"
#include "common_msgs/ir_toapp.h"
#include "std_msgs/String.h"


#define NODE_NAME   "sdk_servo"

/* topics & services */
// for movement control
#define WHEEL_CMD_TOPIC     "WheelCmd"      // send wheel movement cmd
// #define HEAD_CMD_TOPIC       "HeadCmd"       // send head movement cmd
#define HEAD_CTRL_TOPIC     "HeadCtrl"      // send head ctrl to head motor
// for ir_control
// #define IR_CONTROL_CMD_TOPIC     "ir_control_cmd"    // for sending cmd to ir_control
#define IR_CONTROL_ACK_TOPIC        "irc_toapp_topic"   // for receiving ack msg from ir_control
// for ir control
#define IR_CTRL_SRV                 "ircl_central_service"


// typedefs
typedef geometry_msgs::Twist            WheelCmdMsgType;    // for sending wheel movement cmds to robot_control
// typedef common_msgs::ServMotorCmd        HeadCmdMsgType;     // for sending head movement cmds to robot_control
typedef common_msgs::ServMotorControl   HeadCtrlMsgType;    // for sending head ctrl cmd directly to the motor
typedef common_msgs::ir_tv              IrCtrlSrvType;      // for sending ir control msg to ir_control
typedef common_msgs::ir_toapp       IrCtrlAckMsgType;
typedef std_msgs::String            IrCtrlCmdMsgType;

// test
// typedef void* (*robot_callback_t)(void *arg);


typedef void (*ir_learn_callback_t)( const char *brand, const char *model, const char *cmd );

extern "C" {
    extern int sdk_init_ros( int argc, char **argv );
    extern void sdk_shutdown_ros();
    extern void sdk_wheel_move( const double rho, const double theta );
    extern void sdk_head_move( const int h_angle, const int v_angle );
    extern int sdk_ir_ctrl( const char *brand, const char *model, const char *cmd );
    extern int sdk_ir_learn( const char *brand, const char *model, const char *cmd,
                                ir_learn_callback_t ir_learn_callback );
    
    // test
    // extern void sdk_register_callback( const robot_callback_t cb_func );
} // extern "C"


#endif















