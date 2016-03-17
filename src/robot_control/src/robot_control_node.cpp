/*
  Wheel control msg type is "Twist", which consists of a linear(x,y,z) and an angular(x,y,z), 
  is typedefed to WheelCmdMsgType here.
  We use polar coordinate to represent the movement of the wheel, and the cmd msg 
  WheelCmdMsgType::linear.x corresponds to rho; WheelCmdMsgType::angular.z corresponds to theta, 
  the rest of members in the wheel control msg like linear.y(z) and angular.x(y) is useless in this program.
  
  The value WheelCmdMsgType::linear.x ranges in [0,1] which represents the percentage 
  of the max velocity of the wheels.
  
  The value WheelCmdMsgType::angular.z ranges in [0, 2*PI] or [-PI, PI]
  
  The customized HeadCmdMsgType consists of a 4 length bool array, which correspond to 4 direction buttons, 
  when any one is pressed, the head will trun HEAD_MOVE_STEP_LEN degree towards the corresponding direction.
*/

/*
  TODO: all macro constants should be replaced with ros params
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <cstring>
#include <cmath>
#include <bitset>
#include <exception>
#include "common_msgs/StepMotorControl.h"       // for sending cmd to wheel motors
#include "common_msgs/ServMotorControl.h"       // for sending cmd to head motors
#include "common_msgs/ServMotorCmd.h"           // for receiving head control cmds from msgserver


#define NODE_NAME                   "robot_control_node"

#define DEFAULT_WHEEL_CTRL_TOPIC            "robot_control/wheel_ctrl"      // ctrl msg send to wheel motor
#define DEFAULT_WHEEL_CMD_TOPIC             "robot_control/wheel_cmd"       // receive wheel movement cmd
#define DEFAULT_HEAD_CTRL_TOPIC             "robot_control/head_ctrl"       // ctrl msg send to head motor
#define DEFAULT_HEAD_CMD_TOPIC              "robot_control/head_cmd"        // receive head movement cmd

#define EPSILON                     0.0001  // if two float values difference less than it, take them same
#define DEFAULT_FREQUENCY           50
#define SMOOTH_THRESHOLD            50      // (mm/s) if wheel velocity change bigger than it, smooth is required
#define MAX_WHEEL_VELOCITY          500     // (mm/s)
#define MAX_HEAD_HORIZONTAL_ANGLE   600
#define MIN_HEAD_HORIZONTAL_ANGLE   -600    // 1/10 degree
#define MAX_HEAD_VERTICAL_ANGLE     300
#define MIN_HEAD_VERTICAL_ANGLE     -300
#define HEAD_MOVE_STEP_LEN          30      // (1/10 degree) when press head turning control button, the minimum degree the head moves.


using namespace common_msgs;

namespace robot_control {

typedef geometry_msgs::Twist        WheelCmdMsgType;
typedef StepMotorControl            WheelCtrlMsgType;
typedef ServMotorControl            HeadCtrlMsgType;
typedef ServMotorCmd                HeadCmdMsgType;


bool operator== ( const HeadCtrlMsgType &lhs, const int *rhs )
{ return std::equal( lhs.angle.begin(), lhs.angle.end(), rhs ); }

bool operator!= ( const HeadCtrlMsgType &lhs, const int *rhs )
{ return !(lhs == rhs); }

bool operator== ( const WheelCmdMsgType &lhs, const WheelCmdMsgType &rhs )
{
    // here we only consider linear.x and angular.z, 
    // which correspond to rho and theta in Polar coordinate
    if( fabs(lhs.linear.x - rhs.linear.x) >= EPSILON )
        return false;
    if( fabs(lhs.angular.z - rhs.angular.z) >= EPSILON )
        return false;
    return true;
}

bool operator!= ( const WheelCmdMsgType &lhs, const WheelCmdMsgType &rhs )
{ return !(lhs == rhs); }


/* if started not by roslaunch, read params from file specified by paramFileName */
static const char *paramFileName = 0;

/* class definition */
class RobotControl {
public:
    RobotControl();
    void spin();
protected:
    // callback funcs here
    void onWheelCmd( const WheelCmdMsgType::ConstPtr& msg );
    void onHeadCmd( const HeadCmdMsgType::ConstPtr& msg );
    // other helper funcs here
    static void setNewHeadAngle( HeadCtrlMsgType &msgAngle, std::size_t direction );
    bool updateWheelVel( std::bitset<2> &mark ); // update curWheelVel according to targetWheelVel and SMOOTH_THRESHOLD
    int waitForSubscribers( uint32_t loop_count = 1000 );
    bool getParams();
    bool getParamsFromFile();
    // member varaibles
    ros::NodeHandle     nh;
    ros::Publisher      pubWheelCtrl;
    ros::Publisher      pubHeadCtrl;
    ros::Subscriber     subWheelCmd;
    ros::Subscriber     subHeadCmd;
    int                 currentHeadAngle[2];
    WheelCmdMsgType::ConstPtr   pLastWheelCmd;
    double              curWheelVel[2];     // current wheel velocity
    double              targetWheelVel[2];
    double              wheelVelSetp;       // mm/Hz determined by SMOOTH_THRESHOLD and DEFAULT_FREQUENCY
    // msg topics
    std::string WHEEL_CTRL_TOPIC;   // ctrl msg send to wheel motor
    std::string WHEEL_CMD_TOPIC;    // receive wheel movement cmd
    std::string HEAD_CTRL_TOPIC;    // ctrl msg send to head motor
    std::string HEAD_CMD_TOPIC;     // receive head movement cmd
};


/* Implement of RobotControl */
RobotControl::RobotControl() 
        : pLastWheelCmd( new WheelCmdMsgType )
        , wheelVelSetp((double)SMOOTH_THRESHOLD / DEFAULT_FREQUENCY)
{
    if( !getParams() )
        throw std::invalid_argument( "Fail to get params!" );
    
    using namespace std;
    
    // print all params
    cout << "Params on this node:" << endl;
    cout << "WHEEL_CTRL_TOPIC = " << WHEEL_CTRL_TOPIC << endl;
    cout << "WHEEL_CMD_TOPIC = " << WHEEL_CMD_TOPIC << endl;
    cout << "HEAD_CTRL_TOPIC = " << HEAD_CTRL_TOPIC << endl;
    cout << "HEAD_CMD_TOPIC = " << HEAD_CMD_TOPIC << endl;  
    
    memset( currentHeadAngle, 0, sizeof(currentHeadAngle) );
    memset( curWheelVel, 0, sizeof(curWheelVel) );
    memset( targetWheelVel, 0, sizeof(targetWheelVel) );
    pubWheelCtrl = nh.advertise<WheelCtrlMsgType>( WHEEL_CTRL_TOPIC, 1 );
    pubHeadCtrl = nh.advertise<HeadCtrlMsgType>( HEAD_CTRL_TOPIC, 1 );
    subWheelCmd = nh.subscribe<WheelCmdMsgType>( WHEEL_CMD_TOPIC, 1, &RobotControl::onWheelCmd, this );
    subHeadCmd = nh.subscribe<HeadCmdMsgType>( HEAD_CMD_TOPIC, 1, &RobotControl::onHeadCmd, this );
    
    // reset the head position
    waitForSubscribers();
    pubHeadCtrl.publish( HeadCtrlMsgType() );   // assign boost::array in constructor, fill all to 0
}


bool RobotControl::getParamsFromFile()
{
    using namespace std;

    static const char *SEPERATOR = " :\t\f\r\v\n";
    
    typedef map< string, string >   DictType;
    
    ifstream ifs( paramFileName );
    if( !ifs ) {
        cerr << "Cannot read file: " << paramFileName << endl;
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
    
    if( (it = dict.find("WHEEL_CTRL_TOPIC")) == dict.end() ) {
        cerr << "Failed to get param WHEEL_CTRL_TOPIC" << endl;
        return false;
    } // if
    WHEEL_CTRL_TOPIC = it->second;
    
    if( (it = dict.find("HEAD_CTRL_TOPIC")) == dict.end() ) {
        cerr << "Failed to get param HEAD_CTRL_TOPIC" << endl;
        return false;
    } // if
    HEAD_CTRL_TOPIC = it->second;   
    
    return true;    
}

bool RobotControl::getParams()
{
    using namespace std;
    
    if( paramFileName && *paramFileName )
        return getParamsFromFile();
    
    nh.param < string > ( "WHEEL_CTRL_TOPIC", WHEEL_CTRL_TOPIC, DEFAULT_WHEEL_CTRL_TOPIC );
    nh.param < string > ( "WHEEL_CMD_TOPIC", WHEEL_CMD_TOPIC, DEFAULT_WHEEL_CMD_TOPIC );
    nh.param < string > ( "HEAD_CTRL_TOPIC", HEAD_CTRL_TOPIC, DEFAULT_HEAD_CTRL_TOPIC );
    nh.param < string > ( "HEAD_CMD_TOPIC", HEAD_CMD_TOPIC, DEFAULT_HEAD_CMD_TOPIC );
    
    return true;
}


int RobotControl::waitForSubscribers( uint32_t loop_count )
{
    uint32_t i = 0;
    bool loop;
    
    do {
        loop = false;   
        if( pubWheelCtrl.getNumSubscribers() == 0 || pubHeadCtrl.getNumSubscribers() == 0 )
        { loop = true; }
        usleep( 1 );
    } while( loop && (++i) < loop_count );
    
    if( loop ) {
        ROS_INFO( "There maybe some subscribes not started." );
        return -1;
    } // if
    
    return 0;
    // return (loop ? -1 : 0);
}


void RobotControl::onWheelCmd( const WheelCmdMsgType::ConstPtr& pMsg )
{
    ROS_INFO_STREAM( "Received wheel cmd: " << *pMsg );
    if( *pMsg == *pLastWheelCmd ) {
        ROS_INFO( "No need to change wheel speed." );
        return;
    }
    
    // transfer into rectangular coordinate
    double rho = pMsg->linear.x;
    double theta = pMsg->angular.z;
    double x = rho * cos(theta);
    double y = rho * sin(theta);
    
    ROS_ASSERT_MSG( (x >= -1.0 && x <= 1.0), "x coordinate out of bound!" );
    ROS_ASSERT_MSG( (y >= -1.0 && y <= 1.0), "y coordinate out of bound!" );
    
    // y > 0 turn left, < 0 turn right.
    //!! 先临时这么定义
    double ratioLeft = x - y;
    double ratioRight = x + y;
    
    if( ratioLeft < -1.0 ) {
        ratioLeft = -1.0;
    } else if ( ratioLeft > 1.0 ) {
        ratioLeft = 1.0;
    } // if
    
    if( ratioRight < -1.0 ) {
        ratioRight = -1.0;
    } else if( ratioRight > 1.0 ) {
        ratioRight = 1.0;
    } // if
    
    targetWheelVel[WheelCtrlMsgType::LEFT] = ratioLeft * MAX_WHEEL_VELOCITY;
    targetWheelVel[WheelCtrlMsgType::RIGHT] = ratioRight * MAX_WHEEL_VELOCITY;
    
    pLastWheelCmd = pMsg;
}


void RobotControl::onHeadCmd( const HeadCmdMsgType::ConstPtr& msg )
{
    HeadCtrlMsgType newHeadAngle;
    // new = current
    std::copy( currentHeadAngle, currentHeadAngle + 2, newHeadAngle.angle.begin() );

    std::size_t i;
    for( i = 0; i < msg->direction.size(); ++i ) {
        if( msg->direction[i] ) {
            setNewHeadAngle( newHeadAngle, i );
            break;  // only care the first activated direction
        } // if
    } // for
    
    ROS_INFO_COND( newHeadAngle == currentHeadAngle, "No need to change head angle." );
    
    if( newHeadAngle != currentHeadAngle ) {
        // currentHeadAngle = newHeadAngle
        std::copy( newHeadAngle.angle.begin(), newHeadAngle.angle.end(), currentHeadAngle );
        newHeadAngle.updated[ i % 2 ] = true;
        ROS_INFO_STREAM( "Set head to angle: " << newHeadAngle );
        pubHeadCtrl.publish(newHeadAngle);
    } // if
    
    return;
}

void RobotControl::setNewHeadAngle( HeadCtrlMsgType &msgAngle, std::size_t direction )
{
    switch(direction) {
    case HeadCmdMsgType::UP: {
        msgAngle.angle[HeadCtrlMsgType::VERTICAL] = 
                std::min( msgAngle.angle[HeadCtrlMsgType::VERTICAL] + HEAD_MOVE_STEP_LEN, MAX_HEAD_VERTICAL_ANGLE );
        break;
    }
    case HeadCmdMsgType::DOWN: {
        msgAngle.angle[HeadCtrlMsgType::VERTICAL] = 
                std::max( msgAngle.angle[HeadCtrlMsgType::VERTICAL] - HEAD_MOVE_STEP_LEN, MIN_HEAD_VERTICAL_ANGLE );
        break;
    }
    case HeadCmdMsgType::RIGHT: {
        msgAngle.angle[HeadCtrlMsgType::HORIZONTAL] = 
                std::min( msgAngle.angle[HeadCtrlMsgType::HORIZONTAL] + HEAD_MOVE_STEP_LEN, MAX_HEAD_HORIZONTAL_ANGLE );
        break;
    }
    case HeadCmdMsgType::LEFT: {
        msgAngle.angle[HeadCtrlMsgType::HORIZONTAL] = 
                std::max( msgAngle.angle[HeadCtrlMsgType::HORIZONTAL] - HEAD_MOVE_STEP_LEN, MIN_HEAD_HORIZONTAL_ANGLE );
        break;
    }   
    } // switch
}


bool RobotControl::updateWheelVel( std::bitset<2> &mark )
{
    double diff;
    
    // diff > 0 accel < 0 decel
    diff = targetWheelVel[WheelCtrlMsgType::LEFT] - curWheelVel[WheelCtrlMsgType::LEFT];
    if( fabs(diff) >= EPSILON ) {
        mark.set(WheelCtrlMsgType::LEFT);
        curWheelVel[WheelCtrlMsgType::LEFT] = (diff >= 0.0) 
                ? std::min( curWheelVel[WheelCtrlMsgType::LEFT] + wheelVelSetp, targetWheelVel[WheelCtrlMsgType::LEFT] )
                : std::max( curWheelVel[WheelCtrlMsgType::LEFT] - wheelVelSetp, targetWheelVel[WheelCtrlMsgType::LEFT] );
    } // if
    
    diff = targetWheelVel[WheelCtrlMsgType::RIGHT] - curWheelVel[WheelCtrlMsgType::RIGHT];
    if( fabs(diff) >= EPSILON ) {
        mark.set(WheelCtrlMsgType::RIGHT);
        curWheelVel[WheelCtrlMsgType::RIGHT] = (diff >= 0.0) 
                ? std::min( curWheelVel[WheelCtrlMsgType::RIGHT] + wheelVelSetp, targetWheelVel[WheelCtrlMsgType::RIGHT] )
                : std::max( curWheelVel[WheelCtrlMsgType::RIGHT] - wheelVelSetp, targetWheelVel[WheelCtrlMsgType::RIGHT] );
    } // if 
    
    return mark.any();
}

void RobotControl::spin()
{
    ros::Rate                   spin_rate(DEFAULT_FREQUENCY);
    WheelCtrlMsgType::Ptr       pWheelCtrlMsg;
    
    while( ros::ok() ) {
//      ROS_INFO("Program spining...");
        ros::spinOnce();            // get cmd message
        std::bitset<2> mark;
        if( updateWheelVel(mark) ) {
            pWheelCtrlMsg.reset( new WheelCtrlMsgType );
            pWheelCtrlMsg->updated[WheelCtrlMsgType::LEFT] = mark[WheelCtrlMsgType::LEFT];
            pWheelCtrlMsg->updated[WheelCtrlMsgType::RIGHT] = mark[WheelCtrlMsgType::RIGHT];    
            std::copy( curWheelVel, curWheelVel + 2, pWheelCtrlMsg->velocity.begin() );
            ROS_INFO_STREAM( "Set wheel velocity to " << *pWheelCtrlMsg );
            pubWheelCtrl.publish(pWheelCtrlMsg);
        } // if
        spin_rate.sleep();
    } // while
}


} // namespace robot_control

int main( int argc, char **argv )
{
    using namespace robot_control;
    
    const char *paramFileCmd = "--paramfile=";
    const size_t lenParamFileCmd = strlen(paramFileCmd);
    if( argc >= 2 && strncmp(argv[1], paramFileCmd, lenParamFileCmd) == 0 ) { 
        paramFileName = argv[1] + lenParamFileCmd;
    } // if
    
    try {
        
        ros::init( argc, argv, NODE_NAME );
        
        RobotControl    robotCtrl;
        
        ROS_INFO( "Robot Control Module Started!" );
        robotCtrl.spin();
        
    } catch ( const std::exception &ex ) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    
    return 0;
}
























