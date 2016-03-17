#include "robot_sdk_impl.h"
#include <cassert>
#include <unistd.h>


static GeneralRosNode::Ptr gRosNode;

// forward func decl
static void on_ir_toapp( const IrCtrlAckMsgType::ConstPtr &msg );



// test
// #define TEST_TOPIC       "greeting"
// typedef std_msgs::String TestMsgType;
// static robot_callback_t  msg_cb_func = 0;
// void on_greeting_msg( const TestMsgType::ConstPtr &pMsg )
// {
    // ROS_INFO_STREAM( "Received msg: " << *pMsg );
    // if( msg_cb_func )
        // (*msg_cb_func)( (void*)(pMsg->data.c_str()) );
// }


int sdk_init_ros( int argc, char **argv )
{
    ros::init(argc, argv, NODE_NAME);

    gRosNode = GeneralRosNode::getInstance();

    gRosNode->addPublisher <WheelCmdMsgType> ( WHEEL_CMD_TOPIC );
    gRosNode->addPublisher <HeadCtrlMsgType> ( HEAD_CTRL_TOPIC );

    // test
    // gRosNode->addSubscriber <TestMsgType> ( TEST_TOPIC, on_greeting_msg );

    gRosNode->addSubscriber <IrCtrlAckMsgType> ( IR_CONTROL_ACK_TOPIC, on_ir_toapp );

    gRosNode->waitForSubscribers();

/*
    NOTE!!! spinner here cannot defined as static var, for static var only inited the first time
    this function called. However, in such a situation: init() -> shutdown() -> init(), the spinner
    will not work in the second init, for it has been stopped during shutdown()
 */
    // static ros::AsyncSpinner spinner(0);
    static boost::shared_ptr< ros::AsyncSpinner >   spinner;
    // make sure first destruct the old one.
    spinner.reset( (ros::AsyncSpinner*)NULL );
    spinner.reset( new ros::AsyncSpinner(0) );
    spinner->start();

    ROS_INFO( "init_ros OK..." );

    return 0;
}


void sdk_shutdown_ros()
{
    ROS_INFO( "sdk_shutdown_ros" );
    ros::shutdown();
}


void sdk_wheel_move( const double rho, const double theta )
{
    static ros::Publisher *pub = gRosNode->getPublisher( WHEEL_CMD_TOPIC );

    assert( pub != NULL );

    WheelCmdMsgType msg;
    msg.linear.x = rho;
    msg.angular.z = theta;

    ROS_INFO_STREAM("Send wheel cmd: " << msg);

    pub->publish( msg );
}


void sdk_head_move( const int h_angle, const int v_angle )
{
    static int last_h_angle = 0;
    static int last_v_angle = 0;
    static ros::Publisher *pub =
                gRosNode->getPublisher( HEAD_CTRL_TOPIC );

    assert( pub != NULL );

    HeadCtrlMsgType msg;

    if( last_h_angle != h_angle ) {
        msg.updated[ HeadCtrlMsgType::HORIZONTAL ] = true;
        last_h_angle = h_angle;
    } // if

    if( last_v_angle != v_angle ) {
        msg.updated[ HeadCtrlMsgType::VERTICAL ] = true;
        last_v_angle = v_angle;
    } // if

    msg.angle[ HeadCtrlMsgType::HORIZONTAL ] = h_angle;
    msg.angle[ HeadCtrlMsgType::VERTICAL ] = v_angle;

    ROS_INFO_STREAM("Send head cmd: " << msg);

    pub->publish( msg );
}


// ir control -----------------------------------------------------------------------

int sdk_ir_ctrl( const char *_brand, const char *_model, const char *_cmd )
{
    IrCtrlSrvType       srv;

    srv.request.type = "control";
    srv.request.brand = _brand;
    srv.request.model = _model;
    srv.request.command = _cmd;

    if( !(gRosNode->callService( IR_CTRL_SRV, srv )) ) {
        ROS_ERROR_STREAM( "sdk_ir_ctrl failed to call service: " << IR_CTRL_SRV );
        return 1;
    } // if

    return (srv.response.result == 0 ? 0 : -1 );
}

// user assigned ir_learn result callback
static ir_learn_callback_t ir_learn_callback = NULL;

static
void on_ir_toapp( const IrCtrlAckMsgType::ConstPtr &msg )
{
    ROS_DEBUG_STREAM( "on_ir_toapp received msg: " << *msg );

    if( ir_learn_callback )
        ir_learn_callback( msg->brand.c_str(), msg->model.c_str(), msg->command.c_str() );
}


int sdk_ir_learn( const char *_brand, const char *_model, const char *_cmd,
                            ir_learn_callback_t callback )
{
    IrCtrlSrvType       srv;

    ir_learn_callback = callback;

    srv.request.type = "learning";
    srv.request.brand = _brand;
    srv.request.model = _model;
    srv.request.command = _cmd;

    if( !(gRosNode->callService( IR_CTRL_SRV, srv )) ) {
        ROS_ERROR_STREAM( "sdk_ir_learn failed to call service: " << IR_CTRL_SRV );
        return 1;
    } // if

    return 0;
}

// ----------------------------------------------------------------------------------------

// test
// void sdk_register_callback( const robot_callback_t cb_func )
// {
    // msg_cb_func = cb_func;
// }





















