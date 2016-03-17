#include "robot_sdk.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#define LIB_PATH "/mnt/hgfs/Documents/ROS/ROS_Workspace/devel"


static
void test_ir_learn_callback( const char *brand, const char *model, const char *cmd )
{
    printf("test_ir_learn_callback:\n");
    printf("%s %s %s\n", brand, model, cmd);
}

static
void run_test()
{
    robot_wheel_move( 1.0, 0.0 );
    sleep(1);
    robot_head_move( 30, 50 );
    
    // ir control / learn test
    // if( robot_ir_ctrl( "Nikon", "D800", "PowerOn" ) )
        // printf("robot_ir_ctrl fail %s\n", robot_strerror());
    // sleep(1);
    // if( robot_ir_learn( "Nikon", "D800", "PowerOn", test_ir_learn_callback ) )
        // printf("robot_ir_learn fail %s\n", robot_strerror());
}


// void* msg_cb_func( void *arg )
// {
    // const char *msg = (const char*)arg;
    
    // printf("user program received msg on thread %ld: %s\n", (long)pthread_self(), msg);
    
    // return (void*)(0);
// }



int main( int argc, char **argv )
{
    if( robot_init( argc, argv, LIB_PATH ) != 0 ) {
        fprintf( stderr, "robot_init fail: %s\n", robot_strerror() );
        exit( EXIT_FAILURE );
    } // if
    
    printf( "user main thread: %ld\n", (long)pthread_self() );
    
    // printf( "robot started, press any key to continue...\n" );
    printf( "running test...\n" );
    run_test();
    // register_callback( msg_cb_func );
    
    printf( "press enter to shutdown...\n" );
    getchar();
    printf( "robot shutdown...\n" );
    robot_shutdown();
    
    return 0;
}










