#include "robot_sdk.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <dlfcn.h>
#include <linux/limits.h>
//#include <unistd.h>

#ifndef LINE_MAX
#define LINE_MAX    1024
#endif

#define LIB_FILE "/lib/librobotsdk.so"

#define LOAD_FUNC( name ) do {                  \
                            char *__load_func_error_str = 0;    \
                            *(void **) (&(name)) = dlsym(handle, #name);        \
                            if((__load_func_error_str = dlerror()) != NULL) {       \
                                set_error( robot_READ_LIB_ERROR, "locd function %s error: %s", \
                                        #name, __load_func_error_str ); \
                                close_handle();                     \
                                return -1;                          \
                            }                                       \
                        } while(0) 


static void *handle = 0;

static inline
void close_handle()
{
    if(handle) {
        dlclose(handle);
        handle = 0;
    } // if
}


// error handle ----------------------------------
int robot_errno = 0;
static char error_msg[LINE_MAX];

static const char *robot_errlist[] = {
                                    "robot_OK",                     // 0
                                    "robot_Invalid_Argument",       // 1
                                    "robot_Load_Lib_Fail",          // 2
                                    "robot_Read_Lib_Error",         // 3
                                    "robot_Operation_Fail",         // 4
                                    "robot_Call_Service_Fail",      // 5
                                }; // robot_errlist

inline
const char* robot_strerror()
{ return error_msg; }

static inline
void set_error( const int err_no, const char *fmt, ... )
{
    char *s = error_msg;
    va_list ap;
    
    assert( err_no >= 0 && err_no < robot_ERRNO_MAX );
    
    robot_errno = err_no;
    s += sprintf( s, "%s: ", robot_errlist[err_no] );
    va_start(ap, fmt);
    vsprintf( s, fmt, ap );
    va_end(ap);
}

inline
void robot_clear_error()
{
    robot_errno = robot_OK;
    sprintf( error_msg, "%s", robot_errlist[robot_OK] );
}
// --------------------------------------------------


// func ptrs to the ros lib -------------------------
static int (*sdk_init_ros)( int argc, char **argv );
static void (*sdk_shutdown_ros)();
static void (*sdk_wheel_move)( const double rho, const double theta );
static void (*sdk_head_move)( const int h_angle, const int v_angle );
static int (*sdk_ir_ctrl)( const char *brand, const char *model, const char *cmd );
static int (*sdk_ir_learn)( const char *brand, const char *model, const char *cmd,
                                ir_learn_callback_t ir_learn_callback );
// test
// static void (*sdk_register_callback)( const robot_callback_t cb_func );
// ----------------------------------------------------


// test ----------------------------------------------
static
void run_test()
{
    // error handle
    // {
        // printf("error handle test:\n");
        // printf("%d, %s\n", robot_errno, robot_strerror());
        // set_error( robot_INVALID_ARGUMENT, "" );
        // printf("%d, %s\n", robot_errno, robot_strerror());
    // }
    
    exit(0);
}
// -----------------------------------------------------

int robot_init( int argc, char **argv, const char *libpath )
{
    char        *error = NULL;
    char        buf[PATH_MAX];
    int         retval;
    
    robot_clear_error();
    
    // run_test();
    
    if( strlen(libpath) + strlen(LIB_FILE) >= PATH_MAX ) {
        set_error( robot_INVALID_ARGUMENT, "invalid libpath" );
        return -1;
    } // if
    
    if(!libpath) libpath = ".";
    
    strncpy( buf, libpath, PATH_MAX );
    strcat( buf, LIB_FILE );
    
    close_handle();     // prevent multiple call of init
    handle = dlopen(buf, RTLD_LAZY);
    if(!handle) {
        set_error( robot_LOAD_LIB_FAIL, "cannot load robot lib %s", dlerror() );
        return -1;
    } // if
    
    dlerror();    /* Clear any existing error */
    
    // load functions in ros lib
    LOAD_FUNC( sdk_init_ros );
    LOAD_FUNC( sdk_shutdown_ros );
    LOAD_FUNC( sdk_wheel_move );
    LOAD_FUNC( sdk_head_move );
    LOAD_FUNC( sdk_ir_ctrl );
    LOAD_FUNC( sdk_ir_learn );
    // test
    // LOAD_FUNC( sdk_register_callback );
    
    retval = sdk_init_ros( argc, argv );
    
    if( retval )
        set_error( robot_OPERATION_FAIL, "sdk_init_ros fail" );
    
    return retval;
}

inline
void robot_shutdown()
{
    sdk_shutdown_ros();
    close_handle();
}

inline
void robot_wheel_move( const double rho, const double theta )
{
    sdk_wheel_move( rho, theta );
}

inline
void robot_head_move( const int h_angle, const int v_angle )
{
    sdk_head_move( h_angle, v_angle );
}

inline
int robot_ir_ctrl( const char *brand, const char *model, const char *cmd )
{
    int retval = sdk_ir_ctrl( brand, model, cmd );
    
    if( 1 == retval )
        set_error( robot_CALL_SERVICE_FAIL, 
                    "ir control fail, mybe the ir_control module not started" );
    
    return retval;
}

inline
int robot_ir_learn( const char *brand, const char *model, const char *cmd,
                        ir_learn_callback_t callback )
{
    int retval = sdk_ir_learn( brand, model, cmd, callback );
    
    if( 1 == retval )
        set_error( robot_CALL_SERVICE_FAIL, 
                    "ir learn fail, mybe the ir_control module not started" );  
    
    return retval;
}


// test
// inline
// void register_callback( const robot_callback_t cb_func )
// {
    // sdk_register_callback( cb_func );
// }
















