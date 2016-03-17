#ifndef _ROBOT_SDK_H
#define _ROBOT_SDK_H

// typedef void* (*robot_callback_t)(void *arg);

extern int robot_init( int argc, char **argv, const char *libpath );
extern void robot_shutdown();
extern void robot_wheel_move( const double rho, const double theta );
extern void robot_head_move( const int h_angle, const int v_angle );
// test
// extern void register_callback( const robot_callback_t cb_func );


// error handle ----------------------------
typedef enum {
    robot_OK,                       // 0
    robot_INVALID_ARGUMENT,         // 1
    robot_LOAD_LIB_FAIL,            // 2
    robot_READ_LIB_ERROR,           // 3
    robot_OPERATION_FAIL,           // 4
    robot_CALL_SERVICE_FAIL,        // 5
    robot_ERRNO_MAX
} robot_errno_enum_t;

extern int robot_errno;
extern const char* robot_strerror();
extern void robot_clear_error();
// -----------------------------------------



// ir_control ---------------------------------
typedef void (*ir_learn_callback_t)( const char *brand, const char *model, const char *cmd );
extern int robot_ir_ctrl( const char *brand, const char *model, const char *cmd );
extern int robot_ir_learn( const char *brand, const char *model, const char *cmd,
                            ir_learn_callback_t ir_learn_callback );
// --------------------------------------------



#endif















