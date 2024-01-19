#ifndef RUNNER_TASK_H
#define RUNNER_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define RUNNER_TASK_INIT_TIME 300
#define RUNNER_CONTROL_TIME_MS 5

#define RUNNER_MOTOR_SPEED_PID_KP 80.0f//100.0f
#define RUNNER_MOTOR_SPEED_PID_KI 0.0f
#define RUNNER_MOTOR_SPEED_PID_KD 0.0f

#define RUNNER_MOTOR_SPEED_PID_MAX_OUT 8000.f
#define RUNNER_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define RUNNER_MOTOR_ANGLE_PID_KP 600.0f//270.0f
#define RUNNER_MOTOR_ANGLE_PID_KI 15.0f
#define RUNNER_MOTOR_ANGLE_PID_KD -0.30f

#define RUNNER_MOTOR_ANGLE_PID_MAX_OUT 70.0f
#define RUNNER_MOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define ECD_OFFSET 1043
#define ANGLE_INIT 0
#define TURN_ANGLE 1.570796327f

#define ANGLE_DIFF 0.1f
#define SPEED_READY 0.02f
typedef enum
{
  RUNNER_DOWN,
	RUNNER_INIT,
	RUNNER_READY,
	RUNNER_LOAD,
	RUNNER_TURNING,
	RUNNER_REACH
} runner_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 last_err;
	
    fp32 max_out;
    fp32 max_iout;
		fp32 max_dout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
		fp32 dT;
} runner_PID_t;

typedef struct
{
  const motor_measure_t *runner_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
  int16_t give_current;
} runner_motor_t;

typedef struct
{
  const RC_ctrl_t *RC_data;               //????'?õ?????????, the point to remote control
  runner_mode_e last_runner_mode;               //state machine. ???????????
	runner_mode_e runner_mode; 
  runner_motor_t motor_data;          //chassis motor data.??????????
	runner_PID_t runner_angle_pid;
  pid_type_def runner_speed_pid;             //motor speed PID.?????????pid


} runner_act_t;


/**
  * @brief          runner task, osDelay RUNNER_CONTROL_TIME_MS (5ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ????,?? RUNNER_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ?
  * @retval         none
  */
extern void runner_task(void const * argument);
extern uint8_t get_runner_mode(void);
#endif
