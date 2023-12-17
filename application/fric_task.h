#ifndef FRIC_TASK_H
#define FRIC_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define FRIC_TASK_INIT_TIME 357
#define MAX_3508_CAN_CURRENT 16384.0f

//fric task control time  2ms
//???????? 2ms
#define FRIC_CONTROL_TIME_MS 2
//fric motor speed PID
//???????PID

#define FRIC_LEFT_SPEED_PID_KP      5.0f
#define FRIC_LEFT_SPEED_PID_KI			0.0f
#define FRIC_LEFT_SPEED_PID_KD			0.3f

#define FRIC_RIGHT_SPEED_PID_KP		  5.0f
#define FRIC_RIGHT_SPEED_PID_KI     0.0f
#define FRIC_RIGHT_SPEED_PID_KD     0.3f


#define FRIC_LEFT_PID_MAX_OUT   MAX_3508_CAN_CURRENT
#define FRIC_LEFT_PID_MAX_IOUT  5000.0f

#define FRIC_RIGHT_PID_MAX_OUT   MAX_3508_CAN_CURRENT
#define FRIC_RIGHT_PID_MAX_IOUT  5000.0f



#define FRIC_SET_SPEED 1000.0f

typedef enum
{
  FRIC_ON,
	FRIC_OFF,
	FRIC_READY
} fric_mode_e;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} fric_motor_t;

typedef struct
{
  const RC_ctrl_t *RC_data;  
  fric_mode_e fric_mode;  	//????'?õ?????????, the point to remote control
  fric_motor_t motor_data[6];          //chassis motor data.??????????
  pid_type_def motor_speed_pid[6];              //motor speed PID.?????????pid
} fric_act_t;

/**
  * @brief          fric task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ????,?? FRIC_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ?
  * @retval         none
  */
extern void fric_task(void const * argument);

extern uint8_t get_fric_mode(void);

extern fric_act_t fric_act;


#endif
