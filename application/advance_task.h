#ifndef ADVANCE_TASK_H
#define ADVANCE_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define ADV_TASK_INIT_TIME 300
#define ADV_CONTROL_TIME_MS 5

#define ADV_MOTOR_SPEED_PID_KP 10.0f
#define ADV_MOTOR_SPEED_PID_KI 0.0f
#define ADV_MOTOR_SPEED_PID_KD -0.1f

#define ADV_MOTOR_SPEED_PID_MAX_OUT 8000.f
#define ADV_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define ADV_MOTOR_ANGLE_PID_KP 500.0f
#define ADV_MOTOR_ANGLE_PID_KI 15.0f
#define ADV_MOTOR_ANGLE_PID_KD -0.20f

#define ADV_MOTOR_ANGLE_PID_MAX_OUT 70.0f
#define ADV_MOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define ADV_SET_SPEED 7000.0f

typedef enum
{
  //调试模式下各状态
	ADV_FREE,
	ADV_MOVE_F,	//向前移动
	ADV_MOVE_B,	//向后移动
	ADV_LOCK_F,	//前方锁死
	ADV_LOCK_B,	//后方锁死
	
	//比赛情况下各状态
	ADV_GAME_LAUNCH,
	ADV_GAME_INIT
	
} adv_mode_e;

typedef enum
//指示推进板前一次运动的方向
{
  ADV_DIR_NONE,	//初始化时，无方向
	ADV_DIR_F,		//向前
	ADV_DIR_B			//向后

} adv_dir_e;
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
} adv_PID_t;

typedef struct
{
  const motor_measure_t *adv_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
  int16_t give_current;
} adv_motor_t;

typedef struct
{
  const RC_ctrl_t *RC_data;               //????'??????????, the point to remote control
  adv_mode_e last_adv_mode;               //state machine. ???????????
	adv_mode_e adv_mode; 
	adv_dir_e adv_dir; 
  adv_motor_t motor_data;          //chassis motor data.??????????
	adv_PID_t adv_angle_pid;
  pid_type_def adv_speed_pid;             //motor speed PID.?????????pid
	const RC_ctrl_t *last_RC_data;

} adv_act_t;


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
extern void adv_task(void const * argument);
extern uint8_t get_adv_mode(void);
#endif
