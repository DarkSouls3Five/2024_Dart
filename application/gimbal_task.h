#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define GIMBAL_TASK_INIT_TIME 300
#define GIMBAL_CONTROL_TIME_MS 5

//Yaw轴转动pid设置
#define GIMBAL_MOTOR_SPEED_PID_KP 50.0f
#define GIMBAL_MOTOR_SPEED_PID_KI 0.0f
#define GIMBAL_MOTOR_SPEED_PID_KD -0.5f

#define GIMBAL_MOTOR_SPEED_PID_MAX_OUT 6000.0f//8000.0f
#define GIMBAL_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define GIMBAL_MOTOR_ANGLE_PID_KP 500.0f//1200.0f//1500.0f
#define GIMBAL_MOTOR_ANGLE_PID_KI 0.0f
#define GIMBAL_MOTOR_ANGLE_PID_KD -5.0f

#define GIMBAL_MOTOR_ANGLE_PID_MAX_OUT 70.0f
#define GIMBAL_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

//锁死模式下pid
#define GIMBAL_MOTOR_LOCKED_PID_KP 30000.0f
#define GIMBAL_MOTOR_LOCKED_PID_KI 10.0f
#define GIMBAL_MOTOR_LOCKED_PID_KD -500.0f

#define GIMBAL_MOTOR_LOCKED_PID_MAX_OUT 30000.0f
#define GIMBAL_MOTOR_LOCKED_PID_MAX_IOUT 0.0f

//Yaw轴ecd范围
#define MAX_GIMBAL  6080//3680//3650
#define MIN_GIMBAL  4750//2380//2370
#define MIDDLE_GIMBAL  5418//2000

//不同目标对应角度
#define MIDDLE_ANGLE 0.0//0.79000141//0.87135624f//0.79306811			//中值
#define OUTPOST_ANGLE -6.49824905		//前哨站
#define FOUNDATION_ANGLE 7.30579853	//基地

//遥控器灵敏度，分母越大灵敏度越低
#define YAW_RC_COEFF 1.57/20000.0f


typedef enum
{
  GIMBAL_FREE,			//释放yaw轴，可手动调节
	GIMBAL_CONTROL,		//遥控器控制
	GIMBAL_LOCKED,		//锁死在当前位置
} gimbal_mode_e;

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
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
	fp32 relative_angle;     //rad
  fp32 relative_angle_set; //rad
  int16_t give_current;
} gimbal_motor_t;

typedef struct
{
  const RC_ctrl_t *RC_data;               //????'??????????, the point to remote control              //state machine. ???????????
	gimbal_mode_e gimbal_mode; 
	gimbal_mode_e last_gimbal_mode; 
  gimbal_motor_t motor_data;          //chassis motor data.??????????
	gimbal_PID_t gimbal_angle_pid;
  pid_type_def gimbal_speed_pid;             //motor speed PID.?????????pid
	gimbal_PID_t gimbal_locked_pid;
	

} gimbal_act_t;

//定义云台控制变量


/**
  * @brief          "gimbal_act" valiable initialization, include pid initialization, remote control data point initialization, gimbal motor
  *                 data point initialization.
  * @param[out]     gimbal_act_init: "gimbal_act" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_act" 变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_act_init:"gimbal_act"变量指针
  * @retval         none
  */
static void gimbal_init(gimbal_act_t *gimbal_act_init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_act_t"变量指针
  * @retval         none
  */
static void gimbal_set_mode(gimbal_act_t *gimbal_act_mode);


/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_act_t *gimbal_move_data);
	

/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          云台测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_act_update:"gimbal_act"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_act_t *gimbal_act_update);


/**
  * @brief          set gimbal control set-point.
  * @param[out]     gimbal_act_control: "gimbal_act" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     runner_act_control:"runner_act"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_act_t *gimbal_act_control);


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台角度PID初始化
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear);

/*
	* @brief          由pid计算发送电流
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
static void gimbal_current_calc(gimbal_act_t *gimbal_act_current);


/*
  * @brief          云台不同控制模式对应函数，分别为自由状态、遥控器调整状态、锁死状态
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void gimbal_free_control(gimbal_act_t *gimbal_act_free);
void gimbal_rc_control(gimbal_act_t *gimbal_act_rc);
void gimbal_locked_control(gimbal_act_t *gimbal_act_locked);

/*
  * @brief          电机增量角度到输出电流转换
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void cal_from_detla_to_current(gimbal_act_t *gimbal_move_data);
	

#endif
