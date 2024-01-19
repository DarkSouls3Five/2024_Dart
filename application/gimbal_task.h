#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define GIMBAL_TASK_INIT_TIME 300
#define GIMBAL_CONTROL_TIME_MS 5

//Yaw��ת��pid����
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

//����ģʽ��pid
#define GIMBAL_MOTOR_LOCKED_PID_KP 30000.0f
#define GIMBAL_MOTOR_LOCKED_PID_KI 10.0f
#define GIMBAL_MOTOR_LOCKED_PID_KD -500.0f

#define GIMBAL_MOTOR_LOCKED_PID_MAX_OUT 30000.0f
#define GIMBAL_MOTOR_LOCKED_PID_MAX_IOUT 0.0f

//Yaw��ecd��Χ
#define MAX_GIMBAL  6080//3680//3650
#define MIN_GIMBAL  4750//2380//2370
#define MIDDLE_GIMBAL  5418//2000

//��ͬĿ���Ӧ�Ƕ�
#define MIDDLE_ANGLE 0.0//0.79000141//0.87135624f//0.79306811			//��ֵ
#define OUTPOST_ANGLE -6.49824905		//ǰ��վ
#define FOUNDATION_ANGLE 7.30579853	//����

//ң���������ȣ���ĸԽ��������Խ��
#define YAW_RC_COEFF 1.57/20000.0f


typedef enum
{
  GIMBAL_FREE,			//�ͷ�yaw�ᣬ���ֶ�����
	GIMBAL_CONTROL,		//ң��������
	GIMBAL_LOCKED,		//�����ڵ�ǰλ��
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

//������̨���Ʊ���


/**
  * @brief          "gimbal_act" valiable initialization, include pid initialization, remote control data point initialization, gimbal motor
  *                 data point initialization.
  * @param[out]     gimbal_act_init: "gimbal_act" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_act" ����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_act_init:"gimbal_act"����ָ��
  * @retval         none
  */
static void gimbal_init(gimbal_act_t *gimbal_act_init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_act_t"����ָ��
  * @retval         none
  */
static void gimbal_set_mode(gimbal_act_t *gimbal_act_mode);


/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_act_t *gimbal_move_data);
	

/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨�������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_act_update:"gimbal_act"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_act_t *gimbal_act_update);


/**
  * @brief          set gimbal control set-point.
  * @param[out]     gimbal_act_control: "gimbal_act" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     runner_act_control:"runner_act"����ָ��.
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
  * @brief          ��̨�Ƕ�PID��ʼ��
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear);

/*
	* @brief          ��pid���㷢�͵���
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
static void gimbal_current_calc(gimbal_act_t *gimbal_act_current);


/*
  * @brief          ��̨��ͬ����ģʽ��Ӧ�������ֱ�Ϊ����״̬��ң��������״̬������״̬
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void gimbal_free_control(gimbal_act_t *gimbal_act_free);
void gimbal_rc_control(gimbal_act_t *gimbal_act_rc);
void gimbal_locked_control(gimbal_act_t *gimbal_act_locked);

/*
  * @brief          ��������Ƕȵ��������ת��
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void cal_from_detla_to_current(gimbal_act_t *gimbal_move_data);
	

#endif
