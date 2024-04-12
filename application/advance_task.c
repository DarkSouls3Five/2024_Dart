/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       runner_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-12-2024   Ignis              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "advance_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"


//double fabs(double a)
//{
//	if (a >= 0)
//		return a;
//	else 
//		return -a;
//}

/**
  * @brief          "runner_act" valiable initialization, include pid initialization, remote control data point initialization, runner motor
  *                 data point initialization.
  * @param[out]     runner_act_init: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"runner_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     runner_act_init:"runner_act"???????.
  * @retval         none
  */
static void adv_init(adv_act_t *adv_act_init);

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void adv_set_mode(adv_act_t *adv_act_mode);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */
static void adv_feedback_update(adv_act_t *adv_act_init);

/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */
static void adv_control_loop(adv_act_t *adv_act_control);
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"gimbal_control"??????????pid??'???? ?????????'??????????????'????????????????'??
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void adv_PID_init(adv_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 adv_PID_calc(adv_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

adv_act_t adv_act;
extern gimbal_act_t gimbal_act;
int last_s_adv;
/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void advance_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(ADV_TASK_INIT_TIME);
    //chassis init
    //?????'??
    adv_init(&adv_act);
    while(1)
    {
			adv_set_mode(&adv_act);                    //???????????g?
      adv_feedback_update(&adv_act);            //??????????
      adv_control_loop(&adv_act);
			vTaskDelay(ADV_CONTROL_TIME_MS);
			adv_act.last_adv_mode = adv_act.adv_mode;
			

    }
}

/**
  * @brief          "runner_act" valiable initialization, include pid initialization, remote control data point initialization, runner motor
  *                 data point initialization.
  * @param[out]     runner_act_init: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"runner_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     runner_act_init:"runner_act"???????.
  * @retval         none
  */
static void adv_init(adv_act_t *adv_act_init)
{
	  if (adv_act_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		adv_act_init->last_adv_mode = adv_act_init->adv_mode = ADV_FREE;
		const static fp32 motor_speed_pid[3] = {ADV_MOTOR_SPEED_PID_KP, ADV_MOTOR_SPEED_PID_KI, ADV_MOTOR_SPEED_PID_KD};
			
		adv_act_init->RC_data = get_remote_control_point();
		adv_act_init->motor_data.adv_motor_measure = get_motor_measure_point(2, CAN_ADV_ID);
		
		adv_PID_init(&adv_act_init->adv_angle_pid, ADV_MOTOR_ANGLE_PID_MAX_OUT, ADV_MOTOR_ANGLE_PID_MAX_IOUT, ADV_MOTOR_ANGLE_PID_KP, ADV_MOTOR_ANGLE_PID_KI, ADV_MOTOR_ANGLE_PID_KD);
		PID_init(&adv_act_init->adv_speed_pid, PID_POSITION, motor_speed_pid, ADV_MOTOR_SPEED_PID_MAX_OUT, ADV_MOTOR_SPEED_PID_MAX_IOUT);
		
    adv_feedback_update(adv_act_init);
		
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void adv_set_mode(adv_act_t *adv_act_mode)
{
    if (adv_act_mode == NULL)
    {
        return;
    }	
		
		if (switch_is_down(adv_act_mode->RC_data->rc.s[0]))  
		//右拨杆下挡，推进机构自由（自锁）状态
    {
			adv_act_mode->adv_mode = ADV_FREE;
		}
		else
		{
			if (switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
					&& switch_is_mid(last_s_adv) 
					&& adv_act_mode->adv_mode != ADV_MOVE_B
					&& adv_act_mode->last_adv_mode != ADV_LOCK_B)
			//下拨一次，且上一次不在后方锁死，推进机构进入后退状态
			{
				adv_act_mode->adv_mode = ADV_MOVE_B;
			}
			else if(switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
							&& switch_is_mid(last_s_adv) 
							&& adv_act_mode->adv_mode != ADV_MOVE_F
							&& adv_act_mode->last_adv_mode != ADV_LOCK_F)
			//再下拨一次，且上一次不在前方锁死，横移机构进入前推状态
			{
				adv_act_mode->adv_mode = ADV_MOVE_F;			
			}
			else if(adv_act_mode->motor_data.adv_motor_measure->given_current > 5000 )
			//到达前极限位置，电机堵转，电流增大到一定程度，自动锁紧
			{
				adv_act_mode->adv_mode = ADV_LOCK_F;
			}	
			else if(adv_act_mode->motor_data.adv_motor_measure->given_current < -5000 )
			//到达后极限位置，电机堵转，电流增大到一定程度，自动锁紧
			{
				adv_act_mode->adv_mode = ADV_LOCK_B;
			}	
		}
}

/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */

static void adv_feedback_update(adv_act_t *adv_act_update)
{
	if (adv_act_update == NULL)
    {
        return;
    }
		adv_act_update->motor_data.motor_speed = adv_act_update->motor_data.adv_motor_measure->speed_rpm;
		last_s_adv = adv_act_update->RC_data->rc.s[1];
}

/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */

static void adv_control_loop(adv_act_t *adv_act_control)
{
	static fp32 motor_speed = 0;
	if (adv_act_control->adv_mode == ADV_FREE || adv_act_control->adv_mode == ADV_LOCK_F || adv_act_control->adv_mode == ADV_LOCK_B)
	//自由状态、前端自锁、后端自锁，不发电流
		{
			adv_act_control->motor_data.give_current = 0;
		}
	else 
	{
		if (adv_act_control->adv_mode == ADV_MOVE_F)
		//向前移动状态,左拨杆每下拨一次换一次运动方向
		{
			motor_speed = ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);
		}
		else if(adv_act_control->adv_mode == ADV_MOVE_B)
		//向后移动状态
		{
			motor_speed = -ADV_SET_SPEED;//发送等大反向电流
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);			
		}
	}
}


/**
  * @brief          set runner control set-point.
  * @param[out]     runner_act_control: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     runner_act_control:"runner_act"???????.
  * @retval         none
  */
uint8_t get_adv_mode(void)
{
	return adv_act.adv_mode;
}
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"gimbal_control"??????????pid??'???? ?????????'??????????????'????????????????'??
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
  */
static void adv_PID_init(adv_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}


static fp32 adv_PID_calc(adv_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {             
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}




