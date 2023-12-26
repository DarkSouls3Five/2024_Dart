/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.3     Dec-18-2023     Ignis              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"

//消除摇杆微小扰动
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



		
/**
  * @brief          gimbal_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          飞镖架Yaw轴云台任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
	
gimbal_act_t gimbal_act;
void gimbal_task(void const * argument)
{
		
		//wait a time 
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //云台初始化
    gimbal_init(&gimbal_act);
    while(1)
    {


			gimbal_set_mode(&gimbal_act);                    //???????????g?
      gimbal_mode_change_control_transit(&gimbal_act);
			gimbal_feedback_update(&gimbal_act);            //??????????
      gimbal_control_loop(&gimbal_act);
			vTaskDelay(GIMBAL_CONTROL_TIME_MS);
			gimbal_act.last_gimbal_mode = gimbal_act.gimbal_mode;
			

    }
}

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
static void gimbal_init(gimbal_act_t *gimbal_act_init)
{
	  if (gimbal_act_init == NULL)
    {
        return;
    }

    //gimbal motor speed PID
    //????????pid?

		gimbal_act_init->last_gimbal_mode = GIMBAL_FREE;
		gimbal_act_init->gimbal_mode =GIMBAL_FREE;				//状态初始化
		
		const static fp32 motor_speed_pid[3] = {GIMBAL_MOTOR_SPEED_PID_KP, GIMBAL_MOTOR_SPEED_PID_KI, GIMBAL_MOTOR_SPEED_PID_KD};//设置PID
		
		gimbal_act_init->RC_data = get_remote_control_point();//获取遥控器指针
		gimbal_act_init->motor_data.gimbal_motor_measure = get_motor_measure_point(2, CAN_YAW_ID);//设置can线、Yaw电机ID
		
		gimbal_PID_init(&gimbal_act_init->gimbal_angle_pid, GIMBAL_MOTOR_ANGLE_PID_MAX_OUT, GIMBAL_MOTOR_ANGLE_PID_MAX_IOUT, GIMBAL_MOTOR_ANGLE_PID_KP, GIMBAL_MOTOR_ANGLE_PID_KI, GIMBAL_MOTOR_ANGLE_PID_KD);
		PID_init(&gimbal_act_init->gimbal_speed_pid, PID_POSITION, motor_speed_pid, GIMBAL_MOTOR_SPEED_PID_MAX_OUT, GIMBAL_MOTOR_SPEED_PID_MAX_IOUT);
		gimbal_PID_init(&gimbal_act_init->gimbal_locked_pid, GIMBAL_MOTOR_LOCKED_PID_MAX_OUT, GIMBAL_MOTOR_LOCKED_PID_MAX_IOUT, GIMBAL_MOTOR_LOCKED_PID_KP, GIMBAL_MOTOR_LOCKED_PID_KI, GIMBAL_MOTOR_LOCKED_PID_KD);
		
		gimbal_PID_clear(&gimbal_act_init->gimbal_angle_pid);
		PID_clear(&gimbal_act_init->gimbal_speed_pid);

		
    gimbal_feedback_update(gimbal_act_init);
		
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式
  * @param[out]     gimbal_set_mode:"gimbal_act_t"变量指针
  * @retval         none
  */
static void gimbal_set_mode(gimbal_act_t *gimbal_act_mode)
{

    if (gimbal_act_mode == NULL)
    {
        return;
    }

		//进入右拨杆控制Yaw轴调节模式
			if (switch_is_down(gimbal_act_mode->RC_data->rc.s[0]))  		//右拨杆下档，自由模式
			{
				gimbal_act_mode->gimbal_mode = GIMBAL_FREE;
			}
			else if (switch_is_mid(gimbal_act_mode->RC_data->rc.s[0]))	//右拨杆中档，遥控器调节模式
			{
				gimbal_act_mode->gimbal_mode = GIMBAL_CONTROL;
			}
			else if (switch_is_up(gimbal_act_mode->RC_data->rc.s[0]))		//右拨杆上档，锁死模式
			{
				gimbal_act_mode->gimbal_mode = GIMBAL_LOCKED;
			}
		


}

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
static void gimbal_feedback_update(gimbal_act_t *gimbal_act_update)
{
	if (gimbal_act_update == NULL)
    {
        return;
    }
		gimbal_act_update->motor_data.motor_angle = ECD2ANGLE_GIMBAL * (gimbal_act_update->motor_data.gimbal_motor_measure->ecd + ECD_RANGE * gimbal_act_update->motor_data.gimbal_motor_measure->ecd_count - MIDDLE_GIMBAL);
		gimbal_act_update->motor_data.motor_speed = gimbal_act_update->motor_data.gimbal_motor_measure->speed_rpm;
}


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
void gimbal_mode_change_control_transit(gimbal_act_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}

	if(gimbal_move_data->gimbal_mode == GIMBAL_CONTROL && 
		gimbal_move_data->last_gimbal_mode == GIMBAL_FREE)
	{
		gimbal_move_data->motor_data.motor_angle_set = gimbal_move_data->motor_data.motor_angle;
	}
	
	if(gimbal_move_data->gimbal_mode == GIMBAL_LOCKED && 
		gimbal_move_data->last_gimbal_mode != GIMBAL_LOCKED)
	{
		gimbal_move_data->motor_data.motor_angle_set = gimbal_move_data->motor_data.motor_angle;
	}
}

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
static void gimbal_control_loop(gimbal_act_t *gimbal_act_control)
{
	if(gimbal_act_control == NULL)
	{
		return;
	}
	if (gimbal_act_control->gimbal_mode ==GIMBAL_FREE)
	{
		gimbal_free_control(gimbal_act_control);
	}
	else if(gimbal_act_control->gimbal_mode ==GIMBAL_CONTROL)
	{
		gimbal_rc_control(gimbal_act_control);
	}	
	else if(gimbal_act_control->gimbal_mode ==GIMBAL_LOCKED)
	{
		gimbal_locked_control(gimbal_act_control);
	}

}


/*
  * @brief          云台不同控制模式对应控制函数，分别为自由状态、遥控器调整状态、锁死状态
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
//FREE模式
void gimbal_free_control(gimbal_act_t *gimbal_act_free)
{
	if(gimbal_act_free == NULL)
	{
		return;
	}

	gimbal_act_free->motor_data.give_current = 0;
}

//CONTROL模式
void gimbal_rc_control(gimbal_act_t *gimbal_act_rc)
{
	if(gimbal_act_rc == NULL)
	{
		return;
	}
	static int16_t yaw_channel = 0;
	
	rc_deadband_limit(gimbal_act_rc->RC_data->rc.ch[2], yaw_channel, 0);//遥控器死区，为提高微调灵敏度此处设为0
	
	
	if(gimbal_act_rc->motor_data.gimbal_motor_measure->ecd>MAX_GIMBAL&&yaw_channel>0)//电控限幅，判断是否大于最大值
	{
		gimbal_act_rc->motor_data.motor_angle_set += 0;
	}
	else if(gimbal_act_rc->motor_data.gimbal_motor_measure->ecd<MIN_GIMBAL&&yaw_channel<0)//电控限幅，判断是否小于最小值
	{
		gimbal_act_rc->motor_data.motor_angle_set += 0;
	}
	else
		gimbal_act_rc->motor_data.motor_angle_set += yaw_channel*YAW_RC_COEFF;
	
	//pid计算发送电流
		gimbal_current_calc(gimbal_act_rc);
	
}

//LOCKED模式
void gimbal_locked_control(gimbal_act_t *gimbal_act_locked)
{
	if(gimbal_act_locked == NULL)
	{
		return;
	}		
	
		//pid计算发送电流（单环）

		gimbal_act_locked->motor_data.give_current = gimbal_PID_calc(&gimbal_act_locked->gimbal_locked_pid, 
																											                gimbal_act_locked->motor_data.motor_angle, 
																											                gimbal_act_locked->motor_data.motor_angle_set, 
																											                gimbal_act_locked->motor_data.motor_speed);
}

//电机角度到输出电流转换
void cal_from_detla_to_current(gimbal_act_t *gimbal_move_data)
{
	if(gimbal_move_data ==NULL)
	{
		return;
	}
	
	gimbal_move_data->motor_data.motor_speed_set = gimbal_PID_calc(&gimbal_move_data->gimbal_angle_pid,
				gimbal_move_data->motor_data.relative_angle, gimbal_move_data->motor_data.relative_angle_set,gimbal_move_data->motor_data.gimbal_motor_measure->speed_rpm);
	gimbal_move_data->motor_data.give_current = PID_calc(&gimbal_move_data->gimbal_speed_pid, gimbal_move_data->motor_data.gimbal_motor_measure->speed_rpm, gimbal_move_data->motor_data.motor_speed_set);
	
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
uint8_t get_gimbal_mode(void)
{
	return gimbal_act.gimbal_mode;
}
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
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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


static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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

void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->last_err = gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/*
	* @brief          由pid计算发送电流
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
static void gimbal_current_calc(gimbal_act_t *gimbal_act_current)
{
		if(gimbal_act_current ==NULL)
		{
			return;
		}
		gimbal_act_current->motor_data.motor_speed_set = gimbal_PID_calc(&gimbal_act_current->gimbal_angle_pid, 
																											                gimbal_act_current->motor_data.motor_angle, 
																											                gimbal_act_current->motor_data.motor_angle_set, 
																											                gimbal_act_current->motor_data.motor_speed);
		gimbal_act_current->motor_data.give_current = (int16_t)PID_calc(&gimbal_act_current->gimbal_speed_pid,
		                                                                 gimbal_act_current->motor_data.motor_speed, 
		                                                                 gimbal_act_current->motor_data.motor_speed_set);
		
}
