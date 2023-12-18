/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_locked_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-18-2023     Ignis              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "cmsis_os.h"


//引用gimbal_task中的gimbal_act指针
extern gimbal_act_t gimbal_act;


/*
	* @brief          检测摇杆状态，分别自动回到中位/瞄准基地/瞄准前哨站
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void	gimbal_locked_set(gimbal_act_t *gimbal_act_locked_set)
{
	if(gimbal_act_locked_set->gimbal_mode == GIMBAL_LOCKED)
	{
		//锁定状态下进行判断，默认锁死在GIMBAL_CONTROL模式下确定的位置，长推摇杆改变锁死位置
		
		if (gimbal_act_locked_set->RC_data->rc.ch[3] > 600)					//左摇杆向前推维持1s，归中
		{
			vTaskDelay(1000);
			if (gimbal_act_locked_set->RC_data->rc.ch[3] > 600)
			{
				gimbal_act_locked_set->motor_data.motor_angle_set = MIDDLE_ANGLE;
			}
		}
		else if (gimbal_act_locked_set->RC_data->rc.ch[2] > 600)					//左摇杆向右推维持1s，瞄准基地
		{
			vTaskDelay(1000);
			if (gimbal_act_locked_set->RC_data->rc.ch[2] > 600)
			{
				gimbal_act_locked_set->motor_data.motor_angle_set = FOUNDATION_ANGLE;
			}
		}
		else if (gimbal_act_locked_set->RC_data->rc.ch[2] < -600)					//左摇杆向左推维持1s，瞄准前哨站
		{
			vTaskDelay(1000);
			if (gimbal_act_locked_set->RC_data->rc.ch[2] < -600)
			{
				gimbal_act_locked_set->motor_data.motor_angle_set = OUTPOST_ANGLE;
			}
		}
	}
}


/**
  * @brief          gimbal_locked_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          Yaw轴锁定状态下切换锁死位置
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void gimbal_locked_task(void const * argument)
{

		//wait a time 
    vTaskDelay(GIMBAL_TASK_INIT_TIME);	
		while(1)
		{
			
			gimbal_locked_set(&gimbal_act);
			vTaskDelay(GIMBAL_CONTROL_TIME_MS);
			
		}
		
}

