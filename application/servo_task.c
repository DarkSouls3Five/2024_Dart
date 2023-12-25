/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-11-2023     Cherryblossomnight              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "runner_task.h"
#include "fric_task.h"

#define SERVO_PULL_PWM   2040	//舵机蓄力
#define SERVO_PUSH_PWM   1000	//舵机击发



extern TIM_HandleTypeDef htim1;
servo_mode_e servo_mode, last_servo_mode;
const RC_ctrl_t *RC_data;
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
	  servo_pwm_init();
		servo_pwm_set(SERVO_PULL_PWM,1);
	  RC_data = get_remote_control_point();
	  last_servo_mode = servo_mode = SERVO_PULL;

    while(1)
    {
			servo_set_mode();
			if (servo_mode == SERVO_PUSH)	
			{
				servo_pwm_set(SERVO_PUSH_PWM,1);
				vTaskDelay(1000);
			}		
			else if (servo_mode == SERVO_PULL)
			{
				servo_pwm_set(SERVO_PULL_PWM,1);
				vTaskDelay(500);
			}
			vTaskDelay(10);
    }
}

void servo_set_mode()
{
	if (last_servo_mode == SERVO_PULL && get_fric_mode() == FRIC_ON && get_runner_mode() == RUNNER_READY)
//	if (last_servo_mode == SERVO_PULL )//调试用
	{
		if (RC_data->rc.ch[1] > 600)
		{
			vTaskDelay(1000);
			if (RC_data->rc.ch[1] > 600)
				servo_mode = SERVO_PUSH;
			else
				servo_mode = SERVO_PULL;
		}
		else
			servo_mode = SERVO_PULL;
	}
	else 
		servo_mode = SERVO_PULL;
}



