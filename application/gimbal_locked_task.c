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


//����gimbal_task�е�gimbal_actָ��
extern gimbal_act_t gimbal_act;


/*
	* @brief          ���ҡ��״̬���ֱ��Զ��ص���λ/��׼����/��׼ǰ��վ
  * @param[out]     gimbal_init:"gimbal_control"???????.
  * @retval         none
*/
void	gimbal_locked_set(gimbal_act_t *gimbal_act_locked_set)
{
	if(gimbal_act_locked_set->gimbal_mode == GIMBAL_LOCKED)
	{
		//����״̬�½����жϣ�Ĭ��������GIMBAL_CONTROLģʽ��ȷ����λ�ã�����ҡ�˸ı�����λ��
		
		if (gimbal_act_locked_set->RC_data->rc.ch[3] > 600)					//��ҡ����ǰ��ά��1s������
		{
			vTaskDelay(1000);
			if (gimbal_act_locked_set->RC_data->rc.ch[3] > 600)
			{
				gimbal_act_locked_set->motor_data.motor_angle_set = MIDDLE_ANGLE;
			}
		}
		else if (gimbal_act_locked_set->RC_data->rc.ch[2] > 600)					//��ҡ��������ά��1s����׼����
		{
			vTaskDelay(1000);
			if (gimbal_act_locked_set->RC_data->rc.ch[2] > 600)
			{
				gimbal_act_locked_set->motor_data.motor_angle_set = FOUNDATION_ANGLE;
			}
		}
		else if (gimbal_act_locked_set->RC_data->rc.ch[2] < -600)					//��ҡ��������ά��1s����׼ǰ��վ
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
  * @brief          Yaw������״̬���л�����λ��
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

