/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       mode_set_task.c/h
  * @brief      ÊÖ¶¯Ä£Ê½/±ÈÈüÄ£Ê½ÇÐ»»ÈÎÎñ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-24-2024   Ignis             1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "mode_set_task.h"

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
static void mode_init(dart_mode_t *dart_mode_init);

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
static void mode_set(dart_mode_t *dart_mode_set);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????£????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */


dart_mode_t dart_mode;
extern int dart_count;
/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ºáÒÆ»ú¹¹ÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void mode_set_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(MODE_SET_TASK_INIT_TIME);
    //chassis init
    //?????'??
    mode_init(&dart_mode);
    while(1)
    {
			mode_set(&dart_mode);                    //???????????g?
			vTaskDelay(MODE_SET_TIME_MS);
			dart_mode.last_dart_mode = dart_mode.dart_mode;
			

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
static void mode_init(dart_mode_t *dart_mode_init)
{
	  if (dart_mode_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		dart_mode_init->last_dart_mode = dart_mode_init->dart_mode = DART_CONTROL;
			
		dart_mode_init->RC_data = get_remote_control_point();
		
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
static void mode_set(dart_mode_t *dart_mode_set)
{
    if (dart_mode_set == NULL)
    {
        return;
    }

		/*ÓÒÒ¡¸ËÍÆµ½×îÏÂÎ¬³Ö1s£¬½øÈë±ÈÈüÄ£Ê½*/		
		if (dart_mode_set->RC_data->rc.ch[1] < -600)  
    {
			vTaskDelay(1000);
			if (dart_mode_set->RC_data->rc.ch[1] < -600 )
			{
				if(dart_mode_set->dart_mode == DART_CONTROL)
				{
					dart_mode_set->dart_mode = DART_GAME;		
					dart_count=0;//Ã¿´Î½øÈë±ÈÈüÄ£Ê½£¬·¢Éä¼ÆÊýÇåÁã
					
				}
			}
		}
		/*×óÓÒÒ¡¸Ë¶¼ÍÆµ½×îÏÂÎ¬³Ö1s£¬ÇÒÓÒ²¦¸Ë×îÏÂ£¬½øÈëÊÖ¶¯Ä£Ê½*/	
		if (dart_mode_set->RC_data->rc.ch[1] < -600 && dart_mode_set->RC_data->rc.ch[3] < -600 )  
    {
			vTaskDelay(500);
			if (dart_mode_set->RC_data->rc.ch[1] < -600 && dart_mode_set->RC_data->rc.ch[3] < -600 && switch_is_down(dart_mode_set->RC_data->rc.s[0]) )
			{
				if(dart_mode_set->dart_mode == DART_GAME)
				{
					dart_mode_set->dart_mode = DART_CONTROL;				
				}
			}
		}
}




