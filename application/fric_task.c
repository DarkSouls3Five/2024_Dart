/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       fric_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author         	 					Modification
  *  V1.0.0     Oct-13-2023     Cherryblossomnight          1. done
  *	 V1.0.1			Apr-23-2024			Ignis												1.–¬‘ˆƒ¶≤¡¬÷◊™ÀŸ∂ØÃ¨µ˜Ω⁄£¨ø…Õ®π˝∏ƒ±‰.hŒƒº˛÷–µƒ
																														RELATIVE_SPEED≤Œ ˝µ˜Ω⁄√ø“ª∑¢Ô⁄∂‘”¶µƒ◊™ÀŸ
  @verbatim
  ==============================================================================
	*
	*
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "fric_task.h"
#include "main.h"
#include "pid.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "stdio.h"
#include "debug.h"
#include "advance_task.h"
#include "mode_set_task.h"



/**
  * @brief          "fric_act" valiable initialization, include pid initialization, remote control data point initialization, 3508 fric motors
  *                 data point initialization.
  * @param[out]     fric_act_ini: "fric_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"fric_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     fric_act_init:"fric_act"???????.
  * @retval         none
  */
static void fric_init(fric_act_t *fric_act_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ???ı??????g????????'chassis_behaviour_mode_set'??????i?
  * @param[out]     chassis_move_mode:"chassis_move"???????.
  * @retval         none
  */
static void fric_set_mode(fric_act_t *fric_act_mode);

/**
  * @brief          fric some measure data updata, such as motor speed, euler angle?? robot speed
  * @param[out]     fric_act_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????£????????????y???????????????
  * @param[out]     fric_act_update:"fric_act"???????.
  * @retval         none
  */
static void fric_feedback_update(fric_act_t *fric_act_update);
/**
  * @brief          set fric control set-point.
  * @param[out]     fric_act_control: "fric_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     fric_act_control:"fric_act"???????.
  * @retval         none
  */
static void fric_control_loop(fric_act_t *fric_act_control);

fric_act_t fric_act;
extern adv_act_t adv_act;//“˝”√Õ∆Ω¯ª˙ππ÷∏’Î
extern int dart_count;
extern dart_mode_t dart_mode;

int16_t count_j = 0;
/**
  * @brief          fric_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          »ŒŒÒ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void fric_task(void const * argument)
{
	  //wait a time 
    //????h?????
    vTaskDelay(FRIC_TASK_INIT_TIME);
    //chassis init
    //?????'??
    fric_init(&fric_act);
    //make sure all chassis motor is online
    //???????????????

	  while(1)
		{
			  //set gric control mode
        //???ı??????g?
        fric_set_mode(&fric_act);

        //fric data update
        //???????????
			  
        fric_feedback_update(&fric_act);

        //fric control pid calculate
        //???????PID????
			
//			if(count_j == 10)
//				{
//					debug_send(fric_act.motor_data[0].speed,fric_act.motor_data[1].speed,fric_act.motor_data[2].speed ,
//										fric_act.motor_data[3].speed,fric_act.motor_data[4].speed,fric_act.motor_data[5].speed);
//					count_j = 0;
//				}
//			
				fric_control_loop(&fric_act);
				count_j++;
				vTaskDelay(FRIC_CONTROL_TIME_MS);



			
		}
}

/**
  * @brief          "fric_act" valiable initialization, include pid initialization, remote control data point initialization, 3508 fric motors
  *                 data point initialization.
  * @param[out]     fric_act_ini: "fric_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"fric_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     fric_act_init:"fric_act"???????.
  * @retval         none
  */
static void fric_init(fric_act_t *fric_act_init)
{
    if (fric_act_init == NULL)
    {
        return;
    }

    //fric motor speed PID
    //????????pid?
    static const fp32 fric_left1_speed_pid[3] = {FRIC_LEFT_SPEED_PID_KP, FRIC_LEFT_SPEED_PID_KI, FRIC_LEFT_SPEED_PID_KD};
		static const fp32 fric_left2_speed_pid[3] = {FRIC_LEFT_SPEED_PID_KP, FRIC_LEFT_SPEED_PID_KI, FRIC_LEFT_SPEED_PID_KD};
		static const fp32 fric_left3_speed_pid[3] = {FRIC_LEFT_SPEED_PID_KP, FRIC_LEFT_SPEED_PID_KI, FRIC_LEFT_SPEED_PID_KD};
		
		static const fp32 fric_right1_speed_pid[3] = {FRIC_RIGHT_SPEED_PID_KP, FRIC_RIGHT_SPEED_PID_KI, FRIC_RIGHT_SPEED_PID_KD};		
		static const fp32 fric_right2_speed_pid[3] = {FRIC_RIGHT_SPEED_PID_KP, FRIC_RIGHT_SPEED_PID_KI, FRIC_RIGHT_SPEED_PID_KD};
		static const fp32 fric_right3_speed_pid[3] = {FRIC_RIGHT_SPEED_PID_KP, FRIC_RIGHT_SPEED_PID_KI, FRIC_RIGHT_SPEED_PID_KD};
    

    //???????????'
    fric_act_init->fric_mode = FRIC_OFF;
		//get remote control point
		//???????????
		fric_act_init->RC_data = get_remote_control_point();
    
    //get fric motor data point,+  initialize motor speed PID
    //??????????????????'??PID 

		fric_act_init->motor_data[0].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICL1_ID );
		PID_init(&fric_act_init->motor_speed_pid[0], PID_POSITION, fric_left1_speed_pid, FRIC_LEFT_PID_MAX_OUT, FRIC_LEFT_PID_MAX_IOUT);
		fric_act_init->motor_data[1].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICL2_ID );
		PID_init(&fric_act_init->motor_speed_pid[1], PID_POSITION, fric_left2_speed_pid, FRIC_LEFT_PID_MAX_OUT, FRIC_LEFT_PID_MAX_IOUT);
		fric_act_init->motor_data[2].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICL3_ID);
		PID_init(&fric_act_init->motor_speed_pid[2], PID_POSITION, fric_left3_speed_pid, FRIC_LEFT_PID_MAX_OUT, FRIC_LEFT_PID_MAX_IOUT);
		
		fric_act_init->motor_data[3].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICR1_ID);
		PID_init(&fric_act_init->motor_speed_pid[3], PID_POSITION, fric_right1_speed_pid, FRIC_RIGHT_PID_MAX_OUT, FRIC_RIGHT_PID_MAX_IOUT);
		fric_act_init->motor_data[4].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICR2_ID);
		PID_init(&fric_act_init->motor_speed_pid[4], PID_POSITION, fric_right2_speed_pid, FRIC_RIGHT_PID_MAX_OUT, FRIC_RIGHT_PID_MAX_IOUT);
		fric_act_init->motor_data[5].fric_motor_measure = get_motor_measure_point(1, CAN_3508_FRICR3_ID);
		PID_init(&fric_act_init->motor_speed_pid[5], PID_POSITION, fric_right3_speed_pid, FRIC_RIGHT_PID_MAX_OUT, FRIC_RIGHT_PID_MAX_IOUT);


    //update data
    //????h??????
    fric_feedback_update(fric_act_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ???ı??????g????????'chassis_behaviour_mode_set'??????i?
  * @param[out]     chassis_move_mode:"chassis_move"???????.
  * @retval         none
  */
static void fric_set_mode(fric_act_t *fric_act_mode)
{
    if (fric_act_mode == NULL)
    {
        return;
    }
		
		/*±»»¸ƒ£ Ω*/
		if(dart_mode.dart_mode == DART_GAME)
		{
			if(adv_act.adv_mode == ADV_GAME_LAUNCH )
			{
				fric_act_mode->fric_mode = FRIC_ON;
				if(adv_act.adv_mode == ADV_GAME_LAUNCH)
				{
					if(adv_act.motor_data.adv_motor_measure->distance <= 6500 && dart_count == 0)
					//∑¢…‰µ⁄“ª∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_1;
					}
					else if(adv_act.motor_data.adv_motor_measure->distance > 6500 && dart_count == 0)
					//∑¢…‰µ⁄∂˛∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_2;					
					}
					else if(adv_act.motor_data.adv_motor_measure->distance <= 6500 && dart_count == 1)
					//∑¢…‰µ⁄»˝∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_3;					
					}
					else
					//∑¢…‰µ⁄Àƒ∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_4;	
					}
				}
			}
			else 
			{
				fric_act_mode->fric_mode = FRIC_OFF;				
			}
		}

		/* ÷∂Øƒ£ Ω*/
		else
		{
			/*”“≤¶∏Àœ¬µ≤£¨ƒ¶≤¡¬÷πÿ±’*/
			if (switch_is_down(fric_act_mode->RC_data->rc.s[0]))  
			{
				fric_act_mode->fric_mode = FRIC_OFF;
			}
			
			/*”“≤¶∏À÷–µ≤£¨ƒ¶≤¡¬÷πÿ±’*/
			else if (switch_is_mid(fric_act_mode->RC_data->rc.s[0]))
			{
				fric_act_mode->fric_mode = FRIC_READY;
			}
			
			/*”“≤¶∏À…œµ≤£¨ƒ¶≤¡¬÷ø™∆Ù*/
			else if (switch_is_up(fric_act_mode->RC_data->rc.s[0]))
			{
				fric_act_mode->fric_mode = FRIC_ON;
				if(adv_act.adv_mode == ADV_GAME_LAUNCH)
				{
					if(adv_act.motor_data.adv_motor_measure->distance <= 6500 && dart_count == 0)
					//∑¢…‰µ⁄“ª∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_1;
					}
					else if(adv_act.motor_data.adv_motor_measure->distance > 6500 && dart_count == 0)
					//∑¢…‰µ⁄∂˛∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_2;					
					}
					else if(adv_act.motor_data.adv_motor_measure->distance <= 6500 && dart_count == 1)
					//∑¢…‰µ⁄»˝∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_3;					
					}
					else
					//∑¢…‰µ⁄Àƒ∑¢∑…Ô⁄
					{
						fric_act_mode->fric_mode = FRIC_ON_4;	
					}
				}
			}
		}	
}

/**
  * @brief          fric some measure data updata, such as motor speed, euler angle?? robot speed
  * @param[out]     fric_act_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????£????????????y???????????????
  * @param[out]     fric_act_update:"fric_act"???????.
  * @retval         none
  */
static void fric_feedback_update(fric_act_t *fric_act_update)
{
    if (fric_act_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 6; i++)
    {
        //update motor speed
        //???µ?????????????????PID???
        fric_act_update->motor_data[i].speed = fric_act_update->motor_data[i].fric_motor_measure->speed_rpm;
    }
}

/**
  * @brief          set fric control set-point.
  * @param[out]     fric_act_control: "fric_act" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     fric_act_control:"fric_act"???????.
  * @retval         none
  */
static void fric_control_loop(fric_act_t *fric_act_control)
{
		fp32 motor_speed = 0;
		if (fric_act_control->fric_mode == FRIC_OFF || fric_act_control->fric_mode == FRIC_READY)
		{
				for (uint8_t i = 0; i < 6; i++)
				{
					fric_act_control->motor_data[i].speed_set = 0;
					fric_act_control->motor_data[i].give_current = (int16_t)PID_calc(&fric_act_control->motor_speed_pid[i], fric_act_control->motor_data[i].speed, fric_act_control->motor_data[i].speed_set);
				}
		}
		else 
		{
			
			/*ƒ¶≤¡¬÷ø™∆Ù£¨…Ë∂®◊™ÀŸ*/
			if (fric_act_control->fric_mode == FRIC_ON)
			//ø™ƒ¶≤¡¬÷£¨‘≠ ºÀŸ∂»
			{
				motor_speed = FRIC_SET_SPEED;					
			}
			else if(fric_act_control->fric_mode == FRIC_ON_1)
			//∑¢…‰µ⁄“ª∑¢Ô⁄£¨º”…œœ‡∂‘÷µ1	
			{
				motor_speed = FRIC_SET_SPEED + RELATIVE_SPEED_1;	
			}
			else if(fric_act_control->fric_mode == FRIC_ON_2)
			//∑¢…‰µ⁄∂˛∑¢Ô⁄£¨º”…œœ‡∂‘÷µ2	
			{
				motor_speed = FRIC_SET_SPEED + RELATIVE_SPEED_2;					
			}
			else if(fric_act_control->fric_mode == FRIC_ON_3)
			//∑¢…‰µ⁄»˝∑¢Ô⁄£¨º”…œœ‡∂‘÷µ3	
			{
				motor_speed = FRIC_SET_SPEED + RELATIVE_SPEED_3;					
			}		
			else if(fric_act_control->fric_mode == FRIC_ON_4)
			//∑¢…‰µ⁄Àƒ∑¢Ô⁄£¨º”…œœ‡∂‘÷µ4	
			{
				motor_speed = FRIC_SET_SPEED + RELATIVE_SPEED_4;					
			}
			
			/*º∆À„≤¢∑¢ÀÕµÁ¡˜*/
			for (uint8_t i = 0; i < 3; i++)
			{
				fric_act_control->motor_data[i].speed_set = -motor_speed;
			}
			for (uint8_t i = 3; i < 6; i++)
			{
				fric_act_control->motor_data[i].speed_set = motor_speed;
			}
			for (uint8_t i = 0; i < 6; i++)
				{
						fric_act_control->motor_data[i].give_current = (int16_t)PID_calc(&fric_act_control->motor_speed_pid[i], fric_act_control->motor_data[i].speed, fric_act_control->motor_data[i].speed_set);
				}
		}
		
	
		/*∑¢ÀÕ¡˘∏ˆµÁª˙µƒµÁ¡˜*/
		CAN_cmd_fricl(fric_act_control->motor_data[0].give_current, fric_act_control->motor_data[1].give_current, fric_act_control->motor_data[2].give_current);
		CAN_cmd_fricr(fric_act_control->motor_data[3].give_current, fric_act_control->motor_data[4].give_current, fric_act_control->motor_data[5].give_current);


}

 uint8_t get_fric_mode(void)
{
	return fric_act.fric_mode;
}




