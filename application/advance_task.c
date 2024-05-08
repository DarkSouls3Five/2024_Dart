/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       advance_task.c/h
  * @brief      advance control task
								推板发射飞镖任务     
	
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2024   	Ignis           1. 基本功能实现
  *  V1.1.0     Apr-15-2024   	Ignis           1. 增加比赛时云台手自动发射模式
  *  V1.1.1     Apr-20-2024   	Ignis           1. 左拨杆中挡可使推板停下
	*	 V1.1.2			Apr-22-2024			Ignis						1. 丝杆电机控制逻辑修改，改为绝对角度控制
	* @note  
  ==============================================================================
	*	赛场上飞镖控制全流程：
	
	*	一、首局三分钟准备时间-场地人员
	*		1、放置好飞镖系统:
	* 		将飞镖架搬到发射站内 -> 定位，打开磁吸开关
	*		2、开电，装填初始化:
	*			连接场地主控 -> 上电 -> 调试模式下（右拨杆中档）控制推进板复位到后端 -> DOWN模式下（右拨杆下挡）下从发射出口将四枚飞镖装填 
	*		3、调节Yaw轴位置:
	*			将滑台推进去 -> DOWN模式（右拨杆下挡）下手动调节Yaw轴位置 -> 右拨杆在下长推1s进入比赛模式 -> 把遥控器交给云台手
	*
	*	二、比赛中-云台手
	*		1、第一次发射:
	*			(1) 手动流程：选手端选择打开闸门 -> 等待闸门完全打开 -> 向前推右摇杆维持1s，发射2枚飞镖
	*			(2) 自动流程：推进板运动到前极限位置，射出两枚飞镖 -> 推进板复位到后极限位置 -> 横移机构切换到下一组飞镖
	*		2、第二次发射：
	*			(1) 手动流程：选手端选择打开闸门 -> 等待闸门完全打开 -> 向前推右摇杆维持1s，发射2枚飞镖
	*			(2) 自动流程：推进板运动到前极限位置，射出两枚飞镖 -> 停在前极限位置
	* 
	* 三、局间三分钟准备时间
	*		1、场地人员捡飞镖，同时云台手把遥控器拿给场地人员
	*		2、右拨杆在下长推1s退出比赛模式 -> 复位推进板 -> 装填飞镖 -> 手动调节Yaw轴 -> 右拨杆在下长推1s进入比赛模式 -> 把遥控器交给云台手
	
	##注意事项！！
	*		丝杆的零位是上电时的位置，所以每次上电前务必手动把丝杆转到最后方的位置！！否则有把联轴器撅烂的风险！！务必注意！！
	*		

	
  ==============================================================================
	
  @verbatim

  @endverbatim
  ****************************(C) COPYRIGHT 2024 TJSP****************************
  */

#include "advance_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "translate_task.h"
#include "mode_set_task.h"


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
extern void init_ecd_record(motor_measure_t *motor_2006);

adv_act_t adv_act;
extern gimbal_act_t gimbal_act;
extern trans_act_t trans_act;
extern motor_measure_t motor_data[9];
extern dart_mode_t dart_mode;
int last_s_adv;
int dart_count = 0;//统计发射次数

/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          推进机构任务
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

		//记录推进电机初始ecd
		init_ecd_record(&motor_data[8]);
		//初始化推进机构状态
		adv_act_init->last_adv_mode = adv_act_init->adv_mode = ADV_FREE;
		//初始化推进机构运动方向
		adv_act_init->adv_dir = ADV_DIR_NONE;
		
		//获取遥控器指针		
		adv_act_init->RC_data = get_remote_control_point();
		//获取推进机构电机指针	
		adv_act_init->motor_data.adv_motor_measure = get_motor_measure_point(2, CAN_ADV_ID);
		
		//初始化装填机构pid	
		const static fp32 motor_speed_pid[3] = {ADV_MOTOR_SPEED_PID_KP, ADV_MOTOR_SPEED_PID_KI, ADV_MOTOR_SPEED_PID_KD};		
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
		
		/*比赛模式*/
		if(dart_mode.dart_mode == DART_GAME)
		{
			if (adv_act_mode->RC_data->rc.ch[1] > 600 && (trans_act.trans_mode == TRANS_LOCK_R || trans_act.trans_mode == TRANS_LOCK_L))
			//横移机构到位，右摇杆前推维持1s，发射两枚飞镖
			{
				vTaskDelay(1000);
				if (adv_act_mode->RC_data->rc.ch[1] > 600)
				{
					adv_act_mode->adv_mode = ADV_GAME_LAUNCH;
				}
			}
//			else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && trans_act.trans_mode != TRANS_LOCK_L)
			else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && adv_act_mode->adv_mode == ADV_REACH_F)
			//dart_count在LAUNCH模式下会变成1，由0变1代表已经完成两枚飞镖发射，推进板开始自动复位
			{
				adv_act_mode->adv_mode = ADV_GAME_INIT;
			}
		}			

		/*手动模式*/
		else
		{
			/*右拨杆下挡，推进机构自由（自锁）状态*/
			if (switch_is_down(adv_act_mode->RC_data->rc.s[0]))  

			{
				adv_act_mode->adv_mode = ADV_FREE;
			}

			
			/*右拨杆上档，开摩擦轮，比赛中云台手操作模式，自动装填与发射*/
			else if(switch_is_up(adv_act_mode->RC_data->rc.s[0]))
			{	
				if (adv_act_mode->RC_data->rc.ch[1] > 600 && (trans_act.trans_mode == TRANS_LOCK_R || trans_act.trans_mode == TRANS_LOCK_L))
				//横移机构到位，右摇杆前推维持1s，发射两枚飞镖
				{
					vTaskDelay(1000);
					if (adv_act_mode->RC_data->rc.ch[1] > 600)
					{
						adv_act_mode->adv_mode = ADV_GAME_LAUNCH;
					}
				}
//				else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && trans_act.trans_mode != TRANS_LOCK_L)
				else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE )				
				//dart_count在LAUNCH模式下会变成1，在down时清零，由0变1代表已经完成两枚飞镖发射，推进板开始自动复位
				{
					adv_act_mode->adv_mode = ADV_GAME_INIT;
				}
			}
		
			
			/*右拨杆中挡，手动控制推板运动*/
			else
			{
				//左拨杆中挡，推板停止运动
				if(switch_is_mid(adv_act_mode->RC_data->rc.s[1]))
				{
					adv_act_mode->adv_mode = ADV_FREE;				
				}
				
				//下拨一次，且上一次不在后方锁死，上一次运动方向不朝后，推进机构进入后退状态
				else if (switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
						&& switch_is_mid(last_s_adv) 
						&& adv_act_mode->adv_dir != ADV_DIR_B
						&& adv_act_mode->last_adv_mode != ADV_LOCK_B)
				{
					adv_act_mode->adv_mode = ADV_MOVE_B;
				}
				
				//再下拨一次，且上一次不在前方锁死，上一次运动方向不朝前，横移机构进入前推状态		
				else if(switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
								&& switch_is_mid(last_s_adv) 
								&& adv_act_mode->adv_dir != ADV_DIR_F
								&& adv_act_mode->last_adv_mode != ADV_LOCK_F)
				{
					adv_act_mode->adv_mode = ADV_MOVE_F;			
				}
				
				//到达前极限位置，电机堵转，电流增大到一定程度，自动锁紧保护			
				else if(adv_act_mode->motor_data.adv_motor_measure->given_current > 6000 )
				{
					adv_act_mode->adv_mode = ADV_LOCK_F;
				}	
				else if(adv_act_mode->motor_data.adv_motor_measure->given_current < -6000 )
				//到达后极限位置，电机堵转，电流增大到一定程度，自动锁紧保护
				{
					adv_act_mode->adv_mode = ADV_LOCK_B;
				}	
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
	
	/*自由状态、前端自锁、后端自锁，不发电流*/
	if (adv_act_control->adv_mode == ADV_FREE || adv_act_control->adv_mode == ADV_LOCK_F || adv_act_control->adv_mode == ADV_LOCK_B)
		{
			adv_act_control->motor_data.give_current = 0;
		}
	
	else 
	{
		/*向前移动状态,左拨杆每下拨一次换一次运动方向*/
		if (adv_act_control->adv_mode == ADV_MOVE_F)
		{
			//将方向改为朝前
			adv_act_control->adv_dir = ADV_DIR_F;
			//以固定速度移动
			motor_speed = ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);
			//判断是否到达前安全位置，自动在前方制动到设定位置			
			if(adv_act_control->motor_data.adv_motor_measure->distance > ADV_SAFE_ANGLE_F - 300.0f)
			{
				adv_act_control->adv_mode = ADV_REACH_F;
			}
		}
		
		/*向后移动状态*/
		else if(adv_act_control->adv_mode == ADV_MOVE_B)
		{
			//将方向改为朝后
			adv_act_control->adv_dir = ADV_DIR_B;
			//发送等大反向电流
			motor_speed = -ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);			
			//判断是否到达后安全位置，自动在后方制动到设定位置			
			if(adv_act_control->motor_data.adv_motor_measure->distance < ADV_SAFE_ANGLE_B + 300.0f)
			{
				adv_act_control->adv_mode = ADV_REACH_B;
			}
		}
		
		/*进入后方制动模式，位置环控制回到零位*/
		else if(adv_act_control->adv_mode == ADV_REACH_B)
		{
			adv_act_control->motor_data.give_current = adv_PID_calc(&adv_act_control->adv_angle_pid, 
																											                adv_act_control->motor_data.adv_motor_measure->distance, 
																											                ADV_SAFE_ANGLE_B, 
																											                adv_act_control->motor_data.motor_speed);		
		}
	
		/*进入前方制动模式，位置环控制回到前极限位置*/
		else if(adv_act_control->adv_mode == ADV_REACH_F)
		{
			adv_act_control->motor_data.give_current = adv_PID_calc(&adv_act_control->adv_angle_pid, 
																											                adv_act_control->motor_data.adv_motor_measure->distance, 
																											                ADV_SAFE_ANGLE_F, 
																											                adv_act_control->motor_data.motor_speed);		
		}	
		
		/*比赛自动发射状态，一次发射2枚飞镖*/
		else if(adv_act_control->adv_mode == ADV_GAME_LAUNCH)
		{
			motor_speed = ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);
			
			if(adv_act_control->motor_data.adv_motor_measure->distance > ADV_SAFE_ANGLE_F)
			//到达前安全位置，说明前2枚飞镖发射完成
			{
				dart_count = 1 ;			
				adv_act_control->adv_mode = ADV_REACH_F;
				
			}
		}

		/*发射完毕后推进板复位*/
		else if(adv_act_control->adv_mode == ADV_GAME_INIT)
		{
			//以固定速度移动
			motor_speed = -ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);

			if(adv_act_control->motor_data.adv_motor_measure->distance < ADV_SAFE_ANGLE_B + 300.0f)
			//到达后安全位置，进行后复位
			{
					adv_act_control->motor_data.give_current =0;
			}
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





