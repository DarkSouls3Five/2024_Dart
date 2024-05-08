/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       advance_task.c/h
  * @brief      advance control task
								�ư巢���������     
	
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2024   	Ignis           1. ��������ʵ��
  *  V1.1.0     Apr-15-2024   	Ignis           1. ���ӱ���ʱ��̨���Զ�����ģʽ
  *  V1.1.1     Apr-20-2024   	Ignis           1. �󲦸��е���ʹ�ư�ͣ��
	*	 V1.1.2			Apr-22-2024			Ignis						1. ˿�˵�������߼��޸ģ���Ϊ���ԽǶȿ���
	* @note  
  ==============================================================================
	*	�����Ϸ��ڿ���ȫ���̣�
	
	*	һ���׾�������׼��ʱ��-������Ա
	*		1�����ú÷���ϵͳ:
	* 		�����ڼܰᵽ����վ�� -> ��λ���򿪴�������
	*		2�����磬װ���ʼ��:
	*			���ӳ������� -> �ϵ� -> ����ģʽ�£��Ҳ����е��������ƽ��帴λ����� -> DOWNģʽ�£��Ҳ����µ����´ӷ�����ڽ���ö����װ�� 
	*		3������Yaw��λ��:
	*			����̨�ƽ�ȥ -> DOWNģʽ���Ҳ����µ������ֶ�����Yaw��λ�� -> �Ҳ������³���1s�������ģʽ -> ��ң����������̨��
	*
	*	����������-��̨��
	*		1����һ�η���:
	*			(1) �ֶ����̣�ѡ�ֶ�ѡ���բ�� -> �ȴ�բ����ȫ�� -> ��ǰ����ҡ��ά��1s������2ö����
	*			(2) �Զ����̣��ƽ����˶���ǰ����λ�ã������ö���� -> �ƽ��帴λ������λ�� -> ���ƻ����л�����һ�����
	*		2���ڶ��η��䣺
	*			(1) �ֶ����̣�ѡ�ֶ�ѡ���բ�� -> �ȴ�բ����ȫ�� -> ��ǰ����ҡ��ά��1s������2ö����
	*			(2) �Զ����̣��ƽ����˶���ǰ����λ�ã������ö���� -> ͣ��ǰ����λ��
	* 
	* �����ּ�������׼��ʱ��
	*		1��������Ա����ڣ�ͬʱ��̨�ְ�ң�����ø�������Ա
	*		2���Ҳ������³���1s�˳�����ģʽ -> ��λ�ƽ��� -> װ����� -> �ֶ�����Yaw�� -> �Ҳ������³���1s�������ģʽ -> ��ң����������̨��
	
	##ע�������
	*		˿�˵���λ���ϵ�ʱ��λ�ã�����ÿ���ϵ�ǰ����ֶ���˿��ת����󷽵�λ�ã��������а����������õķ��գ������ע�⣡��
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
int dart_count = 0;//ͳ�Ʒ������

/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �ƽ���������
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

		//��¼�ƽ������ʼecd
		init_ecd_record(&motor_data[8]);
		//��ʼ���ƽ�����״̬
		adv_act_init->last_adv_mode = adv_act_init->adv_mode = ADV_FREE;
		//��ʼ���ƽ������˶�����
		adv_act_init->adv_dir = ADV_DIR_NONE;
		
		//��ȡң����ָ��		
		adv_act_init->RC_data = get_remote_control_point();
		//��ȡ�ƽ��������ָ��	
		adv_act_init->motor_data.adv_motor_measure = get_motor_measure_point(2, CAN_ADV_ID);
		
		//��ʼ��װ�����pid	
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
		
		/*����ģʽ*/
		if(dart_mode.dart_mode == DART_GAME)
		{
			if (adv_act_mode->RC_data->rc.ch[1] > 600 && (trans_act.trans_mode == TRANS_LOCK_R || trans_act.trans_mode == TRANS_LOCK_L))
			//���ƻ�����λ����ҡ��ǰ��ά��1s��������ö����
			{
				vTaskDelay(1000);
				if (adv_act_mode->RC_data->rc.ch[1] > 600)
				{
					adv_act_mode->adv_mode = ADV_GAME_LAUNCH;
				}
			}
//			else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && trans_act.trans_mode != TRANS_LOCK_L)
			else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && adv_act_mode->adv_mode == ADV_REACH_F)
			//dart_count��LAUNCHģʽ�»���1����0��1�����Ѿ������ö���ڷ��䣬�ƽ��忪ʼ�Զ���λ
			{
				adv_act_mode->adv_mode = ADV_GAME_INIT;
			}
		}			

		/*�ֶ�ģʽ*/
		else
		{
			/*�Ҳ����µ����ƽ��������ɣ�������״̬*/
			if (switch_is_down(adv_act_mode->RC_data->rc.s[0]))  

			{
				adv_act_mode->adv_mode = ADV_FREE;
			}

			
			/*�Ҳ����ϵ�����Ħ���֣���������̨�ֲ���ģʽ���Զ�װ���뷢��*/
			else if(switch_is_up(adv_act_mode->RC_data->rc.s[0]))
			{	
				if (adv_act_mode->RC_data->rc.ch[1] > 600 && (trans_act.trans_mode == TRANS_LOCK_R || trans_act.trans_mode == TRANS_LOCK_L))
				//���ƻ�����λ����ҡ��ǰ��ά��1s��������ö����
				{
					vTaskDelay(1000);
					if (adv_act_mode->RC_data->rc.ch[1] > 600)
					{
						adv_act_mode->adv_mode = ADV_GAME_LAUNCH;
					}
				}
//				else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE && trans_act.trans_mode != TRANS_LOCK_L)
				else if(dart_count==1 && adv_act_mode->adv_mode != ADV_FREE )				
				//dart_count��LAUNCHģʽ�»���1����downʱ���㣬��0��1�����Ѿ������ö���ڷ��䣬�ƽ��忪ʼ�Զ���λ
				{
					adv_act_mode->adv_mode = ADV_GAME_INIT;
				}
			}
		
			
			/*�Ҳ����е����ֶ������ư��˶�*/
			else
			{
				//�󲦸��е����ư�ֹͣ�˶�
				if(switch_is_mid(adv_act_mode->RC_data->rc.s[1]))
				{
					adv_act_mode->adv_mode = ADV_FREE;				
				}
				
				//�²�һ�Σ�����һ�β��ں���������һ���˶����򲻳����ƽ������������״̬
				else if (switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
						&& switch_is_mid(last_s_adv) 
						&& adv_act_mode->adv_dir != ADV_DIR_B
						&& adv_act_mode->last_adv_mode != ADV_LOCK_B)
				{
					adv_act_mode->adv_mode = ADV_MOVE_B;
				}
				
				//���²�һ�Σ�����һ�β���ǰ����������һ���˶����򲻳�ǰ�����ƻ�������ǰ��״̬		
				else if(switch_is_down(adv_act_mode->RC_data->rc.s[1]) 
								&& switch_is_mid(last_s_adv) 
								&& adv_act_mode->adv_dir != ADV_DIR_F
								&& adv_act_mode->last_adv_mode != ADV_LOCK_F)
				{
					adv_act_mode->adv_mode = ADV_MOVE_F;			
				}
				
				//����ǰ����λ�ã������ת����������һ���̶ȣ��Զ���������			
				else if(adv_act_mode->motor_data.adv_motor_measure->given_current > 6000 )
				{
					adv_act_mode->adv_mode = ADV_LOCK_F;
				}	
				else if(adv_act_mode->motor_data.adv_motor_measure->given_current < -6000 )
				//�������λ�ã������ת����������һ���̶ȣ��Զ���������
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
	
	/*����״̬��ǰ�������������������������*/
	if (adv_act_control->adv_mode == ADV_FREE || adv_act_control->adv_mode == ADV_LOCK_F || adv_act_control->adv_mode == ADV_LOCK_B)
		{
			adv_act_control->motor_data.give_current = 0;
		}
	
	else 
	{
		/*��ǰ�ƶ�״̬,�󲦸�ÿ�²�һ�λ�һ���˶�����*/
		if (adv_act_control->adv_mode == ADV_MOVE_F)
		{
			//�������Ϊ��ǰ
			adv_act_control->adv_dir = ADV_DIR_F;
			//�Թ̶��ٶ��ƶ�
			motor_speed = ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);
			//�ж��Ƿ񵽴�ǰ��ȫλ�ã��Զ���ǰ���ƶ����趨λ��			
			if(adv_act_control->motor_data.adv_motor_measure->distance > ADV_SAFE_ANGLE_F - 300.0f)
			{
				adv_act_control->adv_mode = ADV_REACH_F;
			}
		}
		
		/*����ƶ�״̬*/
		else if(adv_act_control->adv_mode == ADV_MOVE_B)
		{
			//�������Ϊ����
			adv_act_control->adv_dir = ADV_DIR_B;
			//���͵ȴ������
			motor_speed = -ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);			
			//�ж��Ƿ񵽴��ȫλ�ã��Զ��ں��ƶ����趨λ��			
			if(adv_act_control->motor_data.adv_motor_measure->distance < ADV_SAFE_ANGLE_B + 300.0f)
			{
				adv_act_control->adv_mode = ADV_REACH_B;
			}
		}
		
		/*������ƶ�ģʽ��λ�û����ƻص���λ*/
		else if(adv_act_control->adv_mode == ADV_REACH_B)
		{
			adv_act_control->motor_data.give_current = adv_PID_calc(&adv_act_control->adv_angle_pid, 
																											                adv_act_control->motor_data.adv_motor_measure->distance, 
																											                ADV_SAFE_ANGLE_B, 
																											                adv_act_control->motor_data.motor_speed);		
		}
	
		/*����ǰ���ƶ�ģʽ��λ�û����ƻص�ǰ����λ��*/
		else if(adv_act_control->adv_mode == ADV_REACH_F)
		{
			adv_act_control->motor_data.give_current = adv_PID_calc(&adv_act_control->adv_angle_pid, 
																											                adv_act_control->motor_data.adv_motor_measure->distance, 
																											                ADV_SAFE_ANGLE_F, 
																											                adv_act_control->motor_data.motor_speed);		
		}	
		
		/*�����Զ�����״̬��һ�η���2ö����*/
		else if(adv_act_control->adv_mode == ADV_GAME_LAUNCH)
		{
			motor_speed = ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);
			
			if(adv_act_control->motor_data.adv_motor_measure->distance > ADV_SAFE_ANGLE_F)
			//����ǰ��ȫλ�ã�˵��ǰ2ö���ڷ������
			{
				dart_count = 1 ;			
				adv_act_control->adv_mode = ADV_REACH_F;
				
			}
		}

		/*������Ϻ��ƽ��帴λ*/
		else if(adv_act_control->adv_mode == ADV_GAME_INIT)
		{
			//�Թ̶��ٶ��ƶ�
			motor_speed = -ADV_SET_SPEED;
			adv_act_control->motor_data.motor_speed_set = motor_speed;
			adv_act_control->motor_data.give_current = (int16_t)PID_calc(&adv_act_control->adv_speed_pid, 
																												adv_act_control->motor_data.motor_speed, adv_act_control->motor_data.motor_speed_set);

			if(adv_act_control->motor_data.adv_motor_measure->distance < ADV_SAFE_ANGLE_B + 300.0f)
			//�����ȫλ�ã����к�λ
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





