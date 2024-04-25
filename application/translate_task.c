/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       translate_task.c/h
  * @brief      ���ƻ����л�����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-12-2024   Ignis             1. done
  *	 V1.0.1			April-20-2024   Ignis             1. �޸�����ģʽ�µĿ��Ʒ�ʽΪ��λ�û����Ż��ƶ�ģʽ�µ�pid
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "translate_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "advance_task.h"
#include "mode_set_task.h"


double fabs(double a)
{
	if (a >= 0)
		return a;
	else 
		return -a;
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
static void trans_init(trans_act_t *trans_act_init);

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
static void trans_set_mode(trans_act_t *trans_act_mode);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????�????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */
static void trans_feedback_update(trans_act_t *trans_act_init);
/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���ƻ���ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void trans_mode_change_control_transit(trans_act_t *trans_act_transit);

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
static void trans_control_loop(trans_act_t *trans_act_control,gimbal_act_t *gimbal_act_control,adv_act_t *adv_act_contorl);
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
static void trans_PID_init(trans_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 trans_PID_calc(trans_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

trans_act_t trans_act;
int last_s_trans; //���ڶ�ȡ��һ���󲦸�����
extern gimbal_act_t gimbal_act;
extern adv_act_t adv_act;
extern int dart_count;//��¼���ڷ������
extern dart_mode_t dart_mode;//���÷��ڷ����ģʽ�趨ָ��
/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ���ƻ�������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void translate_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(TRANS_TASK_INIT_TIME);
    //chassis init
    //?????'??
    trans_init(&trans_act);
    while(1)
    {
			trans_set_mode(&trans_act);                    //???????????g?
			trans_feedback_update(&trans_act);            //??????????
			trans_mode_change_control_transit(&trans_act);
      trans_control_loop(&trans_act,&gimbal_act,&adv_act);
			vTaskDelay(TRANS_CONTROL_TIME_MS);
			trans_act.last_trans_mode = trans_act.trans_mode;
			

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
static void trans_init(trans_act_t *trans_act_init)
{
	  if (trans_act_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		trans_act_init->last_trans_mode = trans_act_init->trans_mode = TRANS_FREE;
		const static fp32 motor_speed_pid[3] = {TRANS_MOTOR_SPEED_PID_KP, TRANS_MOTOR_SPEED_PID_KI, TRANS_MOTOR_SPEED_PID_KD};
			
		trans_act_init->RC_data = get_remote_control_point();
		trans_act_init->motor_data.trans_motor_measure = get_motor_measure_point(2, CAN_TRANS_ID);
		
		trans_PID_init(&trans_act_init->trans_angle_pid, TRANS_MOTOR_ANGLE_PID_MAX_OUT, TRANS_MOTOR_ANGLE_PID_MAX_IOUT, TRANS_MOTOR_ANGLE_PID_KP, TRANS_MOTOR_ANGLE_PID_KI, TRANS_MOTOR_ANGLE_PID_KD);
		PID_init(&trans_act_init->trans_speed_pid, PID_POSITION, motor_speed_pid, TRANS_MOTOR_SPEED_PID_MAX_OUT, TRANS_MOTOR_SPEED_PID_MAX_IOUT);
		
    trans_feedback_update(trans_act_init);
		
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
static void trans_set_mode(trans_act_t *trans_act_mode)
{
    if (trans_act_mode == NULL)
    {
        return;
    }
		
		/*����ģʽ*/
		if(dart_mode.dart_mode == DART_GAME)
		{
			
			/*����Ϊ0˵���ս������ģʽ��δ���䣬�Զ������ƶ������ұ�����*/
			if(dart_count==0 && trans_act_mode->motor_data.trans_motor_measure->given_current < -2000)
			//�����Ҽ���λ�ã������ת����������һ���̶ȣ����ұ�����
			{
				trans_act_mode->trans_mode = TRANS_LOCK_R;
			}
			else if(dart_count==0 && trans_act_mode->trans_mode != TRANS_LOCK_R)
			//�������ұ���������������ģʽ��ͬʱ�Զ������ƶ�������
			{
				trans_act_mode->trans_mode = TRANS_MOVE_R;
			}
			
			/*����Ϊ1˵��ǰ�������ڷ�����ϣ��Ƕ�С��20˵���ƽ��帴λ����ɣ����ƻ�����ʼ�Զ�����*/
			if(dart_count==1 && adv_act.motor_data.adv_motor_measure->distance < 20)
			{
				if(trans_act_mode->motor_data.trans_motor_measure->given_current > 2000)
				//��������λ�ã������ת����������һ���̶ȣ����������
				{
					trans_act_mode->trans_mode = TRANS_LOCK_L;	
				}
				else if(trans_act_mode->trans_mode != TRANS_LOCK_L)
				//��������������򲻻�������ƶ�ģʽ
				{
					trans_act_mode->trans_mode = TRANS_MOVE_L;	
				}
			}			
		}
		
		/*�ֶ�ģʽ*/
		else
		{
			/*�Ҳ����µ������ƻ�������״̬*/		
			if (switch_is_down(trans_act_mode->RC_data->rc.s[0]))  
			{
				trans_act_mode->trans_mode = TRANS_FREE;
				dart_count = 0; //downʱ�����������0
			}
			
			/*�Ҳ����ϵ�����Ħ���֣���������̨�ֲ���ģʽ*/
			else if(switch_is_up(trans_act_mode->RC_data->rc.s[0]))
			{

				if(dart_count==1 && adv_act.motor_data.adv_motor_measure->distance < 20)
				//����Ϊ1˵��ǰ�������ڷ�����ϣ��Ƕ�С��20˵���ƽ��帴λ�����
				{
					if(trans_act_mode->motor_data.trans_motor_measure->given_current > 2000)
					//��������λ�ã������ת����������һ���̶ȣ����������
					{
						trans_act_mode->trans_mode = TRANS_LOCK_L;	
					}
					else if(trans_act_mode->trans_mode != TRANS_LOCK_L)
					//��������������򲻻�������ƶ�ģʽ
					{
						trans_act_mode->trans_mode = TRANS_MOVE_L;	
					}
				}
			}
			
			/*�Ҳ����е�������ģʽ*/
			else
			{
				if (switch_is_up(trans_act_mode->RC_data->rc.s[1]) 
						&& switch_is_mid(last_s_trans) 
						&& trans_act_mode->trans_mode != TRANS_MOVE_L 
						&& trans_act_mode->last_trans_mode != TRANS_LOCK_L )
				//�ϲ�һ�Σ�����һ�β���������������ƻ������������ƶ�״̬
				{
					trans_act_mode->trans_mode = TRANS_MOVE_L;
				}
				else if(switch_is_up(trans_act_mode->RC_data->rc.s[1]) 
								&& switch_is_mid(last_s_trans) 
								&& trans_act_mode->trans_mode != TRANS_MOVE_R 
								&& trans_act_mode->last_trans_mode != TRANS_LOCK_R)
				//���ϲ�һ�Σ�����һ�β����ұ����������ƻ������������ƶ�״̬
				{
					trans_act_mode->trans_mode = TRANS_MOVE_R;			
				}
				else if(trans_act_mode->motor_data.trans_motor_measure->given_current > 2000)
				//��������λ�ã������ת����������һ���̶ȣ����������
				{
					trans_act_mode->trans_mode = TRANS_LOCK_L;
				}	
				else if(trans_act_mode->motor_data.trans_motor_measure->given_current < -2000)
				//�����Ҽ���λ�ã������ת����������һ���̶ȣ����ұ�����
				{
					trans_act_mode->trans_mode = TRANS_LOCK_R;
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
  * @brief          ???_?????????�????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */

static void trans_feedback_update(trans_act_t *trans_act_update)
{
	if (trans_act_update == NULL)
    {
        return;
    }
		trans_act_update->motor_data.motor_ecd = trans_act_update->motor_data.trans_motor_measure->ecd;//���µ��ecd
		trans_act_update->motor_data.motor_speed = trans_act_update->motor_data.trans_motor_measure->speed_rpm;
		last_s_trans = trans_act_update->RC_data->rc.s[1];
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���ƻ���ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void trans_mode_change_control_transit(trans_act_t *trans_act_transit)
{
	if(trans_act_transit == NULL)
	{
		return;
	}

	if(trans_act_transit->trans_mode == TRANS_LOCK_L && 
		trans_act_transit->last_trans_mode == TRANS_MOVE_L)
	{
		trans_act_transit->motor_data.motor_ecd_set = trans_act_transit->motor_data.motor_ecd;
	}
	
	if(trans_act_transit->trans_mode == TRANS_LOCK_R && 
		trans_act_transit->last_trans_mode == TRANS_MOVE_R)
	{
		trans_act_transit->motor_data.motor_ecd_set = trans_act_transit->motor_data.motor_ecd;
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

static void trans_control_loop(trans_act_t *trans_act_control,gimbal_act_t *gimbal_act_control,adv_act_t *adv_act_control)
{
	//
	static fp32 motor_speed = 0;
	
	/*����״̬*/
	if (trans_act_control->trans_mode == TRANS_FREE)
		{
			trans_act_control->motor_data.give_current = 0;
		}
	
	
	else 
	{
		/*�����ƶ�״̬,�󲦸�ÿ�ϲ�һ�λ�һ���˶�����*/
		if (trans_act_control->trans_mode == TRANS_MOVE_R)
		{
			motor_speed = -TRANS_SET_SPEED;
			trans_act_control->motor_data.motor_speed_set = motor_speed;
			trans_act_control->motor_data.give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																												trans_act_control->motor_data.motor_speed, trans_act_control->motor_data.motor_speed_set);
		}
		
		/*�����ƶ�״̬*/
		else if(trans_act_control->trans_mode == TRANS_MOVE_L)
		{
			motor_speed = TRANS_SET_SPEED;//���͵ȴ������
			trans_act_control->motor_data.motor_speed_set = motor_speed;
			trans_act_control->motor_data.give_current = (int16_t)PID_calc(&trans_act_control->trans_speed_pid, 
																												trans_act_control->motor_data.motor_speed, trans_act_control->motor_data.motor_speed_set);			
		}
		
		/*����״̬��ʹ�ýǶȻ���������*/
		else if (trans_act_control->trans_mode == TRANS_LOCK_L || trans_act_control->trans_mode == TRANS_LOCK_R)
		{
				trans_act_control->motor_data.give_current = trans_PID_calc(&trans_act_control->trans_angle_pid, 
																											                trans_act_control->motor_data.motor_ecd, 
																											                trans_act_control->motor_data.motor_ecd_set, 
																											                trans_act_control->motor_data.motor_speed);
		}
	}
	
	CAN_cmd_can2(gimbal_act_control->motor_data.give_current,trans_act_control->motor_data.give_current,adv_act_control->motor_data.give_current);

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
uint8_t get_trans_mode(void)
{
	return trans_act.trans_mode;
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
static void trans_PID_init(trans_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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


static fp32 trans_PID_calc(trans_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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




