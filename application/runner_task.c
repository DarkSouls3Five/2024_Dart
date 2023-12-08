/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       runner_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-15-2023     Cherryblossomnight              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "runner_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_bus.h"
#include "remote_control.h"
#include "user_lib.h"

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
static void runner_init(runner_act_t *runner_act_init);

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
static void runner_set_mode(runner_act_t *runner_act_mode);
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
static void runner_feedback_update(runner_act_t *runner_act_init);

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
static void runner_control_loop(runner_act_t *runner_act_control);
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
static void runner_PID_init(runner_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 runner_PID_calc(runner_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

runner_act_t runner_act;

/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void runner_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(RUNNER_TASK_INIT_TIME);
    //chassis init
    //?????'??
    runner_init(&runner_act);
    while(1)
    {
			runner_set_mode(&runner_act);                    //???????????g?
      runner_feedback_update(&runner_act);            //??????????
      runner_control_loop(&runner_act);
			vTaskDelay(RUNNER_CONTROL_TIME_MS);
			runner_act.last_runner_mode = runner_act.runner_mode;
			

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
static void runner_init(runner_act_t *runner_act_init)
{
	  if (runner_act_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		runner_act_init->last_runner_mode = runner_act_init->runner_mode = RUNNER_DOWN;
		const static fp32 motor_speed_pid[3] = {RUNNER_MOTOR_SPEED_PID_KP, RUNNER_MOTOR_SPEED_PID_KI, RUNNER_MOTOR_SPEED_PID_KD};
			
		runner_act_init->RC_data = get_remote_control_point();
		runner_act_init->motor_data.runner_motor_measure = get_motor_measure_point(2, CAN_RUNNER_ID);
		
		runner_PID_init(&runner_act_init->runner_angle_pid, RUNNER_MOTOR_ANGLE_PID_MAX_OUT, RUNNER_MOTOR_ANGLE_PID_MAX_IOUT, RUNNER_MOTOR_ANGLE_PID_KP, RUNNER_MOTOR_ANGLE_PID_KI, RUNNER_MOTOR_ANGLE_PID_KD);
		PID_init(&runner_act_init->runner_speed_pid, PID_POSITION, motor_speed_pid, RUNNER_MOTOR_SPEED_PID_MAX_OUT, RUNNER_MOTOR_SPEED_PID_MAX_IOUT);
		
    runner_feedback_update(runner_act_init);
		
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
static void runner_set_mode(runner_act_t *runner_act_mode)
{
    if (runner_act_mode == NULL)
    {
        return;
    }
		if (switch_is_down(runner_act_mode->RC_data->rc.s[0]))  
    {
			runner_act_mode->runner_mode = RUNNER_DOWN;
		}
		else
		{
			if (switch_is_up(runner_act_mode->RC_data->rc.s[1]) && runner_act_mode->last_runner_mode == RUNNER_DOWN)
			{
				runner_act_mode->runner_mode = RUNNER_INIT;
			}
			else if (switch_is_up(runner_act_mode->RC_data->rc.s[1]) && runner_act_mode->last_runner_mode == RUNNER_READY)
			{
				runner_act_mode->runner_mode = RUNNER_LOAD;
			}
			else if (switch_is_down(runner_act_mode->RC_data->rc.s[1]) && runner_act_mode->last_runner_mode == RUNNER_REACH)
			{
				runner_act_mode->runner_mode = RUNNER_READY;
			}
			
		}
		

}

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

static void runner_feedback_update(runner_act_t *runner_act_update)
{
	if (runner_act_update == NULL)
    {
        return;
    }
		runner_act_update->motor_data.motor_angle = ECD2ANGLE * (runner_act_update->motor_data.runner_motor_measure->ecd + ECD_RANGE * runner_act_update->motor_data.runner_motor_measure->ecd_count - ECD_OFFSET);
		runner_act_update->motor_data.motor_speed = runner_act_update->motor_data.runner_motor_measure->speed_rpm;
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
static void runner_control_loop(runner_act_t *runner_act_control)
{
	if (runner_act_control->runner_mode == RUNNER_DOWN)
		{
			runner_act_control->motor_data.give_current = 0;
		}
	else 
	{
		if (runner_act_control->runner_mode == RUNNER_INIT)
			runner_act_control->motor_data.motor_angle_set = (fp32)((int16_t)(runner_act_control->motor_data.motor_angle / TURN_ANGLE) + 1) * TURN_ANGLE;
		else if (runner_act_control->runner_mode == RUNNER_LOAD)
			runner_act_control->motor_data.motor_angle_set += TURN_ANGLE;
		runner_act_control->motor_data.motor_speed_set = runner_PID_calc(&runner_act_control->runner_angle_pid, 
																											runner_act_control->motor_data.motor_angle, 
																											runner_act_control->motor_data.motor_angle_set, 
																											runner_act_control->motor_data.motor_speed);
		runner_act_control->motor_data.give_current = (int16_t)PID_calc(&runner_act_control->runner_speed_pid, runner_act_control->motor_data.motor_speed, runner_act_control->motor_data.motor_speed_set);
		if (runner_act_control->runner_mode != RUNNER_REACH || runner_act_control->runner_mode != RUNNER_READY)
			runner_act_control->runner_mode = RUNNER_TURNING;
	}

	
	if (runner_act_control->runner_mode == RUNNER_TURNING && fabs(runner_act_control->motor_data.motor_angle_set - runner_act_control->motor_data.motor_angle) < ANGLE_DIFF && 
		fabs(runner_act_control->motor_data.motor_speed) < SPEED_READY)
		runner_act_control->runner_mode = RUNNER_REACH;
	
	CAN_cmd_runner(0,runner_act_control->motor_data.give_current,0);
		
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
uint8_t get_runner_mode(void)
{
	return runner_act.runner_mode;
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
static void runner_PID_init(runner_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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


static fp32 runner_PID_calc(runner_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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




