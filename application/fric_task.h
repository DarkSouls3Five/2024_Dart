#ifndef FRIC_TASK_H
#define FRIC_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

//·ÉïÚ²ÎÊýÁÐ±í£¬Ã¿ÖÆÔìÒ»·¢ÐÂïÚ¶¼ÐèÒªÖØÐÂ²âÊÔÈ·¶¨²ÎÊý
#define X1 130.0f
#define X2 50.0f
#define X3 185.0f
#define X4 115.0f

#define A0 130.0f
#define B0 0.0f
#define C0 -25.0f
#define D0 -125.0f

/*»ù×¼×ªËÙ£¨ÎÂ¶ÈÔ¼20¡ã£¬°´Èü³¡¾àÀë»÷´òÇ°ÉÚÕ¾£©Èô¼¯ÌåÆ«²îÇëµ÷½Ú»ù×¼×ªËÙ£¡*/
#define FRIC_SET_SPEED 6270.0f
//6100.0f//6300.0f//6000.0f//7000.0f//73	00.0f//7000.0f//5000.0f

//·¢ÉäÃ¿Ò»·¢·ÉïÚÊ±µÄÏà¶Ô×ªËÙ±ä»¯£¬Çë¸ù¾ÝÊµ¼Ê·¢Éä·ÉïÚ±àºÅÓëË³Ðò½øÐÐÐÞ¸Ä

#define RELATIVE_SPEED_1 X1
#define RELATIVE_SPEED_2 X2
#define RELATIVE_SPEED_3 X3
#define RELATIVE_SPEED_4 X4

//#define RELATIVE_SPEED_1 A0
//#define RELATIVE_SPEED_2 B0
//#define RELATIVE_SPEED_3 C0
//#define RELATIVE_SPEED_4 D0

#define FRIC_TASK_INIT_TIME 357
#define MAX_3508_CAN_CURRENT 16384.0f

//fric task control time  2ms
//???????? 2ms
#define FRIC_CONTROL_TIME_MS 2
//fric motor speed PID
//???????PID

#define FRIC_LEFT_SPEED_PID_KP      10.0f//5.0f
#define FRIC_LEFT_SPEED_PID_KI			0.0f
#define FRIC_LEFT_SPEED_PID_KD			0.5f//0.3f

#define FRIC_RIGHT_SPEED_PID_KP		  10.0f//5.0f
#define FRIC_RIGHT_SPEED_PID_KI     0.0f
#define FRIC_RIGHT_SPEED_PID_KD     0.5f//0.3f


#define FRIC_LEFT_PID_MAX_OUT   MAX_3508_CAN_CURRENT
#define FRIC_LEFT_PID_MAX_IOUT  5000.0f

#define FRIC_RIGHT_PID_MAX_OUT   MAX_3508_CAN_CURRENT
#define FRIC_RIGHT_PID_MAX_IOUT  5000.0f


typedef enum
{
  FRIC_ON,
	FRIC_OFF,
	
	//·¢Éä²»Í¬·ÉïÚ£¬¶ÔÓ¦²»Í¬×ªËÙ
	FRIC_ON_1,
	FRIC_ON_2,
	FRIC_ON_3,
	FRIC_ON_4,
	FRIC_READY
} fric_mode_e;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} fric_motor_t;

typedef struct
{
  const RC_ctrl_t *RC_data;  
  fric_mode_e fric_mode;  	//????'?õ?????????, the point to remote control
  fric_motor_t motor_data[6];          //chassis motor data.??????????
  pid_type_def motor_speed_pid[6];              //motor speed PID.?????????pid
} fric_act_t;

/**
  * @brief          fric task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ????,?? FRIC_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ?
  * @retval         none
  */
extern void fric_task(void const * argument);

extern uint8_t get_fric_mode(void);

extern fric_act_t fric_act;


#endif
