/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收gpio数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_bus.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "arm_math.h"

int count_i = 0;
int start_up_i = 0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static uint8_t              fric_left_can_send_data[8];
static CAN_TxHeaderTypeDef  fric_left_tx_message;
static uint8_t              fric_right_can_send_data[8];
static CAN_TxHeaderTypeDef  fric_right_tx_message;
static uint8_t              runner_can_send_data[8];
static CAN_TxHeaderTypeDef  runner_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal_tx_message;

//计算2006绝对角度
static void M2006_absolute_position_cal(motor_measure_t *adv_act_absolute);

/*
motor data,  [byte0:byte1]:servo1, [byte2:byte3]:servo2,
						[byte4]:gpio1, [byte5]:gpio2, [byte6]:gpio3, [byte7]:gpio4,
电机数据, [byte0:byte1]:舵机1，[byte2:byte3]:舵机2，
					[byte4]:gpio1, [byte5]:gpio2, [byte6]:gpio3, [byte7]:gpio4,*/
motor_measure_t motor_data[9];

/**
  * @brief          hal CAN fifo call back, receive gpio data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收gpio数据
  * @param[in]      ptr:电机数据指针
	* @param[in]      dara:CAN帧数据指针
  * @retval         none
  */
void get_motor_measure(motor_measure_t *ptr, uint8_t *data)                                    
{                                                                   
		(ptr)->last_ecd = (ptr)->ecd;                                   
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
		(ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      
		(ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  
		(ptr)->temperate = (data)[6];
}

/**
  * @brief          hal CAN fifo call back, receive gpio data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收gpio数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if (hcan == &hcan1)
		{
			switch (rx_header.StdId)
			{
				case CAN_3508_FRICL1_ID:
				{
					static uint8_t i = 0;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}
				case CAN_3508_FRICL2_ID:
				{
					static uint8_t i = 1;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}	
				case CAN_3508_FRICL3_ID:
				{
					static uint8_t i = 2;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}	
				
				case CAN_3508_FRICR1_ID:
				{
					static uint8_t i = 3;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}
				case CAN_3508_FRICR2_ID:
				{
					static uint8_t i = 4;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}	
				case CAN_3508_FRICR3_ID:
				{
					static uint8_t i = 5;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}	
			}
		}
		else if (hcan == &hcan2)
		{
			switch (rx_header.StdId)
			{
				case CAN_YAW_ID:
				{
					static uint8_t i = 6;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}
				case CAN_TRANS_ID:
				{
					static uint8_t i = 7;
					get_motor_measure(&motor_data[i], rx_data);		
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					break;
				}
				case CAN_ADV_ID:
				{
					static uint8_t i = 8;
					get_motor_measure(&motor_data[i], rx_data);		
					M2006_absolute_position_cal(&motor_data[i]);
					//detect_hook(CHASSIS_MOTOR1_TOE + i);					
					if(count_i == 0)
					{
						motor_data[i].ecd_count =0;

					}
					count_i++;
					if(count_i>500)
					{
						if(motor_data[i].ecd - motor_data[i].last_ecd > HALF_ECD_RANGE)
						{
							motor_data[i].ecd_count--;
						}
						else if(motor_data[i].ecd - motor_data[i].last_ecd < -HALF_ECD_RANGE)
						{
							motor_data[i].ecd_count++;
						}
					}
					break;				
				}
			}
		}
}
/**
  * @brief          send control current of motor (0x201, 0x202, 0x203)
  * @param[in]      motor1: (0x201)(0x202)(0x203) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
* @brief            发送左摩擦轮电机电机控制电流(0x201,0x202,0x203)
  * @param[in]      motor: (0x201)(0x202)(0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_fricl(int16_t motor1, int16_t motor2, int16_t motor3)
{
    uint32_t send_mail_box;
    fric_left_tx_message.StdId = CAN_TX_LEFT_ID;
    fric_left_tx_message.IDE = CAN_ID_STD;
    fric_left_tx_message.RTR = CAN_RTR_DATA;
    fric_left_tx_message.DLC = 0x08;
	
    fric_left_can_send_data[0] = motor1 >> 8;
    fric_left_can_send_data[1] = motor1;
    fric_left_can_send_data[2] = motor2 >> 8;
    fric_left_can_send_data[3] = motor2;
    fric_left_can_send_data[4] = motor3 >> 8;
    fric_left_can_send_data[5] = motor3;
    fric_left_can_send_data[6] = 0X00;
    fric_left_can_send_data[7] = 0X00;
    HAL_CAN_AddTxMessage(&hcan1, &fric_left_tx_message, fric_left_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207)
  * @param[in]      motor1: (0x205)(0x206)(0x207) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
* @brief            发送右摩擦轮电机电机控制电流(0x205,0x206,0x207)
  * @param[in]      motor: (0x205)(0x206)(0x207) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_fricr(int16_t motor1, int16_t motor2, int16_t motor3)
{
    uint32_t send_mail_box;
    fric_right_tx_message.StdId = CAN_TX_RIGHT_ID;
    fric_right_tx_message.IDE = CAN_ID_STD;
    fric_right_tx_message.RTR = CAN_RTR_DATA;
    fric_right_tx_message.DLC = 0x08;
	
    fric_right_can_send_data[0] = motor1 >> 8;
    fric_right_can_send_data[1] = motor1;
    fric_right_can_send_data[2] = motor2 >> 8;
    fric_right_can_send_data[3] = motor2;
    fric_right_can_send_data[4] = motor3 >> 8;
    fric_right_can_send_data[5] = motor3;
    fric_right_can_send_data[6] = 0X00;
    fric_right_can_send_data[7] = 0X00;
    HAL_CAN_AddTxMessage(&hcan1, &fric_right_tx_message, fric_right_can_send_data, &send_mail_box);
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207)
  * @param[in]      motor1: (0x205)(0x206)(0x207) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
* @brief            发送装填电机电机控制电流(0x205,0x206,0x207)
  * @param[in]      motor: (0x205)(0x206)(0x207) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_can2(int16_t motor1, int16_t motor2, int16_t motor3)
{
    uint32_t send_mail_box;
    runner_tx_message.StdId = CAN_TX_REAR_ID;
    runner_tx_message.IDE = CAN_ID_STD;
    runner_tx_message.RTR = CAN_RTR_DATA;
    runner_tx_message.DLC = 0x08;
	
    runner_can_send_data[0] = motor1 >> 8;
    runner_can_send_data[1] = motor1;
    runner_can_send_data[2] = motor2 >> 8;
    runner_can_send_data[3] = motor2;
    runner_can_send_data[4] = motor3 >> 8;
    runner_can_send_data[5] = motor3;
    runner_can_send_data[6] = 0X00;
    runner_can_send_data[7] = 0X00;
    HAL_CAN_AddTxMessage(&hcan2, &runner_tx_message, runner_can_send_data, &send_mail_box);
}



/**
  * @brief          return the motor data point
  * @param[in]      motor ID
  * @retval         motor data point
  */
/**
* @brief          返回电机数据指针
* @param[in]      电机ID
* @retval         电机数据指针
  */
const motor_measure_t *get_motor_measure_point(uint8_t bus, uint16_t id)
{
	if (bus == 1)
	{
		if (id == CAN_3508_FRICL1_ID)	
			return &motor_data[0];
		else if  (id == CAN_3508_FRICL2_ID)	
			return &motor_data[1];
		else if (id == CAN_3508_FRICL3_ID)	
			return &motor_data[2];
		else if (id == CAN_3508_FRICR1_ID)	
			return &motor_data[3];
		else if (id == CAN_3508_FRICR2_ID)	
			return &motor_data[4];
		else if (id == CAN_3508_FRICR3_ID)	
			return &motor_data[5];
		else
			return NULL;
	}
	else if (bus == 2)
	{
		if (id == CAN_YAW_ID)	
			return &motor_data[6];
		else if (id == CAN_TRANS_ID)	
			return &motor_data[7];
		else if (id == CAN_ADV_ID)
			return &motor_data[8];
	}
	else
		return NULL;
		
}

/**
  * @brief          M2006_absolute_position_cal
  * @param[in]      motor ID
  * @retval         motor data point
  */
/**
* @brief          计算2006电机绝对角度
* @param[in]      电机ID
* @retval         电机数据指针
  */
static void M2006_absolute_position_cal(motor_measure_t *motor_2006)
{
	if(motor_2006->ecd - motor_2006->last_ecd > 4096)
	{
		motor_2006->round --;
	}
	else if(motor_2006->ecd - motor_2006->last_ecd < -4096)
	{
		motor_2006->round ++;	
	}
	
	motor_2006->distance = (motor_2006->round + (fp32)(motor_2006->ecd-motor_2006->init_ecd)/8192.0f)*360/36;
}

/**
  * @brief          M2006_init_ecd
  * @param[in]      motor ID
  * @retval         motor data point
  */
/**
* @brief          记录2006电机初始ecd
* @param[in]      电机ID
* @retval         电机数据指针
  */
void init_ecd_record(motor_measure_t *motor_2006)
{
	motor_2006->init_ecd = motor_2006->ecd;
	if(motor_2006->init_ecd>4096)
		motor_2006->round ++;
}
