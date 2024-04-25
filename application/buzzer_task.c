#include "buzzer_task.h"
#include "mode_set_task.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "cmsis_os.h"

//const uint32_t btnDelay1=20;
//const uint32_t btnDelay2=1.5*300;//0.5
//const uint32_t btnDelay3=600;//1
//const uint32_t btnDelay4=2*150;//0.25
//const uint32_t btnDelay5=700;//1+0.5
const uint32_t btnDelay0=20;
const uint32_t btnDelay1=200;
const uint32_t btnDelay2=400;
const uint32_t btnDelay3=600;
const uint32_t btnDelay4=800;

void tim0(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay0);
}
void tim1(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay1);
}

void tim2(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay2);
}

void tim3(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay3);
}

void tim4(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay4);
}

/*定义及引用变量*/

//进入比赛模式提示音
void Buzzer_game_mode(void);
//进入调试模式提示音
void Buzzer_control_mode(void);

void Buzzer_test(void);

extern dart_mode_t dart_mode;
uint8_t BuzzerCount = 0;


void Buzzer_Task(void const * argument)
{
	
	vTaskDelay(200);


	while(1)
	{
		//进入比赛模式提示音
		if(dart_mode.dart_mode == DART_GAME && BuzzerCount == 0)
		{
			Buzzer_game_mode();
			fn_BuzzerClose();
			BuzzerCount = 1;
		}
		//进入调试模式提示音
		else if(dart_mode.dart_mode == DART_CONTROL && BuzzerCount == 1)
		{
			Buzzer_control_mode();	
			fn_BuzzerClose();
			BuzzerCount = 0;
		}

		vTaskDelay(100);
	}
}

/*勝利のファンファーレ*/
void Buzzer_game_mode(void)
{
	tim1(M3);   
	tim0(0);
	tim1(M3);   
	tim0(0);
	tim1(M3);   
	tim0(0);
	tim1(M3);   //450
	tim0(0);
	
	tim1(0);
	tim2(M1);   //300
	tim0(0);
	tim2(M2);   //450
	tim0(0);
	tim2(M3);   //450
	tim0(0);
	tim1(M2);   //450
	tim0(0);
	tim2(M3);
}

/*Gwyn, Lord of Cinder*/
void Buzzer_control_mode(void)
{
	tim2(H3);   
	tim0(0);
	tim2(H2);   
	tim0(0);
	tim2(M6);   
	tim2(0);
	tim2(0);
	tim2(0);

	tim2(H3);   
	tim0(0);
	tim2(H2);   
	tim0(0);
	tim2(H5);   
	tim2(0);
	tim2(H4);   
	tim0(0);
	tim2(H3);   
	tim0(0);

}


//关闭蜂鸣器
void fn_BuzzerClose(void){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
}
