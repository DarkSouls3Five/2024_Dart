#include "laser_task.h"
#include "main.h"
#include "cmsis_os.h"


void laser_control(const RC_ctrl_t *laser_rc);

void laser_task(void const * argument)
{
	
	while(1)
	{
		const RC_ctrl_t *laser_rc = get_remote_control_point();
		laser_control(laser_rc);
		vTaskDelay(2);
	}
}

//��㼤��������
void laser_control(const RC_ctrl_t *laser_rc)
{
	if(switch_is_down(laser_rc->rc.s[0])||switch_is_mid(laser_rc->rc.s[0]))//�Ҳ����µ����е����򿪼��������ϵ��رռ�����
	{
		HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
	}
}
