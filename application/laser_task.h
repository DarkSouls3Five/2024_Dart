#ifndef LASER_TASK_H
#define LASER_TASK_H

#include "remote_control.h"

//��㼤������������
void laser_task(void const * argument);
//����������ִ��
void laser_control(const RC_ctrl_t *laser_rc);


#endif
