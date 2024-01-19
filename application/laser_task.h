#ifndef LASER_TASK_H
#define LASER_TASK_H

#include "remote_control.h"

//红点激光器控制任务
void laser_task(void const * argument);
//激光器动作执行
void laser_control(const RC_ctrl_t *laser_rc);


#endif
