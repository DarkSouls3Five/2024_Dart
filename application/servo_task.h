#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

typedef enum
{
  SERVO_PULL,
	SERVO_PUSH
} servo_mode_e;

extern void servo_task(void const * argument);
extern servo_mode_e servo_mode, last_servo_mode;
void servo_set_mode(void);
#endif
