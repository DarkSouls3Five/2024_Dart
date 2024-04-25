#ifndef MODE_SET_TASK_H
#define MODE_SET_TASK_H
#include "struct_typedef.h"
#include "CAN_bus.h"
#include "pid.h"
#include "remote_control.h"

#define MODE_SET_TASK_INIT_TIME 300
#define MODE_SET_TIME_MS 5

typedef enum
{
  DART_CONTROL,//ÊÖ¶¯¿ØÖÆÄ£Ê½
	DART_GAME,	//±ÈÈüÄ£Ê½
	
} dart_mode_e;


typedef struct
{
  const RC_ctrl_t *RC_data;               //????'?õ?????????, the point to remote control
  dart_mode_e last_dart_mode;               //state machine. ???????????
	dart_mode_e dart_mode; 

} dart_mode_t;



#endif
