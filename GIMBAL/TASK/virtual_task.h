#ifndef __TASK_VIRTUAL_H  //���δ����
#define __TASK_VIRTUAL_H  //��ô����

#include "gimbal_struct_variables.h"

extern void Virtual_Task(void const * argument);
void gimbal_clear_virtual_recive(void);
const gimbal_auto_control_t **get_auto_control_point(void);
#endif


