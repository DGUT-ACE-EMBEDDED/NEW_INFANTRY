#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "chassis_struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"




void Task_Chassis(void const *argument);
chassis_control_t *Get_Chassis_Control_p(void);
#endif
