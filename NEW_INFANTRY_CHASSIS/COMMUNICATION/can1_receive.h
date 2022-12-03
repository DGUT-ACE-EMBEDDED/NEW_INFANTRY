#ifndef __CAN1_RECEIVE_H
#define __CAN1_RECEIVE_H

#include "can.h"
#include "struct_variables.h"

Supercapacitor_receive_t *get_supercap_control_point(void);
motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern void CAN1_filter_config(void);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void chassis_can1_callback(CAN_HandleTypeDef *hcan);
extern float re_capacitance_voltage(void);

#endif 


