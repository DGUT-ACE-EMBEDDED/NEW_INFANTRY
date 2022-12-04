#ifndef __CAN1_SEND_H
#define __CAN1_SEND_H

#include "chassis_struct_variables.h"

void can1_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
void can1_cap_setmsg(int16_t Chassis_power);

#endif
