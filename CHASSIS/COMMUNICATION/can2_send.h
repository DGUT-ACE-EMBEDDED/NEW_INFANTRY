#ifndef __CAN2_SEND_H
#define __CAN2_SEND_H

#include "chassis_struct_variables.h"

void can2_chassis_to_gimbal(const RC_ctrl_t *can2_MK_send);
void can2_chassis_to_gimbal_referee(const REFEREE_t *can2_referee_send);
#endif
