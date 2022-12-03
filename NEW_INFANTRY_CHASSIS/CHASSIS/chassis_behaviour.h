#ifndef __CHASSIS_BEHAVIOUR_H
#define __CHASSIS_BEHAVIOUR_H

#include "struct_variables.h"

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f);
void chassis_behaviour_choose(chassis_control_t *Chassis_behaviour_f);
void chassis_motion_decomposition(chassis_control_t *chassis_motion_decomposition_f);
void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f);
void chassis_state_choose(chassis_control_t *chassis_state_choose_f);

chassis_behaviour_e *get_chassis_behaviour_point(void);
fp32 *get_chassis_x_point(void);
fp32 *get_chassis_y_point(void);
fp32 *get_chassis_yaw_point(void);

#endif
