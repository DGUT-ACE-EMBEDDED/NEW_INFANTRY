#include "fire_Task.h"
#include "bsp_dr16.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"
#include "can1_receive.h"
#include "can1_send.h"
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "gimbal_config.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

gimbal_fire_control_t *fire_control_p;

static gimbal_fire_control_t *fire_task_init(void);
static void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f);
static void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f);

void fire_Task(void const *argument)
{
	fire_control_p = fire_task_init();
	while (1)
	{
		fire_behaviour_choose(fire_control_p); // 还差裁判系统 TODO:

		fire_pid_calculate(fire_control_p);

		can2_gimbal_setmsg(fire_control_p->left_motor.set_current, fire_control_p->right_motor.set_current, fire_control_p->fire_motor.set_current);
		vTaskDelay(1);
	}
}
gimbal_fire_control_t *fire_task_init(void)
{
	gimbal_fire_control_t *fire_task_init_p;
	fire_task_init_p = malloc(sizeof(gimbal_fire_control_t));
	memset(fire_task_init_p, 0, sizeof(gimbal_fire_control_t));

	// 获得拨弹指针
	fire_task_init_p->right_motor.motor_measure = get_right_motor_measure_point();
	fire_task_init_p->left_motor.motor_measure = get_left_motor_measure_point();
	fire_task_init_p->fire_motor.motor_measure = get_fire_motor_measure_point();
	
	fire_task_init_p->fire_rc  = RC_Get_RC_Pointer();
	
	fire_task_init_p->fire_motor_encoder = Encoder_Init(M2006, 3); 

	// fire
	PidInit(&fire_task_init_p->left_motor_speed_pid, 10, 0, 0, Output_Limit);
	PidInit(&fire_task_init_p->right_motor_speed_pid, 10, 0, 0, Output_Limit);
	PidInit(&fire_task_init_p->fire_motor_speed_pid, 10, 0, 0, Output_Limit);
	PidInit(&fire_task_init_p->fire_motor_position_pid, 1, 0.1, 0, Output_Limit | Integral_Limit);
	PidInitMode(&fire_task_init_p->left_motor_speed_pid, Output_Limit, 16000, 0);
	PidInitMode(&fire_task_init_p->right_motor_speed_pid, Output_Limit, 16000, 0);
	PidInitMode(&fire_task_init_p->fire_motor_speed_pid, Output_Limit, 16000, 0);
	PidInitMode(&fire_task_init_p->fire_motor_position_pid, Output_Limit, 10000, 0);
	PidInitMode(&fire_task_init_p->fire_motor_position_pid, Integral_Limit, 100, 0);

	fire_task_init_p->full_automatic = true;
	fire_task_init_p->feed_buttle = false;

	return fire_task_init_p;
}
void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f)
{
	static uint16_t last_B = 0;
	// static uint16_t last_C = 0;
	uint16_t last_press;

	// B	单发全自动切换
	last_press = last_B;
	last_B = fire_behaviour_choose_f->fire_rc->kb.bit.B;
	if ((last_press == false) && (last_B == true)) // 上升沿触发
	{
		fire_behaviour_choose_f->full_automatic = !fire_behaviour_choose_f->full_automatic;
	}

	// 待定	补弹 TODO:
	// last_press = last_C;
	// last_C = fire_behaviour_choose_rc_f->kb.bit.B;
	// if ((last_press == false) && (last_C == true))
	// {
	// 	fire_behaviour_choose_f->feed_buttle = !fire_behaviour_choose_f->feed_buttle;
	// }

	// 裁判系统弹速设置 TODO:
	//  switch()
	//  {
	//  	case :
	//  	fire_behaviour_choose_f->left_motor.Speed_Set = fire_behaviour_choose_f->right_motor.Speed_Set = fire_speed_15;
	//  }
	fire_behaviour_choose_f->left_motor.Speed_Set = fire_behaviour_choose_f->right_motor.Speed_Set = 100;
	// 弹舱舵机控制 TODO:
}


void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f)
{
	if ((fire_pid_calculate_f->fire_rc->mouse.press_l > 0) || (fire_pid_calculate_f->fire_rc->rc.ch[4] > 0))
	{
		if (fire_pid_calculate_f->full_automatic) // 全自动开环控制
		{
			fire_pid_calculate_f->fire_motor.Speed_Set = 100; // TODO:
			fire_pid_calculate_f->fire_motor.set_current = motor_speed_control(&fire_pid_calculate_f->fire_motor_speed_pid,
																			   fire_pid_calculate_f->fire_motor.Speed_Set,
																			   fire_pid_calculate_f->fire_motor.motor_measure->speed);
		}
		else // 非全自动使用闭环控制
		{
			fire_pid_calculate_f->fire_motor.set_current = motor_position_speed_control(&fire_pid_calculate_f->fire_motor_speed_pid,
																						&fire_pid_calculate_f->fire_motor_position_pid,
																						(fire_pid_calculate_f->fire_motor_encoder->Encode_Record_Val + 1000),
																						fire_pid_calculate_f->fire_motor_encoder->Encode_Record_Val,
																						fire_pid_calculate_f->fire_motor.motor_measure->speed);
		}
	}
	else
	{
		fire_pid_calculate_f->fire_motor.set_current = 0;
	}

	fire_pid_calculate_f->left_motor.set_current = motor_speed_control(&fire_pid_calculate_f->left_motor_speed_pid,
																	   fire_pid_calculate_f->left_motor.Speed_Set,
																	   fire_pid_calculate_f->left_motor.motor_measure->speed);
	fire_pid_calculate_f->right_motor.set_current = motor_speed_control(&fire_pid_calculate_f->right_motor_speed_pid,
																		fire_pid_calculate_f->right_motor.Speed_Set,
																		fire_pid_calculate_f->right_motor.motor_measure->speed);
}
const gimbal_fire_control_t **get_fire_control_point(void)
{
	return (const gimbal_fire_control_t**)&fire_control_p;
}
