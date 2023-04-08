/*
_ooOoo_
o8888888o
88" . "88
(| -_- |)
 O\ = /O
___/`---'\____
.   ' \\| |// `.
/ \\||| : |||// \
/ _||||| -:- |||||- \
| | \\\ - /// | |
| \_| ''\---/'' | |
\ .-\__ `-` ___/-. /
___`. .' /--.--\ `. . __
."" '< `.___\_<|>_/___.' >'"".
| | : `- \`.;`\ _ /`;.`/ - ` : | |
\ \ `-. \_ __\ /__ _/ .-` / /
======`-.____`-.___\_____/___.-`____.-'======
`=---='

		 .............................................
		  ��Ի��bug���ģ�����̱����
*/

//  #include "gimbal_struct_variables.h"
#ifndef __GIMBAL_STRUCT_VARIABLES_H
#define __GIMBAL_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"

/*  ϵͳͷ�ļ� */
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
#include <stdbool.h>
// #include "stdint.h"

///* ************************FreeRTOS******************** */
// #include "freertos.h"
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"
// #include "cmsis_os.h"

/******************** BSP ********************/
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "bsp_referee.h"
/********************ALGORITHM********************/
#include "pid.h"
#include "fifo.h"
#include "lqr.h"
#include "filter.h"
/********************REFEREE********************/
// #include "crc.h"
// #include "referee_deal.h"

#include "imu_task.h"

typedef enum
{
	GIMBAL_MANUAL,		 // �ֶ�״̬
	GIMBAL_AUTOATTACK,	 // ����״̬
	GIMBAL_AUTOBUFF,	 // ���״̬
	GIMBAL_REPLENISHMEN, // ����״̬
} gimbal_behaviour_e;
typedef enum
{
	FIRE_OFF,		 
	FIRE_FULL_AUTO,
} fire_behaviour_e;
typedef enum
{
	GIMBAL_ZERO_FORCE, // ��̨����
	GIMBAL_NORMAL,	   //
} gimbal_state_e;

// rm6020���
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
} motor_measure_t;
typedef struct
{
	motor_measure_t *motor_measure; // ���յ��������
	fp32 actPositon_360;
	fp32 Speed_Set; // �����ٶ�
	int16_t set_voltage;
} Motor6020_t;

typedef struct
{
	motor_measure_t *motor_measure; // ���յ��������
	fp32 Speed_Set;					// �����ٶ�
	fp32 set_current;				//
} Motor3508_t;

// p��
typedef struct
{
	Motor6020_t pitch_motor;
	Encoder_t *pitch_motor_encoder;
	pid_parameter_t pitch_motor_speed_pid;
	pid_parameter_t pitch_motor_position_pid;
	first_order_filter_type_t visual_pitch__first_order_filter;
	#if(PITCH_CONTROLER == PITCH_UES_LQR)
	LQR_t motor_lqr;
	pid_parameter_t qitch_lqr_only_i_pid;
	sliding_mean_filter_type_t motor_filter;
	
	float motor_target;
	float motor_output;
	#endif
} gimbal_pitch_control_t;

// y��
typedef struct
{
	Motor6020_t yaw_motor;
	Encoder_t *yaw_motor_encoder;
	pid_parameter_t yaw_motor_speed_pid;
	pid_parameter_t yaw_motor_position_pid;
	pid_parameter_t yaw_motor_visual_speed_pid;
	pid_parameter_t yaw_motor_visual_position_pid;
	#if(YAW_CONTROLER == YAW_UES_LQR)
	LQR_t motor_lqr;
	pid_parameter_t yaw_lqr_only_i_pid;
	sliding_mean_filter_type_t motor_filter;
	
	float motor_target;
	float motor_output;
	#endif
} gimbal_yaw_control_t;

// ���
typedef struct
{
	fire_behaviour_e fire_behaviour;
	Motor3508_t right_motor;
	Motor3508_t left_motor;
	Motor3508_t fire_motor;
	Encoder_t *fire_motor_encoder;
	pid_parameter_t right_motor_speed_pid;
	pid_parameter_t left_motor_speed_pid;
	pid_parameter_t fire_motor_speed_pid;
	pid_parameter_t fire_motor_position_pid;
	
//	bool full_automatic;
	bool feed_buttle;
	
	const RC_ctrl_t *fire_rc;
	const REFEREE_t *referee;
} gimbal_fire_control_t;

typedef struct
{
	uint8_t visual_buff_send[29];
	fifo_s_t *usb_fifo;
	float auto_yaw;
	float auto_pitch;
	float auto_pitch_speed;
	
	float history_gimbal_data[2]; //��һ��pitch���ڶ���yaw
	float gimbal_use_control[2];

	const gimbal_behaviour_e *gimbal_behaviour;
	const INS_t *Imu_c;
	const float *gimbal_yaw;
	const float *gimbal_pitch;
	const REFEREE_t *referee;
	const gimbal_pitch_control_t *Pitch_c;
} gimbal_auto_control_t;

typedef struct
{
	gimbal_behaviour_e gimbal_behaviour;
	gimbal_state_e gimbal_state;
	gimbal_pitch_control_t Pitch_c;
	gimbal_yaw_control_t Yaw_c;
	float chassis_gimbal_angel;
	
	const RC_ctrl_t *Gimbal_RC;
	const gimbal_fire_control_t **fire_c;
	const gimbal_auto_control_t **auto_c;
	const INS_t *Imu_c;
	const REFEREE_t *referee;
} gimbal_control_t;

#endif
