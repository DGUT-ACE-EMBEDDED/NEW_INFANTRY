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
		  佛曰：bug泛滥，我已瘫痪！
*/
#ifndef __GIMBAL_STRUCT_VARIABLES_H
#define __GIMBAL_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"

/*  系统头文件 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stdint.h"

/* ************************FreeRTOS******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"

/********************CONTROL********************/
#include "bsp_dr16.h"
//#include "imu_task.h"
//#include "can1_receive.h"
//#include "can2_receive.h"
//#include "capacitor_control.h"
//#include "upper_machine.h"
//#include "visual.h"

/********************ALGORITHM********************/
//#include "fifo_buff.h"
#include "pid.h"
//#include "maths.h"
//#include "rm_motor.h"

/********************REFEREE********************/
//#include "crc.h"
//#include "referee_deal.h"

#include "bsp_Motor_Encoder.h"
#include "imu_task.h"
typedef enum
{
	GIMBAL_MANUAL,		 //手动状态
	GIMBAL_AUTOATTACK,	 //自瞄状态
	GIMBAL_AUTOBUFF,	 //打符状态
	GIMBAL_REPLENISHMEN, //补给状态
} gimbal_behaviour_e;

typedef enum
{
	GIMBAL_ZERO_FORCE, //云台无力
	GIMBAL_NORMAL,	   //
} gimbal_state_e;

// rm6020电机
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
} motor_measure_t;
typedef struct
{
	motor_measure_t *motor_measure; //接收电机的数据
	fp32 actPositon_360;
	fp32 Speed_Set; //设置速度
	int16_t set_voltage;
} Motor6020_t;

typedef struct
{
	motor_measure_t *motor_measure; //接收电机的数据
	fp32 Speed_Set;					//设置速度
	fp32 set_current;				//
} Motor3508_t;

// p轴
typedef struct
{
	Motor6020_t pitch_motor;
	Encoder_t *pitch_motor_encoder;
	pid_parameter_t pitch_motor_speed_pid;
	pid_parameter_t pitch_motor_position_pid;
} gimbal_pitch_control_t;

// y轴
typedef struct
{
	Motor6020_t yaw_motor;
	Encoder_t *yaw_motor_encoder;
	pid_parameter_t yaw_motor_speed_pid;
	pid_parameter_t yaw_motor_position_pid;
} gimbal_yaw_control_t;

// 火控
typedef struct
{
	Motor3508_t right_motor;
	Motor3508_t left_motor;
	Motor3508_t fire_motor;
	pid_parameter_t right_motor_speed_pid;
	pid_parameter_t left_motor_speed_pid;
	pid_parameter_t fire_motor_speed_pid;
	pid_parameter_t fire_motor_position_pid;
	bool full_automatic;
} gimbal_fire_control_t;

typedef struct
{
	gimbal_behaviour_e gimbal_behaviour;
	gimbal_state_e gimbal_state;
	RC_ctrl_t *Gimbal_RC;
	gimbal_pitch_control_t Pitch_c;
	gimbal_yaw_control_t Yaw_c;
	gimbal_fire_control_t fire_c;
	const INS_t *Imu_c;
	float chassis_gimbal_angel; //云台和底盘差角
} gimbal_control_t;

#endif
