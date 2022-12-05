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
// rm6020电机
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;

	float actPositon_360;
} motor_measure_t;
typedef struct
{
	motor_measure_t *motor_measure; //接收电机的数据
	fp32 Speed_Set;					//设置速度
	fp32 pid_output;				// pid输出
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
	pid_parameter_t pitch_motor_speed_pid;
	pid_parameter_t pitch_motor_position_pid;
} gimbal_pitch_control_t;

// y轴
typedef struct
{
	Motor6020_t yaw_motor;
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

} gimbal_fire_control_t;

typedef struct
{
	RC_ctrl_t *Gimbal_RC;
	gimbal_pitch_control_t Pitch_c;
	gimbal_yaw_control_t Yaw_c;
	gimbal_fire_control_t fire_c;
	const INS_t *Imu_c;

} gimbal_control_t;

#endif
