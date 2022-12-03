#ifndef __STRUCT_VARIABLES_H
#define __STRUCT_VARIABLES_H

#include "main.h"
#include "parameter.h"
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
#include "rc.h"
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
#include "filter.h"

/********************REFEREE********************/
//#include "crc.h"
//#include "referee_deal.h"

#include "bsp_Motor_Encoder.h"

typedef enum
{
	CHASSIS_FOLLOW,	   //跟随
	CHASSIS_NO_FOLLOW, //不跟随
	CHASSIS_ROTATION,  //小陀螺
	CHASSIS_BATTERY,   //炮台模式
} chassis_behaviour_e;

typedef enum
{
	CHASSIS_ZERO_FORCE,	   //底盘无力
	CHASSIS_LOCK_POSITION, //底盘位置锁死
	CHASSIS_SPEED,		   //速度
} chassis_state_e;

// rm电机统一数据结构体
typedef struct
{
	uint16_t position;
	int16_t speed;
} motor_measure_t;

//超级电容
typedef struct
{
	float input_voltage;	   //输入电压
	float Capacitance_voltage; //电容电压
	float Input_current;	   //输入电流
	float Set_power;		   //设定功率
} Supercapacitor_receive_t;	   //超级电容器

typedef struct
{
	motor_measure_t *chassis_motor_measure; //接收电机的数据
	fp32 Speed_Set;							//设置速度
	fp32 pid_output;						// pid输出
	int16_t give_current;
} Motor3508_t;

typedef struct
{
	RC_ctrl_t *Chassis_RC; //底盘遥控数据

	chassis_behaviour_e *behaviour; //底盘模式
	chassis_state_e chassis_state;

	Motor3508_t Chassis_Motor[4]; //底盘四个电机
	Encoder_t *Motor_encoder[4];
	pid_parameter_t motor_Speed_Pid[4]; //电机速度pid
	pid_parameter_t motor_Position_Pid[4];

	pid_parameter_t Chassis_speedX_Pid; //底盘速度xpid
	pid_parameter_t Chassis_speedY_Pid; //底盘速度ypid
	pid_parameter_t chassis_rotate_pid; //旋转pid

	first_order_filter_type_t LowFilt_chassis_vx; //低通滤波器
	first_order_filter_type_t LowFilt_chassis_vy; //低通滤波器

	fp32 Chassis_Gimbal_Diference_Angle; //底盘与云台的差角

	Supercapacitor_receive_t *super_cap_c; //超电
	fp32 chassis_speed_gain;			   //速度因子
	fp32 chassis_last_speed_gain;

} chassis_control_t;

#endif
