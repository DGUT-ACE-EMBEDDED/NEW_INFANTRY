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
#ifndef __CHASSIS_STRUCT_VARIABLES_H
#define __CHASSIS_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"
#include "chassis_config.h"
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
//#include "can1_receive.h"
//#include "can2_receive.h"
//#include "capacitor_control.h"
//#include "upper_machine.h"
//#include "visual.h"
#include "bsp_referee.h"
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
#include "imu_task.h"

/*模块工作属性*/
//#define WATCH_DOG                //启动看门狗
//#define FIRE_WORK                //射弹模式开启 (开拨弹轮)
#define POWER_LIMIT              //启动功率限制   

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
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
} motor6020_measure_t;
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

#ifdef ACCEL_CONTROL
typedef struct
{
	float accel_x;
	float accel_y;
	float motor_accel[4];
	float theory_accel_x;
	float theory_accel_y;
	float error_accel_x;
	float error_accel_y;
	int last_motor_speed[4];
	sliding_mean_filter_type_t accel_x_sliding_filter;
	sliding_mean_filter_type_t accel_y_sliding_filter;
	first_order_filter_type_t motor_accel_filter_fliter[4];
	sliding_mean_filter_type_t motor_accel_sliding_fliter[4];
	pid_parameter_t Chassis_accel_x_pid;
	pid_parameter_t Chassis_accel_y_pid;
	
}Chassis_accel_control_t;
#endif

typedef struct
{
	RC_ctrl_t *Chassis_RC; //底盘遥控数据

	chassis_behaviour_e behaviour; //底盘模式
	chassis_state_e chassis_state;
	
	motor6020_measure_t *yaw_motor;
	Encoder_t *yaw_motor_encoder;
	Motor3508_t Chassis_Motor[4]; //底盘四个电机
	Encoder_t *Motor_encoder[4];
	pid_parameter_t motor_Speed_Pid[4]; //电机速度pid
	pid_parameter_t motor_Position_Pid[4];

	pid_parameter_t Chassis_speedX_Pid; //底盘速度xpid
	pid_parameter_t Chassis_speedY_Pid; //底盘速度ypid
	pid_parameter_t chassis_rotate_pid; //旋转pid
	
	first_order_filter_type_t Chassis_speedX_filter;
	first_order_filter_type_t Chassis_speedY_filter;
	
	#ifdef POWER_CONTROL
	pid_parameter_t power_pid; //功率
	pid_parameter_t powerbuff_pid; //功率
	#endif

	fp32 Chassis_Gimbal_Diference_Angle; //底盘与云台的差角

	Supercapacitor_receive_t *super_cap_c; //超电
	fp32 chassis_speed_gain;			   //速度因子
	REFEREE_t *referee_p;
	
	#ifdef USE_IMU
	const INS_t *Imu_c;
	float chassis_no_follow_yaw;
	#endif
	
	#ifdef ACCEL_CONTROL
	Chassis_accel_control_t chassis_accel_control;
	#endif
} chassis_control_t;

#endif
