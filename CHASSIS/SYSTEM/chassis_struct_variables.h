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
#ifndef __CHASSIS_STRUCT_VARIABLES_H
#define __CHASSIS_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"
#include "chassis_config.h"
/*  ϵͳͷ�ļ� */
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

/*ģ�鹤������*/
//#define WATCH_DOG                //�������Ź�
//#define FIRE_WORK                //�䵯ģʽ���� (��������)
#define POWER_LIMIT              //������������   

typedef enum
{
	CHASSIS_FOLLOW,	   //����
	CHASSIS_NO_FOLLOW, //������
	CHASSIS_ROTATION,  //С����
	CHASSIS_BATTERY,   //��̨ģʽ
} chassis_behaviour_e;

typedef enum
{
	CHASSIS_ZERO_FORCE,	   //��������
	CHASSIS_LOCK_POSITION, //����λ������
	CHASSIS_SPEED,		   //�ٶ�
} chassis_state_e;

// rm���ͳһ���ݽṹ��
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
//��������
typedef struct
{
	float input_voltage;	   //�����ѹ
	float Capacitance_voltage; //���ݵ�ѹ
	float Input_current;	   //�������
	float Set_power;		   //�趨����
} Supercapacitor_receive_t;	   //����������

typedef struct
{
	motor_measure_t *chassis_motor_measure; //���յ��������
	fp32 Speed_Set;							//�����ٶ�
	fp32 pid_output;						// pid���
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
	RC_ctrl_t *Chassis_RC; //����ң������

	chassis_behaviour_e behaviour; //����ģʽ
	chassis_state_e chassis_state;
	
	motor6020_measure_t *yaw_motor;
	Encoder_t *yaw_motor_encoder;
	Motor3508_t Chassis_Motor[4]; //�����ĸ����
	Encoder_t *Motor_encoder[4];
	pid_parameter_t motor_Speed_Pid[4]; //����ٶ�pid
	pid_parameter_t motor_Position_Pid[4];

	pid_parameter_t Chassis_speedX_Pid; //�����ٶ�xpid
	pid_parameter_t Chassis_speedY_Pid; //�����ٶ�ypid
	pid_parameter_t chassis_rotate_pid; //��תpid
	
	first_order_filter_type_t Chassis_speedX_filter;
	first_order_filter_type_t Chassis_speedY_filter;
	
	#ifdef POWER_CONTROL
	pid_parameter_t power_pid; //����
	pid_parameter_t powerbuff_pid; //����
	#endif

	fp32 Chassis_Gimbal_Diference_Angle; //��������̨�Ĳ��

	Supercapacitor_receive_t *super_cap_c; //����
	fp32 chassis_speed_gain;			   //�ٶ�����
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
