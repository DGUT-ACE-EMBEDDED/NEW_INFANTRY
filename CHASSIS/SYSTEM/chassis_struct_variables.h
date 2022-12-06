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

/********************ALGORITHM********************/
//#include "fifo_buff.h"
#include "pid.h"
//#include "maths.h"
//#include "rm_motor.h"


/********************REFEREE********************/
//#include "crc.h"
//#include "referee_deal.h"

#include "bsp_Motor_Encoder.h"

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

typedef struct
{
	RC_ctrl_t *Chassis_RC; //����ң������

	chassis_behaviour_e behaviour; //����ģʽ
	chassis_state_e chassis_state;

	Motor3508_t Chassis_Motor[4]; //�����ĸ����
	Encoder_t *Motor_encoder[4];
	pid_parameter_t motor_Speed_Pid[4]; //����ٶ�pid
	pid_parameter_t motor_Position_Pid[4];

	pid_parameter_t Chassis_speedX_Pid; //�����ٶ�xpid
	pid_parameter_t Chassis_speedY_Pid; //�����ٶ�ypid
	pid_parameter_t chassis_rotate_pid; //��תpid

	int16_t Chassis_Gimbal_Diference_Angle; //��������̨�Ĳ��

	Supercapacitor_receive_t *super_cap_c; //����
	fp32 chassis_speed_gain;			   //�ٶ�����

} chassis_control_t;

#endif
