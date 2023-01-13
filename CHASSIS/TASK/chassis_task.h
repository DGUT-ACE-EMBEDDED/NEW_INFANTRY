#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "chassis_struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

/************����pid************/
#define CHASSIS_RF_MOTOR_KP 9.0f
#define CHASSIS_RF_MOTOR_KI 0.0f
#define CHASSIS_RF_MOTOR_KD 0.5f

#define CHASSIS_LF_MOTOR_KP 9.0f
#define CHASSIS_LF_MOTOR_KI 0.0f
#define CHASSIS_LF_MOTOR_KD 0.5f

#define CHASSIS_RB_MOTOR_KP 9.0f
#define CHASSIS_RB_MOTOR_KI 0.0f
#define CHASSIS_RB_MOTOR_KD 0.5f

#define CHASSIS_LB_MOTOR_KP 9.0f
#define CHASSIS_LB_MOTOR_KI 0.0f
#define CHASSIS_LB_MOTOR_KD 0.5f

//�����ٶ�pid
#define CHASSIS_LOCATION_KP 8.0f
#define CHASSIS_LOCATION_KI 0.0f
#define CHASSIS_LOCATION_KD 10.8f

//��תpid
#define CHASSIS_SPIN_FOLLOW_KP 100.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

#define NORMAL_FORWARD_BACK_SPEED 660.0f //������ֱͨ���ٶ�
#define NORMAL_LEFT_RIGHT_SPEED 660.0f   //������ͨƽ���ٶ�

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 0.0110f ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���  |  0.26f  |  0.0097f  |  0.0510f
#define CHASSIS_MK_SECOND_FILTERING (0.8f)

/**********************�˶����ٶ�����**********************/

#define STRAIGHT_ACCELERAD 3.5f    //ֱ�е��̼��ٶ�����
#define TRANSLATION_ACCELERAD 5.5f //ƽ�Ƶ��̼��ٶ�����
#define ROTATING_ACCELERAD 19.0f   //��ת���̼��ٶ�����

//#define CHASSIS_ROTATION_SPEED_1 (600)
//#define CHASSIS_ROTATION_SPEED_2 (750)
//#define CHASSIS_ROTATION_SPEED_3 (1200) // 900

#define CHASSIS_ROTATION_SPEED 30               //С���ݵ���ת�ٶ�  2000
//#define CHASSIS_ROTATION_MOVE_SPEED (800 * 0.8f) //С�����ƶ�ʱΪ��ֹ�켣ʧ���ת��   1700
//#define CHASSIS_TWIST_SPEED 600                  //Ť���ٶ�  1600

#define MOTOR_3508_CURRENT_LIMIT 15000
void Task_Chassis(void const *argument);

#endif
