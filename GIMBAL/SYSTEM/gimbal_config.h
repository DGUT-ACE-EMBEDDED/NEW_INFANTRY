/*code is far away from bug with the animal protecting
 *  ��������������
 *�����ߩ��������ߩ�
 *������������������ ��
 *������������������
 *�����ש������ס���
 *������������������
 *���������ߡ�������
 *������������������
 *������������������
 *��������������PC��BJ����
 *��������������������BUG��
 *����������������������
 *���������������������ǩ�
 *������������������������
 *���������������ש�����
 *���������ϩϡ����ϩ�
 *���������ߩ������ߩ�
 *������
 */

// #include "gimbal_config.h"
#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#define FIRE_WORK

//#define VIRTUAL_DELAY_COMPENSATE

#define PITCH_USE_PID 			0
#define PITCH_USE_LQR 			1
#define PITCH_CONTROLER 		PITCH_USE_LQR

#define PITCH_USE_ENCODER   0
#define PITCH_USE_IMU       1
#define PITCH_ANGLE_SENSOR  PITCH_USE_IMU

#define YAW_USE_PID 			0
#define YAW_USE_LQR 			1
#define YAW_CONTROLER 		YAW_USE_LQR

#define PITCH_ZERO_OFFSET 100.0f //p����λƫ��ֵ


/**********************pitch��PID����**********************/
#define GIMBAL_PITCH_P_P 120.0f
#define GIMBAL_PITCH_P_I 5.0f
#define GIMBAL_PITCH_P_D 500.0f

#define GIMBAL_PITCH_S_P 20.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************Yaw��PID����**********************/
#define GIMBAL_YAW_P_P 15.0f //200/ //15
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 5.0f

#define GIMBAL_YAW_S_P 75.0f //15 //75
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

#define GIMBAL_YAW_visual_P_P 150.0f
#define GIMBAL_YAW_visual_P_I 1.0f
#define GIMBAL_YAW_visual_P_D 2000.0f

#define GIMBAL_YAW_visual_S_P 20.0f
#define GIMBAL_YAW_visual_S_I 0.0f
#define GIMBAL_YAW_visual_S_D 0.0f
/**********************��̨pitch�Ƕ�����**********************/
#define PITCH_ANGLE_LIMIT_UP 39.0f
#define PITCH_ANGLE_LIMIT_DOWN -29.0f

/**********************�������ң���ٶ�����**********************/
#define MOUSE_YAW_SPEED 0.011f   //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 0.009f //���pitch���ٶ�����
#define RC_YAW_SPEED 0.0003f     //ң����yaw���ٶ�����
#define RC_PITCH_SPEED 0.0005f   //ң����pitch���ٶ�����

/**********************������**********************/
#define FIRE_SPEED_0 0
#define FIRE_SPEED_15 4510
#define FIRE_SPEED_18 5000
#define FIRE_SPEED_22 5550
#define FIRE_SPEED_30 7300

#define REPLENISH_OFF  0x4940
#define REPLENISH_HALF 0x4820
#define REPLENISH_ON   0x4700
#endif
