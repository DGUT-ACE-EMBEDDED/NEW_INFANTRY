#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

/**********************pitch��PID����**********************/
#define GIMBAL_PITCH_P_P 3.0f
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 50.0f

#define GIMBAL_PITCH_S_P 80.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************Yaw��PID����**********************/
#define GIMBAL_YAW_P_P 30.0f
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 5.0f

#define GIMBAL_YAW_S_P 50.0f
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

/**********************��̨pitch�Ƕ�����**********************/
#define PITCH_ANGLE_LIMIT_UP 45.0f
#define PITCH_ANGLE_LIMIT_DOWN -30.0f

/**********************�������ң���ٶ�����**********************/
#define MOUSE_YAW_SPEED 0.011f   //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 0.009f //���pitch���ٶ�����
#define RC_YAW_SPEED 0.0005f     //ң����yaw���ٶ�����
#define RC_PITCH_SPEED 0.0005f   //ң����pitch���ٶ�����

void Gimbal_Task(void const *argument);

#endif
