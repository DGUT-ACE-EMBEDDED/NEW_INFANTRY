#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

/**********************pitch轴PID参数**********************/
#define GIMBAL_PITCH_P_P 3.0f
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 50.0f

#define GIMBAL_PITCH_S_P 80.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************Yaw轴PID参数**********************/
#define GIMBAL_YAW_P_P 30.0f
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 5.0f

#define GIMBAL_YAW_S_P 50.0f
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

/**********************云台pitch角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP 45.0f
#define PITCH_ANGLE_LIMIT_DOWN -30.0f

/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED 0.011f   //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED 0.009f //鼠标pitch轴速度增益
#define RC_YAW_SPEED 0.0005f     //遥控器yaw轴速度增益
#define RC_PITCH_SPEED 0.0005f   //遥控器pitch轴速度增益

void Gimbal_Task(void const *argument);

#endif
