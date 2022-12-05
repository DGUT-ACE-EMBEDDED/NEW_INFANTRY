#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

/**********************pitch轴PID参数**********************/
#define GIMBAL_PITCH_P_P 3.0f // P位置环
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 50.0f

#define GIMBAL_PITCH_S_P 80.0f // P速度环(不要加i)
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************Yaw轴PID参数**********************/
#define GIMBAL_YAW_P_P 2.0f  // 15.0f   //3 1.0f    //Y位置环   //12
#define GIMBAL_YAW_P_I 0.0f  // 0.5f//1.0f     //1.0
#define GIMBAL_YAW_P_D 45.0f // 0.0f   //45.0f//35.0f//30.0f    //10

#define GIMBAL_YAW_S_P 100.0f // 20不会有齿轮类转动//Y速度环
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

void Gimbal_Task(void const *argument);

#endif
