#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

/**********************pitch��PID����**********************/
#define GIMBAL_PITCH_P_P 3.0f // Pλ�û�
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 50.0f

#define GIMBAL_PITCH_S_P 80.0f // P�ٶȻ�(��Ҫ��i)
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************Yaw��PID����**********************/
#define GIMBAL_YAW_P_P 2.0f  // 15.0f   //3 1.0f    //Yλ�û�   //12
#define GIMBAL_YAW_P_I 0.0f  // 0.5f//1.0f     //1.0
#define GIMBAL_YAW_P_D 45.0f // 0.0f   //45.0f//35.0f//30.0f    //10

#define GIMBAL_YAW_S_P 100.0f // 20�����г�����ת��//Y�ٶȻ�
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

void Gimbal_Task(void const *argument);

#endif
