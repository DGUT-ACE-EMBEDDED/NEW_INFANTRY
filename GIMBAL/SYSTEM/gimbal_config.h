/*code is far away from bug with the animal protecting
 *  ┏┓　　　┏┓
 *┏┛┻━━━┛┻┓
 *┃　　　　　　　┃ 　
 *┃　　　━　　　┃
 *┃　┳┛　┗┳　┃
 *┃　　　　　　　┃
 *┃　　　┻　　　┃
 *┃　　　　　　　┃
 *┗━┓　　　┏━┛
 *　　┃　　　┃PC、BJ保佑
 *　　┃　　　┃代码无BUG！
 *　　┃　　　┗━━━┓
 *　　┃　　　　　　　┣┓
 *　　┃　　　　　　　┏┛
 *　　┗┓┓┏━┳┓┏┛
 *　　　┃┫┫　┃┫┫
 *　　　┗┻┛　┗┻┛
 *　　　
 */

// #include "gimbal_config.h"
#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

//#define FIRE_WORK
#define PITCH_ZERO_OFFSET 100.0f 

/**********************pitch轴PID参数**********************/
#define GIMBAL_PITCH_P_P 20.0f
#define GIMBAL_PITCH_P_I 0.10f
#define GIMBAL_PITCH_P_D 50.0f

#define GIMBAL_PITCH_S_P 75.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 100.0f

/**********************Yaw轴PID参数**********************/
#define GIMBAL_YAW_P_P 15.0f
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 5.0f

#define GIMBAL_YAW_S_P 75.0f
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

/**********************云台pitch角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP 39.0f
#define PITCH_ANGLE_LIMIT_DOWN -29.0f

/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED 0.011f   //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED 0.009f //鼠标pitch轴速度增益
#define RC_YAW_SPEED 0.0003f     //遥控器yaw轴速度增益
#define RC_PITCH_SPEED 0.0005f   //遥控器pitch轴速度增益

#endif
