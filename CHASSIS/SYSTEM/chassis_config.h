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
#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#define WHEEL_RADIUS 0.087f
#define PI 3.1415926f
#define CHASSIS_MOTOR_REDUCATION_RATIO 19.0f

#define MOUSE_YAW_SPEED 0.011f   //鼠标yaw轴速度增益
#define RC_YAW_SPEED 0.0003f     //遥控器yaw轴速度增益

#define MOTOR_3508_CURRENT_LIMIT 10000 //3508最大电流值

#define YAW_ZERO_OFFSET 5960 //yaw归中编码值

//#define GIMBAL_MOTION_PREDICT  //云台运动控制,在跟随模式yaw轴运动时,底盘会先于云台转动,减少云台与地盘之间的相互干扰,依赖于IMU
//#define POWER_CONTROL //功耗控制，依赖于裁判系统 还有bug
#define ACCEL_CONTROL //加速度控制,依赖于IMU
#define USE_IMU //陀螺仪

/************底盘pid************/
//电机电流环pid
#define CHASSIS_RF_MOTOR_KP 3.0f
#define CHASSIS_RF_MOTOR_KI 0.0f
#define CHASSIS_RF_MOTOR_KD 0.5f

#define CHASSIS_LF_MOTOR_KP 3.0f
#define CHASSIS_LF_MOTOR_KI 0.0f
#define CHASSIS_LF_MOTOR_KD 0.5f

#define CHASSIS_RB_MOTOR_KP 3.0f
#define CHASSIS_RB_MOTOR_KI 0.0f
#define CHASSIS_RB_MOTOR_KD 0.5f

#define CHASSIS_LB_MOTOR_KP 3.0f
#define CHASSIS_LB_MOTOR_KI 0.0f
#define CHASSIS_LB_MOTOR_KD 0.5f

//底盘速度pid
#define CHASSIS_PSEED_X_KP 4.0f
#define CHASSIS_PSEED_X_KI 0.0f
#define CHASSIS_PSEED_X_KD 10.8f

#define CHASSIS_PSEED_Y_KP 8.0f
#define CHASSIS_PSEED_Y_KI 0.0f
#define CHASSIS_PSEED_Y_KD 10.8f

//旋转pid
#define CHASSIS_SPIN_FOLLOW_KP 50.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高

//小陀螺的旋转速度
#define CHASSIS_ROTATION_SPEED 30               


#endif
