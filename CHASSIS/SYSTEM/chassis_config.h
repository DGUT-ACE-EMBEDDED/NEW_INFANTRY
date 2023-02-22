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

//功耗控制，依赖于裁判系统 还有bug
//#define POWER_CONTROL

//yaw归中编码值
#define YAW_ZERO_OFFSET 8015

//3508最大电流值
#define MOTOR_3508_CURRENT_LIMIT 15000

/************底盘pid************/
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

//底盘速度pid
#define CHASSIS_LOCATION_KP 8.0f
#define CHASSIS_LOCATION_KI 0.0f
#define CHASSIS_LOCATION_KD 10.8f

//旋转pid
#define CHASSIS_SPIN_FOLLOW_KP 100.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

#define NORMAL_FORWARD_BACK_SPEED 660.0f //键盘普通直行速度
#define NORMAL_LEFT_RIGHT_SPEED 660.0f   //键盘普通平移速度

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高

//小陀螺的旋转速度
#define CHASSIS_ROTATION_SPEED 30               


#endif
