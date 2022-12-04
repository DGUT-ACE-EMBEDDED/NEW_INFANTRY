#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

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
#define CHASSIS_SPIN_FOLLOW_KP 20.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

#define NORMAL_FORWARD_BACK_SPEED 660.0f //键盘普通直行速度
#define NORMAL_LEFT_RIGHT_SPEED 660.0f   //键盘普通平移速度

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f  |  0.0510f
#define CHASSIS_MK_SECOND_FILTERING (0.8f)

/**********************运动加速度限制**********************/

#define STRAIGHT_ACCELERAD 3.5f    //直行底盘加速度限制
#define TRANSLATION_ACCELERAD 5.5f //平移底盘加速度限制
#define ROTATING_ACCELERAD 19.0f   //旋转底盘加速度限制

#define CHASSIS_ROTATION_SPEED_1 (600)
#define CHASSIS_ROTATION_SPEED_2 (750)
#define CHASSIS_ROTATION_SPEED_3 (1200) // 900

#define CHASSIS_ROTATION_SPEED 100               //小陀螺的旋转速度  2000
#define CHASSIS_ROTATION_MOVE_SPEED (800 * 0.8f) //小陀螺移动时为防止轨迹失真减转速   1700
#define CHASSIS_TWIST_SPEED 600                  //扭腰速度  1600

#define MOTOR_3508_CURRENT_LIMIT 15000
void Task_Chassis(void const *argument);

#endif
