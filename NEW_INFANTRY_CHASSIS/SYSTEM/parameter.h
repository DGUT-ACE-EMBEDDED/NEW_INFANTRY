//_ooOoo_
//o8888888o
//88" . "88
//(| -_- |)
// O\ = /O
//___/`---'\____
//.   ' \\| |// `.
/// \\||| : |||// \
/// _||||| -:- |||||- \
//| | \\\ - /// | |
//| \_| ''\---/'' | |
//\ .-\__ `-` ___/-. /
//___`. .' /--.--\ `. . __
//."" '< `.___\_<|>_/___.' >'"".
//| | : `- \`.;`\ _ /`;.`/ - ` : | |
//\ \ `-. \_ __\ /__ _/ .-` / /
//======`-.____`-.___\_____/___.-`____.-'======
//`=---='
//
//         .............................................
//          佛曰：bug泛滥，我已瘫痪！

#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "main.h"



/*模块工作属性*/
//#define WATCH_DOG                //启动看门狗
//#define FIRE_WORK                //射弹模式开启 (开拨弹轮)
#define POWER_LIMIT              //启动功率限制   
#define IMU_WORK                 //启动陀螺仪



/************电机 传动比*减速比 ***************/
#define PITCH_RATIO      (1)//(500*3*19)         //Yaw轴
#define YAW_RATIO        (1)         //Yaw轴
//#define Sec_YAW_RATIO  (3*1)          //副Yaw轴



/* 底盘电机移动速度设定 */
#define M3508_MAX_OUTPUT_CURRENT  (5000)   //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  (9500)   //m2006电机最大电流输出

#define MAX_MOTOR_CAN_INPUT    (2000.0f)   //3508最大电流输入
#define MAX_MOTOR_CAN_OUTPUT   (10000.0f)//(16000.0f)  //3508最大电流输出



/*信息输出*/
#define NQ_DEBUG_ON 1

#define NQ_PRINTF(fmt, arg...) \
    do                         \
    {                          \
        if (NQ_DEBUG_ON)     \
            printf(fmt, ##arg); \
    } while (0)




#endif 
