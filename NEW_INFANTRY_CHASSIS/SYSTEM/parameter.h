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
//          ��Ի��bug���ģ�����̱����

#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "main.h"



/*ģ�鹤������*/
//#define WATCH_DOG                //�������Ź�
//#define FIRE_WORK                //�䵯ģʽ���� (��������)
#define POWER_LIMIT              //������������   
#define IMU_WORK                 //����������



/************��� ������*���ٱ� ***************/
#define PITCH_RATIO      (1)//(500*3*19)         //Yaw��
#define YAW_RATIO        (1)         //Yaw��
//#define Sec_YAW_RATIO  (3*1)          //��Yaw��



/* ���̵���ƶ��ٶ��趨 */
#define M3508_MAX_OUTPUT_CURRENT  (5000)   //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  (9500)   //m2006������������

#define MAX_MOTOR_CAN_INPUT    (2000.0f)   //3508����������
#define MAX_MOTOR_CAN_OUTPUT   (10000.0f)//(16000.0f)  //3508���������



/*��Ϣ���*/
#define NQ_DEBUG_ON 1

#define NQ_PRINTF(fmt, arg...) \
    do                         \
    {                          \
        if (NQ_DEBUG_ON)     \
            printf(fmt, ##arg); \
    } while (0)




#endif 
