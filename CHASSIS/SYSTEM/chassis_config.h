/*code is far away from bug with the animal protecting
 *  ��������������
 *�����ߩ��������ߩ�
 *������������������ ��
 *������������������
 *�����ש������ס���
 *������������������
 *���������ߡ�������
 *������������������
 *������������������
 *��������������PC��BJ����
 *��������������������BUG��
 *����������������������
 *���������������������ǩ�
 *������������������������
 *���������������ש�����
 *���������ϩϡ����ϩ�
 *���������ߩ������ߩ�
 *������
 */
#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#define WHEEL_RADIUS 0.087f
#define PI 3.1415926f
#define CHASSIS_MOTOR_REDUCATION_RATIO 19.0f

#define MOUSE_YAW_SPEED 0.011f   //���yaw���ٶ�����
#define RC_YAW_SPEED 0.0003f     //ң����yaw���ٶ�����

#define MOTOR_3508_CURRENT_LIMIT 10000 //3508������ֵ

#define YAW_ZERO_OFFSET 5960 //yaw���б���ֵ

//#define GIMBAL_MOTION_PREDICT  //��̨�˶�����,�ڸ���ģʽyaw���˶�ʱ,���̻�������̨ת��,������̨�����֮����໥����,������IMU
//#define POWER_CONTROL //���Ŀ��ƣ������ڲ���ϵͳ ����bug
#define ACCEL_CONTROL //���ٶȿ���,������IMU
#define USE_IMU //������

/************����pid************/
//���������pid
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

//�����ٶ�pid
#define CHASSIS_PSEED_X_KP 4.0f
#define CHASSIS_PSEED_X_KI 0.0f
#define CHASSIS_PSEED_X_KD 10.8f

#define CHASSIS_PSEED_Y_KP 8.0f
#define CHASSIS_PSEED_Y_KI 0.0f
#define CHASSIS_PSEED_Y_KD 10.8f

//��תpid
#define CHASSIS_SPIN_FOLLOW_KP 50.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���

//С���ݵ���ת�ٶ�
#define CHASSIS_ROTATION_SPEED 30               


#endif
