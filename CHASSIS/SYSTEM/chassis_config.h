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

//���Ŀ��ƣ������ڲ���ϵͳ ����bug
//#define POWER_CONTROL

//yaw���б���ֵ
#define YAW_ZERO_OFFSET 8015

//3508������ֵ
#define MOTOR_3508_CURRENT_LIMIT 15000

/************����pid************/
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

//�����ٶ�pid
#define CHASSIS_LOCATION_KP 8.0f
#define CHASSIS_LOCATION_KI 0.0f
#define CHASSIS_LOCATION_KD 10.8f

//��תpid
#define CHASSIS_SPIN_FOLLOW_KP 100.0f // 9.0f//7.0f
#define CHASSIS_SPIN_FOLLOW_KI 0.0f  // 0.01f
#define CHASSIS_SPIN_FOLLOW_KD 1.0f  // 0.5f//20.0f

#define NORMAL_FORWARD_BACK_SPEED 660.0f //������ֱͨ���ٶ�
#define NORMAL_LEFT_RIGHT_SPEED 660.0f   //������ͨƽ���ٶ�

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���

//С���ݵ���ת�ٶ�
#define CHASSIS_ROTATION_SPEED 30               


#endif
