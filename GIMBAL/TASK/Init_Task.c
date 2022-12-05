#include "Init_Task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

/************************* hardware ************************/
#include "can1_receive.h"
#include "can2_receive.h"

/************************* Task ************************/
#include "Task_Safe.h"
#include "gimbal_task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

//#include "shoot_Task.h"

extern osThreadId Init_TASKHandle;
extern osThreadId defaultTaskHandle;

osThreadId Safe_TASKHandle;
osThreadId IMUTask_Handler;
osThreadId TASK_GIMBALHandle;
osThreadId ShootTask_Handler;

void Init_Task(void const *argument)
{
	taskENTER_CRITICAL(); //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���

	// CAN�˲�����ʼ��
	CAN1_filter_config();
	CAN2_filter_config();

	//ң������ʼ��
	ECF_RC_Init();

	//����ϵͳ
	//		referee_system_init();

	//������ȫ����
	osThreadDef(Safe_TASK, Safe_Task, osPriorityNormal, 0, 128);
	Safe_TASKHandle = osThreadCreate(osThread(Safe_TASK), NULL);

	//������̨����
	osThreadDef(Gimbal_TASK, Gimbal_Task, osPriorityHigh, 0, 256);
	TASK_GIMBALHandle = osThreadCreate(osThread(Gimbal_TASK), NULL);

	// IMU
	osThreadDef(IMU_TASK, imu_Task, osPriorityNormal, 0, 512);
	IMUTask_Handler = osThreadCreate(osThread(IMU_TASK), NULL);

#ifdef FIRE_WORK //����
				 //�����������
				 //		osThreadDef(SHOOT_TASK, shoot_Task, osPriorityAboveNormal, 0, 128);
				 //		ShootTask_Handler = osThreadCreate(osThread(SHOOT_TASK), NULL);

#endif

	vTaskDelete(Init_TASKHandle); //ɾ����ʼ����
	vTaskDelete(defaultTaskHandle);
	taskEXIT_CRITICAL(); //�˳��ٽ���
}

/**
 * @brief      ʧ�ش���
 * @param[in]  none
 * @retval     none
 * @attention
 */
void out_of_control(void)
{
	//���������
	vTaskSuspend(TASK_GIMBALHandle);
	vTaskSuspend(ShootTask_Handler);

	//���ʧ�ر�����������
	//    vTaskResume(OutOf_Control_THandle);
}

/**
 * @brief      ���� | ���ʧ��
 * @param[in]  none
 * @retval     none
 * @attention
 */
void normal_control(void)
{
	//�������
	vTaskResume(TASK_GIMBALHandle);
	vTaskResume(ShootTask_Handler);

	//ʧ�ر������������������
	//    vTaskSuspend(OutOf_Control_THandle);
}
