#include "Init_Task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"
#include "bsp_referee.h"

/************************* hardware ************************/
#include "can1_receive.h"
#include "can2_receive.h"

/************************* Task ************************/
#include "Task_Safe.h"
#include "chassis_task.h"
#include "imu_task.h"
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
osThreadId UI_TASKHandle;
osThreadId TASK_CHASSISHandle;
osThreadId ShootTask_Handler;
osThreadId IMUTask_Handler;

void Init_Task(void const *argument)
{
	taskENTER_CRITICAL(); //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���

	// CAN�˲�����ʼ��
	CAN1_filter_config();
	CAN2_filter_config();
	
	//ң������ʼ��
	ECF_RC_Init();

	//����ϵͳ
	ECF_referee_uart_init();

	//������ȫ����
	osThreadDef(Safe_TASK, Safe_Task, osPriorityNormal, 0, 128);
	Safe_TASKHandle = osThreadCreate(osThread(Safe_TASK), NULL);

	//������������
	osThreadDef(CHASSIS_TASK, Task_Chassis, osPriorityHigh, 0, 256);
	TASK_CHASSISHandle = osThreadCreate(osThread(CHASSIS_TASK), NULL);

	//����UI����
	//		osThreadDef(UI_TASK, UI_Task, osPriorityLow, 0, 256);
	//		UI_TASKHandle = osThreadCreate(osThread(UI_TASK), NULL);
	
	#ifdef USE_IMU
	osThreadDef(IMU_TASK, imu_Task, osPriorityNormal, 0, 512);
	IMUTask_Handler = osThreadCreate(osThread(IMU_TASK), NULL);
	#endif

	vTaskDelete(Init_TASKHandle); //ɾ����ʼ����
	vTaskDelete(defaultTaskHandle);
	taskEXIT_CRITICAL();		  //�˳��ٽ���
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
	vTaskSuspend(TASK_CHASSISHandle);
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
	vTaskResume(TASK_CHASSISHandle);
	vTaskResume(ShootTask_Handler);

	//ʧ�ر������������������
	//    vTaskSuspend(OutOf_Control_THandle);
}
