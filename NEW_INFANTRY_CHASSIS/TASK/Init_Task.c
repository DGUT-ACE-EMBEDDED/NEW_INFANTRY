#include "Init_Task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

/************************* hardware ************************/
#include "can1_receive.h"
#include "can2_receive.h"

/************************* Task ************************/
#include "Task_Safe.h"
#include "chassis_task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

//#include "shoot_Task.h"

extern osThreadId Init_TASKHandle;

osThreadId Safe_TASKHandle;
osThreadId UI_TASKHandle;
osThreadId TASK_CHASSISHandle;
osThreadId ShootTask_Handler;

void Init_Task(void const *argument)
{
	taskENTER_CRITICAL(); //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区

	// CAN滤波器初始化
	CAN1_filter_config();
	CAN2_filter_config();

	//遥控器初始化
	remote_control_init();

	//裁判系统
	//		referee_system_init();

	//创建安全任务
	osThreadDef(Safe_TASK, Safe_Task, osPriorityNormal, 0, 128);
	Safe_TASKHandle = osThreadCreate(osThread(Safe_TASK), NULL);

	//创建底盘任务
	osThreadDef(CHASSIS_TASK, Task_Chassis, osPriorityHigh, 0, 256);
	TASK_CHASSISHandle = osThreadCreate(osThread(CHASSIS_TASK), NULL);

	//创建UI任务
	//		osThreadDef(UI_TASK, UI_Task, osPriorityLow, 0, 256);
	//		UI_TASKHandle = osThreadCreate(osThread(UI_TASK), NULL);

#ifdef FIRE_WORK //火力
	//创建火控任务
	//		osThreadDef(SHOOT_TASK, shoot_Task, osPriorityAboveNormal, 0, 128);
	//		ShootTask_Handler = osThreadCreate(osThread(SHOOT_TASK), NULL);

#endif

	vTaskDelete(Init_TASKHandle); //删除开始任务
	taskEXIT_CRITICAL();		  //退出临界区
}

/**
 * @brief      失控处理
 * @param[in]  none
 * @retval     none
 * @attention
 */
void out_of_control(void)
{
	//将任务挂起
	vTaskSuspend(TASK_CHASSISHandle);
	vTaskSuspend(ShootTask_Handler);

	//解挂失控保护控制任务
	//    vTaskResume(OutOf_Control_THandle);
}

/**
 * @brief      正常 | 解除失控
 * @param[in]  none
 * @retval     none
 * @attention
 */
void normal_control(void)
{
	//解挂任务
	vTaskResume(TASK_CHASSISHandle);
	vTaskResume(ShootTask_Handler);

	//失控保护控制任务任务挂起
	//    vTaskSuspend(OutOf_Control_THandle);
}
