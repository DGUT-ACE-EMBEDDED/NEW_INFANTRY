#include "Task_Safe.h"

/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/**
 * @brief      ʧ�ر����������
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
	static TickType_t currentTime;

	currentTime = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��

	while (1)
	{

		//�������
		vTaskDelayUntil(&currentTime, 1); //������ʱ//vTaskDelay(2)
	}
}
