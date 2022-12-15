#include "Task_Safe.h"
#include "tim.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"



uint32_t* stack_base_p = 0x00000000 ;
uint32_t stack_base ;
uint32_t heap_base ;

uint32_t stack_now ;
uint32_t stack_free = 0;

 
/**
 * @brief      ʧ�ر����������
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
	stack_base = *stack_base_p;
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);//���TIM��ʱ����־λ
  HAL_TIM_Base_Start_IT(&htim2);//�����ڲ���ʱ��tim2
  

	//	static TickType_t currentTime;

	//	currentTime = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��

	while (1)
	{
		
		//�������
		// vTaskDelayUntil(&currentTime, 1); //������ʱ//vTaskDelay(2)		
		vTaskDelay(2);
	}
}

void TIM2_IRQ_safe(void)
{
	stack_now = __get_MSP();

}
