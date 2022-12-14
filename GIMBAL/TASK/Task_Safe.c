#include "Task_Safe.h"
#include "tim.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

extern int __initial_sp;
extern int __stack_base;
extern int __heap_base;
extern int __heap_limit;


uint32_t stack_base ;
uint32_t heap_base ;

uint32_t stack_now ;
uint32_t stack_free = 0;

 
/**
 * @brief      失控保护检测任务
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
	heap_base = *(int *)__heap_base;
	stack_base = *(int *)__stack_base;
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);//清除TIM定时器标志位
  HAL_TIM_Base_Start_IT(&htim2);//开启内部定时器tim2
  

	//	static TickType_t currentTime;

	//	currentTime = xTaskGetTickCount(); //获取当前系统时间

	while (1)
	{
		
		//检测周期
		// vTaskDelayUntil(&currentTime, 1); //绝对延时//vTaskDelay(2)		
		vTaskDelay(2);
	}
}

void TIM2_IRQ_safe(void)
{
	SEGGER_SYSVIEW_RecordEnterISR();
						 __get_PSP();
	stack_now = __get_MSP();
	stack_free = 0x2000 - (stack_base - stack_now);
	SEGGER_SYSVIEW_RecordExitISR();
}
