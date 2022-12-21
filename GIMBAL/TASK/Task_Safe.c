#include "Task_Safe.h"
#include "stdlib.h"
#include "string.h"
#include "tim.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"



uint32_t* stack_base_p = 0x00000000 ;
uint32_t stack_base ;
uint32_t heap_free ;

uint32_t stack_now ;
uint32_t stack_free = 0;

 static int my_heapprt(void *param, char const *format, ...)
{
    if((*(int*)param)!=0) return 0;
    va_list argp;
    va_start( argp, format );
    int data = va_arg( argp, int);
    va_end( argp );
    *(int*) param = data;
    return -1;
}
//��ȡ���жѴ�С
int heap_free_size()
{
    int size = 0;
    __heapstats((__heapprt)my_heapprt, &size);
    return size;
}
/**
 * @brief      ʧ�ر����������
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
	stack_base = *stack_base_p;

	//	static TickType_t currentTime;

	//	currentTime = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��

	while (1)
	{
		stack_now = __get_MSP();
		stack_free = 0x2000 - (stack_base - stack_now);
		heap_free = heap_free_size();
		//�������
		// vTaskDelayUntil(&currentTime, 1); //������ʱ//vTaskDelay(2)		
		vTaskDelay(2);
	}
}


