#include "Task_Safe.h"
#include "stdlib.h"
#include "string.h"
#include "tim.h"
#include "bsp_buzzer.h"

uint32_t *stack_base_p = 0x00000000;
uint32_t stack_base;
uint32_t heap_free;
uint32_t stack_now;
uint32_t stack_free = 0;

extern QueueHandle_t safe_task_queue;

static int my_heapprt(void *param, char const *format, ...)
{
    if ((*(int *)param) != 0)
        return 0;
    va_list argp;
    va_start(argp, format);
    int data = va_arg(argp, int);
    va_end(argp);
    *(int *)param = data;
    return -1;
}

int heap_free_size()
{
    int size = 0;
    __heapstats((__heapprt)my_heapprt, &size);
    return size;
}

/**
 * @brief      安全任务
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
    uint8_t error_object_flag = 0;
    stack_base = *stack_base_p;
    safe_message_queue rec_message;
    //	static TickType_t currentTime;
    safe_task_queue = xQueueCreate(10, sizeof(safe_message_queue));

    //	currentTime = xTaskGetTickCount();
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    /*------------can1----------*/
//    Protect_object_create(300, "p轴");

    /*------------can2----------*/
//    Protect_object_create(100, "裁判系统");
//    Protect_object_create(100, "遥控1");
//    Protect_object_create(100, "遥控2");
//    Protect_object_create(100, "y轴");

    while (1)
    {
        stack_now = __get_MSP();
        stack_free = 0x2000 - (stack_base - stack_now);
        heap_free = heap_free_size();

//        Lost_check_all();
//        error_object_flag = 0;
//        while (xQueueReceive(safe_task_queue, &rec_message, 0) == pdTRUE)
//        {
//            if (rec_message.error == safe_ERROR)
//            {
//                error_object_flag = 1;
//            }
//        }
//        if (error_object_flag)
//        {
//            buzzer_on(30, 500);
//        }
//        else
//        {
//            buzzer_off();
//        }

        vTaskDelay(1);
    }
}
