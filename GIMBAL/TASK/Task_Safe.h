/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       Task_Safe.c/h
  * @brief      ��ȫ����
  * @note       �ϲ��汾
  * @history
  *
  @verbatim   v1.0
  ==============================================================================

  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  */
#ifndef __TASK_SAFE_H // ���δ����
#define __TASK_SAFE_H // ��ô����
#include "main.h"
#include "Init_Task.h"
#include "safe_check.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/*OS�������������Լ�����ʱ��*/
#define LED_TASK_INIT_TIME 5
#define LED_CONTROL_TIME 5

void Safe_Task(void const *argument);
#endif
