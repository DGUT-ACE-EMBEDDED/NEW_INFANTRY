#include "can2_receive.h"
#include "can.h"
#include "parameter.h"
#include "struct_variables.h"

extern CAN_HandleTypeDef hcan2;

/*--------------------变量-----------------------*/

/**
 * @brief		can2滤波器配置
 * @param		none
 *	@retval		none
 */

void CAN2_filter_config(void)
{
  CAN_FilterTypeDef CAN2_FIilter_InitStruct;

  //开启滤波器
  CAN2_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;  //掩码模式
  CAN2_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32位工作
  CAN2_FIilter_InitStruct.FilterIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterBank = 14; // CAN2 滤波器组为14
  CAN2_FIilter_InitStruct.SlaveStartFilterBank = 14;
  CAN2_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0; //指定接收邮箱
  CAN2_FIilter_InitStruct.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &CAN2_FIilter_InitStruct);            //根据指定配置CAN接收过滤器
  HAL_CAN_Start(&hcan2);                                             //开启can2
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //启动中断
}

/*************************************can2接收*************************************/
/**
 * @brief      底盘板can2接收回调函数
 * @param[in]  *rx_message: can2接收结构体
 * @retval     none
 * @attention  在 chassis_app.h 中注册，中断调用（can2_callback(&rx2_message);）
 */
void chassis_can2_callback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef Rxmessage; //接收信息结构体
  uint8_t Rx_Data[8];            //接收的信息缓存的数组

  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //读取接收的信息
  {
    switch (Rxmessage.StdId)
    {
    }
  }
}
