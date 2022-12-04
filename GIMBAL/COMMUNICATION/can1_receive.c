#include "can1_receive.h"
#include "can2_receive.h"
#include "gimbal_struct_variables.h"
#include "bsp_Motor_Encoder.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void chassis_can2_callback(CAN_HandleTypeDef *hcan);

/*--------------------变量-----------------------*/

/**
 * @brief		CAN1滤波器配置
 * @param		none
 *	@retval		none
 */
void CAN1_filter_config(void)
{
	CAN_FilterTypeDef CAN1_FIilter_InitStruct;

	CAN1_FIilter_InitStruct.FilterActivation = ENABLE;			 //开启滤波器
	CAN1_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 //掩码模式
	CAN1_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32位工作
	CAN1_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterBank = 0;
	//	CAN1_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN1_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0;	   //指定接收邮箱
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FIilter_InitStruct);			   //根据指定配置CAN接收过滤器
	HAL_CAN_Start(&hcan1);											   //开启can1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //启动中断
}

/**
 * @brief		HAl库can1的回调函数
 * @param		传入参数： CAN的句柄
 * @retval   none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		chassis_can1_callback(hcan);
	}
	if (hcan == &hcan2)
	{
		chassis_can2_callback(hcan);
	}
}

/*************************************can1接收*************************************/
/**
 * @brief      底盘板can1接收回调函数
 * @param[in]  *rx_message: can1接收结构体
 * @retval     none
 * @attention  在 chassis_app.h 中注册，中断调用（can1_callback(&rx1_message);）
 */
void chassis_can1_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Rxmessage; //接收信息结构体
	uint8_t Rx_Data[8];			   //接收的信息缓存的数组

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //读取接收的信息
	{
		switch (Rxmessage.StdId)
		{
			
		}
	}
}
