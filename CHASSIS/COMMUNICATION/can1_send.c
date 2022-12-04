#include "can1_send.h"
#include "chassis_struct_variables.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //发送的信息

/**
 * @brief		底盘 CAN1 发送信息（电流）给电机       0x200-3508
 * @param		none
 *	@retval		none
 */
void can1_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data[8];		//发送数据的数组

	Txmessage.StdId = 0x200;	  //根据820r设置标识符
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 8;

	Data[0] = (unsigned char)(ESC_201 >> 8);
	Data[1] = (unsigned char)ESC_201;
	Data[2] = (unsigned char)(ESC_202 >> 8);
	Data[3] = (unsigned char)ESC_202;
	Data[4] = (unsigned char)(ESC_203 >> 8);
	Data[5] = (unsigned char)ESC_203;
	Data[6] = (unsigned char)(ESC_204 >> 8);
	Data[7] = (unsigned char)ESC_204;

	HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}

/**
 * @brief		CAN1 发送电容的值
 * @param		none
 *	@retval		none
 */
void can1_cap_setmsg(int16_t Chassis_power)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data[8];		//发送数据的数组

	Chassis_power = Chassis_power * 100;

	Txmessage.StdId = 0x210;	  //根据820r设置标识符   （根据情况修改）
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 2;

	Data[0] = (Chassis_power >> 8);
	Data[1] = Chassis_power;

	HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}
