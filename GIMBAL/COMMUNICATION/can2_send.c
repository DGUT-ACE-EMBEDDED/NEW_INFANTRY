#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //发送的信息
extern gimbal_control_t Gimbal_Control;
/**
 * @brief		底盘CAN2发送键盘的数据
 * @param		none
 *	@retval		none
 */
void can2_gambal_to_chassis(void)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data[8];		//发送数据的数组
	int16_t chassis_gimbal_angel = (int16_t)Gimbal_Control.chassis_gimbal_angel;
	Txmessage.StdId = 0x300;	  //根据820r设置标识符
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 2;

	Data[0] = (uint8_t)(chassis_gimbal_angel >> 8);
	Data[1] = (uint8_t)(chassis_gimbal_angel);

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}
