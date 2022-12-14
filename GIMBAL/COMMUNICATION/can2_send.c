#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //发送的信息
extern gimbal_control_t Gimbal_Control;

/**
 * @brief		云台发送角度给底盘
 * @param		none
 *	@retval		none
 */
void can2_gimbal_to_chassis(void)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data[5];		//发送数据的数组
	int Register; //防止汇编那警告的
	
	Txmessage.StdId = 0x300;	  //根据820r设置标识符
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 4;

//将浮点转成四个u8，通过内存越界的方法实现
__asm__ volatile
{
	LDR Register , [&Gimbal_Control.chassis_gimbal_angel]
	STR Register , [Data]
}


	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}
