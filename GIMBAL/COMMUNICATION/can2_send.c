#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //���͵���Ϣ
extern gimbal_control_t Gimbal_Control;

/**
 * @brief		��̨���ͽǶȸ�����
 * @param		none
 *	@retval		none
 */
void can2_gimbal_to_chassis(void)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[5];		//�������ݵ�����
	int Register; //��ֹ����Ǿ����
	
	Txmessage.StdId = 0x300;	  //����820r���ñ�ʶ��
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 4;

//������ת���ĸ�u8��ͨ���ڴ�Խ��ķ���ʵ��
__asm__ volatile
{
	LDR Register , [&Gimbal_Control.chassis_gimbal_angel]
	STR Register , [Data]
}


	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
