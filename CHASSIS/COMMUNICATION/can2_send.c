#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //���͵���Ϣ

/**
 * @brief		����CAN2���ͼ��̵�����
 * @param		none
 *	@retval		none
 */
void can2_chassis_to_gimbal(const RC_ctrl_t *can2_MK_send)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����

	Txmessage.StdId = 0x401;	  //
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 8;

	Data[0] = (can2_MK_send->rc.ch[0] >> 8);
	Data[1] = can2_MK_send->rc.ch[0];
	Data[2] = (can2_MK_send->rc.ch[1] >> 8);
	Data[3] = can2_MK_send->rc.ch[1];
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
