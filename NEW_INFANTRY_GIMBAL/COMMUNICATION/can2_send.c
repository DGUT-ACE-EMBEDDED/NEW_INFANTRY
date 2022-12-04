#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //���͵���Ϣ

/**
 * @brief		����CAN2���ͼ��̵�����
 * @param		none
 *	@retval		none
 */
void can2_chassis_to_gambal(const RC_ctrl_t *can2_MK_send)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����

	Txmessage.StdId = 0x401;	  //����820r���ñ�ʶ��
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 8;

	Data[0] = (can2_MK_send->mouse.x >> 8);
	Data[1] = can2_MK_send->mouse.x;
	Data[2] = (can2_MK_send->mouse.y >> 8);
	Data[3] = can2_MK_send->mouse.y;
	Data[4] = (can2_MK_send->kb.key_code >> 8);
	Data[5] = can2_MK_send->kb.key_code;
	Data[6] = (can2_MK_send->rc.s1) * 10 + (can2_MK_send->rc.s2);
	Data[7] = ((can2_MK_send->mouse.press_l + 1) * 10 + can2_MK_send->mouse.press_r);

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
