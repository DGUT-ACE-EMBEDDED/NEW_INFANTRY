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
	Data[1] = (can2_MK_send->rc.ch[0]);
	Data[2] = (can2_MK_send->rc.ch[1] >> 8);
	Data[3] = (can2_MK_send->rc.ch[1]);
	Data[4] = (can2_MK_send->rc.s2);
	Data[5] = (can2_MK_send->kb.key_code >> 8);
	Data[6] = (can2_MK_send->kb.key_code);
	Data[7] = 0xAA;

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
	
	Data[0] = (can2_MK_send->mouse.x >> 8);
	Data[1] = (can2_MK_send->mouse.x);
	Data[2] = (can2_MK_send->mouse.y >> 8);
	Data[3] = (can2_MK_send->mouse.y);
	Data[4] = (can2_MK_send->mouse.press_l);
	Data[5] = (can2_MK_send->mouse.press_r);
	Data[6] = (can2_MK_send->rc.ch[4]);
	Data[7] = (can2_MK_send->rc.ch[4] >> 8);
	
	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}

void can2_chassis_to_gimbal_referee(const REFEREE_t *can2_referee_send)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����

	Txmessage.StdId = 0x402;	  //
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 8;

	Data[0] = (can2_referee_send->Robot_Status.shooter_id1_17mm_cooling_limit >> 8);
	Data[1] = (can2_referee_send->Robot_Status.shooter_id1_17mm_cooling_limit);
	Data[2] = (can2_referee_send->Power_Heat.shooter_id1_17mm_cooling_heat >> 8);
	Data[3] = (can2_referee_send->Power_Heat.shooter_id1_17mm_cooling_heat);
	Data[4] = (can2_referee_send->Robot_Status.shooter_id1_17mm_speed_limit);
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0xAA;

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
