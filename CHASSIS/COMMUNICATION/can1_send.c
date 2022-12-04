#include "can1_send.h"
#include "chassis_struct_variables.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //���͵���Ϣ

/**
 * @brief		���� CAN1 ������Ϣ�������������       0x200-3508
 * @param		none
 *	@retval		none
 */
void can1_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����

	Txmessage.StdId = 0x200;	  //����820r���ñ�ʶ��
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 8;

	Data[0] = (unsigned char)(ESC_201 >> 8);
	Data[1] = (unsigned char)ESC_201;
	Data[2] = (unsigned char)(ESC_202 >> 8);
	Data[3] = (unsigned char)ESC_202;
	Data[4] = (unsigned char)(ESC_203 >> 8);
	Data[5] = (unsigned char)ESC_203;
	Data[6] = (unsigned char)(ESC_204 >> 8);
	Data[7] = (unsigned char)ESC_204;

	HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}

/**
 * @brief		CAN1 ���͵��ݵ�ֵ
 * @param		none
 *	@retval		none
 */
void can1_cap_setmsg(int16_t Chassis_power)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����

	Chassis_power = Chassis_power * 100;

	Txmessage.StdId = 0x210;	  //����820r���ñ�ʶ��   ����������޸ģ�
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 2;

	Data[0] = (Chassis_power >> 8);
	Data[1] = Chassis_power;

	HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
