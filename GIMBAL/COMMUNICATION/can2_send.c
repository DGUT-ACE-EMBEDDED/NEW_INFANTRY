#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //���͵���Ϣ
extern gimbal_control_t Gimbal_Control;
/**
 * @brief		����CAN2���ͼ��̵�����
 * @param		none
 *	@retval		none
 */
void can2_gambal_to_chassis(void)
{
	uint32_t send_mail_box; //��������
	uint8_t Data[8];		//�������ݵ�����
	int16_t chassis_gimbal_angel = (int16_t)Gimbal_Control.chassis_gimbal_angel;
	Txmessage.StdId = 0x300;	  //����820r���ñ�ʶ��
	Txmessage.IDE = CAN_ID_STD;	  //ָ����Ҫ�������Ϣ�ı�ʶ��������
	Txmessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Txmessage.DLC = 2;

	Data[0] = (uint8_t)(chassis_gimbal_angel >> 8);
	Data[1] = (uint8_t)(chassis_gimbal_angel);

	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}
