#include "can2_receive.h"
#include "can.h"
#include "gimbal_struct_variables.h"
#include "bsp_Motor_Encoder.h"
#include "bsp_dr16.h"
#include "Task_Safe.h"
extern CAN_HandleTypeDef hcan2;
extern gimbal_control_t Gimbal_Control;
extern REFEREE_t referee;

/*--------------------����-----------------------*/
static motor_measure_t yaw_motor_measure;

motor_measure_t *get_yaw_motor_measure_point(void)
{
	return &yaw_motor_measure;
}
/**
 * @brief		can2�˲�������
 * @param		none
 *	@retval		none
 */

void CAN2_filter_config(void)
{
	CAN_FilterTypeDef CAN2_FIilter_InitStruct;

	// �����˲���
	CAN2_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 // ����ģʽ
	CAN2_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ����
	CAN2_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN2_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN2_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN2_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN2_FIilter_InitStruct.FilterBank = 14; // CAN2 �˲�����Ϊ14
	CAN2_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN2_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0; // ָ����������
	CAN2_FIilter_InitStruct.FilterActivation = ENABLE;
	

	HAL_CAN_ConfigFilter(&hcan2, &CAN2_FIilter_InitStruct);			   // ����ָ������CAN���չ�����
	HAL_CAN_Start(&hcan2);											   // ����can2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // �����ж�
}

/*************************************can2����*************************************/
/**
 * @brief      ���̰�can2���ջص�����
 * @param[in]  *rx_message: can2���սṹ��
 * @retval     none
 * @attention  �� chassis_app.h ��ע�ᣬ�жϵ��ã�can2_callback(&rx2_message);��
 */
void chassis_can2_callback(CAN_HandleTypeDef *hcan)
{
	static RC_ctrl_t *rc_ctl;
	rc_ctl = RC_Get_RC_Pointer();
	CAN_RxHeaderTypeDef Rxmessage; // ������Ϣ�ṹ��
	uint8_t Rx_Data[8];			   // ���յ���Ϣ���������
//	SEGGER_SYSVIEW_RecordEnterISR();
	while (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK ) // ��ȡ���յ���Ϣ
	{
		switch (Rxmessage.StdId)
		{
		case 0x433:
			rc_lost_time_refresh();
//			Lost_time_fresh_by_name((uint8_t*)"rc");
			rc_ctl->mouse.x = ((Rx_Data[0] << 8) | Rx_Data[1]);
			rc_ctl->mouse.y = ((Rx_Data[2] << 8) | Rx_Data[3]);
			rc_ctl->mouse.press_l = Rx_Data[4];
			rc_ctl->mouse.press_r = Rx_Data[5];
			rc_ctl->rc.ch[4] = ((Rx_Data[7] << 8) | Rx_Data[6]);
			break;
		case 0x411:
			rc_lost_time_refresh();
//			Lost_time_fresh_by_name((uint8_t*)"rc");
			rc_ctl->rc.ch[0] = ((Rx_Data[0] << 8) | Rx_Data[1]);
			rc_ctl->rc.ch[1] = ((Rx_Data[2] << 8) | Rx_Data[3]);
			rc_ctl->rc.s2 = Rx_Data[4];
			rc_ctl->kb.key_code = ((Rx_Data[5] << 8) | Rx_Data[6]);
			break;

		case 0x206: // Y��
		{
//			Lost_time_fresh_by_name("y��");
			yaw_motor_measure.position = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			yaw_motor_measure.speed = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			yaw_motor_measure.current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
			yaw_motor_measure.temperature = Rx_Data[6];
			CAN_DATA_Encoder_Deal(yaw_motor_measure.position, yaw_motor_measure.speed, 2);
			break;
		}
		case 0x422:
		{
//			Lost_time_fresh_by_name((uint8_t*)"referee");
			referee.Robot_Status.shooter_id1_17mm_cooling_limit = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			referee.Power_Heat.shooter_id1_17mm_cooling_heat = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			referee.Robot_Status.shooter_id1_17mm_speed_limit = (int16_t)(Rx_Data[4]);
			break;
		}
		default :
			break;
		}
	}
//	SEGGER_SYSVIEW_RecordExitISR();
}
