#include "can1_receive.h"
#include "can2_receive.h"
#include "gimbal_struct_variables.h"
#include "bsp_Motor_Encoder.h"
#include "Task_Safe.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void chassis_can2_callback(CAN_HandleTypeDef *hcan);

static motor_measure_t pitch_motor_measure;
static motor_measure_t right_motor;
static motor_measure_t left_motor;
static motor_measure_t fire_motor;

motor_measure_t *get_pitch_motor_measure_point(void)
{
	return &pitch_motor_measure;
}
motor_measure_t *get_right_motor_measure_point(void)
{
	return &right_motor;
}
motor_measure_t *get_left_motor_measure_point(void)
{
	return &left_motor;
}
motor_measure_t *get_fire_motor_measure_point(void)
{
	return &fire_motor;
}

/**
 * @brief		CAN1�˲�������
 * @param		none
 *	@retval		none
 */
void CAN1_filter_config(void)
{
	CAN_FilterTypeDef CAN1_FIilter_InitStruct;

	CAN1_FIilter_InitStruct.FilterActivation = ENABLE;			 // �����˲���
	CAN1_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 // ����ģʽ
	CAN1_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ����
	CAN1_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterBank = 0;
	//	CAN1_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN1_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0;	   // ָ����������
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FIilter_InitStruct);			   // ����ָ������CAN���չ�����
	HAL_CAN_Start(&hcan1);											   // ����can1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // �����ж�
}

/**
 * @brief		HAl��can1�Ļص�����
 * @param		��������� CAN�ľ��
 * @retval   none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		chassis_can1_callback(hcan);
	}
	if (hcan == &hcan2)
	{
		chassis_can2_callback(hcan);
	}
}

/*************************************can1����*************************************/
/**
 * @brief      ���̰�can1���ջص�����
 * @param[in]  *rx_message: can1���սṹ��
 * @retval     none
 * @attention  �� chassis_app.h ��ע�ᣬ�жϵ��ã�can1_callback(&rx1_message);��
 */
void chassis_can1_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Rxmessage; // ������Ϣ�ṹ��
	uint8_t Rx_Data[8];			   // ���յ���Ϣ���������

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) // ��ȡ���յ���Ϣ
	{
		switch (Rxmessage.StdId)
		{
		case 0x205: // P��
		{
//			Lost_time_fresh_by_name("p��");
			pitch_motor_measure.position = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			pitch_motor_measure.speed = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			pitch_motor_measure.current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
			pitch_motor_measure.temperature = Rx_Data[6];
			CAN_DATA_Encoder_Deal(pitch_motor_measure.position, pitch_motor_measure.speed, 1);
			break;
		}
		case 0x201:
		{
			right_motor.position = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			right_motor.speed = (uint16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			break;
		}
		case 0x202:
		{
			left_motor.position = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			left_motor.speed = (uint16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			break;
		}
		case 0x203:
		{
			fire_motor.position = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
			fire_motor.speed = (uint16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
			fire_motor.current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
			fire_motor.temperature = Rx_Data[6];
			CAN_DATA_Encoder_Deal(fire_motor.position, fire_motor.speed, 3);
			break;
		}
		default:
		{
			break;
		}
		}
	}
}
