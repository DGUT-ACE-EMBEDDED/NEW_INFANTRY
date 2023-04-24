#include "can2_receive.h"
#include "can.h"
#include "chassis_struct_variables.h"
#include "bsp_Motor_Encoder.h"
extern CAN_HandleTypeDef hcan2;
extern chassis_control_t Chassis_Control;
/*--------------------����-----------------------*/
static motor6020_measure_t yaw_motor_measure;
static Gimbal_data_t Gimbal_data;
motor6020_measure_t *get_yaw_motor_measure_point(void)
{
	return &yaw_motor_measure;
}
Gimbal_data_t *get_gimbal_data_p(void)
{
	return &Gimbal_data;
}
/**
 * @brief		can2�˲�������
 * @param		none
 *	@retval		none
 */

void CAN2_filter_config(void)
{
  CAN_FilterTypeDef CAN2_FIilter_InitStruct;

  //�����˲���
  CAN2_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;  //����ģʽ
  CAN2_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ����
  CAN2_FIilter_InitStruct.FilterIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterBank = 14; // CAN2 �˲�����Ϊ14
  CAN2_FIilter_InitStruct.SlaveStartFilterBank = 14;
  CAN2_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0; //ָ����������
  CAN2_FIilter_InitStruct.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &CAN2_FIilter_InitStruct);            //����ָ������CAN���չ�����
  HAL_CAN_Start(&hcan2);                                             //����can2
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //�����ж�
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
  CAN_RxHeaderTypeDef Rxmessage; //������Ϣ�ṹ��
  uint8_t Rx_Data[8];            //���յ���Ϣ���������
//	int Register;
  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //��ȡ���յ���Ϣ
  {
    switch (Rxmessage.StdId)
    {
//			case 0x300:
//			{
//				__asm__ 
//				(
//					"LDR Register , [Rx_Data] \n"
//					"STR Register , [&Chassis_Control.Chassis_Gimbal_Diference_Angle] \n"
//				);
//				break;
//			}
			case 0x501:
			{
				Gimbal_data.replenish_flag = Rx_Data[0];
				break;
			}
			case 0x206: // Y��
			{
				yaw_motor_measure.position = (int16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
				yaw_motor_measure.speed = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
				yaw_motor_measure.current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]);
				yaw_motor_measure.temperature = Rx_Data[6];
				CAN_DATA_Encoder_Deal(yaw_motor_measure.position, yaw_motor_measure.speed, 5);
				break;
			}
			default :
								break;
		}
  }
}
