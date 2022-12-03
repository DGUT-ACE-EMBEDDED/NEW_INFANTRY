#include "can1_receive.h"
#include "can2_receive.h"
#include "struct_variables.h"
#include "bsp_Motor_Encoder.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void chassis_can2_callback(CAN_HandleTypeDef *hcan);

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                                  \
	{                                                                                     \
		(ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
		(ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);    \
	}

/*--------------------变量-----------------------*/
//申明底盘电机变量 static
static motor_measure_t motor_chassis[4];
//申明电容变量
static Supercapacitor_receive_t Supercap_receive;

//电容
Supercapacitor_receive_t *get_supercap_control_point(void)
{
	return &Supercap_receive;
}

//底盘电机
motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}

/**************数据接收函数*********/
float re_capacitance_voltage(void);

/**
 * @brief		CAN1滤波器配置
 * @param		none
 *	@retval		none
 */
void CAN1_filter_config(void)
{
	CAN_FilterTypeDef CAN1_FIilter_InitStruct;

	CAN1_FIilter_InitStruct.FilterActivation = ENABLE;			 //开启滤波器
	CAN1_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 //掩码模式
	CAN1_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32位工作
	CAN1_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterBank = 0;
	//	CAN1_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN1_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0;	   //指定接收邮箱
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FIilter_InitStruct);			   //根据指定配置CAN接收过滤器
	HAL_CAN_Start(&hcan1);											   //开启can1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //启动中断
}

/**
 * @brief		HAl库can1的回调函数
 * @param		传入参数： CAN的句柄
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

/**********************************************************************************/
/*************************************can1接收*************************************/
/**********************************************************************************/
/**
 * @brief      底盘板can1接收回调函数
 * @param[in]  *rx_message: can1接收结构体
 * @retval     none
 * @attention  在 chassis_app.h 中注册，中断调用（can1_callback(&rx1_message);）
 */
void chassis_can1_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Rxmessage; //接收信息结构体
	uint8_t Rx_Data[8];			   //接收的信息缓存的数组

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //读取接收的信息
	{
		switch (Rxmessage.StdId)
		{
		/*底盘电机*/
		case 0x201:
		{
			motor_chassis[0].position = (uint16_t)(Rx_Data[0] << 8) | Rx_Data[1];
			motor_chassis[0].speed = (uint16_t)(Rx_Data[2] << 8) | Rx_Data[3];
			CAN_DATA_Encoder_Deal(motor_chassis[0].position, motor_chassis[0].speed, 1);
			break;
		}
		case 0x202:
		{
			motor_chassis[1].position = (uint16_t)(Rx_Data[0] << 8) | Rx_Data[1];
			motor_chassis[1].speed = (uint16_t)(Rx_Data[2] << 8) | Rx_Data[3];
			CAN_DATA_Encoder_Deal(motor_chassis[1].position, motor_chassis[1].speed, 2);
			break;
		}
		case 0x203:
		{
			motor_chassis[2].position = (uint16_t)(Rx_Data[0] << 8) | Rx_Data[1];
			motor_chassis[2].speed = (uint16_t)(Rx_Data[2] << 8) | Rx_Data[3];
			CAN_DATA_Encoder_Deal(motor_chassis[2].position, motor_chassis[2].speed, 3);
			break;
		}
		case 0x204:
		{
			motor_chassis[3].position = (uint16_t)(Rx_Data[0] << 8) | Rx_Data[1];
			motor_chassis[3].speed = (uint16_t)(Rx_Data[2] << 8) | Rx_Data[3];
			CAN_DATA_Encoder_Deal(motor_chassis[3].position, motor_chassis[3].speed, 4);
			break;
		}
		case 0x211: //超级电容接收
		{
			Supercap_receive.input_voltage = (float)(Rx_Data[0] / 100.0f);
			Supercap_receive.Capacitance_voltage = (float)(Rx_Data[1] / 100.0f);
			Supercap_receive.Input_current = (float)(Rx_Data[2] / 100.0f);
			Supercap_receive.Set_power = (float)(Rx_Data[3] / 100.0f);
			break;
		}
		default:
		{
			break;
		}
		}
	}
}

/**
 * @brief      返回超级电容电压
 * @param[in]  none
 * @retval     超级电容电压
 * @attention  底盘板调用
 */
float re_capacitance_voltage(void)
{
	return (Supercap_receive.Capacitance_voltage);
}
