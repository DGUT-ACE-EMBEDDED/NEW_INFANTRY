/*--------------------- CONTROL --------------------*/
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"

/*--------------------- TASK --------------------*/
#include "virtual_task.h"
#include "gimbal_task.h"
/*--------------------- FIRMWARE --------------------*/
#include "usbd_cdc_if.h"

#include "stdlib.h"
#include "string.h"

#include "fifo.h"

gimbal_auto_control_t *auto_control_p;

static gimbal_auto_control_t *virtual_task_init(void);
static void Virtual_send(gimbal_auto_control_t *Virtual_send_p);
static void Virtual_recive(gimbal_auto_control_t *Virtual_recive_p);

void Virtual_Task(void const * argument)
{
  auto_control_p = virtual_task_init();
	while(1)
	{
		Virtual_recive(auto_control_p);
		Virtual_send(auto_control_p);
		vTaskDelay(1);
	}
}
void Virtual_recive(gimbal_auto_control_t *Virtual_recive_p)
{
	if(fifo_s_isempty(Virtual_recive_p->usb_fifo) != true)
	{
		uint8_t read_buff[8];
		fifo_s_gets(Virtual_recive_p->usb_fifo,(char*)read_buff,8);
		if(read_buff[0] == 0xFF && read_buff[7] == 0xFE)
		{
			Virtual_recive_p->auto_yaw = -(((float)((int16_t)(read_buff[2]<<8 | read_buff[1]))) / 100.0f);        //自动打击的y轴角度计算/100
			Virtual_recive_p->auto_pitch = -(((float)((int16_t)(read_buff[4]<<8 | read_buff[3]))) / 100.0f);      //自动打击的p轴角度计算
//			Virtual_recive_p->auto_pitch_speed = (read_buff[5]<<8 | read_buff[6]) / 10000; //自动打击的p轴角度计算/100
		}
	}
}
/**
  * @brief      发送数据给视觉
  * @retval     none
  * @attention  协议：帧头0xFF  数据帧尾0xFE
  *             第二帧 data: 敌方装甲板  红：0 | 蓝：1
	* 						第三帧 mode: 模式选择    0：装甲模式  1：能量机关模式  2：陀螺模式
	* 						第四帧 shoot_speed: 射速，目前是平均实际速度，其实就是当前上限射速-2
  */
void Virtual_send(gimbal_auto_control_t *Virtual_send_p)
{
	switch (*Virtual_send_p->gimbal_behaviour)
	{
    case GIMBAL_MANUAL:
				return;
    case GIMBAL_AUTOATTACK:
				Virtual_send_p->visual_buff_send[2] = 0;
        break;
    case GIMBAL_AUTOBUFF:
				Virtual_send_p->visual_buff_send[2] = 1;
        break;
    default:
        break;
	}
//		//裁判系统接收 阵营，射速。
//		//阵营
		Virtual_send_p->visual_buff_send[1] = 1;
//		//射速

		Virtual_send_p->visual_buff_send[3] = 15;
		//浮点转字节
		{
			float register *Register1 = INS.q;
			uint8_t register *Register2 = &Virtual_send_p->visual_buff_send[4];
			int register Register3;
			 __asm__ volatile
			{
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Virtual_send_p->gimbal_yaw]
				STR Register3 , [Register2],#4
				LDR Register3 , [Virtual_send_p->gimbal_pitch]
				STR Register3 , [Register2]
			}
		}
		CDC_Transmit_FS(Virtual_send_p->visual_buff_send, sizeof(Virtual_send_p->visual_buff_send));
}

gimbal_auto_control_t *virtual_task_init(void)
{
	gimbal_auto_control_t *virtual_task_init_p;
	virtual_task_init_p = malloc(sizeof(gimbal_auto_control_t));
	memset(virtual_task_init_p, 0, sizeof(gimbal_auto_control_t));
	
	virtual_task_init_p->usb_fifo = fifo_s_create(48);
	virtual_task_init_p->gimbal_pitch = get_Gimbal_pitch_point();
	virtual_task_init_p->gimbal_yaw = get_Gimbal_yaw_point();
	virtual_task_init_p->gimbal_behaviour = get_gimbal_behaviour_point();
	
	virtual_task_init_p->visual_buff_send[0]  = 0xFF;
	virtual_task_init_p->visual_buff_send[28] = 0xFE;
	
	return virtual_task_init_p;
}
void gimbal_clear_virtual_recive(void)
{
	if(auto_control_p == NULL)
	{
		return;
	}
	auto_control_p->auto_pitch = 0;
	auto_control_p->auto_yaw = 0;
}
const gimbal_auto_control_t **get_auto_control_point(void)
{
	return (const gimbal_auto_control_t**)&auto_control_p;
}
