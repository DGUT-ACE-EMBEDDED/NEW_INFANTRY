/*--------------------- CONTROL --------------------*/
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"
#include "virtual_task.h"

/*--------------------- FIRMWARE --------------------*/
#include "usbd_cdc_if.h"

#include "stdlib.h"
#include "string.h"

gimbal_auto_control_t *auto_control_p;

static gimbal_auto_control_t *virtual_task_init(void);


void Virtual_Task(void const * argument)
{
  auto_control_p = virtual_task_init();
	while(1)
	{
//		//裁判系统接收 阵营，射速。
//		//阵营
//		auto_control_p->visual_buff_send[1] = ;
//		//射速
//		auto_control_p->visual_buff_send[3] = ;
		{
			float register *Register1 = INS.q;
			uint8_t register *Register2 = &auto_control_p->visual_buff_send[4];
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
				LDR Register3 , [auto_control_p->gimbal_yaw]
				STR Register3 , [Register2],#4
				LDR Register3 , [auto_control_p->gimbal_pitch]
				STR Register3 , [Register2]
			}
		}
		CDC_Transmit_FS(auto_control_p->visual_buff_send, sizeof(auto_control_p->visual_buff_send));
		
		vTaskDelay(1);
	}
}

gimbal_auto_control_t *virtual_task_init(void)
{
	gimbal_auto_control_t *virtual_task_init_p;
	virtual_task_init_p = malloc(sizeof(gimbal_auto_control_t));
	memset(virtual_task_init_p, 0, sizeof(gimbal_auto_control_t));
	
	virtual_task_init_p->usb_fifo = fifo_s_create(48);
	virtual_task_init_p->gimbal_pitch = get_Gimbal_pitch_point();
	virtual_task_init_p->gimbal_yaw = get_Gimbal_yaw_point();
	
	virtual_task_init_p->visual_buff_send[0]  = 0xFF;
	virtual_task_init_p->visual_buff_send[28] = 0xFE;
	
	return virtual_task_init_p;
}

const gimbal_auto_control_t **get_auto_control_point(void)
{
	return (const gimbal_auto_control_t**)&auto_control_p;
}
