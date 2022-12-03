/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			remote_control.h
 *
 * @brief 		包括遥控器初始化，遥控器数据获取，遥控器通讯协议的解析
 *
 * @note  		遥控器数据接收采用串口加DMA的模式
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */

#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "usart.h"
#include "stdio.h"


#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR				((uint16_t)0)
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct //遥控统一结构体
{
	struct
	{
		int16_t ch[5];
		char s[2];

	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t M : 1;//uint16_t Z : 1;
            uint16_t N : 1;//uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;

}RC_ctrl_t;

extern void remote_control_init(void);
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
extern RC_ctrl_t *get_remote_control_point(void);
extern void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void sbus_to_usart1(uint8_t *sbus);
extern void Remote_reload(void);
extern uint8_t rc_data_is_error(void);
extern int16_t rc_deadline_limit(int16_t input, int16_t dealine);
extern void rc_lost_time_refresh(void);
extern uint32_t chassis_rc_lost_time(void);



#endif
