/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			remote_control.c
 *
 * @brief 		����ң������ʼ����ң�������ݻ�ȡ��ң����ͨѶЭ��Ľ���
 *
 * @note  		ң�������ݽ��ղ��ô��ڼ�DMA��ģʽ
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */

#include "rc.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700
/* �������ʱ��û���յ��µ�ң�������ݾ���Ϊ�Ѿ�ʧ�� */
#define REMOTE_LOST_TIME ((uint32_t)50) // 50ms

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//ȡ������
static int16_t RC_abs(int16_t value);
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// remote control data
//ң�������Ʊ���static
RC_ctrl_t rc_ctrl;

static TickType_t RCLostTime = 0;

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief          ң������ʼ��
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */

//�ж�ң���������Ƿ����
uint8_t rc_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }

    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;

    return 1;
}

//ң�������߻����ݴ���ʱ����ң����
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//ȡ������
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
 * @brief      ң����������������
 * @param[in]  input
 * @param[in]  dealine
 * @retval
 */
int16_t rc_deadline_limit(int16_t input, int16_t dealine)
{
    if (input > dealine || input < -dealine)
    {
        return input;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief      ����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
 * @param[in]  none
 * @retval     ����ң�������Ʊ��� &rc_ctrl
 * @attention
 */
RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint32_t chassis_rc_lost_time(void)
{
    return RCLostTime;
}

void rc_lost_time_refresh()
{
    /* �����յ�����ʱ��ˢ��ʧ������ʱ   xTaskGetTickCountFromISR  xTaskGetTickCount  */
    RCLostTime = xTaskGetTickCount() + REMOTE_LOST_TIME;
}

/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL)
    {
        return;
    }

    rc_lost_time_refresh();

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->kb.key_code = sbus_buf[14] | (sbus_buf[15] << 8);              //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    rc_ctrl->rc.ch[0] = rc_deadline_limit(rc_ctrl->rc.ch[0], 5); //��������
    rc_ctrl->rc.ch[1] = rc_deadline_limit(rc_ctrl->rc.ch[1], 5); //��������
    rc_ctrl->rc.ch[2] = rc_deadline_limit(rc_ctrl->rc.ch[2], 5); //��������
    rc_ctrl->rc.ch[3] = rc_deadline_limit(rc_ctrl->rc.ch[3], 5); //��������
}

//ң������ʼ��

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //ʹ��DMA���ڽ���
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //    //����DMA���䣬������1�����ݰ��˵�recvive_buff��
    //    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);

    //�ڴ滺����1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);

    //�ڴ滺����2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);

    //���ݳ���
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    //ʹ��˫������
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}

// �����ж�
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{

    //	LEDE0 = 0;

    if (huart3.Instance->SR & UART_FLAG_RXNE) //���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

//����ң����
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}

/**
 * @brief          ҡ��������
 * @param[in]      none
 * @retval         none
 * @attention
 */
void Remote_reload(void)
{
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    //	rc_ctrl.rc.s1 = RC_SW_ERROR;  //���ִ���Ϊ0
    //	rc_ctrl.rc.s2 = RC_SW_ERROR;  //���ִ���Ϊ0
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;
}
