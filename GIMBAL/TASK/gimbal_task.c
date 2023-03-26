/*--------------------- FIRMWARE --------------------*/
#include "string.h"
#include "usbd_cdc_if.h"

/*--------------------- TASK --------------------*/
#include "gimbal_task.h"
#include "fire_Task.h"
#include "virtual_task.h"
/*--------------------- COMMUINICATE --------------------*/
#include "can1_receive.h"
#include "can2_receive.h"
#include "can1_send.h"
#include "can2_send.h"

/*--------------------- CONTROL --------------------*/
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"
#include "gimbal_auto.h"
#include "gimbal_config.h"
/*--------------------- ALGORITHM --------------------*/
#include "pid.h"
#include "maths.h"
#include "filter.h"
#include "lqr.h"
/*--------------------- BSP --------------------*/
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "bsp_referee.h"

gimbal_control_t Gimbal_Control;

static void Gimbal_Work(gimbal_control_t *Gimbal_Work_f);
static void Gimbal_Init(gimbal_control_t *Gimbal_Init_f);
#if(PITCH_CONTROLER == PITCH_USE_LQR)
double k_pitch_lqr[2] = {-5.94183025734807, -0.3};
#endif
#if(YAW_CONTROLER == YAW_USE_LQR)
double k_yaw_lqr[2] = {-7.96227766016838, -0.33467065322830};
#endif
/**
 * @brief  ��̨������
 * @param
 * @retval void
 */
void Gimbal_Task(void const *argument)
{
//    uint32_t currentTime;

    Gimbal_Init(&Gimbal_Control);
//    currentTime = xTaskGetTickCount();
    while (1)
    {
			taskENTER_CRITICAL();         // �����ٽ���
			Gimbal_Work(&Gimbal_Control); // ��̨״̬����    //��̨����
			taskEXIT_CRITICAL();              // �˳��ٽ���
			can1_gimbal_setmsg_to_pitch(Gimbal_Control.Pitch_c.pitch_motor.set_voltage);
			can2_gimbal_setmsg_to_yaw(Gimbal_Control.Yaw_c.yaw_motor.set_voltage);
		
      vTaskDelay(1); // ������ʱ//vTaskDelay(2);
    }
}

void Gimbal_Init(gimbal_control_t *Gimbal_Init_f)
{
    memset(Gimbal_Init_f, 0, sizeof(gimbal_control_t));
    /*--------------------��ȡָ��--------------------*/
    // ��ȡң����ָ��(����)
    Gimbal_Init_f->Gimbal_RC = RC_Get_RC_Pointer();
    // ��ȡ��ָ̨��
    Gimbal_Init_f->Pitch_c.pitch_motor.motor_measure = get_pitch_motor_measure_point();
    Gimbal_Init_f->Yaw_c.yaw_motor.motor_measure = get_yaw_motor_measure_point();
    // ��ȡ���ָ��
    Gimbal_Init_f->fire_c = get_fire_control_point();
		//��ȡ����ָ��
		Gimbal_Init_f->auto_c = get_auto_control_point();
    // ��ȡ������ָ��
    Gimbal_Init_f->Imu_c = get_imu_control_point();
		// ��ȡ����ϵͳָ��
		Gimbal_Init_f->referee = Get_referee_Address();
    /*--------------------��ʼ��������--------------------*/
    Gimbal_Init_f->Pitch_c.pitch_motor_encoder = Encoder_Init(GM6020, 1);
    Gimbal_Init_f->Yaw_c.yaw_motor_encoder = Encoder_Init(GM6020, 2);

		#if(PITCH_CONTROLER == PITCH_USE_LQR)
		PidInit(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, 0, 0.15f, 0, Integral_Limit );
    PidInitMode(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, Integral_Limit, 1000, 0);
		LQR_Init(&Gimbal_Init_f->Pitch_c.motor_lqr, 2, 1, k_pitch_lqr);
		#elif(PITCH_CONTROLER == PITCH_USE_PID)
    /*--------------------pid--------------------*/
    // P��
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, GIMBAL_PITCH_S_P, GIMBAL_PITCH_S_I, GIMBAL_PITCH_S_D, Integral_Limit | Output_Limit | OutputFilter);
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, GIMBAL_PITCH_P_P, GIMBAL_PITCH_P_I, GIMBAL_PITCH_P_D, Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Integral_Limit, 10000, 0);
		PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, OutputFilter, 0.9, 0);
//    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, ChangingIntegrationRate, 20.0f, 0.5f);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, StepIn, 30, 0);
		#endif
		#if(YAW_CONTROLER == YAW_USE_LQR)
		PidInit(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, 0, 0.15f, 0, Integral_Limit );
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, Integral_Limit, 1000, 0);
		LQR_Init(&Gimbal_Init_f->Yaw_c.motor_lqr, 2, 1, k_yaw_lqr);
		#elif(YAW_CONTROLER == YAW_USE_PID)
    // Y��
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, GIMBAL_YAW_S_P, GIMBAL_YAW_S_I, GIMBAL_YAW_S_D, Output_Limit | Integral_Limit);
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, GIMBAL_YAW_P_P,GIMBAL_YAW_P_I, GIMBAL_YAW_P_D, Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Integral_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, StepIn, 60, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Integral_Limit, 300, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Output_Limit, 30000, 0);
		
		//����
		PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, GIMBAL_YAW_visual_S_P, GIMBAL_YAW_visual_S_I, GIMBAL_YAW_visual_S_D, Output_Limit | Integral_Limit );
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, GIMBAL_YAW_visual_P_P, GIMBAL_YAW_visual_P_I, GIMBAL_YAW_visual_P_D, Integral_Limit | Output_Limit );		
		PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, Integral_Limit, 10000, 0);
    
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, Output_Limit, 30000, 0);
		#endif
		
		/*--------------------filter--------------------*/
		#if(PITCH_CONTROLER == PITCH_USE_LQR)
		sliding_mean_filter_init(&Gimbal_Init_f->Pitch_c.motor_filter);
		#endif
		#if(YAW_CONTROLER == YAW_USE_LQR)
		sliding_mean_filter_init(&Gimbal_Init_f->Yaw_c.motor_filter);
		#endif
		first_order_filter_init(&Gimbal_Init_f->Pitch_c.visual_pitch__first_order_filter,0.03);
		
    EncoderValZero(Gimbal_Init_f->Yaw_c.yaw_motor_encoder);
    Gimbal_Init_f->chassis_gimbal_angel = 0;
}

void Gimbal_Work(gimbal_control_t *Gimbal_Work_f)
{
    gimbal_behaviour_choose(Gimbal_Work_f);

    gimbal_behaviour_react(Gimbal_Work_f);

    gimbal_pid_calculate(Gimbal_Work_f);
}

gimbal_behaviour_e *get_gimbal_behaviour_point(void)
{
    return &Gimbal_Control.gimbal_behaviour;
}
