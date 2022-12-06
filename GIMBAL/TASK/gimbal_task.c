#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"
#include "pid.h"
#include "can1_receive.h"
#include "can1_send.h"
#include "string.h"
#include "bsp_Motor_Encoder.h"
#include "maths.h"

#include "bsp_dr16.h"
gimbal_control_t Gimbal_Control;

static void Gimbal_Work(gimbal_control_t *Gimbal_Work_f);
static void Gimbal_Init(gimbal_control_t *Gimbal_Init_f);

/**
 * @brief  ��̨������
 * @param
 * @retval void
 */
void Gimbal_Task(void const *argument)
{
    uint32_t currentTime;

    Gimbal_Init(&Gimbal_Control);
    currentTime = xTaskGetTickCount();
    while (1)
    {
        taskENTER_CRITICAL();         //�����ٽ���
        Gimbal_Work(&Gimbal_Control); //��̨״̬����    //��̨����

        can1_gimbal_setmsg(Gimbal_Control.Pitch_c.pitch_motor.set_voltage, Gimbal_Control.Yaw_c.yaw_motor.set_voltage);
        can2_gimbal_setmsg(Gimbal_Control.fire_c.left_motor.set_current, Gimbal_Control.fire_c.right_motor.set_current, Gimbal_Control.fire_c.fire_motor.set_current);
        can2_gambal_to_chassis();
        taskEXIT_CRITICAL();              //�˳��ٽ���
        vTaskDelayUntil(&currentTime, 1); //������ʱ//vTaskDelay(2);
    }
}

void Gimbal_Init(gimbal_control_t *Gimbal_Init_f)
{
    memset(Gimbal_Init_f, 0, sizeof(gimbal_control_t));
    /*--------------------��ȡָ��--------------------*/
    //��ȡң����ָ��(����)
    Gimbal_Init_f->Gimbal_RC = RC_Get_RC_Pointer();
    //��ȡ��ָ̨��
    Gimbal_Init_f->Pitch_c.pitch_motor.motor_measure = get_pitch_motor_measure_point();
    Gimbal_Init_f->Yaw_c.yaw_motor.motor_measure = get_yaw_motor_measure_point();
    //��ò���ָ��
    Gimbal_Init_f->fire_c.right_motor.motor_measure = get_right_motor_measure_point();
    Gimbal_Init_f->fire_c.left_motor.motor_measure = get_left_motor_measure_point();
    Gimbal_Init_f->fire_c.fire_motor.motor_measure = get_fire_motor_measure_point();

    //��ȡ������ָ��
    Gimbal_Init_f->Imu_c = get_imu_control_point();

    /*--------------------��ʼ��������--------------------*/
    Gimbal_Init_f->Pitch_c.pitch_motor_encoder = Encoder_Init(GM6020, 1);
    Gimbal_Init_f->Yaw_c.yaw_motor_encoder = Encoder_Init(GM6020, 2);

    /*--------------------pid--------------------*/
    // P��
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, GIMBAL_PITCH_S_P, GIMBAL_PITCH_S_I, GIMBAL_PITCH_S_D, Integral_Limit | Output_Limit);
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, GIMBAL_PITCH_P_P, GIMBAL_PITCH_P_I, GIMBAL_PITCH_P_D, ChangingIntegrationRate | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Integral_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, ChangingIntegrationRate, 20.0f, 0.5f);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, StepIn, 30, 0);

    // Y��
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, GIMBAL_YAW_S_P, GIMBAL_YAW_S_I, GIMBAL_YAW_S_D, Output_Limit | Integral_Limit);
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, GIMBAL_YAW_P_P, GIMBAL_YAW_P_I, GIMBAL_YAW_P_D, Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Integral_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, StepIn, 60, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Integral_Limit, 300, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Output_Limit, 10000, 0);

    // fire
    PidInit(&Gimbal_Init_f->fire_c.left_motor_speed_pid, 10, 0, 0, Output_Limit);
    PidInit(&Gimbal_Init_f->fire_c.right_motor_speed_pid, 10, 0, 0, Output_Limit);
    PidInit(&Gimbal_Init_f->fire_c.fire_motor_speed_pid, 10, 0, 0, Output_Limit);
    PidInitMode(&Gimbal_Init_f->fire_c.left_motor_speed_pid, Output_Limit, 16000, 0);
    PidInitMode(&Gimbal_Init_f->fire_c.right_motor_speed_pid, Output_Limit, 16000, 0);
    PidInitMode(&Gimbal_Init_f->fire_c.fire_motor_speed_pid, Output_Limit, 16000, 0);

    Gimbal_Init_f->fire_c.full_automatic = true;
    EncoderValZero(Gimbal_Init_f->Yaw_c.yaw_motor_encoder);
    Gimbal_Init_f->chassis_gimbal_angel = 0;
}

void Gimbal_Work(gimbal_control_t *Gimbal_Work_f)
{
    gimbal_behaviour_choose(Gimbal_Work_f);

    gimbal_behaviour_react(Gimbal_Work_f);

    gimbal_pid_calculate(Gimbal_Work_f);
}
