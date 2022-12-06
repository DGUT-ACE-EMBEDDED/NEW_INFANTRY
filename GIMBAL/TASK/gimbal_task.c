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
 * @brief  云台主任务
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
        taskENTER_CRITICAL();         //进入临界区
        Gimbal_Work(&Gimbal_Control); //云台状态控制    //云台工作

        can1_gimbal_setmsg(Gimbal_Control.Pitch_c.pitch_motor.set_voltage, Gimbal_Control.Yaw_c.yaw_motor.set_voltage);
        can2_gimbal_setmsg(Gimbal_Control.fire_c.left_motor.set_current, Gimbal_Control.fire_c.right_motor.set_current, Gimbal_Control.fire_c.fire_motor.set_current);
        can2_gambal_to_chassis();
        taskEXIT_CRITICAL();              //退出临界区
        vTaskDelayUntil(&currentTime, 1); //绝对延时//vTaskDelay(2);
    }
}

void Gimbal_Init(gimbal_control_t *Gimbal_Init_f)
{
    memset(Gimbal_Init_f, 0, sizeof(gimbal_control_t));
    /*--------------------获取指针--------------------*/
    //获取遥控器指针(数据)
    Gimbal_Init_f->Gimbal_RC = RC_Get_RC_Pointer();
    //获取云台指针
    Gimbal_Init_f->Pitch_c.pitch_motor.motor_measure = get_pitch_motor_measure_point();
    Gimbal_Init_f->Yaw_c.yaw_motor.motor_measure = get_yaw_motor_measure_point();
    //获得拨弹指针
    Gimbal_Init_f->fire_c.right_motor.motor_measure = get_right_motor_measure_point();
    Gimbal_Init_f->fire_c.left_motor.motor_measure = get_left_motor_measure_point();
    Gimbal_Init_f->fire_c.fire_motor.motor_measure = get_fire_motor_measure_point();

    //获取陀螺仪指针
    Gimbal_Init_f->Imu_c = get_imu_control_point();

    /*--------------------初始化编码器--------------------*/
    Gimbal_Init_f->Pitch_c.pitch_motor_encoder = Encoder_Init(GM6020, 1);
    Gimbal_Init_f->Yaw_c.yaw_motor_encoder = Encoder_Init(GM6020, 2);

    /*--------------------pid--------------------*/
    // P轴
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, GIMBAL_PITCH_S_P, GIMBAL_PITCH_S_I, GIMBAL_PITCH_S_D, Integral_Limit | Output_Limit);
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, GIMBAL_PITCH_P_P, GIMBAL_PITCH_P_I, GIMBAL_PITCH_P_D, ChangingIntegrationRate | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Integral_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, ChangingIntegrationRate, 20.0f, 0.5f);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, StepIn, 30, 0);

    // Y轴
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
