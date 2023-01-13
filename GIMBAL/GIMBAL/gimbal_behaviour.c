#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "gimbal_config.h"

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;

static void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f);
static void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f);
static void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f);

float* get_Gimbal_pitch_point(void)
{
	return &Gimbal_pitch;
}

float* get_Gimbal_yaw_point(void)
{
	return &Gimbal_yaw;
}

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // 手柄
    last_behaviour = rc_behaviour;
    switch (gimbal_behaviour_choose_f->Gimbal_RC->rc.s2)
    {
    case RC_SW_UP:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    case RC_SW_MID:
        rc_behaviour = GIMBAL_AUTOATTACK;
        break;
    case RC_SW_DOWN:
        rc_behaviour = GIMBAL_AUTOBUFF;
        break;
    default:
        break;
    }
    // 如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = rc_behaviour;
    }

    // 键鼠
    last_behaviour = kb_behaviour;
    //**c
    if (gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.C)
    {
        kb_behaviour = GIMBAL_MANUAL;
    }
    // 如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
    }
}

void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;
    value_limit(Gimbal_pitch, PITCH_ANGLE_LIMIT_DOWN, PITCH_ANGLE_LIMIT_UP);

    Gimbal_yaw += gimbal_behaviour_react_f->Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;
    //    Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, -180.0f, 180.0f);

    switch (gimbal_behaviour_react_f->gimbal_behaviour)
    {
    case GIMBAL_MANUAL:
        f_GIMBAL_MANUAL(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOATTACK:
        f_GIMBAL_AUTOATTACK(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOBUFF:
        f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
        break;
    default:
        break;
    }
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
}
void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
}

void gimbal_pid_calculate(gimbal_control_t *gimbal_pid_calculate_f)
{
    gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360 = ((float)gimbal_pid_calculate_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
    gimbal_pid_calculate_f->Yaw_c.yaw_motor.actPositon_360 = ((float)(gimbal_pid_calculate_f->Yaw_c.yaw_motor_encoder->Encode_Actual_Val - YAW_ZERO_OFFSET)* 360.0f / 8192.0f);
    gimbal_pid_calculate_f->Yaw_c.yaw_motor.actPositon_360 = loop_fp32_constrain(gimbal_pid_calculate_f->Yaw_c.yaw_motor.actPositon_360,0.0f, 360.0f);
		Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw , 0.0f ,360.0f);
    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid,
                                                                                           &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid,
                                                                                           Gimbal_pitch,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);
    gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Yaw_c.yaw_motor_speed_pid,
                                                                                       &gimbal_pid_calculate_f->Yaw_c.yaw_motor_position_pid,
                                                                                       user_abs(Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 180 ?  ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 0 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) - 360.0f) : (360.0f - (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw))) : (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw),
																																											 0,
//                                                                                       gimbal_pid_calculate_f->Imu_c->YawTotalAngle,
                                                                                       gimbal_pid_calculate_f->Yaw_c.yaw_motor.motor_measure->speed);
}
