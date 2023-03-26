#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "filter.h"
#include "lqr.h"

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;

static void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f);
static void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f);
static void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f);
static float torque_to_voltage_6020(float torque);
static float float_min_distance(float target, float actual, float minValue, float maxValue);
float *get_Gimbal_pitch_point(void)
{
    return &Gimbal_pitch;
}

float *get_Gimbal_yaw_point(void)
{
    return &Gimbal_yaw;
}

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // �ֱ�
    last_behaviour = rc_behaviour;
    switch (gimbal_behaviour_choose_f->Gimbal_RC->rc.s2)
    {
    case RC_SW_UP:
        rc_behaviour = GIMBAL_AUTOATTACK;
        break;
    case RC_SW_MID:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    case RC_SW_DOWN:
        rc_behaviour = GIMBAL_AUTOBUFF;
        break;
    default:
        break;
    }
    // �����λ�����ı䣬���ö�Ӧ��ģʽ
    if (last_behaviour != rc_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = rc_behaviour;
    }

    // ����
    last_behaviour = kb_behaviour;
    //**c
    if (gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.C)
    {
        kb_behaviour = GIMBAL_MANUAL;
    }
    // ���ģʽ�����ı䣬���ö�Ӧ��ģʽ
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
    }
}

void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
    Gimbal_pitch -= gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;

    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;

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
		
		value_limit(Gimbal_pitch, PITCH_ANGLE_LIMIT_DOWN, PITCH_ANGLE_LIMIT_UP);
		#if(YAW_CONTROLER == YAW_USE_PID)
		Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, 0.0f, 360.0f);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
		Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, -180.0f, 180.0f);
		#endif
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
	
}
void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
	
//		static float history_data[4][2] = {0};
//	
//	memmove(&history_data[1][0],&history_data[0][0],sizeof(float) * 6);
//	history_data[0][0] = Gimbal_pitch;
//	history_data[0][1] = Gimbal_yaw;
//	Gimbal_pitch = history_data[3][0] + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
//	Gimbal_yaw = history_data[3][1] + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
//	
//	gimbal_clear_virtual_recive();
	
	if(((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch != 0))
	{
	f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 = ((float)f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
	Gimbal_pitch = f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
	Gimbal_yaw = f_GIMBAL_AUTOATTACK_f->Imu_c->Yaw + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
	
	Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
	
	gimbal_clear_virtual_recive();
	}
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
}

void gimbal_pid_calculate(gimbal_control_t *gimbal_pid_calculate_f)
{
    gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360 = ((float)gimbal_pid_calculate_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);

    
		if(gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOBUFF)//����ģʽ
		{
		#if(PITCH_CONTROLER == PITCH_USE_PID)
		gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid,
                                                                                           &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid,
                                                                                           Gimbal_pitch,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);
		#elif(PITCH_CONTROLER == PITCH_USE_LQR)
		gimbal_pid_calculate_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch,gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360, -180, 180);
		double pitch_system_state[2] = {((-gimbal_pid_calculate_f->Pitch_c.motor_target) / 57.295779513f), gimbal_pid_calculate_f->Imu_c->Gyro[1]};
		
		
		LQR_Data_Update(&gimbal_pid_calculate_f->Pitch_c.motor_lqr, pitch_system_state);
		LQR_Calculate(&gimbal_pid_calculate_f->Pitch_c.motor_lqr);
		PidCalculate(&gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid, Gimbal_pitch, gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360 );
		gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0] = sliding_mean_filter(&gimbal_pid_calculate_f->Pitch_c.motor_filter, gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0],10);
		gimbal_pid_calculate_f->Pitch_c.motor_output = torque_to_voltage_6020(gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0]) + gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid.out;
		gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_voltage = abs_limit(gimbal_pid_calculate_f->Pitch_c.motor_output, 25000);
		#endif
		#if(YAW_CONTROLER == YAW_USE_PID)
    gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Yaw_c.yaw_motor_visual_speed_pid,
                                                                                       &gimbal_pid_calculate_f->Yaw_c.yaw_motor_visual_position_pid,
                                                                                       // �ӵ�ǰλ��Ϊ0��ѰĿ����ӻ��Ƕȼ�Ϊ������
                                                                                       user_abs(Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 180 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 0 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) - 360.0f) : (360.0f - (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw))) : (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw),
                                                                                       0,
                                                                                       gimbal_pid_calculate_f->Yaw_c.yaw_motor.motor_measure->speed);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
		gimbal_pid_calculate_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw,gimbal_pid_calculate_f->Imu_c->Yaw, -180, 180);
		double Yaw_system_state[2] = {((-gimbal_pid_calculate_f->Yaw_c.motor_target) / 57.295779513f), gimbal_pid_calculate_f->Imu_c->Gyro[2]};
		
		
		LQR_Data_Update(&gimbal_pid_calculate_f->Yaw_c.motor_lqr, Yaw_system_state);
		LQR_Calculate(&gimbal_pid_calculate_f->Yaw_c.motor_lqr);
		PidCalculate(&gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid, Gimbal_yaw, gimbal_pid_calculate_f->Imu_c->Yaw );
		gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0] = sliding_mean_filter(&gimbal_pid_calculate_f->Yaw_c.motor_filter, gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0],10);
		gimbal_pid_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0]) + gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid.out;
		gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = abs_limit(gimbal_pid_calculate_f->Yaw_c.motor_output, 25000);
		#endif
		}
		else//����ģʽ
		{
		#if(PITCH_CONTROLER == PITCH_USE_PID)
    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid,
                                                                                           &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid,
                                                                                           Gimbal_pitch,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360,
                                                                                           gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);
		#endif
		#if(YAW_CONTROLER == YAW_USE_PID)
    gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Yaw_c.yaw_motor_speed_pid,
                                                                                       &gimbal_pid_calculate_f->Yaw_c.yaw_motor_position_pid,
                                                                                       // �ӵ�ǰλ��Ϊ0��ѰĿ����ӻ��Ƕȼ�Ϊ������
                                                                                       user_abs(Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 180 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 0 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) - 360.0f) : (360.0f - (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw))) : (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw),
                                                                                       0,
                                                                                       gimbal_pid_calculate_f->Yaw_c.yaw_motor.motor_measure->speed);
		#endif
		}

}
float float_min_distance(float target, float actual, float minValue, float maxValue)
{
	if (maxValue < minValue)
    {
        return 0;
    }
	
	target = loop_float_constrain(target, minValue,maxValue);
	
	if (abs(target - actual) > (maxValue - minValue) / 2.0f)
	{
		if(maxValue - actual < (maxValue - minValue) / 2.0f)
		{
			return maxValue - actual + target - minValue;
		}
		else
		{
			return minValue - actual + target - maxValue;
		}
	}
	else 
	{
		return target - actual;
	}
	
}
float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = abs_limit(voltage,25000);
	
	return voltage;
		
}
