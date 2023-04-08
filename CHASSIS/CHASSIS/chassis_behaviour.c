#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "bsp_dr16.h"
#include "maths.h"
#include "pid.h"
#include "chassis_config.h"
#include "filter.h"
//ԭʼ������
float Chassis_x = 0.0f;
float Chassis_y = 0.0f;
float Chassis_yaw_speed = 0.0f;
float Chassis_yaw_angle = 0.0f;

float Chassis_x_pid_output = 0.0f;
float Chassis_y_pid_output = 0.0f;
float Chassis_yaw_angle_pid_output = 0.0f;
float Chassis_yaw_speed_pid_output = 0.0f;
/********************��������********************/
void f_CHASSIS_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f);

void chassis_behaviour_choose(chassis_control_t *Chassis_behaviour_f)
{
    //���ڼ�¼��һ������
    chassis_behaviour_e last_behaviour;
    static chassis_behaviour_e rc_behaviour = CHASSIS_NO_FOLLOW;
    static chassis_behaviour_e kb_behaviour = CHASSIS_NO_FOLLOW;

    //�ֱ�
    last_behaviour = rc_behaviour;
    switch (Chassis_behaviour_f->Chassis_RC->rc.s1)
    {
    case RC_SW_UP:
        rc_behaviour = CHASSIS_ROTATION;
        break;
    case RC_SW_MID:
        rc_behaviour = CHASSIS_FOLLOW;
        break;
    case RC_SW_DOWN:
        rc_behaviour = CHASSIS_NO_FOLLOW;
        break;
    default:
        break;
    }
    //�����λ�����ı䣬���ö�Ӧ��ģʽ
    if (last_behaviour != rc_behaviour)
    {
			if(rc_behaviour == CHASSIS_NO_FOLLOW)
			{
				Chassis_behaviour_f->chassis_no_follow_yaw = Chassis_behaviour_f->Imu_c->Yaw;
			}
       Chassis_behaviour_f->behaviour = rc_behaviour;
    }

    //����
    last_behaviour = kb_behaviour;
    //**Q
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.Q)
    {
        kb_behaviour = CHASSIS_FOLLOW; //���̸���ģʽ
    }
    //**E
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.E)
    {
        kb_behaviour = CHASSIS_ROTATION; //С����ģʽ
    }
    //**F
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.F)
    {
        kb_behaviour = CHASSIS_FOLLOW; //���̸���ģʽ
    }
    //**G ����ģʽ
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.G)
    {
        kb_behaviour = CHASSIS_NO_FOLLOW; //����ģʽ�£����̲�������̨
    }
    //���ģʽ�����ı䣬���ö�Ӧ��ģʽ
    if (last_behaviour != kb_behaviour)
    {
			if(kb_behaviour == CHASSIS_NO_FOLLOW)
			{
				Chassis_behaviour_f->chassis_no_follow_yaw = Chassis_behaviour_f->Imu_c->Yaw;
			}
      Chassis_behaviour_f->behaviour = kb_behaviour;
    }
}

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = Chassis_behaviour_react_f->Chassis_RC->rc.ch[2] + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.A + Chassis_behaviour_react_f->Chassis_RC->kb.bit.D) * 660;
    value_limit(Chassis_x, -660, 660);
    Chassis_y = Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W) * 660;
    value_limit(Chassis_y, -660, 660);

    switch (Chassis_behaviour_react_f->behaviour)
    {
    case CHASSIS_FOLLOW:
        f_CHASSIS_FOLLOW(Chassis_behaviour_react_f);
        break;
    case CHASSIS_NO_FOLLOW:
        f_CHASSIS_NO_FOLLOW(Chassis_behaviour_react_f);
        break;
    case CHASSIS_ROTATION:
        f_CHASSIS_ROTATION(Chassis_behaviour_react_f);
        break;
    case CHASSIS_BATTERY:
        f_CHASSIS_BATTERY(Chassis_behaviour_react_f);
        break;
    default:
        break;
    }

    //ûͷ��ʱ����ת�õ�
//    Chassis_yaw_speed += Chassis_behaviour_react_f->Chassis_RC->rc.ch[0];
}
void chassis_state_choose(chassis_control_t *chassis_state_choose_f)
{
    chassis_state_e last_state;
    last_state = chassis_state_choose_f->chassis_state;
    if ((user_abs(chassis_state_choose_f->Chassis_speedX_Pid.SetValue) < chassis_state_choose_f->Chassis_speedX_Pid.stepIn) && (user_abs(chassis_state_choose_f->Chassis_speedY_Pid.SetValue) < chassis_state_choose_f->Chassis_speedY_Pid.stepIn) && (user_abs(chassis_state_choose_f->chassis_rotate_pid.SetValue) < 3))
    {
        chassis_state_choose_f->chassis_state = CHASSIS_LOCK_POSITION;
    }
    else
    {
        chassis_state_choose_f->chassis_state = CHASSIS_SPEED;
    }
    if (last_state != chassis_state_choose_f->chassis_state)
    {
//        EncoderValZero(chassis_state_choose_f->Motor_encoder[0]);
//        EncoderValZero(chassis_state_choose_f->Motor_encoder[1]);
//        EncoderValZero(chassis_state_choose_f->Motor_encoder[2]);
//        EncoderValZero(chassis_state_choose_f->Motor_encoder[3]);
//        pid_clear(&chassis_state_choose_f->motor_Position_Pid[0]);
//        pid_clear(&chassis_state_choose_f->motor_Position_Pid[1]);
//        pid_clear(&chassis_state_choose_f->motor_Position_Pid[2]);
//        pid_clear(&chassis_state_choose_f->motor_Position_Pid[3]);
//        pid_clear(&chassis_state_choose_f->Chassis_speedX_Pid);
//        pid_clear(&chassis_state_choose_f->Chassis_speedY_Pid);
//        pid_clear(&chassis_state_choose_f->chassis_rotate_pid);
    }

    if (chassis_state_choose_f->Chassis_RC->rc.s2 == RC_SW_DOWN)
    {
        chassis_state_choose_f->chassis_state = CHASSIS_ZERO_FORCE;
    }
}

void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f)
{
		#ifdef POWER_CONTROL
		static int count=0;
		#endif
		Chassis_x = first_order_filter(&chassis_speed_pid_calculate_f->Chassis_speedX_filter,Chassis_x);
		Chassis_y = first_order_filter(&chassis_speed_pid_calculate_f->Chassis_speedY_filter,Chassis_y);
	
		Chassis_x_pid_output = -PidCalculate(&chassis_speed_pid_calculate_f->Chassis_speedX_Pid, Chassis_x, 0);
		Chassis_y_pid_output = PidCalculate(&chassis_speed_pid_calculate_f->Chassis_speedY_Pid, Chassis_y, 0);
	  if(chassis_speed_pid_calculate_f->behaviour == CHASSIS_FOLLOW || chassis_speed_pid_calculate_f->behaviour == CHASSIS_NO_FOLLOW )
		{
			Chassis_yaw_angle_pid_output = PidCalculate(&chassis_speed_pid_calculate_f->chassis_yaw_pid,Chassis_yaw_angle,0);
			Chassis_yaw_speed_pid_output = -PidCalculate(&chassis_speed_pid_calculate_f->chassis_rotate_pid, Chassis_yaw_angle_pid_output , 0);
		}
		else
		{
			Chassis_yaw_speed_pid_output = -PidCalculate(&chassis_speed_pid_calculate_f->chassis_rotate_pid, Chassis_yaw_speed, 0);
		}
		#ifdef POWER_CONTROL
	//FIXME:��������ȫ����ס���ģ��𲽴򻬡�
		count++;
		if(count == 50)
		{
			chassis_speed_pid_calculate_f->chassis_speed_gain = PidCalculate(&chassis_speed_pid_calculate_f->power_pid,chassis_speed_pid_calculate_f->referee_p->Robot_Status.chassis_power_limit,chassis_speed_pid_calculate_f->referee_p->Power_Heat.chassis_power) + 1.0f;
			chassis_speed_pid_calculate_f->chassis_speed_gain -= PidCalculate(&chassis_speed_pid_calculate_f->powerbuff_pid,60,chassis_speed_pid_calculate_f->referee_p->Power_Heat.chassis_power_buffer);
			if(chassis_speed_pid_calculate_f->chassis_speed_gain<=0)chassis_speed_pid_calculate_f->chassis_speed_gain = 0.0;
			count = 0;
		}
		#endif
		
}

void chassis_motion_decomposition(chassis_control_t *chassis_motion_decomposition_f)
{
    chassis_motion_decomposition_f->Chassis_Motor[0].Speed_Set = (-Chassis_y_pid_output - Chassis_x_pid_output + Chassis_yaw_speed_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[1].Speed_Set = (Chassis_y_pid_output - Chassis_x_pid_output + Chassis_yaw_speed_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[2].Speed_Set = (-Chassis_y_pid_output + Chassis_x_pid_output + Chassis_yaw_speed_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[3].Speed_Set = (Chassis_y_pid_output + Chassis_x_pid_output + Chassis_yaw_speed_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
}

void f_CHASSIS_FOLLOW(chassis_control_t *CHASSIS_FOLLOW_f)
{
	//����̨Ϊ������������ֵ����Ϊ��̨
  float Gimbal_x = Chassis_x;
  float Gimbal_y = Chassis_y;
  //����̨�ٶȷֽ⵽����
  Chassis_x = Gimbal_y * sin_calculate(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle) - Gimbal_x * cos_calculate(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle);
  Chassis_y = Gimbal_y * cos_calculate(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle) + Gimbal_x * sin_calculate(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle);
	Chassis_x = -Chassis_x;
	if(user_abs(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle) >= 90.0f)
	{
		Chassis_yaw_angle = -float_min_distance(180,loop_fp32_constrain(CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle,0.0f,360.0f),-180.0f,180.0f);
	}
	else
	{
		Chassis_yaw_angle = CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle;
	}
}

void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f)
{
    //����̨Ϊ������������ֵ����Ϊ��̨
    float Gimbal_x = Chassis_x;
    float Gimbal_y = Chassis_y;
    //����̨�ٶȷֽ⵽����
    Chassis_x = Gimbal_y * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) - Gimbal_x * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_y = Gimbal_y * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) + Gimbal_x * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_x = -Chassis_x;
		#ifdef USE_IMU
		Chassis_yaw_angle =  float_min_distance(loop_fp32_constrain(Chassis_behaviour_react_f->chassis_no_follow_yaw,-180.0f,180.0f),loop_fp32_constrain(Chassis_behaviour_react_f->Imu_c->Yaw,-180.0f,180.0f),-180.0f,180.0f);
		#else
		Chassis_yaw_angle =0
		#endif
}
void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_yaw_speed = CHASSIS_ROTATION_SPEED;
    //����̨Ϊ������������ֵ����Ϊ��̨
    float Gimbal_x = Chassis_x;
    float Gimbal_y = Chassis_y;
    //����̨�ٶȷֽ⵽����
    Chassis_x = Gimbal_y * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) - Gimbal_x * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_y = Gimbal_y * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) + Gimbal_x * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_x = -Chassis_x;
}
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = 0;
    Chassis_y = 0;
    Chassis_yaw_speed = 0;
}
