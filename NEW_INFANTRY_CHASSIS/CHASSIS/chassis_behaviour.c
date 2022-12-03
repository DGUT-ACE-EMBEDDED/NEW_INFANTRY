#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "rc.h"
#include "maths.h"

chassis_behaviour_e Chassis_Behaviour;
float Chassis_x = 0.0f;
float Chassis_y = 0.0f;
float Chassis_yaw = 0.0f;
float Chassis_x_pid_output = 0.0f;
float Chassis_y_pid_output = 0.0f;
float Chassis_yaw_pid_output = 0.0f;
/********************函数声明********************/
void f_CHASSIS_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f);

void chassis_behaviour_choose(chassis_control_t *Chassis_behaviour_f)
{
    //用于记录上一次数据
    chassis_behaviour_e last_behaviour;
    static chassis_behaviour_e rc_behaviour = CHASSIS_FOLLOW;
    static chassis_behaviour_e kb_behaviour = CHASSIS_FOLLOW;

    //手柄
    last_behaviour = rc_behaviour;
    switch (Chassis_behaviour_f->Chassis_RC->rc.s[0])
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
    //如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        *Chassis_behaviour_f->behaviour = rc_behaviour;
    }

    //键鼠
    last_behaviour = kb_behaviour;
    //**Q
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.Q)
    {
        kb_behaviour = CHASSIS_FOLLOW; //底盘跟随模式
    }
    //**E
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.E)
    {
        kb_behaviour = CHASSIS_ROTATION; //小陀螺模式
    }
    //**F
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.F)
    {
        kb_behaviour = CHASSIS_FOLLOW; //底盘跟随模式
    }
    //**G 补给模式
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.G)
    {
        kb_behaviour = CHASSIS_NO_FOLLOW; //补给模式下，底盘不跟随云台
    }
    //如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        *Chassis_behaviour_f->behaviour = kb_behaviour;
    }
}

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = Chassis_behaviour_react_f->Chassis_RC->rc.ch[0] + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.A + Chassis_behaviour_react_f->Chassis_RC->kb.bit.D) * 660;
    value_limit(Chassis_x, -660, 660);
    Chassis_y = Chassis_behaviour_react_f->Chassis_RC->rc.ch[1] + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W) * 660;
    value_limit(Chassis_y, -660, 660);

    switch (*Chassis_behaviour_react_f->behaviour)
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

    //没头的时候旋转用的
    Chassis_yaw += Chassis_behaviour_react_f->Chassis_RC->rc.ch[2];
}
void chassis_state_choose(chassis_control_t *chassis_state_choose_f)
{
    chassis_state_e last_state;
    last_state = chassis_state_choose_f->chassis_state;
    //如果没有控制量，锁死底盘
    if (((Chassis_x < 5) && (Chassis_x > -5)) && ((Chassis_y < 5) && (Chassis_y > -5)) && ((Chassis_yaw < 5) && (Chassis_yaw > -5)))
    {
        chassis_state_choose_f->chassis_state = CHASSIS_LOCK_POSITION;
    }
    else
    {
        chassis_state_choose_f->chassis_state = CHASSIS_SPEED;
    }
    if (last_state != chassis_state_choose_f->chassis_state)
    {
        EncoderValZero(chassis_state_choose_f->Motor_encoder[0]);
        EncoderValZero(chassis_state_choose_f->Motor_encoder[1]);
        EncoderValZero(chassis_state_choose_f->Motor_encoder[2]);
        EncoderValZero(chassis_state_choose_f->Motor_encoder[3]);
    }

    if (chassis_state_choose_f->Chassis_RC->rc.s[1] == RC_SW_DOWN)
    {
        chassis_state_choose_f->chassis_state = CHASSIS_ZERO_FORCE;
    }
}
void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f)
{
    Chassis_x_pid_output = -PidCalculate(&chassis_speed_pid_calculate_f->Chassis_speedX_Pid, Chassis_x, 0);
    Chassis_y_pid_output = PidCalculate(&chassis_speed_pid_calculate_f->Chassis_speedY_Pid, Chassis_y, 0);
    Chassis_yaw_pid_output = PidCalculate(&chassis_speed_pid_calculate_f->chassis_rotate_pid, Chassis_yaw, 0);
}

void chassis_motion_decomposition(chassis_control_t *chassis_motion_decomposition_f)
{
    chassis_motion_decomposition_f->Chassis_Motor[0].Speed_Set = (-Chassis_y_pid_output - Chassis_x_pid_output + Chassis_yaw_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[1].Speed_Set = (Chassis_y_pid_output - Chassis_x_pid_output + Chassis_yaw_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[2].Speed_Set = (-Chassis_y_pid_output + Chassis_x_pid_output + Chassis_yaw_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
    chassis_motion_decomposition_f->Chassis_Motor[3].Speed_Set = (Chassis_y_pid_output + Chassis_x_pid_output + Chassis_yaw_pid_output) * chassis_motion_decomposition_f->chassis_speed_gain;
}

void f_CHASSIS_FOLLOW(chassis_control_t *CHASSIS_FOLLOW_f)
{
    Chassis_yaw = CHASSIS_FOLLOW_f->Chassis_Gimbal_Diference_Angle;
}

void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f)
{
    //以云台为主，将底盘数值设置为云台
    float Gimbal_x = Chassis_x;
    float Gimbal_y = Chassis_y;
    //将云台速度分解到底盘
    Chassis_x = Gimbal_y * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) - Gimbal_x * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_y = Gimbal_y * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) + Gimbal_x * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_x = -Chassis_x;
    Chassis_yaw = 0;
}
void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_yaw = CHASSIS_ROTATION_SPEED;
    //以云台为主，将底盘数值设置为云台
    float Gimbal_x = Chassis_x;
    float Gimbal_y = Chassis_y;
    //将云台速度分解到底盘
    Chassis_x = Gimbal_y * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) - Gimbal_x * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_y = Gimbal_y * cos_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle) + Gimbal_x * sin_calculate(Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle);
    Chassis_x = -Chassis_x;
}
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = 0;
    Chassis_y = 0;
    Chassis_yaw = 0;
}
/**
 * @brief          返回底盘模式指针
 * @param[in]      none
 * @retval
 */
chassis_behaviour_e *get_chassis_behaviour_point(void)
{
    return (&Chassis_Behaviour);
}
