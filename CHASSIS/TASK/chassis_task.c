#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "can1_receive.h"
#include "can2_receive.h"
#include "can1_send.h"
#include "can2_send.h"
#include "string.h"
#include "bsp_Motor_Encoder.h"
#include "maths.h"
#include "chassis_config.h"
#include "bsp_referee.h"
chassis_control_t Chassis_Control;

static void Chassis_Init(chassis_control_t *Chassis_Control);
static void Chassis_Work(chassis_control_t *Chassis_Control_f);
static void motor_speed_pid_calculate(chassis_control_t *Chassis_pid_calculate_f);
static void motor_position_speed_pid_calculate(chassis_control_t *motor_position_speed_pid_calculate_f);
static void chassis_zero_fore_react(chassis_control_t *chassis_zero_fore_react_f);
static void chassis_state_react(chassis_control_t *chassis_state_react_f);
static void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f);
static void chassis_get_gimbal_differece_angle(chassis_control_t *chassis_get_gimbal_differece_angle_f);


void Task_Chassis(void const *argument)
{
	
	Chassis_Init(&Chassis_Control);
	vTaskDelay(5);

	while (1)
	{
		taskENTER_CRITICAL(); //进入临界区
		can2_chassis_to_gimbal_referee(Chassis_Control.referee_p);
		Chassis_Work(&Chassis_Control);

		can2_chassis_to_gimbal(Chassis_Control.Chassis_RC);
		can1_chassis_setmsg(Chassis_Control.Chassis_Motor[0].give_current,
							Chassis_Control.Chassis_Motor[1].give_current,
							Chassis_Control.Chassis_Motor[2].give_current,
							Chassis_Control.Chassis_Motor[3].give_current);
		can1_cap_setmsg(50);
		taskEXIT_CRITICAL(); //退出临界区

		//vTaskDelayUntil(&currentTime, 1); //绝对延时//vTaskDelay(2)
		vTaskDelay(1);
	}
}

void Chassis_Work(chassis_control_t *Chassis_Control_f)
{
	//底盘云台角度计算
	chassis_get_gimbal_differece_angle(Chassis_Control_f);

	//选择底盘模式
	chassis_behaviour_choose(Chassis_Control_f);

	//根据底盘模式计算x、y、yaw值
	chassis_behaviour_react(Chassis_Control_f);

	//底盘pid速度计算
	chassis_speed_pid_calculate(Chassis_Control_f);

	//底盘状态选择
	chassis_state_choose(Chassis_Control_f);

	//根据底盘状态计算电机输出量
	chassis_state_react(Chassis_Control_f);

	//防止运动失真
	chassis_prevent_motion_distortion(Chassis_Control_f);
}
/**
 * @brief          底盘数据初始化
 * @param[in]      *chassis_move_init_f：底盘主结构体
 * @retval         none
 */
static void Chassis_Init(chassis_control_t *chassis_data_init_f)
{
	memset(chassis_data_init_f, 0, sizeof(chassis_control_t));
	/*--------------------初始化指针--------------------*/
	//获取遥控的指针
	chassis_data_init_f->Chassis_RC = RC_Get_RC_Pointer();
	
	//获取底盘四个电机的指针
	chassis_data_init_f->Chassis_Motor[0].chassis_motor_measure = get_chassis_motor_measure_point(0);
	chassis_data_init_f->Chassis_Motor[1].chassis_motor_measure = get_chassis_motor_measure_point(1);
	chassis_data_init_f->Chassis_Motor[2].chassis_motor_measure = get_chassis_motor_measure_point(2);
	chassis_data_init_f->Chassis_Motor[3].chassis_motor_measure = get_chassis_motor_measure_point(3);
	//获取底盘YAW电机的指针
	chassis_data_init_f->yaw_motor = get_yaw_motor_measure_point();
	//获取超级电容的指针
	chassis_data_init_f->super_cap_c = get_supercap_control_point();
	//获取裁判系统的指针
	chassis_data_init_f->referee_p = Get_referee_Address();

	/*--------------------初始化编码器--------------------*/
	chassis_data_init_f->Motor_encoder[0] = Encoder_Init(M3508, 1);
	chassis_data_init_f->Motor_encoder[1] = Encoder_Init(M3508, 2);
	chassis_data_init_f->Motor_encoder[2] = Encoder_Init(M3508, 3);
	chassis_data_init_f->Motor_encoder[3] = Encoder_Init(M3508, 4);
	chassis_data_init_f->yaw_motor_encoder = Encoder_Init(GM6020, 5);//YAW

	/*--------------------初始化pid--------------------*/
	/*底盘pid初始化*/
	PidInit(&chassis_data_init_f->motor_Speed_Pid[0], CHASSIS_RF_MOTOR_KP, CHASSIS_RF_MOTOR_KI, CHASSIS_RF_MOTOR_KD, Output_Limit);
	PidInit(&chassis_data_init_f->motor_Speed_Pid[1], CHASSIS_LF_MOTOR_KP, CHASSIS_LF_MOTOR_KI, CHASSIS_LF_MOTOR_KD, Output_Limit);
	PidInit(&chassis_data_init_f->motor_Speed_Pid[2], CHASSIS_RB_MOTOR_KP, CHASSIS_RB_MOTOR_KI, CHASSIS_RB_MOTOR_KD, Output_Limit);
	PidInit(&chassis_data_init_f->motor_Speed_Pid[3], CHASSIS_LB_MOTOR_KP, CHASSIS_LB_MOTOR_KI, CHASSIS_LB_MOTOR_KD, Output_Limit);
	PidInitMode(&chassis_data_init_f->motor_Speed_Pid[0], Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->motor_Speed_Pid[1], Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->motor_Speed_Pid[2], Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->motor_Speed_Pid[3], Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);

	PidInit(&chassis_data_init_f->motor_Position_Pid[0], 0.05f, 0, 0, NONE);
	PidInit(&chassis_data_init_f->motor_Position_Pid[1], 0.05f, 0, 0, NONE);
	PidInit(&chassis_data_init_f->motor_Position_Pid[2], 0.05f, 0, 0, NONE);
	PidInit(&chassis_data_init_f->motor_Position_Pid[3], 0.05f, 0, 0, NONE);

	//底盘位置环pid
	PidInit(&chassis_data_init_f->Chassis_speedX_Pid, CHASSIS_LOCATION_KP, CHASSIS_LOCATION_KI, CHASSIS_LOCATION_KD, StepIn | OutputFilter);
	PidInit(&chassis_data_init_f->Chassis_speedY_Pid, CHASSIS_LOCATION_KP, CHASSIS_LOCATION_KI, CHASSIS_LOCATION_KD, StepIn | OutputFilter);
	PidInitMode(&chassis_data_init_f->Chassis_speedX_Pid, StepIn, 3.0F, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedY_Pid, StepIn, 3.0F, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedX_Pid, OutputFilter, CHASSIS_FIRST_ORDER_FILTER_K, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedY_Pid, OutputFilter, CHASSIS_FIRST_ORDER_FILTER_K, 0);
	
	#ifdef POWER_CONTROL
	PidInit(&chassis_data_init_f->power_pid, 0.03, 0.005, 0.0, Output_Limit | Integral_Limit | OutputFilter );
	PidInitMode(&chassis_data_init_f->power_pid , Output_Limit , 3.0,0);
	PidInitMode(&chassis_data_init_f->power_pid , Integral_Limit , 200,0);
	PidInitMode(&chassis_data_init_f->power_pid , OutputFilter , 0.04,0);

	PidInit(&chassis_data_init_f->powerbuff_pid, 1.0/55.0, 0.0, 0.0, NULL);
	#endif
	
	//底盘旋转跟随pid
	PidInit(&chassis_data_init_f->chassis_rotate_pid, CHASSIS_SPIN_FOLLOW_KP, CHASSIS_SPIN_FOLLOW_KI, CHASSIS_SPIN_FOLLOW_KD, Deadzone | ChangingIntegrationRate | Integral_Limit);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, Deadzone, 0.0f, 0);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, ChangingIntegrationRate, 180.0f, 0.5f);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, Integral_Limit, 1000, 0);

	chassis_data_init_f->behaviour = CHASSIS_NO_FOLLOW;
	//速度因子
	chassis_data_init_f->chassis_speed_gain = 1;
}
void chassis_state_react(chassis_control_t *chassis_state_react_f)
{
	switch (chassis_state_react_f->chassis_state)
	{
	case CHASSIS_LOCK_POSITION:
		//位置速度环串级pid
		motor_position_speed_pid_calculate(chassis_state_react_f);
		break;
	case CHASSIS_SPEED:

		//运动分解
		chassis_motion_decomposition(chassis_state_react_f);

		//电机pid速度计算
		motor_speed_pid_calculate(chassis_state_react_f);
		break;
	case CHASSIS_ZERO_FORCE:
		chassis_zero_fore_react(chassis_state_react_f);
		break;
	default:
		break;
	}
}
void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f)
{
	float divisor = 1.0f;
	if (chassis_prevent_motion_distortion_f->Chassis_Motor[0].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[1].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[2].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[3].pid_output > MOTOR_3508_CURRENT_LIMIT)
	{
		divisor = user_abs(MOTOR_3508_CURRENT_LIMIT / max(max(chassis_prevent_motion_distortion_f->Chassis_Motor[0].pid_output, chassis_prevent_motion_distortion_f->Chassis_Motor[1].pid_output), max(chassis_prevent_motion_distortion_f->Chassis_Motor[2].pid_output, chassis_prevent_motion_distortion_f->Chassis_Motor[3].pid_output)));
	}
	chassis_prevent_motion_distortion_f->Chassis_Motor[0].give_current = chassis_prevent_motion_distortion_f->Chassis_Motor[0].pid_output * divisor;
	chassis_prevent_motion_distortion_f->Chassis_Motor[1].give_current = chassis_prevent_motion_distortion_f->Chassis_Motor[1].pid_output * divisor;
	chassis_prevent_motion_distortion_f->Chassis_Motor[2].give_current = chassis_prevent_motion_distortion_f->Chassis_Motor[2].pid_output * divisor;
	chassis_prevent_motion_distortion_f->Chassis_Motor[3].give_current = chassis_prevent_motion_distortion_f->Chassis_Motor[3].pid_output * divisor;
}
void chassis_zero_fore_react(chassis_control_t *chassis_zero_fore_react_f)
{
	chassis_zero_fore_react_f->Chassis_Motor[0].pid_output = 0;
	chassis_zero_fore_react_f->Chassis_Motor[1].pid_output = 0;
	chassis_zero_fore_react_f->Chassis_Motor[2].pid_output = 0;
	chassis_zero_fore_react_f->Chassis_Motor[3].pid_output = 0;
}
void motor_position_speed_pid_calculate(chassis_control_t *motor_position_speed_pid_calculate_f)
{
	motor_position_speed_pid_calculate_f->Chassis_Motor[0].pid_output = motor_position_speed_control(&motor_position_speed_pid_calculate_f->motor_Speed_Pid[0], &motor_position_speed_pid_calculate_f->motor_Position_Pid[0], 0, motor_position_speed_pid_calculate_f->Motor_encoder[0]->Encode_Record_Val, motor_position_speed_pid_calculate_f->Chassis_Motor[0].chassis_motor_measure->speed);
	motor_position_speed_pid_calculate_f->Chassis_Motor[1].pid_output = motor_position_speed_control(&motor_position_speed_pid_calculate_f->motor_Speed_Pid[1], &motor_position_speed_pid_calculate_f->motor_Position_Pid[1], 0, motor_position_speed_pid_calculate_f->Motor_encoder[1]->Encode_Record_Val, motor_position_speed_pid_calculate_f->Chassis_Motor[1].chassis_motor_measure->speed);
	motor_position_speed_pid_calculate_f->Chassis_Motor[2].pid_output = motor_position_speed_control(&motor_position_speed_pid_calculate_f->motor_Speed_Pid[2], &motor_position_speed_pid_calculate_f->motor_Position_Pid[2], 0, motor_position_speed_pid_calculate_f->Motor_encoder[2]->Encode_Record_Val, motor_position_speed_pid_calculate_f->Chassis_Motor[2].chassis_motor_measure->speed);
	motor_position_speed_pid_calculate_f->Chassis_Motor[3].pid_output = motor_position_speed_control(&motor_position_speed_pid_calculate_f->motor_Speed_Pid[3], &motor_position_speed_pid_calculate_f->motor_Position_Pid[3], 0, motor_position_speed_pid_calculate_f->Motor_encoder[3]->Encode_Record_Val, motor_position_speed_pid_calculate_f->Chassis_Motor[3].chassis_motor_measure->speed);
}

void motor_speed_pid_calculate(chassis_control_t *Chassis_pid_calculate_f)
{
	Chassis_pid_calculate_f->Chassis_Motor[0].pid_output = motor_speed_control(&Chassis_pid_calculate_f->motor_Speed_Pid[0], Chassis_pid_calculate_f->Chassis_Motor[0].Speed_Set, Chassis_pid_calculate_f->Chassis_Motor[0].chassis_motor_measure->speed);
	Chassis_pid_calculate_f->Chassis_Motor[1].pid_output = motor_speed_control(&Chassis_pid_calculate_f->motor_Speed_Pid[1], Chassis_pid_calculate_f->Chassis_Motor[1].Speed_Set, Chassis_pid_calculate_f->Chassis_Motor[1].chassis_motor_measure->speed);
	Chassis_pid_calculate_f->Chassis_Motor[2].pid_output = motor_speed_control(&Chassis_pid_calculate_f->motor_Speed_Pid[2], Chassis_pid_calculate_f->Chassis_Motor[2].Speed_Set, Chassis_pid_calculate_f->Chassis_Motor[2].chassis_motor_measure->speed);
	Chassis_pid_calculate_f->Chassis_Motor[3].pid_output = motor_speed_control(&Chassis_pid_calculate_f->motor_Speed_Pid[3], Chassis_pid_calculate_f->Chassis_Motor[3].Speed_Set, Chassis_pid_calculate_f->Chassis_Motor[3].chassis_motor_measure->speed);
}
void chassis_get_gimbal_differece_angle(chassis_control_t *chassis_get_gimbal_differece_angle_f)
{
	chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = ((float)(chassis_get_gimbal_differece_angle_f->yaw_motor_encoder->Encode_Actual_Val - YAW_ZERO_OFFSET)* 360.0f / 8192.0f);
  chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = loop_fp32_constrain(chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle, -180.0f, 180.0f);
}
