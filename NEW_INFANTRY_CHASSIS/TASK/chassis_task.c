#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "can1_receive.h"
#include "can1_send.h"
#include "string.h"
#include "filter.h"
#include "bsp_Motor_Encoder.h"
#include "maths.h"
chassis_control_t Chassis_Control;

static void Chassis_Init(chassis_control_t *Chassis_Control);
static void Chassis_Work(chassis_control_t *Chassis_Control_f);
static void motor_speed_pid_calculate(chassis_control_t *Chassis_pid_calculate_f);
static void motor_position_speed_pid_calculate(chassis_control_t *motor_position_speed_pid_calculate_f);
static void chassis_zero_fore_react(chassis_control_t *chassis_zero_fore_react_f);
void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f);

void Task_Chassis(void const *argument)
{
	uint32_t currentTime;

	Chassis_Init(&Chassis_Control);
	// vTaskDelay(5);

	while (1)
	{
		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��

		taskENTER_CRITICAL(); //�����ٽ���

		Chassis_Work(&Chassis_Control);

		//		Chassis_TO_Gimbal(&Chassis_Control);
		can1_chassis_setmsg(Chassis_Control.Chassis_Motor[0].give_current,
							Chassis_Control.Chassis_Motor[1].give_current,
							Chassis_Control.Chassis_Motor[2].give_current,
							Chassis_Control.Chassis_Motor[3].give_current);

		taskEXIT_CRITICAL(); //�˳��ٽ���

		vTaskDelayUntil(&currentTime, 2); //������ʱ//vTaskDelay(2)
	}
}
/**
 * @brief          �������ݳ�ʼ��
 * @param[in]      *chassis_move_init_f���������ṹ��
 * @retval         none
 */
static void Chassis_Init(chassis_control_t *chassis_data_init_f)
{
	memset(chassis_data_init_f, 0, sizeof(chassis_control_t));
	/*--------------------��ʼ��ָ��--------------------*/
	//��ȡң�ص�ָ��
	chassis_data_init_f->Chassis_RC = get_remote_control_point();

	//��ȡ�����ĸ������ָ��
	chassis_data_init_f->Chassis_Motor[0].chassis_motor_measure = get_chassis_motor_measure_point(0);
	chassis_data_init_f->Chassis_Motor[1].chassis_motor_measure = get_chassis_motor_measure_point(1);
	chassis_data_init_f->Chassis_Motor[2].chassis_motor_measure = get_chassis_motor_measure_point(2);
	chassis_data_init_f->Chassis_Motor[3].chassis_motor_measure = get_chassis_motor_measure_point(3);

	//��ȡ�������ݵ�ָ��
	chassis_data_init_f->super_cap_c = get_supercap_control_point();

	//��ȡ����ģʽ��ָ��
	chassis_data_init_f->behaviour = get_chassis_behaviour_point();

	/*--------------------��ʼ���˲�--------------------*/
	//��ʼ����ͨ�˲�  ��Ҫ���ֵ,������ң�غͼ��̵�ֵ���˲�
	first_order_filter_init(&(chassis_data_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
	first_order_filter_init(&(chassis_data_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);

	/*--------------------��ʼ��������--------------------*/
	chassis_data_init_f->Motor_encoder[0] = Encoder_Init(M3508, 1);
	chassis_data_init_f->Motor_encoder[1] = Encoder_Init(M3508, 2);
	chassis_data_init_f->Motor_encoder[2] = Encoder_Init(M3508, 3);
	chassis_data_init_f->Motor_encoder[3] = Encoder_Init(M3508, 4);

	/*--------------------��ʼ��pid--------------------*/
	/*����pid��ʼ��*/
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

	//����λ�û�pid
	PidInit(&chassis_data_init_f->Chassis_speedX_Pid, CHASSIS_LOCATION_KP, CHASSIS_LOCATION_KI, CHASSIS_LOCATION_KD, StepIn | OutputFilter);
	PidInit(&chassis_data_init_f->Chassis_speedY_Pid, CHASSIS_LOCATION_KP, CHASSIS_LOCATION_KI, CHASSIS_LOCATION_KD, StepIn | OutputFilter);
	PidInitMode(&chassis_data_init_f->Chassis_speedX_Pid, StepIn, 3.0F, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedY_Pid, StepIn, 3.0F, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedX_Pid, OutputFilter, CHASSIS_FIRST_ORDER_FILTER_K, 0);
	PidInitMode(&chassis_data_init_f->Chassis_speedY_Pid, OutputFilter, CHASSIS_FIRST_ORDER_FILTER_K, 0);

	//������ת����pid
	PidInit(&chassis_data_init_f->chassis_rotate_pid, CHASSIS_SPIN_FOLLOW_KP, CHASSIS_SPIN_FOLLOW_KI, CHASSIS_SPIN_FOLLOW_KD, Deadzone | ChangingIntegrationRate | Integral_Limit);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, Deadzone, 0.0f, 0);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, ChangingIntegrationRate, 180.0f, 0.5f);
	PidInitMode(&chassis_data_init_f->chassis_rotate_pid, Integral_Limit, 1000, 0);

	//�ٶ�����
	chassis_data_init_f->chassis_speed_gain = 1;
}
void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f)
{
	float divisor = 1.0f;
	if (chassis_prevent_motion_distortion_f->Chassis_Motor[0].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[1].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[2].pid_output > MOTOR_3508_CURRENT_LIMIT ||
		chassis_prevent_motion_distortion_f->Chassis_Motor[3].pid_output > MOTOR_3508_CURRENT_LIMIT)
	{
		divisor = abs(MOTOR_3508_CURRENT_LIMIT / max(max(chassis_prevent_motion_distortion_f->Chassis_Motor[0].pid_output, chassis_prevent_motion_distortion_f->Chassis_Motor[1].pid_output), max(chassis_prevent_motion_distortion_f->Chassis_Motor[2].pid_output, chassis_prevent_motion_distortion_f->Chassis_Motor[3].pid_output)));
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

void Chassis_Work(chassis_control_t *Chassis_Control_f)
{
	//ѡ�����ģʽ
	chassis_behaviour_choose(Chassis_Control_f);

	//���ݵ���ģʽ����x��y��yawֵ
	chassis_behaviour_react(Chassis_Control_f);

			//����pid�ٶȼ���
		chassis_speed_pid_calculate(Chassis_Control_f);
	
	//����״̬ѡ��
	chassis_state_choose(Chassis_Control_f);

	switch (Chassis_Control_f->chassis_state)
	{
	case CHASSIS_LOCK_POSITION:
		//λ���ٶȻ�����pid
		motor_position_speed_pid_calculate(Chassis_Control_f);
		break;
	case CHASSIS_SPEED:


		//�˶��ֽ�
		chassis_motion_decomposition(Chassis_Control_f);

		//���pid�ٶȼ���
		motor_speed_pid_calculate(Chassis_Control_f);
		break;
	case CHASSIS_ZERO_FORCE:
		chassis_zero_fore_react(Chassis_Control_f);
		break;
	default:
		break;
	}
	//��ֹ�˶�ʧ��
	chassis_prevent_motion_distortion(Chassis_Control_f);
}
