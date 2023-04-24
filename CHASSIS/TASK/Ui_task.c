#include "Ui_task.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "rm_cilent_ui.h"

#define MAX_X 1920
#define MAX_Y 1080

const static char *chassis_mode_ui[4] = {
    "FOLLOW",      //����
    "NO_FOLLOW",   //������   
    "ROTATION",    //С����
    "Battery"};    //��̨ģʽ

chassis_control_t *Chassis_Control_p = NULL;
Graph_Data diff_ang;

Float_Data CAP_VOLTAGE;
String_Data CH_MODE,voltage,replenish;
		
static void ui_init(void);           //ui��ʼ��
static void print_mode(void);        //����ģʽ��ʾ
static void print_chassis_gimbal_angle(void);
static void print_cap_voltage(void);
/**
  * @brief      ui��ʾ����
  * @param[in]  *pvParameters
  * @retval     void    
  * @attention  
  */
void UI_Task(void const * argument)
{
	int count =0;
	vTaskDelay(10);
	Chassis_Control_p = Get_Chassis_Control_p();
	ui_init();
	while (1)
	{
		print_cap_voltage();
		print_chassis_gimbal_angle();
		print_mode();

		count++;
		if(count == 100)
		{
			ui_init();
			count = 0;
		}
		vTaskDelay(1);
	}
}


	
/** 
  * @brief      ui��ʼ��
  * @param[in]  void
  * @retval     void    
  * @attention  û�еĻ�������һ��
  */
static void ui_init(void)
{
		
		/*--------------------����ui����--------------------*/
		Char_Draw(&voltage, "078", UI_Graph_ADD, 7, UI_Color_Green, 20, 4, 1080, 300, "Voltage:");
		My_Char_Refresh(voltage);
		Float_Draw(&CAP_VOLTAGE, "075", UI_Graph_ADD, 6, UI_Color_Green, 20, 32, 3, 1230, 300, (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000));		//x��ƫ��Ϊ150���ұȽ�����
		My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);

		/*--------------------������̨���ui����--------------------*/
		//���е�ΪԲ�ģ���̨����Ϊǰ������200Ϊ�뾶�������̵�����λ��
		uint32_t start_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle -45 -180 ,0,360);
		uint32_t end_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle +45 -180,0,360);
		Arc_Draw(&diff_ang,"001",UI_Graph_ADD ,6 ,UI_Color_Main ,start_angle,end_angle,5,MAX_X/2,MAX_Y/2,200,200);
		My_Graph_Refresh(&diff_ang);

		/*--------------------����ģʽui����--------------------*/
		Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 8, UI_Color_Main, 25, 5, 80, 880-80, chassis_mode_ui[Chassis_Control_p->behaviour]);
		My_Char_Refresh(CH_MODE);
	
		/*--------------------��̨����ui����--------------------*/
		if(Chassis_Control_p->gimbal_data_p->replenish_flag)
			Char_Draw(&replenish, "002", UI_Graph_ADD, 7, UI_Color_Green, 20, 4, 80, 600, "replenish:");
		else
			Char_Draw(&replenish, "002", UI_Graph_ADD, 7, UI_Color_Orange, 20, 4, 80, 600, "replenish:");
		My_Char_Refresh(replenish);
}

/**
  * @brief      ��̨������ģʽ��ʾ
  */
static void print_mode(void)
{
	//����
    Char_Draw(&CH_MODE, "093", UI_Graph_Change, 8, UI_Color_Main, 25, 5, 80, 880-80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);
		if(Chassis_Control_p->gimbal_data_p->replenish_flag)
			Char_Draw(&replenish, "002", UI_Graph_Change, 7, UI_Color_Green, 20, 4, 80, 600, "replenish:");
		else
			Char_Draw(&replenish, "002", UI_Graph_Change, 7, UI_Color_Orange, 20, 4, 80, 600, "replenish:");
		My_Char_Refresh(replenish);
	//��̨
//	GI_MODE.Graph_Control.end_angle = strlen(gimbal_mode_ui[re_gimbal_behaviour()]);
//	strcpy(GI_MODE.show_Data, gimbal_mode_ui[re_gimbal_behaviour()]);
//	GI_MODE.Graph_Control.operate_tpye = UI_Graph_Change;
//	My_Char_Refresh(GI_MODE);
}

/**
  * @brief      ���ݵ�ѹ��ʾ
  */
static void print_cap_voltage(void)
{
    CAP_VOLTAGE.graph_Float = (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000);
    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}
/**
  * @brief      ������̨�Ƕ���ʾ
  */
static void print_chassis_gimbal_angle(void)
{
		uint32_t start_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle -45 -180,0,360);
		uint32_t end_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle +45 -180,0,360);
		Arc_Draw(&diff_ang,"001",UI_Graph_Change ,6 ,UI_Color_Main ,start_angle,end_angle,5,MAX_X/2,MAX_Y/2,200,200);
		My_Graph_Refresh(&diff_ang);
}
