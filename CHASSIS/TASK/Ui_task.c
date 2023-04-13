#include "Ui_task.h"

#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "rm_cilent_ui.h"

Graph_Data G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11;

/**
  * @brief      ui显示任务
  * @param[in]  *pvParameters
  * @retval     void    
  * @attention  
  */
void UI_Task(void const * argument)
{
	vTaskDelay(10);
	uint16_t i = 0;
	char test[3];
    while (1)
    {
			for(i = 0; i<1920 ; i+=3)
			{
				test[0] = i>>8;
				test[1] = i;
				Rectangle_Draw(&G1,test, UI_Graph_ADD, 9, UI_Color_Cyan, 3, i, 0, 1920-i, 1080);
				My_Graph_Refresh((Graph_Data *)&G1);
				
			}
	/*--------------------吊射ui--------------------*/
			Line_Draw(&G1, "091", UI_Graph_ADD, 4, UI_Color_Cyan, 1, 0, 1919, 0, 1079);
			My_Graph_Refresh((Graph_Data *)&G1);
			Line_Draw(&G1, "092", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 880, 580, 1040, 580);
			My_Graph_Refresh((Graph_Data *)&G1);
//    Line_Draw(&G1, "093", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 800, 540, 1120, 540);
//			My_Graph_Refresh((Graph_Data *)&G1);
//    Line_Draw(&G1, "094", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 880, 500, 1040, 500);
//			My_Graph_Refresh((Graph_Data *)&G1);
//    Line_Draw(&G1, "095", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 900, 420, 1020, 420);
//			My_Graph_Refresh((Graph_Data *)&G1);
//    Line_Draw(&G1, "096", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 920, 370, 1000, 370);
			
//		My_Graph_Refresh((Graph_Data *)&G1);
   
			
//	  Line_Draw(&G1, "091", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 960, 330, 960, 620);
//    Line_Draw(&G2, "092", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 880, 580, 1040, 580);
//    Line_Draw(&G3, "093", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 800, 540, 1120, 540);
//    Line_Draw(&G4, "094", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 880, 500, 1040, 500);
//    Line_Draw(&G5, "095", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 900, 420, 1020, 420);
//    Line_Draw(&G6, "096", UI_Graph_Del, 4, UI_Color_Purplish_red, 1, 920, 370, 1000, 370);
//		
//		My_Graph_Refresh((Graph_Data *)&G1);
//    My_Graph_Refresh((Graph_Data *)&G2);
//    My_Graph_Refresh((Graph_Data *)&G3);
//    My_Graph_Refresh((Graph_Data *)&G4);
//    My_Graph_Refresh((Graph_Data *)&G5);
//    My_Graph_Refresh((Graph_Data *)&G6);
        vTaskDelay(10);//频率控制
    }
}
