#ifndef __UI_H__
#define __UI_H__

#include "stdint.h"
#include "main.h"
#include "uart.h"
//#include "flash.h"

extern BUTTON_STATUS startBtnStatus;
extern BUTTON_STATUS stopBtnStatus;
extern BUTTON_STATUS enterBtnStatus;
extern BUTTON_STATUS upBtnStatus;
extern BUTTON_STATUS downBtnStatus;
extern BUTTON_STATUS rightBtnStatus;
extern BUTTON_STATUS leftBtnStatus;
extern BUTTON_STATUS upHourBtnStatus;
extern BUTTON_STATUS downHourBtnStatus;
extern BUTTON_STATUS upMinBtnStatus;
extern BUTTON_STATUS downMinBtnStatus;
extern BUTTON_STATUS upSecBtnStatus;
extern BUTTON_STATUS downSecBtnStatus;

void Set_Parameters(void);
void Set_Main_Parameters(void);
void Set_Auto_Mode_Hours(void);
void Set_Auto_Mode_Minutess(void);
void Set_Auto_Mode_Seconds(void);
void Update_Up_Down_Btn_count(void);
void Indicate_Actual_Movement(int data);
void Indicate_Execution_Time();
#endif
