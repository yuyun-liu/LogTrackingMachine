#include "hmi.h"
#include "ui.h"

extern uint8_t* hmi_buftx;
extern uint8_t* pc_buftx;

uint8_t* buffer = NULL;
uint8_t data_length = 0;

static char cMsg[30];
static char cLmsg[50];
static char Par[20];
static char cPar[24];

int last_title_index = -1;
int _error_page_index = -1;

_Bool hmi_btn_update_flag = 0;
_Bool hmi_in_led_page_flag = 0;

static char* cInitPage = "page TM_init";
static char* cMainPage = "page TM_main";
static char* cAutoPage = "page TM_auto";

static char* cDPS = "TM_main.t0.txt="; 
static char* cDegree = "TM_main.t1.txt=";
static char* cValueAutoModeTime = "TM_main.t2.txt=";
static char* cValueExecutionTime = "TM_main.t3.txt=";
static char* cValueMovement = "TM_main.t4.txt=";
static char* cAutoModeTimeIntervalBcgColor = "TM_main.t2.bco="; 
static char* cAutoHour = "TM_auto.t0.txt="; 
static char* cAutoMin = "TM_auto.t1.txt=";
static char* cAutoSec = "TM_auto.t2.txt="; 

void reset_buffer(void)
{
	for(int i = 0; i < MAX_BUFFER_SIZE; i++)
	{
		buffer[i] = 0;
	}
}

uint8_t* createMessageBuffer_HMI_DMA(char* str)
{
	buffer = msg_AllocateQ();
	
	int i = 0;
	while(str[i]!='\0')
	{
		buffer[i] = str[i];
		i++;
	}
	buffer[i] = 0xFF;
	buffer[i + 1] = 0xFF;
	buffer[i + 2] = 0xFF;
	data_length = i + 3;
	
	return buffer;
}

uint8_t* createMessageBuffer_HMI(char* str)
{
	msg_FreeQ(hmi_buftx);
	buffer = msg_AllocateQ();
	
	int i = 0;
	while(str[i]!='\0')
	{
		buffer[i] = str[i];
		i++;
	}
	buffer[i] = 0xFF;
	buffer[i + 1] = 0xFF;
	buffer[i + 2] = 0xFF;
	data_length = i + 3;
	
	return buffer;
}

void Write_AutoModeTimeInterval_Background_to_HMI(int auto_mode_color)
{	
	memset(cMsg, 0x00, 30);
	memset(Par, 0x00, 5);
	
	strcpy(cMsg, cAutoModeTimeIntervalBcgColor);
	itoa2(auto_mode_color, Par, 5);
	strcat(cMsg, Par);
	Transmit_Msg(&huart1, createMessageBuffer_HMI(cMsg), data_length);
}

void WritePage_to_HMI(uint8_t title_index)
{
	if(title_index == last_title_index)
		return;
	page_index = title_index;
	last_title_index = title_index;
	
	memset(cMsg, 0x00, 30);
	
	switch(title_index)
	{
		case PAGE_INIT:
			strcpy(cMsg, cInitPage);
			break;
		case PAGE_MAIN:
			strcpy(cMsg, cMainPage);
			break;
		case PAGE_AUTO:
			strcpy(cMsg, cAutoPage);
			break;
		default:
			return;
			break;
	}
	Transmit_Msg(&huart1, createMessageBuffer_HMI(cMsg), data_length);
}

void Write_Parameters_To_HMI(uint8_t parameter_index,int parameter_value)
{
	memset(cMsg, 0x00, 30);
	memset(Par, 0x00, 13);
	memset(cPar, 0x00, 17);
	
	char cT[3];
	switch(parameter_index)
	{
		case PARAM_INCH_PER_SECOND:	
			strcpy(cMsg, cDPS);
			itoa2(parameter_value, Par, 4);
			strcat(Par, " inch");
		
			break;
		case PARAM_DEGREE:
			strcpy(cMsg, cDegree);
			itoa2(parameter_value, Par, 3);
			strcat(Par, " deg");
			break;
		case VALUE_AUTOMODE_TIME:
			strcpy(cMsg, cValueAutoModeTime);
			itoa2(automode_parameters.hour, cT, 2);
			strcat(Par, cT);
			strcat(Par, "H:");
			itoa2(automode_parameters.minute, cT, 2);
			strcat(Par, cT);
			strcat(Par, "M:");
			itoa2(automode_parameters.second, cT, 2);
			strcat(Par, cT);
			strcat(Par, "S");
			break;
		case VALUE_MOVEMENT:
			strcpy(cMsg, cValueMovement);
			itoa2((parameter_value % 10000), Par, 5);
			strcat(Par, " inch");
			break;
		case VALUE_EXECUTION_TIME:
			strcpy(cMsg, cValueExecutionTime);
			itoa2(hmi_parameters.hour, cT, 2);
			strcat(Par, cT);
			strcat(Par, "H:");
			itoa2(hmi_parameters.minute, cT, 2);
			strcat(Par, cT);
			strcat(Par, "M:");
			itoa2(hmi_parameters.second, cT, 2);
			strcat(Par, cT);
			strcat(Par, "S");
			break;
		case PARAM_AUTOHOUR:
			strcpy(cMsg, cAutoHour);
			itoa2(parameter_value, Par, 2);
			strcat(Par, " hour");
			break;
		case PARAM_AUTOMIN:
			strcpy(cMsg, cAutoMin);
			itoa2(parameter_value, Par, 2);
			strcat(Par, " min");
			break;
		case PARAM_AUTOSEC:
			strcpy(cMsg, cAutoSec);
			itoa2(parameter_value, Par, 2);
			strcat(Par, " sec");
			break;
		default:
			return;
			break;
	}
	
	char *t = "\"";
	strcpy(cPar, t);
	if(parameter_value < 0)
	{
		strcat(cPar, "-");
	}
	strcat(cPar, Par);
	strcat(cPar, "\"");
	strcat(cMsg, cPar);
	Transmit_Msg(&huart1, createMessageBuffer_HMI(cMsg), data_length);
}

 void Write_ErrorIindex_To_HMI(int error_page_index)
{
	/*
	if(error_page_index == _error_page_index)
		return;
	_error_page_index = error_page_index;
	
	memset(cMsg, 0x00, 30);
	strcpy(cMsg, cParameter);
	switch(error_page_index)
	{
		case P_ERROR1:
			strcat(cMsg, "\"(1)EEPROM错误\"");
			break;
		case P_ERROR2:
			strcat(cMsg, "\"(2)ADC错误\"");
			break;
		case P_ERROR3:
			strcat(cMsg, "\"(3)过电压\"");
			break;
		case P_ERROR4:
			strcat(cMsg, "\"(4)低电压警报\"");
			break;
		case P_ERROR5:
			strcat(cMsg, "\"(5)过电流\"");
			break;
		case P_ERROR6:
			strcat(cMsg, "\"(6)过负载\"");
			break;
		case P_ERROR7:
			strcat(cMsg, "\"(7)电机过速\"");
			break;
		case P_ERROR8:
			strcat(cMsg, "\"(8)传感器错误\"");
			break;
		case P_ERROR9:
			strcat(cMsg, "\"(9)过温警报\"");
			break;
		case P_ERROR10:
			strcat(cMsg, "\"(10)电机位置超差\"");
			break;
		case P_ERROR11:
			strcat(cMsg, "\"(11)驱动器错误\"");
			break;
		case P_ERROR12:
			strcat(cMsg, "\"(12)未知错误\"");
			break;
		case P_ERROR13:
			strcat(cMsg, "\"(13)通讯无应答\"");
			break;
		case P_ERROR14:
			strcat(cMsg, "\"(14)应答错误\"");
			break;
		case P_ERROR15:
			strcat(cMsg, "\"(15)断线\"");
			break;
		default:
			return;
			break;
	}
	Transmit_Msg(&huart1, createMessageBuffer_HMI(cMsg), data_length);*/
}
