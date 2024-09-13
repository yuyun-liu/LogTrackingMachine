#ifndef __HMI_H__
#define	__HMI_H__

#include "stdint.h"
#include "main.h"
#include "msg_queue.h"
#include "string.h"
#include "uart.h"
#include "utility.h"
#include "math.h"

void Write_AutoModeTimeInterval_Background_to_HMI(int auto_mode_color);
void WritePage_to_HMI(uint8_t title_index);
void Write_Parameters_To_HMI(uint8_t parameter_index,int parameter_value);
void Write_ErrorIindex_To_HMI(int error_page_index);
void EnterInit_Page();
void EnterMain_Page();

uint8_t* createMessageBuffer_HMI_DMA(char* str);
uint8_t* createMessageBuffer_HMI(char* str);
#endif