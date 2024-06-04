#include "ui.h"
#include "math.h"
#include "uart.h"
#include "hmi.h"

/*------------VAR-------------*/
_Bool md_message_transmit_delay_flag = 0;	//Motor Driver need >= 10ms delay for continuous message transmit
_Bool isRecovery = 0;
_Bool led_blink = 0;
uint8_t led_blink_state = 0;

uint16_t bones_value_array[10];

int recovery_i = 0;
/*------------VAR-------------*/
BUTTON_STATUS startBtnStatus = {0, 0, 0, 0, 0, 0, 60};
BUTTON_STATUS enterBtnStatus = {0, 0, 0, 0, 0, 0, 60};
BUTTON_STATUS upBtnStatus = {0, 0, 0, 0, 0, 0, 60};
BUTTON_STATUS downBtnStatus = {0, 0, 0, 0, 0, 0, 60};
BUTTON_STATUS rightBtnStatus = {0, 0, 0, 0, 0, 0, 60};
BUTTON_STATUS leftBtnStatus = {0, 0, 0, 0, 0, 0, 60};

void Single_Add_Minus_Advanced_settings(float* parameter, float upperLimit, float lowerLimit, float interval, int* mode, _Bool isFloat)
{
	float offset = isFloat ? 0.000 : 0.000;
	
	switch(*mode)
	{
		case 1:
			*parameter = *parameter - interval - offset;
			if(*parameter <= lowerLimit + offset)
			{
				*parameter = lowerLimit;
			}
			break;
		case 2:
			*parameter = *parameter + interval + offset;
			if(*parameter >= upperLimit)
			{
				*parameter = upperLimit;
			}
			break;
		default:
			break;
	}
}
void Continuous_Add_Minus_Advanced_settings(float* parameter, float upperLimit, float lowerLimmit, int interval, int* mode, _Bool isFloat)
{
	float offset = isFloat ? 0.00 : 0.00;
	
	if(upBtnStatus.newBtnStatus)
	{
		if(isElapsedTime(500, &upBtnStatus.count))
		{
			*mode = 0;
			upBtnStatus.count = new_tick;
			if(isFloat)
			{
				int r = 10 - (int)((int)((*parameter * interval) + 0.5) % 10);
				*parameter += ((float)r / (float)interval);
			}
			else
			{
				int r = interval - ((int)*parameter % interval);
				*parameter += r;
			}
			if(*parameter >= upperLimit)
				*parameter = upperLimit;
		}
	}
	if(downBtnStatus.newBtnStatus)
	{
		if(isElapsedTime(500, &downBtnStatus.count))
		{
			*mode = 0;
			downBtnStatus.count = new_tick;
			if(isFloat)
			{
				int r = (int)((int)((*parameter * interval) + 0.5)%10);
				r = r == 0 ? 10 : r;
				*parameter -= ((float)r / (float)interval);
			}
			else
			{
				int r = 0;
				if(*parameter > 0)
				{
					r = ((int)*parameter % interval);
					r = r==0 ? interval : r;
				}
				else
				{
					r = interval - ((int)*parameter % interval);
				}
				*parameter -= r;
			}
			if(*parameter <= (lowerLimmit + offset))
				*parameter = lowerLimmit;
		}
	}
}

void Broadcast_To_PeripheralDevice(float* buffer_element, float* parameter, int index, uint16_t parameter_address, _Bool isAdvanced)
{
	if(*buffer_element != *parameter)
	{
		//Write_LcdParameter_To_HMI(index, lcdParameters, advancedTime_setting);
		/*
		if(isAdvanced)
			Write_MotorDriver(1, parameter_address, (uint16_t)(*parameter * 100 + 0.5));
		else if(!isAdvanced && (parameter_address != 0))
			Write_MotorDriver(1, parameter_address, (uint16_t)(*parameter));*/
		
		Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
		Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
	}
	*buffer_element = *parameter;
}

void Indicate_Actual_Movement(int data)
{
	if(m2.motor_motion_flag == MOTOR_MOVING)
	{
		return;
	}
#if 1
	if(isElapsedTime(250, &lastTimeTick_value_movement))
	{
		Write_Parameters_To_HMI(VALUE_MOVEMENT, data);
	}
#else
	if(isElapsedTime(100, &lastTimeTick_value_movement))
	{
		Write_Parameters_To_HMI(VALUE_MOVEMENT, velocity_0);
	}
#endif
}

void Indicate_Execution_Time()
{
	if(m2.motor_motion_flag == MOTOR_MOVING)
	{
		return;
	}
	
	if(isElapsedTime(1000, &lastTimeTick_execution_time))
	{
		Write_Parameters_To_HMI(VALUE_EXECUTION_TIME, 0);
	}
}

void Set_Parameters(void)
{
	if(m2.motor_motion_flag == MOTOR_IDLE)
	{
		int mode = 0;
		int tmpUpperLimit;					
		int tmpLowerLimit;						
		int tmpInterval;							
		float* tmpBuffer_element; 			
		float* tmpParameter; 						
		int tmpIndex;										
		uint16_t tmpParameter_address;
		
		if(upBtnStatus.newBtnStatus && !upBtnStatus.lastBtnStatus)
			mode = 2;
		if(downBtnStatus.newBtnStatus && !downBtnStatus.lastBtnStatus)
			mode = 1;
		switch(page_index)
		{
			case PAGE_MAIN:
				switch(event_id)
				{
					case EVENT_INCREASE_RPM:
						tmpBuffer_element = &hmi_parameters_buffer[0];
						tmpParameter = &hmi_parameters.inch_per_second;
						tmpUpperLimit = 80;
						tmpLowerLimit = -80;
						tmpInterval = 50;
						break;
					case EVENT_DECREASE_RPM:
						tmpBuffer_element = &hmi_parameters_buffer[0];
						tmpParameter = &hmi_parameters.inch_per_second;
						tmpUpperLimit = 80;
						tmpLowerLimit = -80;
						tmpInterval = 50;
						break;
					case EVENT_TURN_RIGHT:
						tmpBuffer_element = &hmi_parameters_buffer[1];
						tmpParameter = &hmi_parameters.degree;
						tmpUpperLimit = 180;
						tmpLowerLimit = 0;
						tmpInterval = 450;
						break;
					case EVENT_TURN_LEFT:
						tmpBuffer_element = &hmi_parameters_buffer[1];
						tmpParameter = &hmi_parameters.degree;
						tmpUpperLimit = 180;
						tmpLowerLimit = 0;
						tmpInterval = 450;
						break;
					case EVENT_STOP:
						return;
						break;
					default:
						return;
						break;
				}
				Continuous_Add_Minus_Advanced_settings(tmpParameter, tmpUpperLimit, tmpLowerLimit, tmpInterval, &mode, 0);
				Single_Add_Minus_Advanced_settings(tmpParameter, tmpUpperLimit, tmpLowerLimit, tmpInterval/10, &mode, 0);
				Broadcast_To_PeripheralDevice(tmpBuffer_element, tmpParameter, tmpIndex, 0, 0);
				break;
			default:
				break;
		}
	}
}

void Update_Up_Down_Btn_count(void)
{
	upBtnStatus.lastBtnStatus = upBtnStatus.newBtnStatus;
	downBtnStatus.lastBtnStatus = downBtnStatus.newBtnStatus;
	if(!upBtnStatus.newBtnStatus)
		upBtnStatus.count = new_tick;
	if(!downBtnStatus.newBtnStatus)
		downBtnStatus.count = new_tick;
}
