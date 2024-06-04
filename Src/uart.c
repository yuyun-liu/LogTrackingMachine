/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "msg_queue.h"
#include "string.h"

/* USER CODE BEGIN 0 */
uint8_t* buffer_MD = NULL;	//Motor Driver Message Buffer
uint8_t data_length_MD = 0;
uint8_t CRC_LEN_MD = 2;

uint8_t* hmi_buftx = NULL;
static uint8_t hmi_bufrx[MAX_BUFFER_SIZE];
uint8_t* md_buftx = NULL;
static uint8_t md_bufrx[MAX_BUFFER_SIZE];

uint32_t MD_Address = 0;

uint32_t uart1_transmitting_state = HAL_UART_STATE_READY;
uint32_t uart1_state = 0;
uint32_t uart3_transmitting_state = HAL_UART_STATE_READY;
uint32_t uart3_state = 0;

_Bool uart1_offline = 0;
/* USER CODE END 0 */

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* USART1 init function */

/* USER CODE BEGIN 1 */
void USART1_START_RX(void)
{
	HAL_UART_Receive_DMA(&huart1, &hmi_bufrx[0], 1);
}

void USART3_START_RX(void)
{
	HAL_UART_Receive_DMA(&huart3, &md_bufrx[0], 1);
}

void Transmit_Msg(UART_HandleTypeDef* huart, uint8_t* msg, uint8_t len)
{
	if(huart->Instance == USART1)
	{
		if(uart1_transmitting_state == HAL_UART_STATE_READY)
		{
			hmi_buftx = msg;
			//uart1_transmitting_state = HAL_UART_STATE_BUSY;
			HAL_UART_Transmit(huart, msg, len, 100);
		}
		else
		{
			msg_Enqueue(HMI_TX_QID, msg, len);
		}
	}
	if(huart->Instance == USART3)
	{
		if(uart3_transmitting_state == HAL_UART_STATE_READY)
		{
			hmi_buftx = msg;
			//uart3_transmitting_state = HAL_UART_STATE_BUSY;
			HAL_UART_Transmit(huart, msg, len, 100);
		}
		else
		{
			msg_Enqueue(MD_TX_QID, msg, len);
		}
	}
}

/*----------Motor Driver----------*/
uint16_t createCRC_MD(uint8_t *data, uint8_t len)
{
	uint8_t j;
		uint16_t crc = 0xFFFF;

		for (int i = 0; i < len; i++)
		{
				crc ^= data[i];
				for (j = 0; j < 8; j++)
				{
						if ((crc & 0x01) > 0)
								crc = (uint16_t)((crc >> 1) ^ 0xA001);
						else
								crc = (uint16_t)((crc >> 1));
				}
		}

		return crc;
}
void Read_MotorDriver(uint8_t address, uint16_t parameter, uint16_t length)
{
	if(huart3.ErrorCode != 0)
		return;
	
	buffer_MD = msg_AllocateQ();
	
	_Bool isRead = 1;
	
	uint8_t function_code = 0x00;
	if(isRead)
		function_code = 0x03;
	else
		function_code = 0x06;
	
	buffer_MD[0] = address;
	buffer_MD[1] = function_code;
	buffer_MD[2] = ((parameter>>8)&0xFF);
	buffer_MD[3] = (parameter&0xFF);
	buffer_MD[4] = ((length>>8)&0xFF);
	buffer_MD[5] = (length&0xFF);
	
	data_length_MD = 8;
	
	uint16_t crc = createCRC_MD(buffer_MD, data_length_MD - CRC_LEN_MD);
	
	buffer_MD[6] = crc&0xFF;
	buffer_MD[7] = (crc>>8)&0xFF;
	
	MD_Address = parameter;
	
	Transmit_Msg(&huart3, buffer_MD, data_length_MD);
}

void Write_MotorDriver(uint8_t address, uint16_t parameter, uint16_t data)
{
	if(huart3.ErrorCode != 0)
		return;
	
	msg_FreeQ(md_buftx);
	buffer_MD = msg_AllocateQ();
	
	_Bool isRead = 0;
	
	uint8_t function_code = 0x00;
	if(isRead)
		function_code = 0x03;
	else
		function_code = 0x06;
	
	buffer_MD[0] = address;
	buffer_MD[1] = function_code;
	buffer_MD[2] = ((parameter>>8)&0xFF);
	buffer_MD[3] = (parameter&0xFF);
	buffer_MD[4] = ((data>>8)&0xFF);
	buffer_MD[5] = (data&0xFF);
	
	data_length_MD = 8;
	
	uint16_t crc = createCRC_MD(&buffer_MD[0], data_length_MD - CRC_LEN_MD);
	
	buffer_MD[6] = (uint8_t)(crc&0xFF);
	buffer_MD[7] = (uint8_t)((crc>>8)&0xFF);
	
	MD_Address = parameter;
	
	Transmit_Msg(&huart3, buffer_MD, data_length_MD);
}
/*----------Motor Driver----------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uint16_t crc = 0;
		
		switch(uart1_state)
		{
			case 0:
				if(hmi_bufrx[0] == 0x02)
				{
					uart1_state++;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[uart1_state], 1);
				}
				else
				{
					uart1_state = 0;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[0], 1);
				}
				break;
			case 1:
				if(hmi_bufrx[1] == 0x0F0)
				{
					uart1_state++;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[uart1_state], 1);
				}
				else
				{
					uart1_state = 0;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[0], 1);
				}
				break;
			case 2:
				uart1_state++;
				HAL_UART_Receive_DMA(huart, &hmi_bufrx[uart1_state], 1);
				break;
			case 3:
				if((hmi_bufrx[3] == 8) && (hmi_bufrx[3] < MAX_BUFFER_SIZE))
				{
					uart1_state++;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[uart1_state], hmi_bufrx[3] - MSG_HEAD_SIZE);
				}
				else
				{
					uart1_state = 0;
					HAL_UART_Receive_DMA(huart, &hmi_bufrx[0], 1);
				}
				break;
			default:
				msg_UpdateRXQ(hmi_bufrx);
				uart1_state = 0;
				HAL_UART_Receive_DMA(huart, &hmi_bufrx[uart1_state], 1);
				break;
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_AbortTransmit(huart);
	uint8_t* msgBuff;
	uint8_t msgLen;
	
	if(huart->Instance == USART1)
	{
		msg_FreeQ(hmi_buftx);
		uart1_transmitting_state = HAL_UART_STATE_READY;
		
		msgBuff = msg_Dequeue(HMI_TX_QID);
		msgLen = msgLen_Dequeue(HMI_TX_QID);
		
		if(msgBuff != NULL)
		{
			hmi_buftx = msgBuff;
			uart1_transmitting_state = HAL_UART_STATE_BUSY;
			HAL_UART_Transmit_DMA(huart, msgBuff, msgLen);
		}
		else
		{
			hmi_buftx = NULL;
		}
		return;
	}
	if(huart->Instance == USART3)
	{
		msg_FreeQ(hmi_buftx);
		uart3_transmitting_state = HAL_UART_STATE_READY;
		
		msgBuff = msg_Dequeue(HMI_TX_QID);
		msgLen = msgLen_Dequeue(HMI_TX_QID);
		
		if(msgBuff != NULL)
		{
			hmi_buftx = msgBuff;
			uart3_transmitting_state = HAL_UART_STATE_BUSY;
			HAL_UART_Transmit_DMA(huart, msgBuff, msgLen);
		}
		else
		{
			hmi_buftx = NULL;
		}
		return;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	/*---------Reset Flags-------------*/
	//md_read_pulse_flag = 0;
	
	/*---------Reset UART--------------*/
	HAL_UART_DeInit(huart);
	
	if(huart->Instance == USART1)
	{
		//MX_USART1_UART_Init();
		USART1_START_RX();
	}
	
	if(huart->Instance == USART3)
	{
		//MX_USART3_UART_Init();
		USART3_START_RX();
	}
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
