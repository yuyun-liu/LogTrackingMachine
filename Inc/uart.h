/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void USART1_START_RX(void);
void USART3_START_RX(void);
void Transmit_Msg(UART_HandleTypeDef* huart, uint8_t* msg, uint8_t len);
void Read_MotorDriver(uint8_t address, uint16_t parameter, uint16_t length);
void Write_MotorDriver(uint8_t address, uint16_t parameter, uint16_t data);

void ChangeInchPerSecond();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
