#ifndef __MSG_QUEUE_H__
#define __MSG_QUEUE_H__

#include "stdint.h"
#include "main.h"

#ifdef __cplusplus
extern "C"{
#endif
typedef struct
{
	uint8_t AckMsg[MAX_BUFFER_SIZE];
	uint8_t AckMsg_Len;
	uint32_t timeout_resend;
	uint8_t timeout_times;
	uint8_t timeout_counts;
	uint32_t lastTimeTick;
	UART_HandleTypeDef* huart;
}ACK_STRUCT;

typedef struct msg_queue_element
{
	uint8_t msg_buff[MAX_BUFFER_SIZE];
	struct msg_queue_element* next;
}MSG_QUEUE_ELEMENTS;

typedef struct msg_queue_len
{
	uint8_t msg_len;
	struct msg_queue_len* next;
}MSG_QUEUE_LEN;
typedef struct
{
	uint8_t buffer[MAX_BUFFER_SIZE];
	_Bool hasReceived;
}MSG_RX_BUFFER;

extern uint8_t remain_tx;
extern const uint8_t MAX_QUEUE_BUFFER_SIZE;

extern ACK_STRUCT dev_Ack;

extern const uint8_t HMI_TX_QID;
extern const uint8_t HMI_RX_QID;
extern const uint8_t PC_TX_QID;
extern const uint8_t PC_RX_QID;
extern const uint8_t DEV_TX_QID;
extern const uint8_t DEV_RX_QID;

void MSG_InitializeQeueue();
void msg_UpdateRXQ(uint8_t* msg);
MSG_RX_BUFFER msg_GetRXQData();
uint8_t* msg_AllocateQlen(void);
uint8_t* msg_AllocateQ(void);
void msg_FreeQ(uint8_t* buff);
void msg_Enqueue(uint8_t qid, uint8_t *msgBuff, uint8_t len);
uint8_t* msg_Dequeue(uint8_t qid);
uint8_t msgLen_Dequeue(uint8_t qid);
#ifdef __cplusplus
}
#endif
#endif