#include "msg_queue.h"
#include "string.h"


static MSG_QUEUE_ELEMENTS* hmi_txMsgQ;
static MSG_QUEUE_ELEMENTS* hmi_rxMsgQ;
static MSG_QUEUE_LEN* hmi_txLen;
static MSG_QUEUE_LEN* hmi_rxLen;

static MSG_QUEUE_ELEMENTS* dev_txMsgQ;
static MSG_QUEUE_ELEMENTS* dev_rxMsgQ;
static MSG_QUEUE_LEN* dev_txLen;
static MSG_QUEUE_LEN* dev_rxLen;

ACK_STRUCT dev_Ack;
static MSG_QUEUE_ELEMENTS* pc_txMsgQ;
static MSG_QUEUE_ELEMENTS* pc_rxMsgQ;
static MSG_QUEUE_LEN* md_txLen;
static MSG_QUEUE_LEN* md_rxLen;

const uint8_t HMI_TX_QID = 1;
const uint8_t HMI_RX_QID = 2;
const uint8_t PC_TX_QID = 3;
const uint8_t PC_RX_QID = 4;
const uint8_t DEV_TX_QID = 5;
const uint8_t DEV_RX_QID = 6;

const uint8_t MAX_QUEUE_BUFFER_SIZE = MAX_BUFFER_SIZE;
uint8_t bufferOccupied = MAX_QUEUE_BUFFER_SIZE;
uint8_t rx_bufferOccupied = 0;
uint8_t remain_tx = 0;

static uint8_t buffer_occupied[MAX_QUEUE_BUFFER_SIZE];
static MSG_QUEUE_ELEMENTS buffer_memory[MAX_QUEUE_BUFFER_SIZE];
static uint8_t buffer_len[MAX_QUEUE_BUFFER_SIZE];
static MSG_RX_BUFFER rx_buffer[MAX_QUEUE_BUFFER_SIZE];

void MSG_InitializeQeueue()
{
	hmi_rxMsgQ = NULL;
	hmi_txMsgQ = NULL;
	hmi_rxLen = NULL;
	hmi_txLen = NULL;
	dev_rxMsgQ = NULL;
	dev_txMsgQ = NULL;
	dev_rxLen = NULL;
	dev_txLen = NULL;
	memset(buffer_len, 0x00, MAX_QUEUE_BUFFER_SIZE);
	memset(buffer_occupied, 0x00, MAX_QUEUE_BUFFER_SIZE);
}
void msg_UpdateRXQ(uint8_t* msg)
{
	memset(rx_buffer[rx_bufferOccupied].buffer, 0x00, MAX_BUFFER_SIZE);
	memcpy(rx_buffer[rx_bufferOccupied].buffer, msg, MAX_BUFFER_SIZE);
	rx_buffer[rx_bufferOccupied].hasReceived = 1;
	rx_bufferOccupied++;
}
MSG_RX_BUFFER msg_GetRXQData()
{
	MSG_RX_BUFFER init;
	init.hasReceived = 0;
	if(rx_bufferOccupied == 0)
		return init;
	
	uint8_t i;
	MSG_RX_BUFFER returnValue = rx_buffer[0];
	for(i = 0; i < MAX_QUEUE_BUFFER_SIZE - 1; i++)
	{
		rx_buffer[i] = rx_buffer[i + 1];
	}
	rx_buffer[MAX_QUEUE_BUFFER_SIZE - 1] = init;
	rx_bufferOccupied--;
	return returnValue;
}
uint8_t* msg_AllocateQ(void)
{
	uint8_t i;
	
	for(i = 0; i < MAX_QUEUE_BUFFER_SIZE; i++)
	{
		if(buffer_occupied[i]==0)
		{
			buffer_occupied[i] = 1;
			break;
		}
	}
	if(i < MAX_QUEUE_BUFFER_SIZE)
	{
		bufferOccupied--;
		return &buffer_memory[i].msg_buff[0];
	}
	else
	{
		return NULL;
	}
}
void msg_FreeQ(uint8_t* buff)
{
	uint8_t i;
	
	for(i = 0; i < MAX_QUEUE_BUFFER_SIZE; i++)
	{
		if(buff == buffer_memory[i].msg_buff)
		{
			buffer_occupied[i] = 0;
			bufferOccupied++;
			break;
		}
	}
}
void msg_Enqueue(uint8_t qid, uint8_t *msgBuff, uint8_t len)
{
	MSG_QUEUE_ELEMENTS **qHead;
	MSG_QUEUE_ELEMENTS *ptr, *element;
	
	switch(qid)
	{
		case HMI_TX_QID:
			qHead = &hmi_txMsgQ;
			break;
		case HMI_RX_QID:
			qHead = &hmi_rxMsgQ;
			break;
		case PC_TX_QID:
			qHead = &pc_txMsgQ;
			break;
		case PC_RX_QID:
			qHead = &pc_rxMsgQ;
			break;
		case DEV_TX_QID:
			qHead = &dev_txMsgQ;
			break;
		case DEV_RX_QID:
			qHead = &dev_rxMsgQ;
			break;
		default:
			break;
	}
	element = (MSG_QUEUE_ELEMENTS*)msgBuff;
	element->next = NULL;
	
	ptr = *qHead;	//Get 2D array heads
	
	if(ptr == NULL)	//2D array is null
	{
		*qHead = element;
		if(qid != PC_TX_QID && qid != PC_RX_QID)
			buffer_len[0] = len;
	}
	else	//2D array is not null and enqueue element.
	{
		uint8_t i = 1;
		while(ptr->next != NULL)
		{
			ptr = ptr->next;
			i++;
		}
		ptr->next = element;
		if(qid != PC_TX_QID && qid != PC_RX_QID)
			buffer_len[i] = len;
		remain_tx = i;
	}
}
uint8_t* msg_Dequeue(uint8_t qid)
{
	MSG_QUEUE_ELEMENTS *ptr;
	
	switch(qid)
	{
		case HMI_TX_QID:
			ptr = hmi_txMsgQ;
			if(ptr != NULL)
				hmi_txMsgQ = ptr->next;
			break;
		case HMI_RX_QID:
			ptr = hmi_rxMsgQ;
			if(ptr != NULL)
				hmi_rxMsgQ = ptr->next;
			break;
		case PC_TX_QID:
			ptr = pc_txMsgQ;
			if(ptr != NULL)
				pc_txMsgQ = ptr->next;
			break;
		case PC_RX_QID:
			ptr = pc_rxMsgQ;
			if(ptr != NULL)
				pc_rxMsgQ = ptr->next;
			break;
		case DEV_TX_QID:
			ptr = dev_txMsgQ;
			if(ptr!=NULL)
				dev_txMsgQ = ptr->next;
			break;
		case DEV_RX_QID:
			ptr = dev_rxMsgQ;
			if(ptr!=NULL)
				dev_rxMsgQ = ptr->next;
			break;
		default:
			ptr = NULL;
			break;
	}
	
	return (uint8_t*)ptr;
}
uint8_t msgLen_Dequeue(uint8_t qid)
{
	uint8_t returnLength = buffer_len[0];
	switch(qid)
	{
		case HMI_TX_QID:
			for(int i = 0; i < MAX_QUEUE_BUFFER_SIZE - 1; i++)
			{
				buffer_len[i] = buffer_len[i+1];
			}
			buffer_len[MAX_QUEUE_BUFFER_SIZE - 1] = 0;
			break;
		case HMI_RX_QID:
			break;
		case DEV_TX_QID:
			for(int i = 0; i < MAX_QUEUE_BUFFER_SIZE - 1; i++)
			{
				buffer_len[i] = buffer_len[i+1];
			}
			buffer_len[MAX_QUEUE_BUFFER_SIZE - 1] = 0;
			break;
		case DEV_RX_QID:
			break;
		default:
			//ptr_len = NULL;
			break;
	}
	return returnLength;
}