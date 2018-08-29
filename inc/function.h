#ifndef __FUNCTION_H
#define __FUNCTION_H
 
#ifdef __cplusplus
extern "C" {
#endif

	
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "RF24.h"
#include "delay.h"
#include "stm32f10x_wwdg.h"
	
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF
	
// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

typedef enum
{
	None = 0, // Not access
	Preview,	// Waiting in 
	Active		// On stream
} NotiStatus;
	
void InitBuzzer(void);
void BuzzerRight(void);
void BuzzerFalse(void);
	
void Timer_Init(void);	
void Initialize_RF24(void);
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);
void Simple_Receive_Init(void);
NotiStatus GetStatus(uint8_t data_recv[], uint8_t index);
NotiStatus GetStatus_INT(int value, uint8_t index);
void LedStatusOnOff(NotiStatus stt);
void led_toggle(void);

void Wwdg_Init(void);
	
#ifdef __cplusplus
}
#endif
 
#endif
