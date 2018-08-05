/**
  ******************************************************************************
  * @file    	GPIO/JTAG_Remap/main.h
  * @author  	ARMVN Application Team
  * @version 	V1.0.0
  * @date    	01/30/2010
  * @brief   	Header file for main.c module.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 ARMVietNam</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include "stm32f10x.h"

void led_toggle(void);
void initialize(void);

#define IDBOARD 1
#define IDDOOR 2

/* Private define ------------------------------------------------------------*/  
#define PSTR(s) s  
#define USARTz    USART2 
#define TxBufferSize2  (countof(TxBuffer2) - 1)
#define TxBufferSize3   (countof(TxBuffer3) - 1)
#define RxBufferSize2   TxBufferSize3
#define RxBufferSize3   TxBufferSize2
/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus; 

typedef unsigned long	ULONG;
typedef unsigned long	DWORD;

/* Private variables ---------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  

/* For TIMER */
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
			
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(void);
void initUART2(void);
void I2C(void);
void Delay(uint32_t);

int  GetKey(void);
int  Send_int_uart(int);
void Send_string_uart(const char*);
void Send_char_uart(char); 
int counterChar(const char* logData);
void netAnalyzer(void);
void EXTI5_Config(void);
/* Private function prototypes -----------------------------------------------*/
void EXTI0_Config(void);
void EXTI9_5_Config(void);

unsigned char bufferRFID[30];
unsigned char MyID[5] = {0x82, 0x1f, 0x2f, 0x00, 0xb2};	//My card on my keys
	
	
void unless_loop(void);
 /*
*************************************************************************************************************************************
*							  						   		GLOBAL FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/




#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */

/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 ARMVietNam *****END OF FILE****/
