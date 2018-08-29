#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


#include "delay.h"
#include <stdio.h>
#include "main.h"
#include "RFID.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_i2c.h"
#include "ttp229.h"
#include "fonts.h"
#include "function.h"
#include "DataProtocol.h"
#include <stdlib.h>
#include <stdint.h>
//#include "u8g_font_fub11.c"
//#include "cortexm/ExceptionHandlers.h"

#define LEDPORTCLK          RCC_APB2Periph_GPIOC
#define LEDPORT             GPIOC
#define LEDB                GPIO_Pin_13


#define BUFSIZE 25
#define IDLE_WAIT_MS 1000


#define BUTTONPORTSOURCE    GPIO_PortSourceGPIOA
#define BUTTONPINSOURCE     GPIO_PinSource0

/* Private function prototypes -----------------------------------------------*/
  #ifdef __GNUC__
    /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
       set to 'Yes') calls __io_putchar() */
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
  #endif /* __GNUC__ */


void DelayInit(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);
		
uint8_t CardID[5];
DataBlock dB;
uint32_t timer = 0;
TTP229_KEY touchKey;
char key[2];
		
//----------- RF24 Variables -------------
// Pipe number
nRF24_RXResult pipe;
// Buffer to read
uint8_t nRF24_payload[32];
// Length of received payload
uint8_t payload_length;


int main(int argc, char* argv[])
{
	char i;
	uint8_t *buffer = malloc(sizeof(uint8_t) * 12);
	char temperature[2] = {'9','9'}, humidity[2] = {'9','9'}, pressure[2] = {'9','9'};
	
	InitBuzzer();
	initialize();
	//TIM_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	DelayInit();
	Timer_Init();
	SSD1306_Init();
	
	BuzzerRight();
	
	SSD1306_UpdateScreen();
	
	SSD1306_GotoXY(10,10);
	SSD1306_Puts("Initialized", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	DelayMs(500);
	
	Initialize_RF24(); // !IMPORTANT, with SPI1, remember to set baud prescaler is 8 for lower speed, so it can read true data
	
	//----RFID-----
	TM_MFRC522_Init();  
	DelayMs(300);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(10,10);
	SSD1306_Puts("RC522", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	DelayMs(500);
	
	//----- TTP229 ------
	TTP229_Init(KEYS_16_ACTIVE_LOW);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(10,10);
	SSD1306_Puts("TTP229", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	DelayMs(500);
	
	//--------- RF24---------
	Simple_Receive_Init();
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(10,10);
	SSD1306_Puts("RF24", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	DelayMs(500);
	
	led_toggle();
	printf("Started Application \n");
	
	//------wwdg init -----
	Wwdg_Init();
	
	// Init timer = 0
	timer = 0;
	
	while (1)
	{
		//----------------------------- Update Environment Data ---------------------------------------------
		if (timer > 10000)
		{
			timer = 0;
			// Init RF24 again
			//--------- RF24---------
			Simple_Receive_Init();
			SSD1306_Fill(SSD1306_COLOR_BLACK);
			SSD1306_GotoXY(10,10);
			SSD1306_Puts("RF24", &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			DelayMs(500);
		}
		//-----------------------------Print OLED ---------------------------------------------
		//SSD1306_UpdateScreen();
		//-----------------------------TTP229--------------------------------------------------
		touchKey = TTP229_GetKey();
		if (touchKey != KEY_NONE)
		{
			//printf("%d", touchKey);
			sprintf(key, "%d", touchKey);
			SSD1306_Fill(SSD1306_COLOR_BLACK);
			SSD1306_GotoXY(10,10);
			SSD1306_Puts(key, &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			DelayMs(100);
		}
		
		//-----------------------------RF24 READ------------------------------------------
		if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) 
		{
			timer = 0;
			led_toggle();
			pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

    	// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();
			
			//Convert temperature & humidity & pressure
			sprintf(temperature, "%d", nRF24_payload[0]);
			sprintf(humidity, "%d", nRF24_payload[1]);
			sprintf(pressure, "%d", nRF24_payload[2]);
		}
		
		//-----------------------------RFID Analayzer------------------------------------------
		if (TM_MFRC522_Check(dB.RFID) == MI_OK) 
		{
			led_toggle();
			dB.IDboard = IDBOARD;
			dB.command = IDCHECK;
			dB.door = IDDOOR;
			buffer = BuildBuffer(dB);
			for (i = 0; i < 12; i++)
			{
				printf("%c", buffer[i]); 
			}
			//printf("%02x %02x %02x %02x",CardID[0],CardID[1],CardID[2],CardID[3]);
			SSD1306_Fill(SSD1306_COLOR_BLACK);
			SSD1306_GotoXY(10,10);
			SSD1306_Puts("Hello", &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			DelayMs(100);
			BuzzerRight();
			//BuzzerFalse();
		}
		else 
		{
			SSD1306_Fill(SSD1306_COLOR_BLACK);
			
			SSD1306_GotoXY(5,5);
			SSD1306_Puts("Temper", &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(75,5);
			SSD1306_Puts(temperature, &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(100,5);
			SSD1306_Puts("C", &Font_11x18, SSD1306_COLOR_WHITE);
			
			SSD1306_GotoXY(5,25);
			SSD1306_Puts("Humi", &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(53,25);
			SSD1306_Puts(humidity, &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(80,25);
			SSD1306_Puts("%", &Font_11x18, SSD1306_COLOR_WHITE);
			
			SSD1306_GotoXY(5,45);
			SSD1306_Puts("Press", &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(65,45);
			SSD1306_Puts(pressure, &Font_11x18, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(90,45);
			SSD1306_Puts("atm", &Font_11x18, SSD1306_COLOR_WHITE);
			
			SSD1306_UpdateScreen();
		}
//		else if (TM_MFRC522_Check(CardID) == MI_NOTAGERR)
//		{
//			led_toggle();
//		}
//		else if (TM_MFRC522_Check(CardID) == MI_ERR)
//		{
//			led_toggle();
//		}
		
		DelayMs(100);
	}
}

void NVIC_Configuration(void)
{
  //NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART3 Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 // NVIC_Init(&NVIC_InitStructure);
    
  /*Enable the TIMER 2 Interrupt*/
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);  
  
}

void GPIO_Configuration(void)
{
	/* Enable GPIOA B C E and AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
   	/*------------------------------------------------------------*/
		/*--------------------------------------------------------*/   
		/* Configure  PC.3, PC.4, PC.5 as output push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Configure Buzzer  A0*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM_Configuration(void)
{

 	/*Config for Timer 2*/

	/* Time base configuration */

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* TIMER 2 enable */
 	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	
}

void initialize()
{
	GPIO_InitTypeDef GPIO;
	USART_InitTypeDef USART;
	
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
	initialize the PLL and update the SystemFrequency variable. */
	SystemInit();

	



	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
/* Set Led */
		GPIO.GPIO_Pin = GPIO_Pin_13;
	  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC, &GPIO);
		
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	
/*************************************************************************************
*																	USART_Init
**************************************************************************************/
			
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	USART.USART_BaudRate = 9600;
	USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART.USART_StopBits = USART_StopBits_1;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_Parity = USART_Parity_No;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		/*---- Configure USART1 ----*/
			USART_Init(USART1, &USART);
			//USART_Init(USART2, &USART);
    /*---- Enable RXNE interrupt ----*/
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		/*---- Enable TXE ----*/
			//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    /*---- Enable USART1 global interrupt ----*/
			//NVIC_EnableIRQ(USART1_IRQn);
		/*---- USART ENABLE ----*/
			USART_Cmd(USART1, ENABLE);
			//USART_Cmd(USART2, ENABLE);
		/*------ TX-Pin PA9 & RX-Pin PA10 -----*/
			
			GPIO.GPIO_Pin = GPIO_Pin_9;
			GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, & GPIO);
			
			GPIO.GPIO_Pin = GPIO_Pin_10;
			GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO);
		/*---- TX-Pin PA2 & RX-Pin PA3 ----*/
			GPIO.GPIO_Pin = GPIO_Pin_2;
			GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, & GPIO);
			
			GPIO.GPIO_Pin = GPIO_Pin_3;
			GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//GETCHAR_PROTOTYPE
//{
//  return GetUSART(USART1);
//}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

void USART1_IRQHandler(void)
{
    /* RXNE handler */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
			printf("%d",USART_ReceiveData(USART1));
    }
     
    /* ------------------------------------------------------------ */
    /* Other USART1 interrupts handler can go here ...             */
}

void TIM4_IRQHandler()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
				timer ++;
				//time_now++;
			if (timer == UINT32_MAX) timer = 0;
    }
}

void led_toggle(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
     
    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    /* If LED output clear, set it */
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        //led_toggle();
    }
}

void EXTI0_IRQHandler()
{
		if(EXTI_GetITStatus(EXTI_Line0) != RESET)//Judge whether a line break
		{
			EXTI_ClearITPendingBit(EXTI_Line0);   //Remove LINE interrupt flag bit
			led_toggle();
		} 	    
} 

void EXTI9_5_IRQHandler()
{
		if(EXTI_GetITStatus(EXTI_Line5) != RESET)//Judge whether a line break
		{
			EXTI_ClearITPendingBit(EXTI_Line5);   //Remove LINE interrupt flag bit
			led_toggle();
		} 	    
} 

void WWDG_IRQHandler(void) {
//	WWDG_ClearFlag(); //This function reset flag WWDG->SR and cancel the resetting
//	WWDG_SetCounter(100);

	/* Toggle LED which connected to PC13*/
    GPIOC->ODR ^= GPIO_Pin_13;
}
