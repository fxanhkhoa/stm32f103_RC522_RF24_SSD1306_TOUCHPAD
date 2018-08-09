#include "function.h"
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

# define BUZZER_CNT_MAX 2000
# define BUZZER_MIN_PSC 0
# define BUZZER_MAX_PSC (BUZZER_CNT_MAX / 4)
#define MAX_DELAY 150

#define Si2 0x07D5
#define La 0x08E1
#define Sol 0x09E8
#define Fa 0x0B20
#define Mi 0x0BC8
#define Re 0x0D4A
#define Do 0x0EE4
#define Si 0x0F86


void InitBuzzer(void)
{
	GPIO_InitTypeDef GPIO;
	
	/********* RCC ***********/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	/********* PWM PIN ***********/
	GPIO.GPIO_Pin = GPIO_Pin_0;
	GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO);
	
	/********* PWM INIT ***********/
	TIM_TimeBaseInitTypeDef TIM_BASE;
	TIM_OCInitTypeDef OC_InitStructure;
	
	TIM_BASE.TIM_Prescaler = 4;
	TIM_BASE.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BASE.TIM_Period = BUZZER_CNT_MAX;
	TIM_BASE.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseInit(TIM2, &TIM_BASE);
	
	TIM_OCStructInit(&OC_InitStructure);
  OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	OC_InitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM2, &OC_InitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
	
	TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
	
}

/* CCR is for Duty Circle (Volume of note) 
	 ARR is for Frequence (Tone of note Do Re mi Fa sol La Si)
*/

void BuzzerRight(void)
{
	TIM2->CCR1 = 2000;
	TIM2->ARR = Si2;
	DelayMs(MAX_DELAY);
	TIM2->ARR = Do;
	DelayMs(MAX_DELAY);
	TIM2->ARR = Re;
	DelayMs(MAX_DELAY);
	TIM2->ARR = Mi;
	DelayMs(MAX_DELAY);
	TIM2->CCR1 = 0x0000;
//	for (uint16_t i = 0; i < 0xFFFF; i += 5000)
//	{
//		TIM2->CCR1 = i;
//		DelayMs(300);
//	}
}

void BuzzerFalse(void)
{
	TIM2->CCR1 = 2000;
	TIM2->ARR = Si;
	DelayMs(MAX_DELAY*2);
	TIM2->CCR1 = 0x0000;
}

void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure; 
		timerInitStructure.TIM_Prescaler = 36000;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 2-1;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &timerInitStructure);
		TIM_Cmd(TIM4, ENABLE);
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
		/*----- NVIC Timer interrupt -----*/
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
}

nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
//	UART_SendStr("[");
//	UART_SendHex8(status);
//	UART_SendStr("] ");

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

void Simple_Receive_Init(void)
{
	// This is simple receiver with one RX pipe:
	//   - pipe#1 address: '0xE7 0x1C 0xE3'
	//   - payload: 5 bytes
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(10);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_1Mbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(5);

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { 0xE8, 0xE8, 0xF0 , 0xF0, 0xE1};
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32); // Auto-ACK: enabled, payload length: 3 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();
}

void Initialize_RF24(void)
{
	GPIO_InitTypeDef PORT;
  SPI_InitTypeDef SPI;

	
    // Enable SPI1 peripheral
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO,ENABLE);

    // Enable SPI1 GPIO peripheral (PORTB)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);


  // Configure nRF24 IRQ pin
	PORT.GPIO_Mode  = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin   = nRF24_IRQ_PIN;
	GPIO_Init(nRF24_IRQ_PORT, &PORT);

	// Configure SPI pins (SPI1)
    PORT.GPIO_Mode  = GPIO_Mode_AF_PP;
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    PORT.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &PORT);


    // Initialize SPI1
    SPI.SPI_Mode = SPI_Mode_Master;
    SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI.SPI_CPOL = SPI_CPOL_Low;
    SPI.SPI_CPHA = SPI_CPHA_1Edge;
    SPI.SPI_CRCPolynomial = 7;
    SPI.SPI_DataSize = SPI_DataSize_8b;
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(nRF24_SPI_PORT, &SPI);
    SPI_NSSInternalSoftwareConfig(nRF24_SPI_PORT, SPI_NSSInternalSoft_Set);
    SPI_Cmd(nRF24_SPI_PORT, ENABLE);
		
		// Initialize the nRF24L01 GPIO pins
    nRF24_GPIO_Init();

    // RX/TX disabled
    nRF24_CE_L();

    // Configure the nRF24L01+
    //UART_SendStr("nRF24L01+ check: ");
    if (!nRF24_Check()) {
    	//UART_SendStr("FAIL\r\n");
    	while (1);
    }
	//UART_SendStr("OK\r\n");

  // Initialize the nRF24L01 to its default state
  nRF24_Init();
		
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	// Configure GPIO for LED (PC14, PC15, PA12, PA0, PA1)
//	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12;
//	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
//	PORT.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &PORT);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	PORT.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
//	PORT.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &PORT);
//		
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	PORT.GPIO_Pin = GPIO_Pin_1;
//	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
//	PORT.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &PORT);

}

NotiStatus GetStatus(uint8_t data_recv[], uint8_t index)
{
	uint8_t result = 0x00;
	//UART_SendInt(data_recv[0]);
	//if (int(data_recv[0]) == 263) data_recv[0] = 129;
	//else if (data_recv[0] == ) data_recv[0] = 130;
	//else if (data_recv[0] == 272) data_recv[0] = 132;
	result |= (data_recv[0] >> (index - 1)) & (0x01);
	result |= (data_recv[0] >> (index + 2)) & (0x02);
	if (result == 0x01) return Active;
	else if (result == 0x02) return Preview;
	else if (result == 0x00) return None;
	return None;
}

NotiStatus GetStatus_INT(int value, uint8_t index)
{
	uint8_t result = 0x00;
	//UART_SendInt(value);
	
	result |= (value >> (index - 1)) & (0x01);
	result |= (value >> (index + 2)) & (0x02);
	//UART_SendInt(result);
	if (result == 0x01) return Active;
	else if (result == 0x02) return Preview;
	else if (result == 0x00) return None;
	return None;
}

void LedStatusOnOff(NotiStatus stt)
{
	// Led Red = GPIO Pin 12
	// Led Yellow = GPIO Pin 1
	// Led Green = GPIO Pin 0
//	GPIO_SetBits(GPIOA, LED_ACTIVE | LED_NONE | LED_PREVIEW);
//	if (stt == Active) GPIO_ResetBits(LED_PORT, LED_ACTIVE);
//	else if (stt == Preview) GPIO_ResetBits(LED_PORT, LED_PREVIEW);
//	else if (stt == None) GPIO_ResetBits(LED_PORT, LED_NONE);
}

