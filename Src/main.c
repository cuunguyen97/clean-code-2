/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2021 Lumi
 * Copyright (c) 2021
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: main.c
 *
 * Description: Input interrupt used to read button state.
 *
 * Author: CuuNV
 *
 * Last Changed By:  $Author: CuuNV $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $January 9 2023
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <timer.h>
#include <misc.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_exti.h>
#include <stm32f401re_syscfg.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define OUTPUT_PIN_COUNT				7
//Led Red: 1.PA1 2.PB13
#define LED_RED_2_GPIO_PIN				GPIO_Pin_1
#define LED_RED_2_GPIO_PORT				GPIOA
#define	LED_RED_2_INIT_SETCLOCK			RCC_AHB1Periph_GPIOA
#define LED_RED_2_PIN					1
#define LED_RED_2_INIT					{LED_RED_2_GPIO_PIN,LED_RED_2_GPIO_PORT,LED_RED_2_INIT_SETCLOCK,LED_RED_2_PIN}

#define LED_RED_1_GPIO_PIN				GPIO_Pin_13
#define LED_RED_1_GPIO_PORT				GPIOB
#define	LED_RED_1_INIT_SETCLOCK			RCC_AHB1Periph_GPIOB
#define LED_RED_1_PIN					13
#define LED_RED_1_INIT					{LED_RED_1_GPIO_PIN,LED_RED_1_GPIO_PORT,LED_RED_1_INIT_SETCLOCK,LED_RED_1_PIN}
//Led Blue: 1.PA3	2.PA10
#define LED_BLUE_1_GPIO_PIN				GPIO_Pin_10
#define LED_BLUE_1_GPIO_PORT			GPIOA
#define	LED_BLUE_1_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_BLUE_1_PIN					10
#define LED_BLUE_1_INIT					{LED_BLUE_1_GPIO_PIN,LED_BLUE_1_GPIO_PORT,LED_BLUE_1_INIT_SETCLOCK,LED_BLUE_1_PIN}
//Led Green: 1.PA0	2.PA11
#define LED_GREEN_2_GPIO_PIN			GPIO_Pin_0
#define LED_GREEN_2_GPIO_PORT			GPIOA
#define	LED_GREEN_2_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_GREEN_2_PIN					0
#define LED_GREEN_2_INIT				{LED_GREEN_2_GPIO_PIN,LED_GREEN_2_GPIO_PORT,LED_GREEN_2_INIT_SETCLOCK,LED_GREEN_2_PIN}

#define LED_GREEN_1_GPIO_PIN			GPIO_Pin_11
#define LED_GREEN_1_GPIO_PORT			GPIOA
#define	LED_GREEN_1_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_GREEN_1_PIN					11
#define LED_GREEN_1_INIT				{LED_GREEN_1_GPIO_PIN,LED_GREEN_1_GPIO_PORT,LED_GREEN_1_INIT_SETCLOCK,LED_GREEN_1_PIN}
//Led Board: PA5
#define LED_BOARD_GPIO_PIN				GPIO_Pin_5
#define LED_BOARD_GPIO_PORT				GPIOA
#define	LED_BOARD_INIT_SETCLOCK			RCC_AHB1Periph_GPIOA
#define LED_BOARD_PIN					5
#define LED_BOARD_INIT					{LED_BOARD_GPIO_PIN,LED_BOARD_GPIO_PORT,LED_BOARD_INIT_SETCLOCK,LED_BOARD_PIN}
// define id_led
#define LED_RED_1						0
#define LED_GREEN_1						1
#define LED_BLUE_1						2
#define LED_RED_2						3
#define LED_GREEN_2						4
#define LED_BOARD						5
//Buzzer: PC9
#define BUZZER_GPIO_PIN					GPIO_Pin_9
#define BUZZER_GPIO_PORT				GPIOC
#define	BUZZER_INIT_SETCLOCK			RCC_AHB1Periph_GPIOC
#define BUZZER_PIN						9
#define BUZZER_INIT						{BUZZER_GPIO_PIN,BUZZER_GPIO_PORT,BUZZER_INIT_SETCLOCK,BUZZER_PIN}
//Define cac nut B2,B3,B4----------------------------------------------------------------/
#define BUTTON_COUNT					3
//Button B2:PB3
#define BUTTON_B2_GPIO_PIN				GPIO_Pin_3
#define BUTTON_B2_GPIO_PORT				GPIOB
#define BUTTON_B2_INIT_SETCLOCK 		RCC_AHB1Periph_GPIOB

//Cac define ve ngat cua B2
#define BUTTON_B2_EXTI_PORT_SOURCEGPIOX	EXTI_PortSourceGPIOB
#define BUTTON_B2_EXTI_PINSOURCEX		EXTI_PinSource3
#define BUTTON_B2_EXTI_LINE				EXTI_Line3
#define BUTTON_B2_NVIC_IRQCHANNEL 		EXTI3_IRQn
#define BUTTON_B2_EXTI_TRIGGER_TYPE_DEF EXTI_Trigger_Rising_Falling

#define BUTTON_B2_INIT					{BUTTON_B2_GPIO_PIN,BUTTON_B2_GPIO_PORT,BUTTON_B2_INIT_SETCLOCK,BUTTON_B2_EXTI_PORT_SOURCEGPIOX,BUTTON_B2_EXTI_PINSOURCEX,BUTTON_B2_EXTI_LINE,BUTTON_B2_NVIC_IRQCHANNEL,BUTTON_B2_EXTI_TRIGGER_TYPE_DEF}

//Button B3:PA4
#define BUTTON_B3_GPIO_PIN				GPIO_Pin_4
#define BUTTON_B3_GPIO_PORT				GPIOA
#define BUTTON_B3_INIT_SETCLOCK 		RCC_AHB1Periph_GPIOA

//Cac define ve ngat cua B3
#define BUTTON_B3_EXTI_PORT_SOURCEGPIOX	EXTI_PortSourceGPIOA
#define BUTTON_B3_EXTI_PINSOURCEX		EXTI_PinSource4
#define BUTTON_B3_EXTI_LINE				EXTI_Line4
#define BUTTON_B3_NVIC_IRQCHANNEL 		EXTI4_IRQn
#define BUTTON_B3_EXTI_TRIGGER_TYPE_DEF EXTI_Trigger_Rising_Falling

#define BUTTON_B3_INIT					{BUTTON_B3_GPIO_PIN,BUTTON_B3_GPIO_PORT,BUTTON_B3_INIT_SETCLOCK,BUTTON_B3_EXTI_PORT_SOURCEGPIOX,BUTTON_B3_EXTI_PINSOURCEX,BUTTON_B3_EXTI_LINE,BUTTON_B3_NVIC_IRQCHANNEL,BUTTON_B3_EXTI_TRIGGER_TYPE_DEF}
//Button B4:PB0
#define BUTTON_B4_GPIO_PIN				GPIO_Pin_0
#define BUTTON_B4_GPIO_PORT				GPIOB
#define BUTTON_B4_INIT_SETCLOCK 		RCC_AHB1Periph_GPIOB

//Cac define ve ngat cua B4
#define BUTTON_B4_EXTI_PORT_SOURCEGPIOX	EXTI_PortSourceGPIOB
#define BUTTON_B4_EXTI_PINSOURCEX		EXTI_PinSource0
#define BUTTON_B4_EXTI_LINE				EXTI_Line0
#define BUTTON_B4_NVIC_IRQCHANNEL 		EXTI0_IRQn
#define BUTTON_B4_EXTI_TRIGGER_TYPE_DEF EXTI_Trigger_Rising_Falling

#define BUTTON_B4_INIT					{BUTTON_B4_GPIO_PIN,BUTTON_B4_GPIO_PORT,BUTTON_B4_INIT_SETCLOCK,BUTTON_B4_EXTI_PORT_SOURCEGPIOX,BUTTON_B4_EXTI_PINSOURCEX,BUTTON_B4_EXTI_LINE,BUTTON_B4_NVIC_IRQCHANNEL,BUTTON_B4_EXTI_TRIGGER_TYPE_DEF}

//define SYSCFG clock
#define SYSCFGINIT_SETCLOCK				RCC_APB2Periph_SYSCFG

//define other--------------------------------------------------------------------------------//
#define GPIO_PIN_SET 					1
#define GPIO_PIN_RESET 					0

#define BUTTON_DEBOUNCE_TIME      		5u
#define BUTTON_NORMAL_PRESS_TIME  		100u
#define BUTTON_LONG_PRESS_TIME    		500u
#define BW2PRESS_TIME		            400u

/* Button status */
#define BUTTON_EDGE_RISING				1
#define BUTTON_EDGE_FALLING            	2
#define Null							0

/* Button states */
#define BUTTON_STATE_START        		0
#define BUTTON_STATE_DEBOUNCE     		1
#define BUTTON_STATE_PRESSED      		2
#define BUTTON_STATE_WAITPRESS			3
#define BUTTON_STATE_WAITRELEASE  		4
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/**
 * @brief  Event of Button types
 */
typedef enum{
	 EVENT_OF_BUTTON_PRESS_1_TIMES,
	 EVENT_OF_BUTTON_PRESS_2_TIMES,
	 EVENT_OF_BUTTON_PRESS_5_TIMES,
	 EVENT_OF_BUTTON_HOLD_500MS,
	 EVENT_OF_BUTTON_RELEASED,
	 EVENT_OF_BUTTON_NOCLICK
}EventButton_e;
/**
 * @brief  Button structure
 */
typedef struct
{
	EventButton_e buttonEven ;
	uint8_t pressCnt ;												/*!< Time when button was pressed */
	uint8_t pressCntEnd;
	uint32_t timeInit;

	uint32_t timeCurrent;
	uint32_t timePress;
	uint32_t timeOut;
	uint8_t State;														/*!< Current button state */
	uint8_t Status;
}Button_t;

Button_t BtnProperty[BUTTON_COUNT];

typedef struct
{
	uint16_t pin;
	GPIO_TypeDef *port;
	uint32_t clock;
	uint8_t extiPort;
	uint8_t extiPin;
	uint32_t extiLine;
	uint8_t irqChannel;
	EXTITrigger_TypeDef EXTI_Trigger;
}BtnInit_t;
BtnInit_t BtnArray[BUTTON_COUNT]={BUTTON_B2_INIT,BUTTON_B3_INIT,BUTTON_B4_INIT};

typedef struct
{
	uint16_t pin;
	GPIO_TypeDef *port;
	uint32_t clock;
	uint8_t pinnumber;
}OutputInit_t;

OutputInit_t outputPinArray[OUTPUT_PIN_COUNT] = {LED_RED_2_INIT, LED_RED_1_INIT, LED_BLUE_1_INIT, LED_GREEN_2_INIT, LED_GREEN_1_INIT, LED_BOARD_INIT, BUZZER_INIT};
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static
void outputInit(void);

static
void inputInterruptInit(void);

static
void buttonInit(void);

static
void ledControlSetState(uint8_t byLedId,uint8_t byLedState);

void delayMs(uint32_t dwMs);

void blinkledStatusPower(void);

void buzzerControlSetBeep(void);

uint32_t dwCalculatorTime(uint32_t dwTimeInit,uint32_t dwTimeCurrent);

void scanButton(Button_t* pButtonStruct_t);

void processEventButton(void);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void)
{
	SystemCoreClockUpdate();
	outputInit();
	buttonInit();
	blinkledStatusPower();
	TimerInit();
    /* Loop forever */
	while(1)
		{
			for(int i = 0;i<BUTTON_COUNT; i++)
			{
				scanButton(&BtnProperty[i]);
			}
				processEventButton();
		}
}
/**
 * @func   outputInit
 * @brief  Ham setup dau ra
 * @param  SetClock: Xung cap, GPIO_Pin: Pin dau ra, GPIO_Port: Port dau ra
 * @retval None
 */
static
void outputInit(void)
{
	for(int i = 0; i< OUTPUT_PIN_COUNT; i++)
	{
		GPIO_InitTypeDef  GPIO_Intructure;

		RCC_AHB1PeriphClockCmd(outputPinArray[i].clock,ENABLE);

		GPIO_Intructure.GPIO_Pin  = outputPinArray[i].pin;
		GPIO_Intructure.GPIO_Mode = GPIO_Mode_OUT ;
		GPIO_Intructure.GPIO_Speed = GPIO_Fast_Speed;
		GPIO_Intructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Intructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(outputPinArray[i].port,&GPIO_Intructure);
	}

}
/**
 * @func   inputInterruptInit
 * @brief  Ham setup dau ra
 * @param  None
 * @retval None
 */
static
void inputInterruptInit(void)
{
	for(uint8_t i;i<BUTTON_COUNT;i++)
	{
			GPIO_InitTypeDef GPIO_Intructure;
			EXTI_InitTypeDef EXTI_InitStruct;
			NVIC_InitTypeDef NVIC_InitStruct;
			// Bat xung clock cho nut nhan
			RCC_AHB1PeriphClockCmd(BtnArray[i].clock,ENABLE);
			// Cau hinh nut bam
			GPIO_Intructure.GPIO_Pin = BtnArray[i].pin;
			GPIO_Intructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_Intructure.GPIO_Speed = GPIO_Fast_Speed;
			GPIO_Intructure.GPIO_PuPd =  GPIO_PuPd_UP;
			GPIO_Init(BtnArray[i].port,&GPIO_Intructure);
			// Bat xung clock cho ngat
			RCC_APB2PeriphClockCmd(SYSCFGINIT_SETCLOCK, ENABLE);
			SYSCFG_EXTILineConfig(BtnArray[i].extiPort, BtnArray[i].extiPin);
			// Cau hinh EXTI
			EXTI_InitStruct.EXTI_Line = BtnArray[i].extiLine;
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = BtnArray[i].EXTI_Trigger;
			EXTI_InitStruct.EXTI_LineCmd = ENABLE ;
			EXTI_Init(&EXTI_InitStruct);
			// Cau hinh trinh phuc vu ngat
			NVIC_InitStruct.NVIC_IRQChannel = BtnArray[i].irqChannel;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

			NVIC_Init(&NVIC_InitStruct);
	}

}

/**
 * @func   buttonInit
 * @brief
 * @param  None
 * @retval None
 */
static
void buttonInit(void)
{
	for(int i = 0; i<BUTTON_COUNT; i++)
	{
		BtnProperty[i].State = BUTTON_STATE_START;
	}

	inputInterruptInit();
}
/**
 * @func   ledControlSetState
 * @brief  This function will control led state when it is called
 * @param
 * byLedId:
 * byLedState: ENABLE/DISABLE
 * @retval None
 */
static
void ledControlSetState(uint8_t byLedId,uint8_t byLedState)
{
	if(byLedState == ENABLE)
	{
		if(byLedId == LED_RED_1)
		{
			(LED_RED_1_GPIO_PORT->BSRRL) |= 1<<LED_RED_1_PIN;
		}else if(byLedId == LED_BLUE_1)
		{
			(LED_BLUE_1_GPIO_PORT->BSRRL) |= 1<<LED_BLUE_1_PIN;
		}else if(byLedId == LED_GREEN_1)
		{
			(LED_GREEN_1_GPIO_PORT->BSRRL) |= 1<<LED_GREEN_1_PIN;
		}else if(byLedId == LED_RED_2)
		{
			(LED_RED_2_GPIO_PORT->BSRRL) |= 1<<LED_RED_2_PIN;
		}else if(byLedId == LED_GREEN_2)
		{
			(LED_GREEN_2_GPIO_PORT->BSRRL) |= 1<<LED_GREEN_2_PIN;
		}else if(byLedId == LED_BOARD)
		{
			(LED_BOARD_GPIO_PORT->BSRRL) |= 1<<LED_BOARD_PIN;
		}
	}
	if(byLedState == DISABLE)
	{
		if(byLedId == LED_RED_1)
		{
			(LED_RED_1_GPIO_PORT->BSRRH) |= 1<<LED_RED_1_PIN;
		}else if(byLedId == LED_BLUE_1)
		{
			(LED_BLUE_1_GPIO_PORT->BSRRH) |= 1<<LED_BLUE_1_PIN;
		}else if(byLedId == LED_GREEN_1)
		{
			(LED_GREEN_1_GPIO_PORT->BSRRH) |= 1<<LED_GREEN_1_PIN;
		}else if(byLedId == LED_RED_2)
		{
			(LED_RED_2_GPIO_PORT->BSRRH) |= 1<<LED_RED_2_PIN;
		}else if(byLedId == LED_GREEN_2)
		{
			(LED_GREEN_2_GPIO_PORT->BSRRH) |= 1<<LED_GREEN_2_PIN;
		}else if(byLedId == LED_BOARD)
		{
			(LED_BOARD_GPIO_PORT->BSRRH) |= 1<<LED_BOARD_PIN;
		}
	}
}
/**
 * @func   delayMs
 * @brief
 * @param  dwMs : thoi gian tre (milisecond)
 * @retval None
 */

void delayMs(uint32_t dwMs)
{
	uint32_t dwTimeCurrent,dwTimeInit,dwTimeTotal;
	dwTimeCurrent = GetMilSecTick();
	do{
		dwTimeInit = GetMilSecTick();
		dwTimeTotal = dwCalculatorTime(dwTimeInit, dwTimeCurrent);
	}while(dwTimeTotal<dwMs);
}
/**
 * @func   blinkledStatusPower
 * @brief  when the function is called, the function will blink led to 5 time
 * @param  None
 * @retval None
 */

void blinkledStatusPower(void)
{
	for(int i = 0;i<4;i++)
	{
		ledControlSetState(LED_BOARD, GPIO_PIN_SET);
		delayMs(1000);
		ledControlSetState(LED_BOARD, GPIO_PIN_RESET);
		delayMs(1000);
	}
}
/**
 * @func   buzzerControlSetBeep
 * @brief  This function will control buzzer state.
 * @param  None
 * @retval None
 */
void buzzerControlSetBeep(void)
{
	for(int i = 0;i<2;i++)
	{
		GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		delayMs(1000);
		GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		delayMs(1000);

	}
}
/**
 * @func   dwCalculatorTime
 * @brief  Ham tinh toan thoi gian tu dwTimeCurrent den dwTimeInit
 * @param
 * dwTimeInit: thoi gian luc truoc(ms)
 * dwTimeCurrent: thoi gian luc sau(ms)
 * @retval None
 */
uint32_t dwCalculatorTime(uint32_t dwTimeInit,uint32_t dwTimeCurrent)
{
	uint32_t dwTimeTotal;
	if(dwTimeCurrent>=dwTimeInit)
	{
		dwTimeTotal = dwTimeCurrent - dwTimeInit;
	}
	else
	{
		dwTimeTotal = 0xFFFFFFFFU + dwTimeCurrent - dwTimeInit;
	}
	return dwTimeTotal;
}
/**
 * @func   scanButton
 * @brief  This function will scan the event of buttons
 * @param  Button_t *
 * @retval None
 */
void scanButton(Button_t* pButtonStruct_t)
{
	uint32_t now,timeTemp ;
	static uint8_t temp = 0;
	now = GetMilSecTick();											/*Thoi gian lay tai thoi diem quet*/
	if(pButtonStruct_t->State == BUTTON_EDGE_RISING)
	{
		//Neu nut nhan o trang thai dang nhan
		timeTemp = dwCalculatorTime(pButtonStruct_t->timeInit, now);
		if(timeTemp >= BUTTON_LONG_PRESS_TIME)
		{
			//Nhan giu qua 500ms thi update event cua nut nhan la nhan giu
			pButtonStruct_t->buttonEven = EVENT_OF_BUTTON_HOLD_500MS;
			temp = 1;

		}
	}
	if(pButtonStruct_t->State == BUTTON_EDGE_FALLING)
	{
		//Neu nut nhan duoc nha ra
		if((temp == 0))
		{
			timeTemp = dwCalculatorTime(pButtonStruct_t->timeCurrent, now);
			if(timeTemp >= BW2PRESS_TIME)
			{
				pButtonStruct_t->pressCntEnd = pButtonStruct_t->pressCnt;
				pButtonStruct_t->timeInit = 0;
				pButtonStruct_t->timeCurrent = 0;
				pButtonStruct_t->pressCnt = 0;
				switch(pButtonStruct_t->pressCntEnd)
				{
				case 1:
				{
					pButtonStruct_t->buttonEven = EVENT_OF_BUTTON_PRESS_1_TIMES;
					pButtonStruct_t->pressCntEnd = 0;
					pButtonStruct_t->pressCnt = 0;
					break;
				}
				case 2:
				{
					pButtonStruct_t->buttonEven = EVENT_OF_BUTTON_PRESS_2_TIMES;
					pButtonStruct_t->pressCntEnd = 0;
					pButtonStruct_t->pressCnt = 0;
					break;
				}
				case 5:
				{
					pButtonStruct_t->buttonEven = EVENT_OF_BUTTON_PRESS_5_TIMES;
					pButtonStruct_t->pressCntEnd = 0;
					pButtonStruct_t->pressCnt = 0;
					break;
				}
				default:
					pButtonStruct_t->pressCntEnd = 0;
					pButtonStruct_t->pressCnt = 0;
					break;
				}
				pButtonStruct_t->State = Null;
			}
		}else
		{	pButtonStruct_t->buttonEven = EVENT_OF_BUTTON_RELEASED;
			pButtonStruct_t->pressCntEnd = 0;
			pButtonStruct_t->pressCnt = 0;
			temp = 0;
			pButtonStruct_t->State = Null;
		}


	}
}
/**
 * @func   processEventButton
 * @brief  For each event of button, the system will control led state and buzzer state.
 * @param  None
 * @retval None
 */
void processEventButton(void)
{
	// Xu ly cac event nut B2,B3,B4
	for(int i = 0; i< BUTTON_COUNT; i++)
	{
		switch(BtnProperty[i].buttonEven)
				{
					case EVENT_OF_BUTTON_HOLD_500MS:
						{
							if(i == 0)
							{
								//Tương ứng với sự kiện của nút B2
								ledControlSetState(LED_BLUE_1, GPIO_PIN_SET);
							}else if(i == 1)
							{
								//Tương ứng với sự kiện của nút B3

							}else if(i == 2)
							{
								//Tương ứng với sự kiện của nút B4
								ledControlSetState(LED_RED_2, GPIO_PIN_SET);
							}
							BtnProperty[i].buttonEven = EVENT_OF_BUTTON_NOCLICK;
						}
						break;
					case EVENT_OF_BUTTON_PRESS_5_TIMES:
						{
							if(i == 0)
							{
								//Tương ứng với sự kiện của nút B2

							}else if(i == 1)
							{
								//Tương ứng với sự kiện của nút B3
								for(int i = 0;i<5;i++)
								{
									ledControlSetState(LED_GREEN_1, GPIO_PIN_SET);
									ledControlSetState(LED_GREEN_2, GPIO_PIN_SET);
									delayMs(1000);
									ledControlSetState(LED_GREEN_1, GPIO_PIN_RESET);
									ledControlSetState(LED_GREEN_2, GPIO_PIN_RESET);
									delayMs(1000);
								}
								buzzerControlSetBeep();

							}else if(i == 2)
							{
								//Tương ứng với sự kiện của nút B4

							}
							BtnProperty[i].buttonEven = EVENT_OF_BUTTON_NOCLICK;
						}
						break;
					case EVENT_OF_BUTTON_PRESS_2_TIMES:
						{
							if(i == 0)
							{
								//Tương ứng với sự kiện của nút B2
								ledControlSetState(LED_BLUE_1, GPIO_PIN_SET);
							}else if(i == 1)
							{
								//Tương ứng với sự kiện của nút B3

							}else if(i == 2)
							{
								//Tương ứng với sự kiện của nút B4
								ledControlSetState(LED_RED_2, GPIO_PIN_SET);
							}

							BtnProperty[i].buttonEven = EVENT_OF_BUTTON_NOCLICK;
						}
						break;
					case EVENT_OF_BUTTON_PRESS_1_TIMES:
						{
							if(i == 0)
							{
								//Tương ứng với sự kiện của nút B2
								ledControlSetState(LED_BLUE_1, GPIO_PIN_RESET);
							}else if(i == 1)
							{
								//Tương ứng với sự kiện của nút B3

							}else if(i == 2)
							{
								//Tương ứng với sự kiện của nút B4
								ledControlSetState(LED_RED_2, GPIO_PIN_RESET);
							}

							BtnProperty[i].buttonEven = EVENT_OF_BUTTON_NOCLICK;
						}
						break;
					case EVENT_OF_BUTTON_RELEASED:
						{
							if(i == 0)
							{
								//Tương ứng với sự kiện của nút B2
								ledControlSetState(LED_BLUE_1, GPIO_PIN_RESET);
							}else if(i == 1)
							{
								//Tương ứng với sự kiện của nút B3

							}else if(i == 2)
							{
								//Tương ứng với sự kiện của nút B4
								ledControlSetState(LED_RED_2, GPIO_PIN_RESET);
							}

							BtnProperty[i].buttonEven = EVENT_OF_BUTTON_NOCLICK;
						}
						break;
					default:
						break;
				}
	}
}

//Chuong trinh thuc thi ngat nut bam B2
void EXTI3_IRQHandler(void)
{
	static uint8_t flagPress = 0,flagOut = 0;
	//Khi nut nhan duoc nhan
	if(GPIO_ReadInputDataBit(BUTTON_B2_GPIO_PORT, BUTTON_B2_GPIO_PIN)==0)
		{
			BtnProperty[0].timeInit = GetMilSecTick();
			BtnProperty[0].State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B2_GPIO_PORT, BUTTON_B2_GPIO_PIN)!=0)
		{
		BtnProperty[0].timeCurrent = GetMilSecTick();
		BtnProperty[0].State = BUTTON_EDGE_FALLING;
			flagPress = 1;
		}
	//Thuc hien tron 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
		BtnProperty[0].timePress = dwCalculatorTime(BtnProperty[0].timeInit, BtnProperty[0].timeCurrent);
			if((BtnProperty[0].timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnProperty[0].timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnProperty[0].pressCnt ++;
				}
			flagPress = 0;
			BtnProperty[0].Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 2)
	{
		BtnProperty[0].timeOut = dwCalculatorTime(BtnProperty[0].timeCurrent, BtnProperty[0].timeInit);
			if(BtnProperty[0].timeOut >= BW2PRESS_TIME)
				{
					BtnProperty[0].pressCntEnd = BtnProperty[0].pressCnt;
					BtnProperty[0].pressCnt = 0;
				}
			flagOut = 0;
		}


	EXTI_ClearITPendingBit(EXTI_Line3);
}

//Chuong trinh thuc thi ngat nut bam B3
void EXTI4_IRQHandler(void)
{
	static uint8_t flagPress = 0,flagOut = 0;
	//Khi nut nhan duoc nhan
	if(GPIO_ReadInputDataBit(BUTTON_B3_GPIO_PORT, BUTTON_B3_GPIO_PIN)==0)
		{
			BtnProperty[1].timeInit = GetMilSecTick();
			BtnProperty[1].State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B3_GPIO_PORT, BUTTON_B3_GPIO_PIN)!=0)
		{
			BtnProperty[1].timeCurrent = GetMilSecTick();
			BtnProperty[1].State = BUTTON_EDGE_FALLING;
			BtnProperty[1].timePress = dwCalculatorTime(BtnProperty[1].timeInit, BtnProperty[1].timeCurrent);
			flagPress = 1;
		}
	//Thuc hien tron 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
			BtnProperty[1].timePress = dwCalculatorTime(BtnProperty[1].timeInit, BtnProperty[1].timeCurrent);
			if((BtnProperty[1].timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnProperty[1].timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnProperty[1].pressCnt ++;
				}
			flagPress = 0;
			BtnProperty[1].Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 3)
	{
		BtnProperty[1].timeOut = dwCalculatorTime(BtnProperty[1].timeCurrent, BtnProperty[1].timeInit);
			if(BtnProperty[1].timeOut >= BW2PRESS_TIME)
				{
					BtnProperty[1].pressCntEnd = BtnProperty[1].pressCnt;
					BtnProperty[1].pressCnt = 0;
				}
			flagOut = 0;
		}
	EXTI_ClearITPendingBit(EXTI_Line4);
}

//Chuong trinh thuc thi ngat nut bam B4
void EXTI0_IRQHandler(void)
{
	static uint8_t flagPress = 0,flagOut = 0;
	//Khi nut nhan duoc nhan
	if(GPIO_ReadInputDataBit(BUTTON_B4_GPIO_PORT, BUTTON_B4_GPIO_PIN)==0)
		{
			BtnProperty[2].timeInit = GetMilSecTick();
			BtnProperty[2].State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B4_GPIO_PORT, BUTTON_B4_GPIO_PIN)!=0)
		{
			BtnProperty[2].timeCurrent = GetMilSecTick();
			BtnProperty[2].State = BUTTON_EDGE_FALLING;
			BtnProperty[2].timePress = dwCalculatorTime(BtnProperty[2].timeInit, BtnProperty[2].timeCurrent);
			flagPress = 1;
		}
	//Thuc hien du 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
			BtnProperty[2].timePress = dwCalculatorTime(BtnProperty[2].timeInit, BtnProperty[2].timeCurrent);
			if((BtnProperty[2].timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnProperty[2].timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnProperty[2].pressCnt ++;
				}
			flagPress = 0;
			BtnProperty[2].Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 3)
	{
		BtnProperty[2].timeOut = dwCalculatorTime(BtnProperty[2].timeCurrent, BtnProperty[2].timeInit);
			if(BtnProperty[2].timeOut >= BW2PRESS_TIME)
				{
					BtnProperty[2].pressCntEnd = BtnProperty[2].pressCnt;
					BtnProperty[2].pressCnt = 0;
				}
			flagOut = 0;
		}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

