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
//Led Red: 1.PA1 2.PB13
#define LED_RED_2_GPIO_PIN				GPIO_Pin_1
#define LED_RED_2_GPIO_PORT				GPIOA
#define	LED_RED_2_INIT_SETCLOCK			RCC_AHB1Periph_GPIOA
#define LED_RED_2_PIN					1

#define LED_RED_1_GPIO_PIN				GPIO_Pin_13
#define LED_RED_1_GPIO_PORT				GPIOB
#define	LED_RED_1_INIT_SETCLOCK			RCC_AHB1Periph_GPIOB
#define LED_RED_1_PIN					13


//Led Blue: 1.PA3	2.PA10
#define LED_BLUE_1_GPIO_PIN				GPIO_Pin_10
#define LED_BLUE_1_GPIO_PORT			GPIOA
#define	LED_BLUE_1_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_BLUE_1_PIN					10
//Led Green: 1.PA0	2.PA11
#define LED_GREEN_2_GPIO_PIN			GPIO_Pin_0
#define LED_GREEN_2_GPIO_PORT			GPIOA
#define	LED_GREEN_2_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_GREEN_2_PIN					0

#define LED_GREEN_1_GPIO_PIN			GPIO_Pin_11
#define LED_GREEN_1_GPIO_PORT			GPIOA
#define	LED_GREEN_1_INIT_SETCLOCK		RCC_AHB1Periph_GPIOA
#define LED_GREEN_1_PIN					11
//Led Board: PA5
#define LED_BOARD_GPIO_PIN				GPIO_Pin_5
#define LED_BOARD_GPIO_PORT				GPIOA
#define	LED_BOARD_INIT_SETCLOCK			RCC_AHB1Periph_GPIOA
#define LED_BOARD_PIN					5
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
Button_t BtnB2;
Button_t BtnB3;
Button_t BtnB4;
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
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static
void outputInit(uint32_t dwSetClock,uint16_t wGpioPin,GPIO_TypeDef *pGpioPort);

static
void Input_Interrupt(void);

static
void LedBuzz_Init(void);

static
void Button_Init(void);

static
void LedControl_SetState(uint8_t led_id,uint8_t led_state);

void Delay_ms(int ms);

void Blinkled_StatusPower(void);

void BuzzerControl_SetBeep(void);

uint32_t CalculatorTime(uint32_t dwTimeInit,uint32_t dwTimeCurrent);

void ScanButton(Button_t* ButtonStruct);

void processEventButton(void);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void)
{
	SystemCoreClockUpdate();
	LedBuzz_Init();
	Button_Init();
	Blinkled_StatusPower();
	TimerInit();
    /* Loop forever */
	while(1)
		{
			ScanButton(&BtnB2);
			ScanButton(&BtnB3);
			ScanButton(&BtnB4);
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
void outputInit(uint32_t dwSetClock,uint16_t wGpioPin,GPIO_TypeDef *pGpioPort)
{
	GPIO_InitTypeDef  GPIO_Intructure;

	RCC_AHB1PeriphClockCmd(dwSetClock,ENABLE);

	GPIO_Intructure.GPIO_Pin  = wGpioPin;
	GPIO_Intructure.GPIO_Mode = GPIO_Mode_OUT ;
	GPIO_Intructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Intructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Intructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(pGpioPort,&GPIO_Intructure);
}
/**
 * @func   Input_Interrupt
 * @brief  Ham setup dau ra
 * @param
 * SetClock: Xung cap
 * GPIO_Pin: Pin dau vao
 * GPIO_Port: Port dau vao
 * EXTI_PortSourceGPIOx
 * EXTI_PinSourcex
 * EXTI_Line
 * NVIC_IRQChannel
 * EXTI_Trigger
 * @retval None
 */
static
void Input_Interrupt(void)
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
 * @func   LedBuzz_Init
 * @brief  Ham setup cho Led va loa Buzzer
 * @param  None
 * @retval None
 */
static
void LedBuzz_Init(void)
{
	outputInit(LED_RED_1_INIT_SETCLOCK, LED_RED_1_GPIO_PIN, LED_RED_1_GPIO_PORT);
	outputInit(LED_RED_2_INIT_SETCLOCK, LED_RED_2_GPIO_PIN, LED_RED_2_GPIO_PORT);
	outputInit(LED_BLUE_1_INIT_SETCLOCK, LED_BLUE_1_GPIO_PIN, LED_BLUE_1_GPIO_PORT);
	outputInit(LED_GREEN_1_INIT_SETCLOCK, LED_GREEN_1_GPIO_PIN, LED_GREEN_1_GPIO_PORT);
	outputInit(LED_GREEN_2_INIT_SETCLOCK, LED_GREEN_2_GPIO_PIN, LED_GREEN_2_GPIO_PORT);
	outputInit(LED_BOARD_INIT_SETCLOCK, LED_BOARD_GPIO_PIN, LED_BOARD_GPIO_PORT);
	outputInit(BUZZER_INIT_SETCLOCK, BUZZER_GPIO_PIN, BUZZER_GPIO_PORT);
}
/**
 * @func   Button_Init
 * @brief  Ham setup nut nhan
 * @param  None
 * @retval None
 */
static
void Button_Init(void)
{
	BtnB2.State = BUTTON_STATE_START;
	BtnB3.State = BUTTON_STATE_START;
	BtnB4.State = BUTTON_STATE_START;
	Input_Interrupt();
}
/**
 * @func   LedControl_SetState
 * @brief  Ham setup Led sang va tat
 * @param
 * led_id: Led can set trang thai
 * led_state: ENABLE(bat), DISABLE(tat)
 * @retval None
 */
static
void LedControl_SetState(uint8_t led_id,uint8_t led_state)
{
	if(led_state == ENABLE)
	{
		if(led_id == LED_RED_1)
		{
			(LED_RED_1_GPIO_PORT->BSRRL) |= 1<<LED_RED_1_PIN;
		}else if(led_id == LED_BLUE_1)
		{
			(LED_BLUE_1_GPIO_PORT->BSRRL) |= 1<<LED_BLUE_1_PIN;
		}else if(led_id == LED_GREEN_1)
		{
			(LED_GREEN_1_GPIO_PORT->BSRRL) |= 1<<LED_GREEN_1_PIN;
		}else if(led_id == LED_RED_2)
		{
			(LED_RED_2_GPIO_PORT->BSRRL) |= 1<<LED_RED_2_PIN;
		}else if(led_id == LED_GREEN_2)
		{
			(LED_GREEN_2_GPIO_PORT->BSRRL) |= 1<<LED_GREEN_2_PIN;
		}else if(led_id == LED_BOARD)
		{
			(LED_BOARD_GPIO_PORT->BSRRL) |= 1<<LED_BOARD_PIN;
		}
	}
	if(led_state == DISABLE)
	{
		if(led_id == LED_RED_1)
		{
			(LED_RED_1_GPIO_PORT->BSRRH) |= 1<<LED_RED_1_PIN;
		}else if(led_id == LED_BLUE_1)
		{
			(LED_BLUE_1_GPIO_PORT->BSRRH) |= 1<<LED_BLUE_1_PIN;
		}else if(led_id == LED_GREEN_1)
		{
			(LED_GREEN_1_GPIO_PORT->BSRRH) |= 1<<LED_GREEN_1_PIN;
		}else if(led_id == LED_RED_2)
		{
			(LED_RED_2_GPIO_PORT->BSRRH) |= 1<<LED_RED_2_PIN;
		}else if(led_id == LED_GREEN_2)
		{
			(LED_GREEN_2_GPIO_PORT->BSRRH) |= 1<<LED_GREEN_2_PIN;
		}else if(led_id == LED_BOARD)
		{
			(LED_BOARD_GPIO_PORT->BSRRH) |= 1<<LED_BOARD_PIN;
		}
	}
}
/**
 * @func   Delay_ms
 * @brief  Ham tao do tre
 * @param
 * ms : thoi gian tre (milisecond)
 * @retval None
 */

void Delay_ms(int ms)
{
	for(int i = 0;i<1000*ms;i++);
}
/**
 * @func   Blinkled_StatusPower
 * @brief  Ham bat tat led tren board 4 lan
 * @param  None
 * @retval None
 */

void Blinkled_StatusPower(void)
{
	for(int i = 0;i<4;i++)
	{
		LedControl_SetState(LED_BOARD, GPIO_PIN_SET);
		Delay_ms(1000);
		LedControl_SetState(LED_BOARD, GPIO_PIN_RESET);
		Delay_ms(1000);
	}
}
/**
 * @func   BuzzerControl_SetBeep
 * @brief  Ham dieu khien loa keu 2 tieng bip
 * @param  None
 * @retval None
 */
void BuzzerControl_SetBeep(void)
{
	for(int i = 0;i<2;i++)
	{
		GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		Delay_ms(1000);
		GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		Delay_ms(1000);

	}
}
/**
 * @func   CalculatorTime
 * @brief  Ham tinh toan thoi gian tu dwTimeCurrent den dwTimeInit
 * @param
 * dwTimeInit: thoi gian luc truoc(ms)
 * dwTimeCurrent: thoi gian luc sau(ms)
 * @retval None
 */
uint32_t CalculatorTime(uint32_t dwTimeInit,uint32_t dwTimeCurrent)
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
 * @func   ScanButton
 * @brief  Ham quet event nut nhan
 * @param  Con tro cau truc nut nhan dang Button_t
 * @retval None
 */
void ScanButton(Button_t* ButtonStruct)
{
	uint32_t now,timeTemp;
	static uint8_t temp = 0;
	now = GetMilSecTick();											/*Thoi gian lay tai thoi diem quet*/
	if(ButtonStruct->State == BUTTON_EDGE_RISING)
	{
		//Neu nut nhan o trang thai dang nhan
		timeTemp = CalculatorTime(ButtonStruct->timeInit, now);
		if(timeTemp >= BUTTON_LONG_PRESS_TIME)
		{
			//Nhan giu qua 500ms thi update event cua nut nhan la nhan giu
			ButtonStruct->buttonEven = EVENT_OF_BUTTON_HOLD_500MS;
			temp = 1;

		}
	}
	if(ButtonStruct->State == BUTTON_EDGE_FALLING)
	{
		//Neu nut nhan duoc nha ra
		if((temp == 0))
		{
			timeTemp = CalculatorTime(ButtonStruct->timeCurrent, now);
			if(timeTemp >= BW2PRESS_TIME)
			{
				ButtonStruct->pressCntEnd = ButtonStruct->pressCnt;
				ButtonStruct->timeInit = 0;
				ButtonStruct->timeCurrent = 0;
				ButtonStruct->pressCnt = 0;
				switch(ButtonStruct->pressCntEnd)
				{
				case 1:
				{
					ButtonStruct->buttonEven = EVENT_OF_BUTTON_PRESS_1_TIMES;
					ButtonStruct->pressCntEnd = 0;
					ButtonStruct->pressCnt = 0;
					break;
				}
				case 2:
				{
					ButtonStruct->buttonEven = EVENT_OF_BUTTON_PRESS_2_TIMES;
					ButtonStruct->pressCntEnd = 0;
					ButtonStruct->pressCnt = 0;
					break;
				}
				case 5:
				{
					ButtonStruct->buttonEven = EVENT_OF_BUTTON_PRESS_5_TIMES;
					ButtonStruct->pressCntEnd = 0;
					ButtonStruct->pressCnt = 0;
					break;
				}
				default:
					ButtonStruct->pressCntEnd = 0;
					ButtonStruct->pressCnt = 0;
					break;
				}
				ButtonStruct->State = Null;
			}
		}else
		{	ButtonStruct->buttonEven = EVENT_OF_BUTTON_RELEASED;
			ButtonStruct->pressCntEnd = 0;
			ButtonStruct->pressCnt = 0;
			temp = 0;
			ButtonStruct->State = Null;
		}


	}
}

void processEventButton(void)
{
	// Xu ly cac event nut B2
	switch(BtnB2.buttonEven)
		{
			case EVENT_OF_BUTTON_HOLD_500MS:
				{
					LedControl_SetState(LED_BLUE_1, GPIO_PIN_SET);
					BtnB2.buttonEven = EVENT_OF_BUTTON_NOCLICK;
				}
				break;
			case EVENT_OF_BUTTON_PRESS_5_TIMES:
				{
					BtnB2.buttonEven = EVENT_OF_BUTTON_NOCLICK;
				}
				break;
			case EVENT_OF_BUTTON_PRESS_2_TIMES:
				{
					LedControl_SetState(LED_BLUE_1, GPIO_PIN_SET);
					BtnB2.buttonEven = EVENT_OF_BUTTON_NOCLICK;
				}
				break;
			case EVENT_OF_BUTTON_PRESS_1_TIMES:
				{
					LedControl_SetState(LED_BLUE_1, GPIO_PIN_RESET);
					BtnB2.buttonEven = EVENT_OF_BUTTON_NOCLICK;
				}
				break;
			case EVENT_OF_BUTTON_RELEASED:
				{
					LedControl_SetState(LED_BLUE_1, GPIO_PIN_RESET);
					BtnB2.buttonEven = EVENT_OF_BUTTON_NOCLICK;
				}
				break;
			default:
				break;
		}
	// Xu ly cac event nut B3
	switch(BtnB3.buttonEven)
			{
				case EVENT_OF_BUTTON_HOLD_500MS:
					{

						BtnB3.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_5_TIMES:
					{
						for(int i = 0;i<5;i++)
						{
							LedControl_SetState(LED_GREEN_1, GPIO_PIN_SET);
							LedControl_SetState(LED_GREEN_2, GPIO_PIN_SET);
							Delay_ms(1000);
							LedControl_SetState(LED_GREEN_1, GPIO_PIN_RESET);
							LedControl_SetState(LED_GREEN_2, GPIO_PIN_RESET);
							Delay_ms(1000);
						}
						BuzzerControl_SetBeep();

						BtnB3.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_2_TIMES:
					{
						BtnB3.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_1_TIMES:
					{
						BtnB3.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_RELEASED:
					{
						BtnB3.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				default:
					break;
			}
	// Xu ly cac event nut B4
	switch(BtnB4.buttonEven)
			{
				case EVENT_OF_BUTTON_HOLD_500MS:
					{
						LedControl_SetState(LED_RED_2, GPIO_PIN_SET);
						BtnB4.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_5_TIMES:
					{
						BtnB4.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_2_TIMES:
					{
						LedControl_SetState(LED_RED_2, GPIO_PIN_SET);
						BtnB4.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_PRESS_1_TIMES:
					{
						LedControl_SetState(LED_RED_2, GPIO_PIN_RESET);
						BtnB4.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				case EVENT_OF_BUTTON_RELEASED:
					{
						LedControl_SetState(LED_RED_2, GPIO_PIN_RESET);
						BtnB4.buttonEven = EVENT_OF_BUTTON_NOCLICK;
					}
					break;
				default:
					break;
			}
}
//Chuong trinh thuc thi ngat nut bam B2
void EXTI3_IRQHandler(void)
{
	static uint8_t flagPress = 0,flagOut = 0;
	//Khi nut nhan duoc nhan
	if(GPIO_ReadInputDataBit(BUTTON_B2_GPIO_PORT, BUTTON_B2_GPIO_PIN)==0)
		{
			BtnB2.timeInit = GetMilSecTick();
			BtnB2.State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B2_GPIO_PORT, BUTTON_B2_GPIO_PIN)!=0)
		{
			BtnB2.timeCurrent = GetMilSecTick();
			BtnB2.State = BUTTON_EDGE_FALLING;
			flagPress = 1;
		}
	//Thuc hien tron 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
			BtnB2.timePress = CalculatorTime(BtnB2.timeInit, BtnB2.timeCurrent);
			if((BtnB2.timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnB2.timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnB2.pressCnt ++;
				}
			flagPress = 0;
			BtnB2.Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 2)
	{
		BtnB2.timeOut = CalculatorTime(BtnB2.timeCurrent, BtnB2.timeInit);
			if(BtnB2.timeOut >= BW2PRESS_TIME)
				{
					BtnB2.pressCntEnd = BtnB2.pressCnt;
					BtnB2.pressCnt = 0;
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
			BtnB3.timeInit = GetMilSecTick();
			BtnB3.State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B3_GPIO_PORT, BUTTON_B3_GPIO_PIN)!=0)
		{
			BtnB3.timeCurrent = GetMilSecTick();
			BtnB3.State = BUTTON_EDGE_FALLING;
			BtnB3.timePress = CalculatorTime(BtnB3.timeInit, BtnB3.timeCurrent);
			flagPress = 1;
		}
	//Thuc hien tron 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
			BtnB3.timePress = CalculatorTime(BtnB3.timeInit, BtnB3.timeCurrent);
			if((BtnB3.timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnB3.timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnB3.pressCnt ++;
				}
			flagPress = 0;
			BtnB3.Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 3)
	{
		BtnB3.timeOut = CalculatorTime(BtnB3.timeCurrent, BtnB3.timeInit);
			if(BtnB3.timeOut >= BW2PRESS_TIME)
				{
					BtnB3.pressCntEnd = BtnB3.pressCnt;
					BtnB3.pressCnt = 0;
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
			BtnB4.timeInit = GetMilSecTick();
			BtnB4.State = BUTTON_EDGE_RISING;
			flagOut++;
		}
	//khi nut nhan duoc nha ra
	if(GPIO_ReadInputDataBit(BUTTON_B4_GPIO_PORT, BUTTON_B4_GPIO_PIN)!=0)
		{
			BtnB4.timeCurrent = GetMilSecTick();
			BtnB4.State = BUTTON_EDGE_FALLING;
			BtnB4.timePress = CalculatorTime(BtnB4.timeInit, BtnB4.timeCurrent);
			flagPress = 1;
		}
	//Thuc hien du 1 chu trinh nhan xong nha cua nut bam
	if(flagPress == 1)
		{
			BtnB4.timePress = CalculatorTime(BtnB4.timeInit, BtnB4.timeCurrent);
			if((BtnB4.timePress >=BUTTON_NORMAL_PRESS_TIME)&&(BtnB4.timePress < BUTTON_LONG_PRESS_TIME))
				{
					BtnB4.pressCnt ++;
				}
			flagPress = 0;
			BtnB4.Status = BUTTON_STATE_WAITPRESS;
		}
	//Khi nut nhan duoc nhan lien tiep nhau
	if(flagOut == 3)
	{
		BtnB4.timeOut = CalculatorTime(BtnB4.timeCurrent, BtnB4.timeInit);
			if(BtnB4.timeOut >= BW2PRESS_TIME)
				{
					BtnB4.pressCntEnd = BtnB4.pressCnt;
					BtnB4.pressCnt = 0;
				}
			flagOut = 0;
		}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

