/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "stm32f10x_adc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_can.h"
#include "stm32f10x_cec.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/

typedef enum {
	FALSE = 0, TRUE=!FALSE,
} BOOLEAN;

typedef enum {
	SYSTEM_RUNNING = 0, SYSTEM_STOPED, UPDATE_OK, UPDATE_ERROR, UPDATTING, DETECTED, NOT_ID, NOTHING,
} NoticeInit_TypeDef;

typedef struct flagstatus_t {
	FunctionalState debug;
	FunctionalState info;
	FunctionalState err;
	FunctionalState warn;
} flagstatus;
flagstatus logFlag;

typedef struct flagdb_t {
	ErrorStatus writeFlash;
	ErrorStatus readFlash;
	ErrorStatus writeBoot;
	ErrorStatus writeApp;
	ErrorStatus eraseFlash;
	BOOLEAN command;
	NoticeInit_TypeDef status;
	BOOLEAN saveID;
	BOOLEAN eraseID;
	BOOLEAN flashIDSave;
} flagdb;
flagdb sysFlag;

typedef struct flag_t {
	FunctionalState writeFlash;
	FunctionalState readFlash;
	FunctionalState writeBoot;
	FunctionalState writeApp;
	FunctionalState erase;
} flag;
flag cmdFlag;

/*-----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#define debugOut		USART1
#define debugBaud		230400
#define debugBufSize	512

#define deviceOut		USART2
#define deviceBaud		115200
#define deviceBufSize	512

#define camOut		UART5
#define camBaud		115200
#define camBufSize	512

#define TIMEOUT			5000

#define Buzzer_pin		GPIO_Pin_1
#define Buzzer_port		GPIOA
#define Status_pin		GPIO_Pin_6
#define Status_port		GPIOB
#define Error_pin		GPIO_Pin_5
#define Error_port		GPIOB
#define Ok_pin			GPIO_Pin_7
#define Ok_port			GPIOB

#define Start_bt_pin	GPIO_Pin_8
#define Start_bt_port	GPIOB

#define LCD_LED_pin		GPIO_Pin_9
#define LCD_LED_port	GPIOB
#define TFT_CLK_pin		GPIO_Pin_5
#define TFT_CLK_port	GPIOA
#define TFT_SDI_pin		GPIO_Pin_7
#define TFT_SDI_port	GPIOA
#define TFT_RS_pin		GPIO_Pin_4 //dc
#define TFT_RS_port		GPIOC
#define TFT_RST_pin		GPIO_Pin_0
#define TFT_RST_port	GPIOB
#define TFT_CS_pin		GPIO_Pin_4
#define TFT_CS_port		GPIOA

/*-----------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
