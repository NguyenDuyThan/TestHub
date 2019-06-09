/**
  ******************************************************************************
  * @file    prtBootSTM32.h
  * @author  Nguyen Duy Than
  * @version V0.0.1
  * @date    15-September-2017
  * @brief   This file contains all the functions prototypes for the protocol bootloader STM32
  *          firmware library functions.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRTBOOTSTM32_H
#define __PRTBOOTSTM32_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* USART bootloader commands*/
#define END						0xFF
#define ACK						0x79
#define NACK					0x1F
#define GET						0x00 //Gets the version and the allowed commands supported by the current version of the bootloader
#define GETV_RPS				0x01 //Gets the bootloader version and the Read Protection status of the Flash memory
#define GETID					0x02 //Gets the chip ID
#define READMEM					0x11 //Reads up to 256 bytes of memory starting from an address specified by the application
#define GO						0x21 //Jumps to user application code located in the internal Flash memory or in SRAM
#define WRITEMEM				0x31 //Writes up to 256 bytes to the RAM or Flash memory starting from an address specified by the application
#define ERASE					0x43 //Erases from one to all the Flash memory pages
#define EXT_ERASE				0x44 //Erases from one to all the Flash memory pages using two byte addressing mode (available only for v3.0 usart bootloader versions and above).
#define WRITEPROTEC				0x63 //Enables the write protection for some sectors
#define WRITEUNPROTEC			0x73 //Disables the write protection for all Flash memory sectors
#define READOUTPROTEC			0x82 //Enables the read protection
#define READOUTUNPROTEC			0x92 //Disables the read protection

#define PROTECTED				0x80

#define MASSERASE				0xFFFF
#define BANK1ERASE				0xFFFE
#define BANK2ERASE				0xFFFD

#ifdef __cplusplus
}
#endif

#endif /* __PRTBOOTSTM32_H */

/************************ (C) COPYRIGHT ECE-G *****END OF FILE****/
