/*=============================================================================
**                              STM32_Study
**                            Copyright 2014
**-----------------------------------------------------------------------------
** Supported MCUs      : MCU STM32F103
** Supported Compilers : Compiler independent
**-----------------------------------------------------------------------------
** File name         : EEpromEmulator.h
**
** Module name       : Flash treated as EEPROM (0 <= memory < 2048 bytes(16K))
**
** Requirement specification :
**
** Summary: 
**
**= History ====================================================================
** 01.00  27/07/2014  dungvt
** -  Creation
==============================================================================*/
#ifndef __EEPROMEMULATOR_H_
#define __EEPROMEMULATOR_H_
#include "stm32f10x.h"
/*****************************************************************************/
/* DECLARATION OF CONST                                                       */
/*****************************************************************************/
//807F800	807FFFF
#define FLASH_START_ADDRESS       ((uint32_t)0x807F800)
#define FLASH_END_ADDRESS         ((uint32_t)0x807FFFF)

#define FLASH_PAGE_SIZE           ((uint16_t)0x800) /* == 256 Bytes */
#define FLASH_BUFFER_START_ADDRESS   ((uint32_t)(FLASH_END_ADDRESS + FLASH_PAGE_SIZE))
#define FLASH_PAGES_MAXIMUM        ((FLASH_END_ADDRESS - FLASH_START_ADDRESS)/FLASH_PAGE_SIZE) /* 8 pages */
#define FLASH_PAGE_BYTE_SIZE       256 /* == 256 Bytes */

/*******************************************************************************/
/* DECLARATION OF DATATYPE                                              */
/*******************************************************************************/
#ifndef BOOLEAN
#define BOOLEAN       uint8_t
#endif /* #ifndef BOOLEAN */

#ifndef TRUE
#define TRUE      1
#endif /* #ifndef TRUE */

#ifndef FALSE
#define FALSE     0
#endif /* #ifndef FALSE */

/*****************************************************************************/
/* DECLARATION OF GLOBAL FUNCTION                                            */
/*****************************************************************************/
/**
  * @brief  The function inits EEpromEmulator modules
  * @param  None
  * @retval None
  */
void EEpromEmulator_Configuration(void);
/**
  * @brief  The function used to read n bytes from Flash
  * @param  memory_position: buffer memory position to be read (0 <= memory_position <= 2047)
  * @param  data_out: pointer to save data
  * @param  len: bytes to be read; 0 <= len < 2048;
  * @retval number byte which is read.
  */
uint16_t EEpromEmulator_ReadBytes(uint16_t memory_position, u8* data_out, uint16_t len);
/**
  * @brief  The function used to write n bytes from flash
  * @param  memory_position: buffer memory position to be written (0 <= memory_position <= 2047)
  * @param  dataWrite: pointer contains data to be written
  * @param  len: bytes to be read; 0 <= len < 2048;
  * @retval status of FLASH processing.
  */
FLASH_Status EEpromEmulator_WriteBytes(uint16_t memory_position, uint8_t* dataWrite, uint16_t len);
#endif /*__FLASH_H_*/
