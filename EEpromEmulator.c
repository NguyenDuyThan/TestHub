/*=============================================================================
**                              STM32_Study
**                            Copyright 2014
**-----------------------------------------------------------------------------
** Supported MCUs      : MCU STM32F103
** Supported Compilers : Compiler independent
**-----------------------------------------------------------------------------
** File name         : EEpromEmulator.c
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
/******************************************************************************/
/* INCLUSIONS                                                                 */
/******************************************************************************/
#include "../EEpromEmulator.h"
/******************************************************************************/
/* DEFINITION OF DATATYPES                                               */
/******************************************************************************/
typedef struct
{
  BOOLEAN         IsPageToBeReadWrite;
  uint16_t        addressOffset; /* 0 <= addressOffset < FLASH_PAGE_BYTE_SIZE */
  uint16_t        data_len; 
}EEpPageType;

/******************************************************************************/
/* DEFINITION OF LOCAL VARIABLES                                               */
/******************************************************************************/
static EEpPageType EEP_PageToBeReadWrite[8];
volatile FLASH_Status FLASHStatus;

/******************************************************************************/
/* DECLARATIOM OF INTERNAL FUNCTIONS                                          */
/******************************************************************************/
static uint32_t EEpromEmulator_iBytesToDword(uint8_t* data);
static void EEpromEmulator_iCalculatePageToBeReadWrite(uint32_t memory_position, uint32_t len);

/******************************************************************************/
/* DEFINITION OF GLOBAL FUNCTIONS                                             */
/******************************************************************************/
/**
  * @brief  The function inits EEpromEmulator modules
  * @param  None
  * @retval None
  */
void EEpromEmulator_Configuration(void)
{
  uint16_t pageIdx;
  for(pageIdx = 0; pageIdx < FLASH_PAGES_MAXIMUM; pageIdx++)
  {
    EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite = FALSE;
    EEP_PageToBeReadWrite[pageIdx].addressOffset = 0;
    EEP_PageToBeReadWrite[pageIdx].data_len = 0;
  }
  FLASHStatus = FLASH_COMPLETE;
}

/**
  * @brief  The function used to read n bytes from Flash
  * @param  memory_position: read data  from this position
  * @param  data_out: pointer to save data
  * @param  len: bytes to be read; 0 <= len < 2048;
  * @retval number byte which is read.
  */
uint16_t EEpromEmulator_ReadBytes(uint16_t memory_position, u8* data_out, uint16_t len)
{
  uint32_t address;
  uint32_t EndAddressCurrentPage;
  uint32_t data;
	uint16_t local_offset;
	uint16_t qty;
  uint16_t pageIdx;
  uint8_t tmp;
  uint8_t i;

  EEpromEmulator_iCalculatePageToBeReadWrite(memory_position, len);
  qty = 0;
  for(pageIdx = 0; (pageIdx < FLASH_PAGES_MAXIMUM); pageIdx++)
  {
    if (EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite == TRUE)
    {
      address = FLASH_START_ADDRESS + (FLASH_PAGE_SIZE * pageIdx);
      EndAddressCurrentPage = address + FLASH_PAGE_SIZE;

      local_offset = 0;       
      while((address < EndAddressCurrentPage) && (FLASHStatus == FLASH_COMPLETE) && (qty < len))
      {
        /* read from current page */
        data = (*(__IO uint32_t*) address);
        tmp = 32;
        for(i = 0; i < 4; i++)
        {
          tmp -= 8;
          if (local_offset >= EEP_PageToBeReadWrite[pageIdx].addressOffset)
          {
            *data_out++ = (uint8_t)(data >> tmp);
            qty++;
          }
          local_offset++;
        } 
        /* update address for next reading */
        address += 4;
      }
    } /* if (EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite == TRUE) */
    else
    {
      /* do nothing */
    }
  }
  return qty;
}

/**
  * @brief  The function used to write n bytes from flash
  * @param  memory_position: read data  from this position
  * @param  dataWrite: pointer contains data to be written
  * @param  len: bytes to be read; 0 <= len < 2048;
  * @retval status of FLASH processing.
  */
FLASH_Status EEpromEmulator_WriteBytes(uint16_t memory_position, uint8_t* dataWrite, uint16_t len)
{
  uint16_t pageIdx;
  uint32_t address;
  uint32_t addressBuffer;
  uint32_t EndAddressCurrentPage;
  uint32_t data;
  uint16_t local_offset;
  uint16_t qty;
  uint8_t local_data[4];
  uint8_t tmp;
  uint8_t i;
  /* Unlock the Flash Program Erase controller */
	FLASH_Unlock();

	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* caculate page where address is locate */
  EEpromEmulator_iCalculatePageToBeReadWrite(memory_position, len);
  FLASHStatus = FLASH_COMPLETE;
  qty = 0;
  /* Start to write */
  for(pageIdx = 0; (pageIdx < FLASH_PAGES_MAXIMUM) && (FLASHStatus == FLASH_COMPLETE); pageIdx++)
  {
    if (EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite == TRUE)
    {
      /* 1. Read all data from current pageIdx */
      /* 1.1 Erase all BUFFER page */
      FLASHStatus = FLASH_ErasePage(FLASH_BUFFER_START_ADDRESS);
      if (FLASHStatus == FLASH_COMPLETE)
      {
        /* 1.2 Read all data from one page and save to BUFFER */
        address = FLASH_START_ADDRESS + (FLASH_PAGE_SIZE * pageIdx);
        EndAddressCurrentPage = address + FLASH_PAGE_SIZE;

        addressBuffer = FLASH_BUFFER_START_ADDRESS;
        //EndAddressBuffer = FLASH_BUFFER_END_ADDRESS;
        local_offset = 0;        
        while((address < EndAddressCurrentPage) && (FLASHStatus == FLASH_COMPLETE))
	      {
          /* read from current page */
          data = (*(__IO uint32_t*) address);
          tmp = 32;
          for(i = 0; i < 4; i++)
          {            
            tmp -= 8;
            if ((local_offset >= EEP_PageToBeReadWrite[pageIdx].addressOffset) && (qty < len))
            {
              local_data[i] = dataWrite[qty++];            
            }
            else
            {
              local_data[i] = (uint8_t)(data >> tmp);
            }
            local_offset++;
          }
          /* update gain */
          data = EEpromEmulator_iBytesToDword(&local_data[0]);
          /* Write again to buffer page */
          FLASHStatus = FLASH_ProgramWord(addressBuffer, data);
          /* update address for next write cycle */
          address += 4;
          addressBuffer += 4;
        }/* while(address < EndAddressCurrentPage) */
      }
      /* 1.3 erase current page */ 
      address = FLASH_START_ADDRESS + (FLASH_PAGE_SIZE * pageIdx);
      EndAddressCurrentPage = address + FLASH_PAGE_SIZE; 
      addressBuffer = FLASH_BUFFER_START_ADDRESS;
      /* Erase current page */ 
      FLASHStatus = FLASH_ErasePage(address);
      
      /* 1.4 read from buffer and then write again to current page */ 
      while((address < EndAddressCurrentPage) && (FLASHStatus == FLASH_COMPLETE))
      {
        /* read from buffer */
        data = (*(__IO uint32_t*) addressBuffer);
        /* Write again to current page */
        FLASHStatus = FLASH_ProgramWord(address, data);
        address += 4;
        addressBuffer += 4;        
      }
    } /*if (EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite == TRUE)*/
    else
    {
      /* do nothing */
    }
  }
  return FLASHStatus;
}
/******************************************************************************/
/* DEFINITION OF INTERNAL FUNCTIONS                                             */
/******************************************************************************/
/**
  * @brief  The function used to read n bytes 32 bits (len) from startAddress
  * @param  memory_position: (0 <= memory_position <= 2047)
  * @param  len: length of data which is calculate ?
  * @retval none.
  */
static void EEpromEmulator_iCalculatePageToBeReadWrite(uint32_t memory_position, uint32_t len)
{
  uint16_t pageIdx;
  uint16_t firstPageIdx;
  uint16_t lastPageIdx;

  /* First, reset all pages */
  for(pageIdx = 0; pageIdx < FLASH_PAGES_MAXIMUM; pageIdx++)
  {
    EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite = FALSE;
    EEP_PageToBeReadWrite[pageIdx].addressOffset = 0;
    EEP_PageToBeReadWrite[pageIdx].data_len = 0;
  }
  firstPageIdx = memory_position / FLASH_PAGE_BYTE_SIZE;
  lastPageIdx = (memory_position + len) / FLASH_PAGE_BYTE_SIZE;
  if (firstPageIdx == lastPageIdx)
  {
    /* First page is last page */
    EEP_PageToBeReadWrite[firstPageIdx].IsPageToBeReadWrite = TRUE;
    EEP_PageToBeReadWrite[firstPageIdx].addressOffset = memory_position % FLASH_PAGE_BYTE_SIZE;
    EEP_PageToBeReadWrite[firstPageIdx].data_len = (uint8_t)len;
  }
  else
  {
    /* First page */
    EEP_PageToBeReadWrite[firstPageIdx].IsPageToBeReadWrite = TRUE;
    EEP_PageToBeReadWrite[firstPageIdx].addressOffset = memory_position % FLASH_PAGE_BYTE_SIZE;
    EEP_PageToBeReadWrite[firstPageIdx].data_len = FLASH_PAGE_BYTE_SIZE - EEP_PageToBeReadWrite[firstPageIdx].addressOffset;
    /* Next pages */
    for(pageIdx = firstPageIdx + 1; pageIdx < lastPageIdx; pageIdx++)
    {
      EEP_PageToBeReadWrite[pageIdx].IsPageToBeReadWrite = TRUE;
      EEP_PageToBeReadWrite[pageIdx].addressOffset = 0;
      EEP_PageToBeReadWrite[pageIdx].data_len = FLASH_PAGE_BYTE_SIZE;      
    }
    /* The last pages */
    EEP_PageToBeReadWrite[lastPageIdx].IsPageToBeReadWrite = TRUE;
    EEP_PageToBeReadWrite[lastPageIdx].addressOffset = 0;
    EEP_PageToBeReadWrite[lastPageIdx].data_len = (memory_position + len) % FLASH_PAGE_BYTE_SIZE;    
  }
}
/**
  * @brief  Convert 4 bytes (uint8_t) to Dword (uint32_t)
  * @param  data: pointer will be converted
  * @retval Dword (uint32_t).
  */
static uint32_t EEpromEmulator_iBytesToDword(uint8_t* data)
{
  uint16_t dataWriteMSB;
  uint16_t dataWriteLSB;  
  uint32_t dataWrite;

  dataWriteMSB = *data++;
  dataWriteMSB = (dataWriteMSB << 8) | *data++;
  dataWriteLSB = *data++;
  dataWriteLSB = (dataWriteLSB << 8) | *data;
  dataWrite = (dataWriteMSB << 16) | dataWriteLSB;

  return dataWrite;
}
