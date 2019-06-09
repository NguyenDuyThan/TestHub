/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "prtBootSTM32.h"

#include "string.h"
#include "stdio.h"
#include "spi.h"
/************** LCD **************/
#include "TFT_22_ILI9225.h"
//#include "DefaultFonts.c"
/************** END LCD **************/
/******************************* Config printf *********************************/
/*
 * define printf to default printing over usart
 * must add file syscalls.c into source and config function _write()
 * define int __io_putchar(int ch) into syscalls.c file
 * define putchar_prototype to next define
 * */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private function prototypes */
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	FAILED = 0, PASSED = !FAILED
} TestStatus1;
///* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
#define DATA_321                 ((uint32_t)0x12345678)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t EraseCounter1 = 0x00, Address1 = 0x00;
uint32_t Data1 = 0x3210ABCD;
uint32_t NbrOfPage1 = 0x00;
__IO FLASH_Status FLASHStatus1 = FLASH_COMPLETE;
__IO TestStatus1 MemoryProgramStatus1 = PASSED;

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 * @note	must configure usart before use this function
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
}
/*******************************************************************************/
#define log_debug(fmt, ...) do { if(logFlag.debug == ENABLE) printf("\n\r[DEBUG] \t%s:\t%d:\t%s(): " fmt, __FILE__, __LINE__, __func__,## __VA_ARGS__); } while (0)
#define log_err(fmt, ...) do { if(logFlag.err == ENABLE) { if(logFlag.debug == ENABLE) printf("\n[ERROR] \t%s:\t%d:\t%s(): " fmt, __FILE__,  __LINE__, __func__,## __VA_ARGS__); else printf("\n[ERROR] \t: " fmt,## __VA_ARGS__);} } while (0)
#define log_warn(fmt, ...) do { if(logFlag.warn == ENABLE) { if(logFlag.debug == ENABLE) printf("\n[WARN] \t%s:\t%d:\t%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__); else printf("\n[WARN] \t: " fmt,## __VA_ARGS__);} } while (0)
#define log_info(fmt, ...)  do { if(logFlag.info == ENABLE) { if(logFlag.debug == ENABLE) printf("\n\r[INFO] \t%s:\t%d:\t%s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__); else printf("\n[INFO] \t: " fmt,## __VA_ARGS__);} } while (0)
/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.  The event semaphore task is
 given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
 soon as the semaphore is given. */
#define configSUPPORT_DYNAMIC_ALLOCATION  	1
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )
#define MAIN_TASK_DEPTH						1024
#define CMD_TASK_DEPTH						512
#define LCD_TASK_DEPTH						4096
#define MAIN_TASK_PRIORITY					1
#define CMD_TASK_PRIORITY					3
#define LCD_TASK_PRIORITY					2

/* The number of items the queue can hold.  This is 1 as the receive task
 will remove items as they are added, meaning the send task should always find
 the queue empty. */
// 255	807F800	807FFFF
#define DEVICE_START_ADDR		((uint32_t)0x08000000)
#define DEVICE_ID_ADDR			((uint32_t)0x0805B800) //ID: page 183
#define DEVICE_ID_LENGHT		((uint8_t)10)
#define LENGHT_MAX_BLOCK_WRITE	((int)256)

#define FW_START_ADDR			((uint32_t)0x08010000)
#define FW_STOP_ADDR			((uint32_t)0x0806B000)
#define FW_SIZE					(FW_STOP_ADDR - FW_START_ADDR)
#define DATA_USER_START_ADDR	((uint32_t)0x0807F000)
#define DATA_USER_END_ADDR 		((uint32_t)0x0807F7FF)
/*-----------------------------------------------------------*/
char debugBuf[debugBufSize];
char deviceBuf[deviceBufSize];
volatile int debugBufIndex = 0;
volatile int deviceBufIndex = 0;
extern int buttonPressed;
uint8_t ID_STORE[DEVICE_ID_LENGHT + 1];
uint8_t ID_GET[DEVICE_ID_LENGHT + 1];
/*-----------------------------------------------------------*/
extern uint8_t tux;

xTaskHandle xHandleMainTask = NULL;
xTaskHandle xHandleAccessCmdTask = NULL;
xTaskHandle xHandleLCDTask = NULL;
/************** LCD **************/
TestStatus1 Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2,
		uint16_t BufferLength);

void GPIO_Configuration(void);
void EXTI0_Config(void);
void NVIC_Configuration(void);
static void USART_Config(USART_TypeDef* USARTx, int BaudRate, uint16_t parity);
void USARTx_Sendchar(USART_TypeDef* USARTx, char Data);
void USARTx_SendString(USART_TypeDef* USARTx, char *Str);
/*-----------------------------------------------------------*/
void FLASH_WriteByte(uint32_t addr, uint8_t *p, uint16_t Byte_Num);
void FLASH_ERASE(uint32_t addr);
void IFLASH_rd(uint32_t addr, uint16_t size, void *dat);
uint32_t IFLASH_rdWord(uint32_t Address);
static char checkSumXor(uint8_t *data, int numbyte);
static int waitACK(int value, int timeOut);
static int checkStr(char *source, char *strSign, int timeOut);
static ErrorStatus resetBuf(char *strBuf, volatile int *indexBuf, int lenBuf);
extern void debugGetData(void);
extern void deviceGetData(void);
/*-----------------------------------------------------------*/
static void statusWarning(NoticeInit_TypeDef type);
extern void bip(uint8_t times, uint8_t longtimes);
/*-----------------------------------------------------------*/
static void sendCommandToChip(char cmd);
static ErrorStatus confirmDevice(void);
static ErrorStatus getCommand(void);
//static ErrorStatus getVersionReadProtectStatus(void);
static ErrorStatus getID(void);
static ErrorStatus writeMem(uint32_t address, uint8_t *data, uint8_t len);
static ErrorStatus eraseMem(uint8_t numpages);
//static ErrorStatus extEraseMem(uint16_t numpages);
static ErrorStatus readMem(uint32_t address, uint8_t len);
//static ErrorStatus writeUnPro(void);
//static ErrorStatus readOutPro(void);
static ErrorStatus readOutUnPro(void);

void Delayus(__IO uint32_t nCount);
/*-----------------------------------------------------------*/

static void prvSetupHardware(void);
static void prvMainTask(void *pvParameters);
static void prvAccessCmdTask(void *pvParameters);
static void prvLCDTask(void *pvParameters);
/*-----------------------------------------------------------*/
/* The semaphore (in this case binary) that is used by the Frnnnnnnn
 * eeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;
/*-----------------------------------------------------------*/
/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
//static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
//static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0;
//static volatile uint32_t ulCountOfReceivedSemaphores = 0;
/*-----------------------------------------------------------*/

int main(void)
{
	logFlag.err = ENABLE;
	logFlag.debug = DISABLE;
	logFlag.info = ENABLE;
	logFlag.warn = ENABLE;

	sysFlag.writeFlash = ERROR;
	sysFlag.writeBoot = ERROR;
	sysFlag.writeApp = ERROR;
	sysFlag.eraseFlash = ERROR;
	sysFlag.saveID = FALSE;
	sysFlag.eraseID = FALSE;
	sysFlag.status = SYSTEM_RUNNING;
	sysFlag.flashIDSave = FALSE;

	cmdFlag.writeFlash = DISABLE;
	cmdFlag.writeBoot = DISABLE;
	cmdFlag.writeApp = DISABLE;
	cmdFlag.erase = DISABLE;

	prvSetupHardware();
	log_info("------------***------------");
	log_info("	Firmware Version 1.1");
	log_info("	Hardware Version 1.0");
	log_info("	Updated 26/10/2017");
	log_info("------------***------------");
	log_info("Start");

	BaseType_t xReturned;
	xReturned = xTaskCreate(prvMainTask, "MainTask", MAIN_TASK_DEPTH, NULL,
	MAIN_TASK_PRIORITY, &xHandleMainTask);
	if (xReturned == pdPASS)
	{
		log_info("Create MainTask success");
	}
	else
	{
		log_info("Create MainTask fail");
	}
	xReturned = xTaskCreate(prvAccessCmdTask, "AccessCmdTask",
	CMD_TASK_DEPTH, NULL,
	CMD_TASK_PRIORITY, &xHandleAccessCmdTask);
	if (xReturned == pdPASS)
	{
		log_info("Create AccessCmdTask success");
	}
	else
	{
		log_info("Create AccessCmdTask fail");
	}
	xReturned = xTaskCreate(prvLCDTask, "LCDTask",
	LCD_TASK_DEPTH, NULL,
	LCD_TASK_PRIORITY, &xHandleLCDTask);
	if (xReturned == pdPASS)
	{
		log_info("Create LCDTask success");
	}
	else
	{
		log_info("Create LCDTask fail");
	}
	/* Start the tasks and timer running. */
	vTaskStartScheduler();
	log_err("RTOS not running!!!");
	sysFlag.status = SYSTEM_STOPED;
	for (;;)
	{
		statusWarning(sysFlag.status);
	}
	return 0;
}
/*-----------------------------------------------------------*/

static void prvMainTask(void *pvParameters)
{
	BOOLEAN trueID = TRUE;
	BOOLEAN compareID = TRUE;
	ErrorStatus get = ERROR;
	ErrorStatus writeApp = ERROR;
	uint8_t data[256];
	BOOLEAN choseID = TRUE;
	bip(1, 50);
	for (;;)
	{
		if (sysFlag.saveID && trueID)
		{
			trueID = FALSE;
			sysFlag.saveID = FALSE;
			FLASH_WriteByte(DATA_USER_START_ADDR, ID_GET, 12);
			log_info("Save ID to Flash");
		}
		if (sysFlag.eraseID)
		{
			sysFlag.eraseID = FALSE;
			FLASH_ERASE(DATA_USER_START_ADDR);
			log_warn("Erase ID in Flash");
			bip(3, 255);
		}
		if (cmdFlag.writeFlash == ENABLE)
		{
			vTaskDelay(1000);
			cmdFlag.writeFlash = DISABLE;
			resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
			get = confirmDevice();
			if (get == SUCCESS)
			{
				log_info("Confirm device OK");
				resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
				get = getCommand();
				if (get == SUCCESS)
				{
					log_info("Version: %X.%X", (deviceBuf[2] >> 4) & 0x0F,
							(deviceBuf[2] & 0x0F));
					log_debug("ACK: 0x%X", deviceBuf[0]);
					log_debug("Num of bytes: 0x%X", deviceBuf[1]);
					log_debug("Version: 0x%X", deviceBuf[2]);
					log_debug("Get command: 0x%X", deviceBuf[3]);
					log_debug("Get VRPS: 0x%X", deviceBuf[4]);
					log_debug("GET ID: 0x%X", deviceBuf[5]);
					log_debug("Read mem: 0x%X", deviceBuf[6]);
					log_debug("Go: 0x%X", deviceBuf[7]);
					log_debug("Write mem: 0x%X", deviceBuf[8]);
					log_debug("erase: 0x%X", deviceBuf[9]);
					log_debug("WP: 0x%X", deviceBuf[10]);
					log_debug("WUP: 0x%X", deviceBuf[11]);
					log_debug("RP: 0x%X", deviceBuf[12]);
					log_debug("RUP: 0x%X", deviceBuf[13]);
					log_debug("ACK: 0x%X", deviceBuf[14]);
				}
				else
				{
					log_err("Can not get version Bootloader");
					log_debug("deviceBuf: %X %X %X %X %X %X %X %X",
							deviceBuf[0], deviceBuf[1], deviceBuf[2],
							deviceBuf[3], deviceBuf[4], deviceBuf[5],
							deviceBuf[6], deviceBuf[7]);
				}
				resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
				get = getID();
				if (get == SUCCESS)
				{
					sysFlag.status = DETECTED;
					log_info("PID: %X%X", deviceBuf[2], deviceBuf[3]);
				}
				else
				{
					log_err("Can not get PID");
					log_debug("deviceBuf: %X %X %X %X %X %X %X %X",
							deviceBuf[0], deviceBuf[1], deviceBuf[2],
							deviceBuf[3], deviceBuf[4], deviceBuf[5],
							deviceBuf[6], deviceBuf[7]);
				}
				resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
				get = readMem(DEVICE_START_ADDR, 3);
				if (get == SUCCESS)
				{
					log_debug("deviceBuf: %X %X %X %X %X %X %X %X",
							deviceBuf[0], deviceBuf[1], deviceBuf[2],
							deviceBuf[3], deviceBuf[4], deviceBuf[5],
							deviceBuf[6], deviceBuf[7]);
					resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
					get = readMem(DEVICE_ID_ADDR, DEVICE_ID_LENGHT);
					if (get == SUCCESS)
					{
						bip(2, 50);
						cmdFlag.writeApp = ENABLE;
						vTaskDelay(10);
						memcpy(ID_GET, &deviceBuf[3], DEVICE_ID_LENGHT);
						IFLASH_rd(DATA_USER_START_ADDR, sizeof(ID_STORE),
								ID_STORE);
						for (uint8_t i = 0; i < 10; i++)
						{
							if (((ID_GET[0] == 0x30) && (ID_GET[1] == 0x30))
									|| (ID_GET[i] < 0x30) || (ID_GET[i] > 0x39))
							{
								trueID = FALSE;
								log_debug("ID_GET[%d]: 0x%X", i, ID_GET[i]);
							}
							if (ID_GET[i] != ID_STORE[i])
							{
								compareID = FALSE;
								log_debug("ID_GET[%d]: /t0x%X", i, ID_GET[i]);
								log_debug("ID_STORE[%d]: /t0x%X", i,
										ID_STORE[i]);
							}
						}
						if (trueID)
						{
							ID_GET[DEVICE_ID_LENGHT] = '\0';
							log_info("ID iTracking get: %s", ID_GET);
						}
						else
						{
							log_warn("Device haven't ID");
						}
					}
					else
					{
						log_err("Cannot read data ID from device");
						log_debug("deviceBuf: %X %X %X %X %X %X", deviceBuf[0],
								deviceBuf[1], deviceBuf[2], deviceBuf[3],
								deviceBuf[4], deviceBuf[5]);
					}
				}
				else if (deviceBuf[0] == NACK)
				{
					bip(7, 50);
					log_warn("Device was protected");
					resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
					get = readOutUnPro();
					vTaskDelay(1000);
					if (get == SUCCESS)
					{
						log_info("Unprotected success");
						resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
						get = confirmDevice();
						if (get == SUCCESS)
						{
							cmdFlag.writeApp = ENABLE;
							resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
							get = readMem(DEVICE_START_ADDR, 3);
							log_debug(
									"deviceBuf: %X %X %X %X %X %X %X %X %X %X %X %X",
									deviceBuf[0], deviceBuf[1], deviceBuf[2],
									deviceBuf[3], deviceBuf[4], deviceBuf[5],
									deviceBuf[6], deviceBuf[7], deviceBuf[8],
									deviceBuf[9], deviceBuf[10], deviceBuf[11]);
							resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
							getID();
							log_debug(
									"deviceBuf: %X %X %X %X %X %X %X %X %X %X %X %X",
									deviceBuf[0], deviceBuf[1], deviceBuf[2],
									deviceBuf[3], deviceBuf[4], deviceBuf[5],
									deviceBuf[6], deviceBuf[7], deviceBuf[8],
									deviceBuf[9], deviceBuf[10], deviceBuf[11]);
							resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
							getCommand();
							log_debug(
									"deviceBuf: %X %X %X %X %X %X %X %X %X %X %X %X",
									deviceBuf[0], deviceBuf[1], deviceBuf[2],
									deviceBuf[3], deviceBuf[4], deviceBuf[5],
									deviceBuf[6], deviceBuf[7], deviceBuf[8],
									deviceBuf[9], deviceBuf[10], deviceBuf[11]);
							resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
							get = readMem(DEVICE_ID_ADDR, DEVICE_ID_LENGHT);
							if (get == SUCCESS)
							{
								cmdFlag.writeApp = ENABLE;
								vTaskDelay(10);
								memcpy(ID_GET, &deviceBuf[3], DEVICE_ID_LENGHT);
								ID_GET[DEVICE_ID_LENGHT] = '\0';
								log_info("ID iTracking: %s", ID_GET);
								IFLASH_rd(DATA_USER_START_ADDR,
										sizeof(ID_STORE), ID_STORE);
								for (uint8_t i = 0; i < 10; i++)
								{
									if (((ID_GET[0] == 0x30)
											&& (ID_GET[1] == 0x30))
											|| (ID_GET[i] < 0x30)
											|| (ID_GET[i] > 0x39))
									{
										trueID = FALSE;
									}
									if (ID_GET[i] != ID_STORE[i])
									{
										compareID = FALSE;
									}
								}
							}
							else
							{
								log_debug("deviceBuf: %X %X %X %X %X %X",
										deviceBuf[0], deviceBuf[1],
										deviceBuf[2], deviceBuf[3],
										deviceBuf[4], deviceBuf[5]);
							}
						}
					}
					else
					{
						log_debug("deviceBuf: %X %X %X %X %X %X", deviceBuf[0],
								deviceBuf[1], deviceBuf[2], deviceBuf[3],
								deviceBuf[4], deviceBuf[5]);
					}
				}
				else
				{
					log_debug("deviceBuf: %X %X %X %X %X %X", deviceBuf[0],
							deviceBuf[1], deviceBuf[2], deviceBuf[3],
							deviceBuf[4], deviceBuf[5]);
				}
			}
			else
			{
				log_err("Cannot detect device");
			}

			if (cmdFlag.writeApp == ENABLE)
			{
				cmdFlag.writeApp = DISABLE;
				resetBuf(deviceBuf, &deviceBufIndex, deviceBufSize);
				get = eraseMem(0xFF);
				if (get == SUCCESS)
				{
					log_info("Erase Flash device ok");
				}
				else
				{
					log_warn("Erase Flash fail");
					log_debug("deviceBuf: %X %X %X %X %X %X %X %X",
							deviceBuf[0], deviceBuf[1], deviceBuf[2],
							deviceBuf[3], deviceBuf[4], deviceBuf[5],
							deviceBuf[6], deviceBuf[7]);
				}
				log_info("Start write APP");
				log_info("Waiting...");
				statusWarning(NOTHING);
				for (int i = 0; i < ((FW_SIZE / 256) + 1); i++)
				{
					sysFlag.status = UPDATTING;
					if ((i % 2) == 0)
					{
						GPIO_ResetBits(Status_port, Status_pin);
					}
					else
					{
						GPIO_SetBits(Status_port, Status_pin);
					}
					memset(data, 0, sizeof(data));
					IFLASH_rd((FW_START_ADDR + (i * 256)), sizeof(data), data);
					uint8_t timeloop = 0;
					AGAINAPP: resetBuf(deviceBuf, &deviceBufIndex,
					deviceBufSize);
					get = writeMem(DEVICE_START_ADDR + (i * 256), &data[0],
							255);
					if (get == SUCCESS)
					{
						writeApp = SUCCESS;
						log_debug("Written block %d success", i);
					}
					else
					{
						writeApp = ERROR;
						log_debug("Write block %d failed", i);
						timeloop++;
						if (timeloop < 1)
						{
							goto AGAINAPP;
						}
						else
						{
							break;
						}
					}
				}
				if (writeApp == SUCCESS)
				{
					log_info("Firmware written success");
					sysFlag.status = UPDATE_OK;
					if (compareID && trueID)
					{
						choseID = TRUE;
					}
					else
					{
						if (sysFlag.flashIDSave)
						{
							choseID = FALSE;
						}
						else
						{
							if (trueID)
							{
								choseID = TRUE;
							}
							else
							{
								get = ERROR;
								goto CANCEL_WRITE_ID;
							}
						}
					}
					if (choseID)
					{
						get = writeMem(DEVICE_ID_ADDR, &ID_GET[0],
						DEVICE_ID_LENGHT);
					}
					else
					{
						get = writeMem(DEVICE_ID_ADDR, &ID_STORE[0],
						DEVICE_ID_LENGHT);
					}
					CANCEL_WRITE_ID: if (get == SUCCESS)
					{
						log_info("Flash ID comeback success");
						sysFlag.status = UPDATE_OK;
						bip(1, 50);
					}
					else
					{
						sysFlag.saveID = TRUE;
						sysFlag.status = NOT_ID;
						log_warn("Cannot write ID comeback to device");
						bip(3, 50);
					}
				}
				else
				{
					sysFlag.saveID = TRUE;
					bip(2, 200);
					log_err("Firmware written FAIL!!!");
					sysFlag.status = UPDATE_ERROR;
				}
			}
		}
		statusWarning(sysFlag.status);
		vTaskDelay(1000);
	}
}
/*-----------------------------------------------------------*/

static void prvAccessCmdTask(void *pvParameters)
{
	uint8_t reset = 0;
	for (;;)
	{
		if (buttonPressed == 3)
		{
			sysFlag.status = SYSTEM_RUNNING;
			sysFlag.eraseID = TRUE;
		}
		else if (buttonPressed == 2)
		{
			sysFlag.status = SYSTEM_RUNNING;
			uint8_t checkID = 1;
			IFLASH_rd(DATA_USER_START_ADDR, sizeof(ID_STORE), ID_STORE);
			for (uint8_t i = 0; i < 10; i++)
			{
				if ((ID_STORE[i] < 0x30) || (ID_STORE[i] > 0x39))
				{
					checkID = 0;
					log_debug("ID[%d]: 0x%X", i, ID_STORE[i]);
				}
			}
			if (checkID)
			{
				cmdFlag.writeFlash = ENABLE;
				sysFlag.flashIDSave = TRUE;
			}
			else
			{
				bip(1, 255);
				log_info("Don't have ID to be save in flash");
			}
		}
		else if (buttonPressed == 1)
		{
			sysFlag.status = SYSTEM_RUNNING;
			sysFlag.flashIDSave = FALSE;
			cmdFlag.writeFlash = ENABLE;
		}
		buttonPressed = 0;
		if (checkStr(debugBuf, "#", 100))
		{
			if (checkStr(debugBuf, "debug0", 1))
			{
				log_info("Received command \"debug0\"");
				logFlag.debug = DISABLE;
				logFlag.err = DISABLE;
				logFlag.warn = DISABLE;
			}
			else if (checkStr(debugBuf, "debug1", 1))
			{
				log_info("Received command \"debug1\"");
				logFlag.debug = ENABLE;
				logFlag.info = ENABLE;
				logFlag.err = ENABLE;
				logFlag.warn = ENABLE;
			}
			else if (checkStr(debugBuf, "info0", 1))
			{
				log_info("Received command \"info0\"");
				logFlag.info = DISABLE;
			}
			else if (checkStr(debugBuf, "flash", 1))
			{
				log_info("Received command \"flash\"");
				cmdFlag.writeFlash = ENABLE;
			}
			else if (checkStr(debugBuf, "buflog", 1))
			{
				log_info("Received command \"devicebuf\"");
				log_info("deviceBuf: %X %X %X %X %X %X %X %X", deviceBuf[0],
						deviceBuf[1], deviceBuf[2], deviceBuf[3], deviceBuf[4],
						deviceBuf[5], deviceBuf[6], deviceBuf[7]);
			}
			else if (checkStr(debugBuf, "ID2", 1))
			{
				log_info("Received command \"ID2\"");
				uint8_t temp[11];
				IFLASH_rd(DATA_USER_START_ADDR, sizeof(temp), temp);
				log_info("temp: %X %X %X %X %X %X %X %X %X %X %X", temp[0],
						temp[1], temp[2], temp[3], temp[4], temp[5], temp[6],
						temp[7], temp[8], temp[9], temp[10]);

			}
			else if (checkStr(debugBuf, "LCD0", 1))
			{
				log_info("Received command \"LCD0\"");
				LCD_LED_RESET;
			}
			else if (checkStr(debugBuf, "LCD1", 1))
			{
				log_info("Received command \"LCD1\"");
				LCD_LED_SET;
			}
			else if (checkStr(debugBuf, "pic0", 1))
			{
				log_info("Received command \"pic0\"");
				setDisplay(FALSE);
			}
			else if (checkStr(debugBuf, "pic1", 1))
			{
				log_info("Received command \"pic1\"");
				setDisplay(TRUE);
			}
			else
			{
				log_info("Command not found");
			}
			reset = resetBuf(debugBuf, &debugBufIndex, debugBufSize);
			if (reset == ERROR)
			{
				log_info("reset debugBuf error");
			}
		}
		vTaskDelay(1000);
	}
}
/*-----------------------------------------------------------*/

static void prvLCDTask(void *pvParameters)
{
	uint8_t i=0;
				setOrientation(0);
				setBackgroundColor(COLOR_BLACK);
				drawBitmap(0, 0, &tux, 180, 220, COLOR_WHITE);
	for (;;)
	{
//		USARTx_Sendchar(UART5, '1');
		i++;
		if (i>=1)
		{
			i = 0;
//			setDisplay(TRUE);
//			setBacklight(TRUE);
//			clear();
			setFont(Terminal12x16);
			setBackgroundColor(COLOR_YELLOW);
			drawText(1, 1, "Mr.THAN_Test LCD", COLOR_RED);
			setBackgroundColor(COLOR_BLACK);
			vTaskDelay(10);
			drawRectangle(0, 0, maxX() - 1, maxY() - 1, COLOR_WHITE);
			setFont(Terminal6x8);
			drawText(1, 19, "hello!", COLOR_WHITE);
			vTaskDelay(10);
			drawText(1, 29, "text small", COLOR_WHITE);
//			setBackgroundColor(COLOR_YELLOW);
//
			drawRectangle(10, 39, 160, 50, COLOR_BLUE);
			drawText(30, 40, "rectangle", COLOR_WHITE);
//			vTaskDelay(1000);
//
//			clearxy(0,16,180,220);
			fillRectangle(20, 52, 160, 65, COLOR_RED);
			drawText(30, 54, "solidRectangle", COLOR_WHITE);
//			vTaskDelay(1000);
//
//			clearxy(0,16,180,220);
			drawCircle(40, 98, 30, COLOR_YELLOW);
			drawText(20, 130, "circle", COLOR_WHITE);
//			vTaskDelay(1000);
//
//			clearxy(0,16,180,220);
			fillCircle(110, 98, 30, COLOR_GREEN);
			drawText(100, 130, "solidCircle", COLOR_WHITE);
//			vTaskDelay(1000);
//
//			clearxy(0,16,180,220);
			drawLine(50, 144, 160, 144, COLOR_CYAN);
			drawText(1, 140, "line", COLOR_WHITE);
//			vTaskDelay(1000);

//			clearxy(0,16,180,220);
//			setOrientation(0);
//			clearxy(0,16,180,220);
//			drawText(10, 100, "drawing bitmap", COLOR_WHITE);
//			vTaskDelay(1000);
//			clearxy(0,16,180,220);
//			setBackgroundColor(COLOR_BLACK);
//			drawBitmap(0, 0, tux, 180, 220, COLOR_WHITE);
//			vTaskDelay(1000);

//			clearxy(0,16,180,220);
//			setOrientation(0);
//			clearxy(0,16,180,220);
			setFont(Terminal12x16);
			setBackgroundColor(COLOR_YELLOW);
			drawText(70, 160, "bye!", COLOR_RED);
			setBackgroundColor(COLOR_BLACK);
//			setFont(Terminal6x8);
//			vTaskDelay(1000);
//
//			clearxy(0,16,180,220);
//			drawText(10, 60, "off", COLOR_WHITE);
//			vTaskDelay(1000);
//
//			setBacklight(FALSE);
//			setDisplay(FALSE);
		}
		vTaskDelay(1000);
	}
}
/*-----------------------------------------------------------*/

TestStatus1 Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2,
		uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
			return FAILED;
		}

		pBuffer1++;
		pBuffer2++;
	}

	return PASSED;
}

void GPIO_Configuration(void)
{
//	GPIO_DeInit(GPIOA);
//	GPIO_DeInit(GPIOB);
//	GPIO_DeInit(GPIOC);
	/*________________________________ OUTPUT ___________________________________*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = Buzzer_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Buzzer_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Ok_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Ok_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Error_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Error_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = Status_pin;
	GPIO_Init(Status_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = LCD_LED_pin;
	GPIO_Init(LCD_LED_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4); //PC4	DC

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0); //PB0	LCD_RST
	/*________________________________ INPUT ____________________________________*/

	/*

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // TX - USART3
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);


	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    // RX - USART3
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	 ___________________________________________________________________________
	 */

}

void EXTI0_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure PB.09 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = Start_bt_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(Start_bt_port, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Connect EXTI9 Line to PB.09 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	/* Configure EXTI9 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI9_5 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

void USARTx_Sendchar(USART_TypeDef* USARTx, char Data)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USARTx, Data);
}

static void USART_Config(USART_TypeDef* USARTx, int BaudRate, uint16_t parity)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	if (USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (USARTx == USART2)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (USARTx == USART3)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (USARTx == UART4)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if (USARTx == UART5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}
	/* Enable USART clock */
	if (USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else if (USARTx == USART2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	else if (USARTx == USART3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	else if (USARTx == UART4)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	else if (USARTx == UART5)
	{
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, DISABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	}
//RCC_AHBPeriph_SDIO
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
	/* Remap pin to USART */

	if (USARTx == UART5)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	}
	else
	{
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	}
	if (USARTx == USART1)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;    // TX - USART1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // RX - USART1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if (USARTx == USART2)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    // TX - USART2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    // RX - USART2
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if (USARTx == USART3)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // TX - USART3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    // RX - USART3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	else if (USARTx == UART5)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;    // TX - USART3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    // RX - USART3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	}

	USART_InitStructure.USART_BaudRate = BaudRate;
	if (parity != USART_Parity_No)
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	}
	else
	{
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	}
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = parity;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTx, &USART_InitStructure);

	/* NVIC configuration */
	/* Configure the Priority Group to 2 bits */
	if (USARTx == USART1)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

	}
	else if (USARTx == USART2)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else if (USARTx == UART5)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}

	if (USARTx != UART5){
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); //cho phep ngat
	}

	/* Enable USART */
	USART_Cmd(USARTx, ENABLE);
	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//LCD
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//LCD

}

char USARTx_Getchar(USART_TypeDef* USARTx)
{
	char Data;
	while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
		;
	Data = (char) USART_ReceiveData(USARTx);
	return Data;
}

void USARTx_SendString(USART_TypeDef* USARTx, char *Str)
{
	while (*Str)
	{
		USARTx_Sendchar(USARTx, *Str);
		Str++;
	}
}
/*-----------------------------------------------------------*/
void FLASH_WriteByte(uint32_t addr, uint8_t *p, uint16_t Byte_Num)
{
	uint32_t HalfWord;
	Byte_Num = Byte_Num / 2;
	FLASH_Unlock();
	FLASH_ClearFlag(
	FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(addr);
	while (Byte_Num--)
	{
		HalfWord = *(p++);
		HalfWord |= *(p++) << 8;
		FLASH_ProgramHalfWord(addr, HalfWord);
		addr += 2;
	}
	FLASH_Lock();
}
/*-----------------------------------------------------------*/
void FLASH_ERASE(uint32_t addr)
{
	FLASH_Unlock();
	FLASH_ClearFlag(
	FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(addr);
	FLASH_Lock();
}

/*-----------------------------------------------------------*/

void IFLASH_rd(uint32_t addr, uint16_t size, void *dat)
{
	memcpy(dat, (void*) addr, size);
}
/********************************************************
 * @brief:		read_flash
 * @param[in]:	U32
 * @param[out]: none
 * @retval;		unsigned int
 *
 * @details:	read data at flash address
 ***********************************************************/
uint32_t IFLASH_rdWord(uint32_t Address)
{
	/* Check the correction of written data */
	uint32_t Data;
	Data = (*(uint32_t*) Address);
	return Data;
}
/*-----------------------------------------------------------*/

static char checkSumXor(uint8_t *data, int numbyte)
{
	char ret = 0;
	if (data != NULL)
	{
		for (int i = 0; i < numbyte; i++)
		{
			ret = ret ^ data[i];
		}
	}
	return ret;
}
/*-----------------------------------------------------------*/

static ErrorStatus resetBuf(char *strBuf, volatile int *indexBuf, int lenBuf)
{
	USART_Cmd(USART1, DISABLE);
	USART_Cmd(USART2, DISABLE);
	Delayus(10);
	memset(strBuf, 0, 256);
	*indexBuf = 0;
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
	Delayus(10);
	if (strBuf[0] != '\0')
		return ERROR;
	else
		return SUCCESS;
}
/*-----------------------------------------------------------*/

static int waitACK(int value, int timeOut)
{
	for (int i = 0; i < timeOut; i++)
	{
		uint8_t temp = 0;
		for (int j = 0; j < 100; j++)
		{
			if (deviceBuf[j] == ACK)
			{
				temp++;
				if (temp == value)
				{
					return 1;
				}
				/*			} else if (deviceBuf[j] == NACK) {
				 return 0;*/
			}
		}
		vTaskDelay(1);
	}
	return 0;
}
/*-----------------------------------------------------------*/

void debugGetData(void)
{
	unsigned char data;
	data = USART_ReceiveData(debugOut);
	if (debugBufIndex == debugBufSize)
		debugBufIndex = 0;
	debugBuf[debugBufIndex] = data;
	debugBufIndex++;
}
/*-----------------------------------------------------------*/

void deviceGetData(void)
{
	unsigned char data;
	data = (USART_ReceiveData(deviceOut));
	if (deviceBufIndex == deviceBufSize)
		deviceBufIndex = 0;
	deviceBuf[deviceBufIndex] = data;
	deviceBufIndex++;
}
/*-----------------------------------------------------------*/

static void statusWarning(NoticeInit_TypeDef type)
{
	if (type == UPDATE_OK)
	{
		GPIO_SetBits(Status_port, Status_pin);
		GPIO_SetBits(Error_port, Error_pin);
		GPIO_ResetBits(Ok_port, Ok_pin);
	}
	else if (type == UPDATE_ERROR)
	{
		GPIO_SetBits(Status_port, Status_pin);
		GPIO_ResetBits(Error_port, Error_pin);
		GPIO_SetBits(Ok_port, Ok_pin);
	}
	else if ((type == SYSTEM_RUNNING) || (type == DETECTED))
	{
		vTaskDelay(1000);
		GPIO_ResetBits(Status_port, Status_pin);
		GPIO_ResetBits(Error_port, Error_pin);
		GPIO_ResetBits(Ok_port, Ok_pin);
		vTaskDelay(10);
		GPIO_SetBits(Status_port, Status_pin);
		GPIO_SetBits(Error_port, Error_pin);
		GPIO_SetBits(Ok_port, Ok_pin);
	}
	else if (type == SYSTEM_STOPED)
	{
		GPIO_ResetBits(Status_port, Status_pin);
		GPIO_ResetBits(Error_port, Error_pin);
		GPIO_ResetBits(Ok_port, Ok_pin);
		bip(1, 50);
		vTaskDelay(1000);
	}
	else if (type == UPDATTING)
	{
		GPIO_ResetBits(Status_port, Status_pin);
		GPIO_SetBits(Error_port, Error_pin);
		GPIO_SetBits(Ok_port, Ok_pin);
		vTaskDelay(50);
		GPIO_SetBits(Status_port, Status_pin);
		vTaskDelay(20);
		GPIO_ResetBits(Status_port, Status_pin);
	}
	else if (type == NOT_ID)
	{
		GPIO_SetBits(Status_port, Status_pin);
		GPIO_ResetBits(Error_port, Error_pin);
		GPIO_ResetBits(Ok_port, Ok_pin);
	}
	else if (type == NOTHING)
	{
		GPIO_SetBits(Status_port, Status_pin);
		GPIO_SetBits(Error_port, Error_pin);
		GPIO_SetBits(Ok_port, Ok_pin);
	}
}

/*-----------------------------------------------------------*/

void bip(uint8_t times, uint8_t longtimes)
{
	for (uint8_t i = 0; i < times; i++)
	{
//		vTaskDelay(longtimes);
//		GPIO_SetBits(Buzzer_port, Buzzer_pin);
//		vTaskDelay(longtimes);
//		GPIO_ResetBits(Buzzer_port, Buzzer_pin);
	}
}
/*-----------------------------------------------------------*/

/*--------------- checkStr ----------------------
 * Brief: check a string in other string
 * Param:	source		|	IN	|	input source string.
 * 			strSign		|	IN	|	string will be search in source.
 * 			timeOut		|	IN	|	times (1 is 10 ms) to waiting for data in source to search.
 * Ret:		response if success 1, otherwise 0.
 ----------------------------------------------*/

static int checkStr(char *source, char *strSign, int timeOut)
{
	uint8_t timePass = 0;
	if ((source != NULL) && (strSign != NULL))
	{
		for (timePass = 0; timePass < timeOut; timePass++)
		{
			if (strstr(source, strSign) != NULL)
			{
				return 1;
			}
			else
			{
				vTaskDelay(10);
			}
		}
	}
	return 0;
}
/*-----------------------------------------------------------*/

void sendCommandToChip(char cmd)
{
	USARTx_Sendchar(deviceOut, cmd);
	Delayus(50);
}
/*-----------------------------------------------------------*/

static ErrorStatus confirmDevice(void)
{
	ErrorStatus ret = ERROR;
	sendCommandToChip(0x7F);
	if (waitACK(1, TIMEOUT + 2000))
	{
		ret = SUCCESS;
	}
	return ret;
}
/*-----------------------------------------------------------*/

static ErrorStatus getCommand(void)
{
	ErrorStatus ret = ERROR;
	sendCommandToChip(GET);
	sendCommandToChip(0xFF);
	if (waitACK(2, TIMEOUT))
	{
		ret = SUCCESS;
	}
	return ret;
}
/*-----------------------------------------------------------*/
//
//static ErrorStatus getVersionReadProtectStatus(void) {
//	ErrorStatus ret = ERROR;
//	sendCommandToChip(GETV_RPS);
//	sendCommandToChip(0xFE);
//	if (waitACK(2, TIMEOUT)) {
//		ret = SUCCESS;
//	}
//	return ret;
//}
/*-----------------------------------------------------------*/

static ErrorStatus getID(void)
{
	ErrorStatus ret = ERROR;
	sendCommandToChip(GETID);
	sendCommandToChip(0xFD);
	if (waitACK(2, TIMEOUT))
	{
		ret = SUCCESS;
	}
	return ret;
}
/*-----------------------------------------------------------*/

static ErrorStatus writeMem(uint32_t address, uint8_t *data, uint8_t len)
{
	if (len > 255)
	{
		log_debug("input lenght of data max is 256");
		return 0;
	}
	ErrorStatus ret = ERROR;
	uint8_t addtemp[4];
	char checksum = 0;
	sendCommandToChip(WRITEMEM);
	sendCommandToChip(0xCE);
	if (waitACK(1, TIMEOUT))
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			addtemp[i] = (address >> (24 - (i * 8))) & 0xFF;
			sendCommandToChip(addtemp[i]);
		}
		checksum = checkSumXor(addtemp, 4);
		sendCommandToChip(checksum);
		if (waitACK(2, TIMEOUT))
		{
			sendCommandToChip(len);
			for (int i = 0; i <= len; i++)
			{
				sendCommandToChip(data[i]);
			}
			checksum = checkSumXor(data, len + 1);
			checksum ^= len;
			sendCommandToChip(checksum);
			if (waitACK(3, TIMEOUT))
			{
				ret = SUCCESS;
			}
			else
			{
				log_err("deviceBuf: %X %X %X %X %X %X %X %X", deviceBuf[0],
						deviceBuf[1], deviceBuf[2], deviceBuf[3], deviceBuf[4],
						deviceBuf[5], deviceBuf[6], deviceBuf[7]);
			}
		}
		else
		{
			log_err("deviceBuf: %X %X %X %X %X %X %X %X", deviceBuf[0],
					deviceBuf[1], deviceBuf[2], deviceBuf[3], deviceBuf[4],
					deviceBuf[5], deviceBuf[6], deviceBuf[7]);
		}
	}
	else
	{
		log_err("deviceBuf: %X %X %X %X %X %X %X %X", deviceBuf[0],
				deviceBuf[1], deviceBuf[2], deviceBuf[3], deviceBuf[4],
				deviceBuf[5], deviceBuf[6], deviceBuf[7]);
	}
	return ret;
}
/*-----------------------------------------------------------*/

static ErrorStatus eraseMem(uint8_t numpages)
{
	uint8_t temp = 0;
	ErrorStatus ret = ERROR;
	sendCommandToChip(ERASE);
	sendCommandToChip(0xBC);
	if (waitACK(1, TIMEOUT))
	{
		if (numpages == 0xFF)
		{
			sendCommandToChip(numpages);
			sendCommandToChip(0x00);
			if (waitACK(2, TIMEOUT))
			{
				ret = SUCCESS;
			}
		}
		else
		{
			sendCommandToChip(numpages);
			temp = numpages + 1;
			sendCommandToChip(temp);
			char check = numpages ^ temp;
			sendCommandToChip(check);
			log_info("numpages: %X %X %X", numpages, temp, check);
			if (waitACK(2, TIMEOUT))
			{
				ret = SUCCESS;
			}
		}
	}
	return ret;
}
/*-----------------------------------------------------------*/

/*

 static ErrorStatus extEraseMem(uint16_t numpages){
 uint8_t ret = 0;
 sendCommandToChip(EXT_ERASE);
 sendCommandToChip(0xBB);
 if(waitFlag(deviceFlag.ack, 1000)){
 deviceFlag.ack = FALSE;
 //		char check = checkSumXor(&numpages, 2);
 //		sendCommandToChip(check);
 if(waitFlag(deviceFlag.ack, 1000)){
 deviceFlag.ack = FALSE;
 ret = ack_e;
 } else if(deviceFlag.nack){
 deviceFlag.nack = FALSE;
 ret = nack_e;
 }
 } else if(deviceFlag.nack){
 deviceFlag.nack = FALSE;
 ret = nack_e;
 }
 return ret;
 }
 -----------------------------------------------------------
 */
/*-----------------------------------------------------------*/

static ErrorStatus readMem(uint32_t address, uint8_t len)
{
	ErrorStatus ret = ERROR;
	uint8_t addtemp[3];
	uint8_t checksum = 0;
	sendCommandToChip(READMEM);
	sendCommandToChip(0xEE);
	if (waitACK(1, TIMEOUT))
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			addtemp[i] = (address >> (24 - i * 8)) & 0xFF;
			sendCommandToChip(addtemp[i]);
		}
		checksum = checkSumXor(addtemp, 4);
		sendCommandToChip(checksum);
		if (waitACK(2, TIMEOUT))
		{
			sendCommandToChip((len));
			checksum = 0;
			checksum = 255 - len;
			sendCommandToChip(checksum);
			if (waitACK(3, TIMEOUT))
			{
				ret = SUCCESS;
			}
		}
	}
	return ret;
}
/*-----------------------------------------------------------*/

//static ErrorStatus writeUnPro(void) {
//	ErrorStatus ret = ERROR;
//	sendCommandToChip(WRITEUNPROTEC);
//	sendCommandToChip(0x8C);
//	if (waitACK(2, TIMEOUT)) {
//		ret = SUCCESS;
//	}
//	return ret;
//}
/*-----------------------------------------------------------*/

//static ErrorStatus readOutPro(void) {
//	ErrorStatus ret = ERROR;
//	sendCommandToChip(READOUTPROTEC);
//	sendCommandToChip(0x7D);
//	if (waitACK(2, TIMEOUT)) {
//		ret = SUCCESS;
//	}
//	return ret;
//}
/*-----------------------------------------------------------*/

static ErrorStatus readOutUnPro(void)
{
	ErrorStatus ret = ERROR;
	sendCommandToChip(READOUTUNPROTEC);
	sendCommandToChip(0x6D);
	if (waitACK(1, TIMEOUT))
	{
		ret = SUCCESS;
	}
	return ret;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t ulCount = 0;

	/* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
	 1 in FreeRTOSConfig.h.

	 "Give" the semaphore on every 500th tick interrupt. */
	ulCount++;
	if (ulCount >= 500UL)
	{
		/* This function is called from an interrupt context (the RTOS tick
		 interrupt),	so only ISR safe API functions can be used (those that end
		 in "FromISR()".

		 xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
		 pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
		 task that has equal or higher priority than the interrupted task.
		 http://www.freertos.org/a00124.html */
		xSemaphoreGiveFromISR(xEventSemaphore, &xHigherPriorityTaskWoken);
		ulCount = 0UL;
	}

	/* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
	 normally be performed before leaving the interrupt (because during the
	 execution of the interrupt a task of equal or higher priority than the
	 running task was unblocked).  The syntax required to context switch from
	 an interrupt is port dependent, so check the documentation of the port you
	 are using.  http://www.freertos.org/a00090.html

	 In this case, the function is running in the context of the tick interrupt,
	 which will automatically check for the higher priority task to run anyway,
	 so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* The malloc failed hook is enabled by setting
	 configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	 Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	 configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected.  pxCurrentTCB can be
	 inspected in the debugger if the task name passed into this function is
	 corrupt. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	 FreeRTOSConfig.h.

	 This function is called on each cycle of the idle task.  In this case it
	 does nothing useful, other than report the amount of FreeRTOS heap that
	 remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if (xFreeStackSpace > 100)
	{
		/* By now, the kernel has allocated everything it is going to, so
		 if there is a lot of heap remaining unallocated then
		 the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		 reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	SystemInit();
	NVIC_SetPriorityGrouping(0);
	SPI1_Init();

	USART_Config(debugOut, debugBaud, USART_Parity_No);
	log_info("setup hardware");
	USART_Config(deviceOut, deviceBaud, USART_Parity_Even);
//	USART_Config(camOut, camBaud, USART_Parity_No);
	GPIO_Configuration();

	GPIO_SetBits(Status_port, Status_pin);
	GPIO_SetBits(Error_port, Error_pin);
	GPIO_SetBits(Ok_port, Ok_pin);
	GPIO_ResetBits(Buzzer_port, Buzzer_pin);

	EXTI0_Config();
	initTFT();
	log_info("done");
}
/*-----------------------------------------------------------*/

void Delayus(__IO uint32_t nCount)
{
	while (nCount--)
	{
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func,
		const char *failedexpr)
{
	while (1)
	{
	}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
	__assert_func(file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval sEE_FAIL.
 */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
	/* Return with error code */
	return sEE_FAIL;
}
#endif
#endif /* USE_SEE */

/************** LCD **************/
/************** END LCD **************/
