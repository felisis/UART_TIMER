/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f1xx_hal_flash.h"
#include <ring_buffer.h>
#include <debug.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_FRAME_MAX 30
#define FLASH_USER  ADDR_FLASH_PAGE_127 //
#define START_ADDR  FLASH_USER
#define END_ADDR    FLASH_USER + 1024 // 1024 bytes
#define ADDR_FLASH_PAGE_0 ((uint32_t)0x08000000)
#define ADDR_FLASH_PAGE_127_END ((uint32_t )0x0801FFF0)
#define ADDR_FLASH_PAGE_127 ((uint32_t )0x0801FC00)

#define SectorSize			0x1000
#define PageSize			256
#define SectorCount			15 * 16
#define PageCount			(SectorCount * SectorSize) / PageSize
#define BlockSize			SectorSize * 16
#define CapacityInKiloByte	SectorCount * SectorSize / 1024;

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x00000000) /* Base @ of Sector 0, 4 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x00001000) /* Base @ of Sector 1, 4 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x00002000) /* Base @ of Sector 2, 4 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x00003000) /* Base @ of Sector 3, 4 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x00004000) /* Base @ of Sector 4, 4 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x00005000) /* Base @ of Sector 5, 4 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x00006000) /* Base @ of Sector 6, 4 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x00007000) /* Base @ of Sector 7, 4 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x00008000) /* Base @ of Sector 8, 4 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x00009000) /* Base @ of Sector 9, 4 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x0000A000) /* Base @ of Sector 10, 4 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x0000B000) /* Base @ of Sector 11, 4 Kbytes */
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x0000C000) /* Base @ of Sector 12, 4 Kbytes */
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x0000D000) /* Base @ of Sector 13, 4 Kbytes */
#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x0000E000) /* Base @ of Sector 14, 4 Kbytes */
#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x0000F000) /* Base @ of Sector 15, 4 Kbytes */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ring_buffer_t rbuf;

int cnt_num = 0;
int digit = 0;
int check = 0;
int blk_cnt;
int flag = 0;

double use_console_run = 0;

char rx_frame[RX_FRAME_MAX];
char *frame1;
char *frame2;
char *frame3;
char *next_ptr;

uint32_t Temp = 0;
uint8_t mode = 0;
uint8_t mode_key1 = 0;
uint8_t num;
uint8_t indata;
uint8_t rcv_data;
uint8_t rx_buffer[RX_FRAME_MAX];
uint8_t rx_frame_cnt = 0;
uint8_t s_digit_10 = 0;
uint8_t s_digit_1 = 0;
uint8_t digit_10 = 0;
uint8_t digit_1 = 0;
uint8_t hex0 = 0;

uint8_t buff[4];
uint8_t uniqID[8];
uint8_t Stts_rgstr;
uint8_t Source_buf[4];
uint8_t Flash_buf[4096];

uint32_t hex1 = 0;
uint32_t hex2 = 0;
uint32_t min = 1;
uint32_t count;
uint32_t use_console_start, use_console_end;
uint32_t flash_add;
uint32_t flash_point;
uint32_t flash_SectorToPage(uint32_t SectorAddress);
uint32_t flash_BlockToPage(uint32_t BlockAddress);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void FND_CLR();
void printnum(uint8_t num);
static u_int32_t StringToHexa(const char *frame2);
uint8_t read_pin();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t GetSector(uint32_t Address) {
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
		sector = ADDR_FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1)) {
		sector = ADDR_FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2)) {
		sector = ADDR_FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3)) {
		sector = ADDR_FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4)) {
		sector = ADDR_FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5)) {
		sector = ADDR_FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6)) {
		sector = ADDR_FLASH_SECTOR_6;
	}
	else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7)) {
		sector = ADDR_FLASH_SECTOR_7;
	}
	else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8)) {
		sector = ADDR_FLASH_SECTOR_8;
	}
	else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9)) {
		sector = ADDR_FLASH_SECTOR_9;
	}
	else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10)) {
		sector = ADDR_FLASH_SECTOR_10;
	}
	else if ((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11)) {
		sector = ADDR_FLASH_SECTOR_11;
	}
	else if ((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12)) {
		sector = ADDR_FLASH_SECTOR_12;
	}
	else if ((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13)) {
		sector = ADDR_FLASH_SECTOR_13;
	}
	else if ((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14)) {
		sector = ADDR_FLASH_SECTOR_14;
	}
	else {
		sector = ADDR_FLASH_SECTOR_15;
	}

	return sector;
}

uint8_t flash_spi(uint8_t data) {
	uint8_t ret;
	HAL_SPI_TransmitReceive(&hspi1, &data, &ret, 1, 100);
	return ret;
}

uint8_t buf_copy(uint32_t Bytes_Address) {
	uint8_t val;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x13);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	val = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); //pBuffer result value
	return val;
}

void flash_wait_end(void) { // Read Status Register-1
	uint8_t stat = 0;

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x05);
	do {
		stat = flash_spi(0xA5);
		HAL_Delay(1);
	} while ((stat & 0x01) == 0x01);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); // CS to low
}

void flash_write_enable(void) { // Write Enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x06);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_write_disable(void) { // Write Disable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x04);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_write_byte(uint8_t pBuffer, uint32_t WriteAddr_inBytes) {
	// Page Program
	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x12);
	flash_spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	flash_spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	flash_spi((WriteAddr_inBytes & 0xFF00) >> 8);
	flash_spi(WriteAddr_inBytes & 0xFF);
	flash_spi(pBuffer);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
}

void sector_write_byte(uint32_t WriteAddr_inBytes) {
	// Page Program
	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x12);
	flash_spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	flash_spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	flash_spi((WriteAddr_inBytes & 0xFF00) >> 8);
	flash_spi(WriteAddr_inBytes & 0xFF);
	for (int i = 256 * flag; i < 256 + 256 * flag; i++) {
		flash_spi(Flash_buf[i]);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
}

void flash_write_page(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize) {
	if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > PageSize) || (NumByteToWrite_up_to_PageSize == 0)) NumByteToWrite_up_to_PageSize = PageSize
			- OffsetInByte;
	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > PageSize) NumByteToWrite_up_to_PageSize = PageSize - OffsetInByte;

	flash_wait_end();
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x02);
	Page_Address = (Page_Address * PageSize) + OffsetInByte;
	flash_spi((Page_Address & 0xFF0000) >> 16);
	flash_spi((Page_Address & 0xFF00) >> 8);
	flash_spi(Page_Address & 0xFF);
	HAL_SPI_Transmit(&hspi1, pBuffer, NumByteToWrite_up_to_PageSize, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

void flash_write_sector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize) {
	if ((NumByteToWrite_up_to_SectorSize > SectorSize) || (NumByteToWrite_up_to_SectorSize == 0)) NumByteToWrite_up_to_SectorSize = SectorSize;

	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > SectorSize) BytesToWrite = SectorSize - OffsetInByte;
	else BytesToWrite = NumByteToWrite_up_to_SectorSize;
	StartPage = flash_SectorToPage(Sector_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;

	do {
		flash_write_page(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

void flash_write_block(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize) {
	if ((NumByteToWrite_up_to_BlockSize > BlockSize) || (NumByteToWrite_up_to_BlockSize == 0)) NumByteToWrite_up_to_BlockSize = BlockSize;

	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;

	if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > BlockSize) BytesToWrite = BlockSize - OffsetInByte;
	else BytesToWrite = NumByteToWrite_up_to_BlockSize;
	StartPage = flash_BlockToPage(Block_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_write_page(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

uint32_t flash_SectorToPage(uint32_t SectorAddress) {
	return (SectorAddress * SectorSize) / PageSize;
}

uint32_t flash_BlockToPage(uint32_t BlockAddress) {
	return (BlockAddress * BlockSize) / PageSize;
}

void flash_read(uint32_t Bytes_Address) { // Read(0Ch)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0C);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	for (int i = 0; i < 4; i++) {
		buff[i] = flash_spi(0xA5);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);

	PRINTF_DEBUG("Read(0CH) :");
	for (int i = 0; i < 4; i++) {
		PRINTF_DEBUG(" 0x%02X", buff[i]);
	}
	PRINTF_DEBUG("\r\n\r\n");
}

void sector_backup(uint32_t Bytes_Address) { // Read(03h)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0C);
	flash_spi((Bytes_Address & 0xFF000000) >> 24);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	for (int i = 0; i < 4096; i++) {
		Flash_buf[i] = flash_spi(0xA5);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
}

void flash_read_byte(uint8_t *pBuffer, uint32_t Bytes_Address) {
	// Fast Read
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((Bytes_Address & 0xFF0000) >> 16);
	flash_spi((Bytes_Address & 0xFF00) >> 8);
	flash_spi(Bytes_Address & 0xFF);
	flash_spi(0); // DUMMY
	*pBuffer = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);

	PRINTF_DEBUG("Fast_read(0Bh) : 0x%02X\n", *pBuffer);
}

void flash_read_bytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) {
	// Fast Read
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((ReadAddr & 0xFF0000) >> 16);
	flash_spi((ReadAddr & 0xFF00) >> 8);
	flash_spi(ReadAddr & 0xFF);
	flash_spi(0); // DUMMY
	HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead, 2000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_read_page(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize) {
	if ((NumByteToRead_up_to_PageSize > PageSize) || (NumByteToRead_up_to_PageSize == 0)) NumByteToRead_up_to_PageSize = PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > PageSize) NumByteToRead_up_to_PageSize = PageSize - OffsetInByte;

	Page_Address = Page_Address * PageSize + OffsetInByte;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x0B);
	flash_spi((Page_Address & 0xFF0000) >> 16);
	flash_spi((Page_Address & 0xFF00) >> 8);
	flash_spi(Page_Address & 0xFF);
	flash_spi(0);
	HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead_up_to_PageSize, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	HAL_Delay(1);
}

void flash_read_sector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize) {
	if ((NumByteToRead_up_to_SectorSize > SectorSize) || (NumByteToRead_up_to_SectorSize == 0)) NumByteToRead_up_to_SectorSize = SectorSize;

	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > SectorSize) BytesToRead = SectorSize - OffsetInByte;
	else BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = flash_SectorToPage(Sector_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_read_page(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

void flash_read_block(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize) {
	if ((NumByteToRead_up_to_BlockSize > BlockSize) || (NumByteToRead_up_to_BlockSize == 0)) NumByteToRead_up_to_BlockSize = BlockSize;

	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > BlockSize) BytesToRead = BlockSize - OffsetInByte;
	else BytesToRead = NumByteToRead_up_to_BlockSize;
	StartPage = flash_BlockToPage(Block_Address) + (OffsetInByte / PageSize);
	LocalOffset = OffsetInByte % PageSize;
	do {
		flash_read_page(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= PageSize - LocalOffset;
		pBuffer += PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

uint8_t flash_read_Status(uint8_t register_num) { // Read Status Register (05h - 1) (35h - 2)
	uint8_t status = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	if (register_num == 1) {
		flash_spi(0x05);
		status = flash_spi(0xA5);
	}
	else if (register_num == 2) {
		flash_spi(0x35);
		status = flash_spi(0xA5);
	}
	else if (register_num == 3) {
		flash_spi(0x15);
		status = flash_spi(0xA5);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	return status;
}

void flash_erase_chip(void) { //Chip Erase
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0xC7);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(10);
}

void flash_erase_sector(uint32_t SectorAddr) { //Sector Erase (4KB)
	flash_wait_end();
//	SectorAddr = SectorAddr * SectorSize;
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0x21);
	flash_spi((SectorAddr & 0xFF000000) >> 24);
	flash_spi((SectorAddr & 0xFF0000) >> 16);
	flash_spi((SectorAddr & 0xFF00) >> 8);
	flash_spi(SectorAddr & 0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

void flash_erase_block(uint32_t BlockAddr) {
	flash_wait_end();
	BlockAddr = BlockAddr * SectorSize * 16;
	flash_write_enable();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // CS to low
	flash_spi(0xD8);
	flash_spi((BlockAddr & 0xFF0000) >> 16);
	flash_spi((BlockAddr & 0xFF00) >> 8);
	flash_spi(BlockAddr & 0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
	flash_wait_end();
	HAL_Delay(1);
}

uint32_t flash_read_ID(void) { //JEDEC ID
	uint32_t Temp0 = 0, Temp1 = 0, Temp2 = 0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	flash_spi(0x9F);
	Temp0 = flash_spi(0xA5);
	Temp1 = flash_spi(0xA5);
	Temp2 = flash_spi(0xA5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}

void flash_read_uniqID(void) { //Read Unique ID
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	flash_spi(0x4B);
	for (uint8_t i = 0; i < 4; i++)
		flash_spi(0x4B);
	for (uint8_t i = 0; i < 8; i++)
		uniqID[i] = flash_spi(0x4B);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

	for (uint8_t i = 0; i < 8; i++)
		PRINTF_DEBUG("%02X ", uniqID[i]);
	PRINTF_DEBUG("\n");
}

void save_flash(uint32_t hex1, uint64_t hex2) {
	uint32_t PAGEError = 0;
	static FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = START_ADDR;
	EraseInitStruct.NbPages = (END_ADDR - START_ADDR) / 1024;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t*) hex1, (uint64_t*) hex2); // 32bit

	HAL_FLASH_Lock();
	PRINTF_DEBUG("Address : 0x%08x --> Hex Value : %08x\r\n\r\n", (uint32_t*) hex1, (uint64_t*) hex2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {

	}
}

void count_sub(void) {
	count++; // 1ms
}

void fnd_sub(void) {

	if (mode == 2) {
		cnt_num = 0;
		digit ^= 1;
		printnum(cnt_num);
		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {

				if (blk_cnt == 100) {
					blk_cnt = 0;
				}
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		}
		else if (mode_key1 == 0) {
			FND_CLR();
		}

	}

	else if (mode == 3) {
		cnt_num++;
		if (cnt_num > 9999) cnt_num = 0;
		digit ^= 1;
		digit_10 = cnt_num / 1000;
		digit_1 = (cnt_num % 1000) / 100;

		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {

				if (blk_cnt == 100) {
					blk_cnt = 0;
				}

				switch (digit) {
				case 0:
					printnum(digit_10);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
					break;
				case 1:
					printnum(digit_1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
					break;
				}
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		}
		else if (mode_key1 == 0) {
			switch (digit) {
			case 0:
				printnum(digit_10);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
				break;
			case 1:
				printnum(digit_1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
				break;
			}
		}
	}

	else if (mode == 4) {
		digit_10 = cnt_num / 1000;
		digit_1 = (cnt_num % 1000) / 100;
		digit ^= 1;

		if (mode_key1 == 1) {
			blk_cnt++;
			if (blk_cnt > 50) {
				if (blk_cnt == 100) {
					blk_cnt = 0;
				}

				switch (digit) {
				case 0:
					printnum(digit_10);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
					break;
				case 1:
					printnum(digit_1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
					break;
				}
			}
			if (blk_cnt < 50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
			}
		}
		else if (mode_key1 == 0) {
			switch (digit) {
			case 0:
				printnum(digit_10);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // COM2
				break;
			case 1:
				printnum(digit_1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // COM1 1 ON, 0 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2
				break;
			}
		}

	}
}

void FND_CLR() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // COM1 1 ON, 0 OFF
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // COM2

}

void printnum(uint8_t num) {
	switch (num) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // A 1 OFF, 0 ON -> anode
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0); // B
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // C
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // E
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // F
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // G
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // DP
		break;
	}
}

void use_console() {

	if (ring_buf_pop(&rbuf, &indata)) {
		if (indata == '\r' || indata == '\n') {
			use_console_start = (TIM3->CNT) + count * 1000; //start count

			rx_frame[rx_frame_cnt] = 0;
			frame1 = strtok_r(rx_frame, " ", &next_ptr);
			frame2 = strtok_r(NULL, " ", &next_ptr);
			frame3 = strtok_r(NULL, " ", &next_ptr);

			if (!strcmp(frame1, "START") || !strcmp(frame1, "start")) {
				if (mode == 4) {
					mode = 3;
					PRINTF_DEBUG(">>>> START\r\n");
				}
				else if (mode == 3) {

				}
				else {
					mode = 3;
					cnt_num = 0;
					PRINTF_DEBUG(">>>> START\r\n");
				}

			}
			else if (!strcmp(frame1, "CLR") || !strcmp(frame1, "clr")) {
				mode = 2;
				cnt_num = 0;
				PRINTF_DEBUG(">>>> CLEAR\r\n");
			}
			else if (!strcmp(frame1, "STOP") || !strcmp(frame1, "stop")) {
				if (mode == 3) {
					mode = 4;
					PRINTF_DEBUG(">>>> STOP\r\n");
				}
				else if (mode == 4) {

				}
				else if (mode == 2) {

				}
			}
			else if (!strcmp(frame1, "MEMRD") || !strcmp(frame1, "memrd")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					if (hex1 >= ADDR_FLASH_PAGE_0 //&& hex1 <= ADDR_FLASH_PAGE_127_END
					) {
						PRINTF_DEBUG("read value : 0x%02x", (*(uint32_t*) hex1 & 0xFF000000) >> 24);
						PRINTF_DEBUG(" 0x%02x", (*(uint32_t*) hex1 & 0xFF0000) >> 16);
						PRINTF_DEBUG(" 0x%02x", (*(uint32_t*) hex1 & 0xFF00) >> 8);
						PRINTF_DEBUG(" 0x%02x\r\n", *(uint32_t*) hex1 & 0xFF);
					}
					else {
						PRINTF_DEBUG("Segmentation Fault\r\n\r\n");
					}
				}
			}
			else if (!strcmp(frame1, "MEMWT") || !strcmp(frame1, "memwt")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Hex-Value : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);
					if (hex1 >= ADDR_FLASH_PAGE_0 //&& hex1 <= ADDR_FLASH_PAGE_127_END
					) {
						save_flash(hex1, hex2);
					}
				}
			}
			else if (!strcmp(frame1, "flashwt") || !strcmp(frame1, "FLASHWT")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Value : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex2 = StringToHexa(frame2);
					hex0 = StringToHexa(frame3);
					flash_write_byte(hex0, hex2);
					flash_read(hex2);
				}
			}
			else if (!strcmp(frame1, "flashrd") || !strcmp(frame1, "FLASHRD")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex2 = StringToHexa(frame2);
					flash_read(hex2);
				}
			}
			else if (!strcmp(frame1, "flash9fh") || !strcmp(frame1, "FLASH9FH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				Temp = flash_read_ID();
				PRINTF_DEBUG("JEDEC ID : %X\r\n", Temp);

			}
			else if (!strcmp(frame1, "flash4bh") || !strcmp(frame1, "FLASH4BH")) {
				PRINTF_DEBUG("Command : %s", frame1);
				PRINTF_DEBUG("\r\n");
				flash_read_uniqID();

			}
			else if (!strcmp(frame1, "flashrdrg") || !strcmp(frame1, "FLASHRDRG")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Rgst : %s", frame2);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);
					if (!strcmp(frame2, "1") || !strcmp(frame2, "1")) {
						Stts_rgstr = flash_read_Status(1);
						PRINTF_DEBUG("sr1 : %X\r\n", Stts_rgstr);
					}
					else if (!strcmp(frame2, "2") || !strcmp(frame2, "2")) {
						Stts_rgstr = flash_read_Status(2);
						PRINTF_DEBUG("sr2 : %X\r\n", Stts_rgstr);
					}
					else if (!strcmp(frame2, "3") || !strcmp(frame2, "3")) {
						Stts_rgstr = flash_read_Status(3);
						PRINTF_DEBUG("sr3 : %X\r\n", Stts_rgstr);
					}
				}
			}
			else if (!strcmp(frame1, "flashcp") || !strcmp(frame1, "FLASHCP")) {
				if (frame2 != NULL) {
					PRINTF_DEBUG("Command : %s", frame1);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Source Address : %s", frame2);
					PRINTF_DEBUG("\r\n");
					PRINTF_DEBUG("Flash Address : %s", frame3);
					PRINTF_DEBUG("\r\n");
					hex1 = StringToHexa(frame2);
					hex2 = StringToHexa(frame3);

					Source_buf[0] = ((*(uint32_t*) hex1 & 0xFF000000) >> 24);
					Source_buf[1] = ((*(uint32_t*) hex1 & 0xFF0000) >> 16);
					Source_buf[2] = ((*(uint32_t*) hex1 & 0xFF00) >> 8);
					Source_buf[3] = ((*(uint32_t*) hex1 & 0xFF));
					flash_point = (hex2 & 0x000FFFFF);

					flash_add = GetSector(hex2); //sector explorer
					sector_backup(flash_add); // sector backup

					for (int i = 0; i < 4; i++) {
						if (flash_point >= 4096) {
							flash_point = flash_point % 4096;
						}
						Flash_buf[flash_point + i] = Source_buf[i];
					}

					flash_erase_sector(flash_add);
					for (flag = 0; flag < 16; flag++) {
						sector_write_byte(flash_add + flag * 256);
					}
					//+ ram memory equals flash memory comparing
					/*
					 * if(flahsread ==
					 */
					flash_read(hex2);

				}

			}
			else if (!strcmp(frame1, "flashclr") || !strcmp(frame1, "FLASHCLR")) {
				flash_erase_chip();
				PRINTF_DEBUG("---CHIP ERASE---");
			}

//
//		else {
//			PRINTF_DEBUG("Invalid Command\r\n\r\n");
//		}
			buf_clear(&rbuf);
			ring_buf_clr(&rbuf);
			memset(rx_frame, 0, RX_FRAME_MAX);
			rx_frame_cnt = 0;

			use_console_end = (TIM3->CNT) + count * 1000; //end count
			use_console_run = (double) (use_console_end - use_console_start);
			PRINTF_DEBUG("\r\nuse_console running time : %.2fus", use_console_run);
			PRINTF_DEBUG("\r\n\r\n");
			use_console_start = 0, use_console_end = 0, use_console_run = 0;
		}
		else {
			if (rx_frame_cnt == 0) {
				buf_clear(&rbuf);

			}
			rx_frame[rx_frame_cnt++] = indata;
			if (rx_frame_cnt >= RX_FRAME_MAX) {
				buf_clear(&rbuf);
				memset(rx_frame, 0, RX_FRAME_MAX);
				rx_frame_cnt = 0;
			}
		}
	}
}

uint8_t read_pin() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != '\0') {
			mode_key1 ^= 1;
			PRINTF_DEBUG(">>>> SELECT MODE 1\r\n");
		}
		return 0;
	}
	else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == '\0') {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) != '\0') {
			mode = 2;
			PRINTF_DEBUG(">>>> SELECT MODE 2\r\n");
		}
		return 0;
	}
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == '\0' && check == 0) {
		HAL_Delay(30);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != '\0') {
			if (mode == 4) {
				mode = 3;
				PRINTF_DEBUG(">>>> SELECT MODE 3\r\n");
			}
			else if (mode == 3) {
				mode = 4;
				PRINTF_DEBUG(">>>> SELECT MODE 4\r\n");
			}
			else {
				mode = 3;
				cnt_num = 0;
				PRINTF_DEBUG(">>>> SELECT MODE 3\r\n");
			}
			return 0;
		}
	}
	return 1;
}

static u_int32_t StringToHexa(const char *frame2) {
	uint32_t hex = 0;
	int count = strlen(frame2), i = 0;

	for (i = 0; i < count; i++) {
		if (*frame2 >= '0' && *frame2 <= '9') hex = hex * 16 + *frame2 - '0';
		else if (*frame2 >= 'A' && *frame2 <= 'F') hex = hex * 16 + *frame2 - 'A' + 10;
		else if (*frame2 >= 'a' && *frame2 <= 'f') hex = hex * 16 + *frame2 - 'a' + 10;
		frame2++;
	}

	return hex;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		ring_buf_push(&rbuf, rcv_data);
		HAL_UART_Transmit(&huart2, &rcv_data, 1, 100);
		HAL_UART_Receive_IT(&huart2, &rcv_data, 1);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi1.Instance) {

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	HAL_NVIC_EnableIRQ(USART2_IRQn);
//	HAL_SPI_MspInit(&hspi1);
	ring_buf_init(&rbuf, rx_buffer, RX_FRAME_MAX);
	ring_buf_clr(&rbuf);
	HAL_UART_Receive_IT(&huart2, &rcv_data, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
//	flash_erase_sector(1);
//	flash_write_sector(buffer1, 1, 0, 2048);
//	flash_read_sector(buffer2, 1, 0, 2048);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		use_console();
		if (read_pin() == '\0') {
			read_pin();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 640 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 64;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Flash_SS_GPIO_Port, Flash_SS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	SEG_A_Pin | SEG_B_Pin | SEG_F_Pin | SEG_G_Pin | SEG_DP_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BT_2_Pin */
	GPIO_InitStruct.Pin = BT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BT_2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY1_Pin */
	GPIO_InitStruct.Pin = KEY1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Flash_SS_Pin */
	GPIO_InitStruct.Pin = Flash_SS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Flash_SS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_F_Pin SEG_G_Pin
	 SEG_DP_Pin SEG_C_Pin SEG_D_Pin SEG_E_Pin */
	GPIO_InitStruct.Pin = SEG_A_Pin | SEG_B_Pin | SEG_F_Pin | SEG_G_Pin | SEG_DP_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : FND_COM1_Pin FND_COM2_Pin */
	GPIO_InitStruct.Pin = FND_COM1_Pin | FND_COM2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY2_Pin */
	GPIO_InitStruct.Pin = KEY2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
