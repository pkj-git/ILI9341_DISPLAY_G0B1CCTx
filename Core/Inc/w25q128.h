/*
 * w25q128.h
 *
 *  Created on: Dec 7, 2025
 *      Author: jainp
 */

#ifndef INC_W25Q128_H_
#define INC_W25Q128_H_
#include <stdint.h>

#define MEMORY_FLASH_SIZE 0x01000000
#define MEMORY_PAGE_SIZE 0x100
#define MEMORY_SECTOR_SIZE 0x00001000
#define MEMORY_START_ADDRESS 0x90000000

void W25Q_Reset (void);

uint32_t W25Q_ReadID (void);

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_Read_Bytes(uint32_t mem_offset, uint32_t size, uint8_t *rData);

void W25Q_Erase_Sector (uint16_t numsector);

void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

void W25Q_Erase_Chip (void);

void DataReader_WaitForReceiveDone();
void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length);
void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length);

#endif /* INC_W25Q128_H_ */
