/*
 * w25q128.c
 *
 *  Created on: Dec 7, 2025
 *      Author: jainp
 */


#include "main.h"
#include "w25q128.h"

extern SPI_HandleTypeDef hspi2;
#define W25Q_SPI hspi2
#define W25Q_Delay(time) HAL_Delay(time)
#define numBLOCK 256  // number of total blocks for 16MB flash

void csLOW (void)
{
	HAL_GPIO_WritePin (NSS_Flash_GPIO_Port, NSS_Flash_Pin, GPIO_PIN_RESET);
}

void csHIGH (void)
{
	HAL_GPIO_WritePin (NSS_Flash_GPIO_Port, NSS_Flash_Pin, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

/**************************************************************************************************/

void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	W25Q_Delay(100);
}

uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9F;  // Read JEDEC ID
	uint8_t rData[3];
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);
	csHIGH();
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 5);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}

void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x0C;  // Fast Read with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
		tData[5] = 0;  // Dummy clock
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 5);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 6);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}

void write_enable (void)
{
	uint8_t tData = 0x06;  // enable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  // 5ms delay
}

void write_disable(void)
{
	uint8_t tData = 0x04;  // disable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  // 5ms delay
}

uint32_t bytestowrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}

void W25Q_Erase_Sector (uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector*16*256;   // Each sector contains 16 pages * 256 bytes

	write_enable();

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x20;  // Erase sector
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address

		csLOW();
		SPI_Write(tData, 4);
		csHIGH();
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x21;  // ERASE Sector with 32bit address
		tData[1] = (memAddr>>24)&0xFF;
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = memAddr&0xFF;

		csLOW();  // pull the CS LOW
		SPI_Write(tData, 5);
		csHIGH();  // pull the HIGH
	}

	W25Q_Delay(450);  // 450ms delay for sector erase

	write_disable();

}


void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

	uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;
	for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}

	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);
		uint32_t indx = 0;

		write_enable();

		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

		else // we use 32bit memory address for chips >= 256Mb
		{
			tData[0] = 0x12;  // page program with 4-Byte Address
			tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;
			tData[4] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 5;
		}

		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		if (bytestosend > 200)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytestosend-100);
			csHIGH();
		}

		else
		{
			csLOW();
			SPI_Write(tData, bytestosend);
			csHIGH();
		}

		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

		W25Q_Delay(5);
		write_disable();

	}
}

	void W25Q_Erase_Chip ()
	{
		uint8_t tData = 0xC7;  // Bulk Erase instruction (0xC7 or 0x60)

	    // 1. Enable Write Operation
		write_enable();

	    // 2. Send the Bulk Erase Instruction
		csLOW();
		SPI_Write(&tData, 1);
		csHIGH();

	    // 3. Wait for the Erase Cycle to Complete
	    // A bulk erase is a very long operation.
	    // The W25Q128FV datasheet suggests a typical time of 50 seconds.
	    // We will use a conservative delay. A robust driver would poll the Status Register (0x05)
	    // to check the Write In Progress (WIP) bit (Bit 0).

		W25Q_Delay(60000);  // 60,000ms = 60 seconds (Using a large delay for simplicity)

	    // 4. Disable Write Operation
		write_disable();
	}

	void W25Q_Read_Bytes(uint32_t mem_offset, uint32_t size, uint8_t *rData) {
	    uint32_t startPage = mem_offset / MEMORY_PAGE_SIZE;
	    uint8_t offset = mem_offset % MEMORY_PAGE_SIZE;

	    // Using W25Q_Read as a simpler interface, but W25Q_FastRead is usually better.
	    // Assuming size does not cross multiple pages for simplicity, but W25Q_Read
	    // implementation in w25q128.c handles multi-page reads implicitly.
	    W25Q_Read(startPage, offset, size, rData);
	}
	/**
	 * @brief Reads data from W25Q flash memory across potential page boundaries.
	 * * @param mem_offset The starting linear address (byte index) in the memory.
	 * @param size The total number of bytes to read.
	 * @param rData Pointer to the buffer where the read data will be stored.
	 */
	void W25Q_Read_Bytes_MultiPage(uint32_t mem_offset, uint32_t size, uint8_t *rData) {
	    // Current linear address (used for calculation and tracking)
	    uint32_t current_addr = mem_offset;
	    // Remaining bytes to read
	    uint32_t bytes_remaining = size;
	    // Pointer to the current position in the destination buffer
	    uint8_t *current_rData = rData;

	    // --- 1. Handle the first partial page (if the start is not page-aligned) ---

	    // Calculate the page and offset for the start of the read
	    uint32_t startPage = current_addr / MEMORY_PAGE_SIZE;
	    uint8_t offset_in_page = current_addr % MEMORY_PAGE_SIZE;

	    // Bytes available in the first page starting from the offset
	    uint32_t bytes_left_in_first_page = MEMORY_PAGE_SIZE - offset_in_page;

	    // The amount to read in this first operation:
	    // It's the smaller of bytes_remaining and the available space in the current page.
	    uint32_t read_len = (bytes_remaining < bytes_left_in_first_page) ?
	                        bytes_remaining : bytes_left_in_first_page;

	    if (read_len > 0) {
	        // Use the specified interface: W25Q_Read(startPage, offset, size, rData)
	        //
	        W25Q_Read(startPage, offset_in_page, read_len, current_rData);

	        // Update tracking variables
	        current_addr += read_len;
	        current_rData += read_len;
	        bytes_remaining -= read_len;
	    }

	    // --- 2. Handle the full middle pages (now page-aligned) ---

	    // The current address is now either page-aligned or the read is complete.
	    while (bytes_remaining >= MEMORY_PAGE_SIZE) {
	        read_len = MEMORY_PAGE_SIZE;
	        startPage = current_addr / MEMORY_PAGE_SIZE;
	        // Since we are now reading full pages, the offset is always 0
	        offset_in_page = 0;

	        // Read the full page
	        W25Q_Read(startPage, offset_in_page, read_len, current_rData);

	        // Update tracking variables
	        current_addr += read_len;
	        current_rData += read_len;
	        bytes_remaining -= read_len;
	    }

	    // --- 3. Handle the last partial page (if any bytes remain) ---

	    if (bytes_remaining > 0) {
	        read_len = bytes_remaining;
	        startPage = current_addr / MEMORY_PAGE_SIZE;
	        // The last read will be page-aligned from the start, so offset is 0
	        offset_in_page = 0;

	        // Read the remaining bytes
	        W25Q_Read(startPage, offset_in_page, read_len, current_rData);

	        // No further updates needed
	    }
	}

   void DataReader_WaitForReceiveDone()
    {
      return;
    }

    void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
    {
    	W25Q_Read_Bytes_MultiPage((address24 - MEMORY_START_ADDRESS), length, buffer);
    }

    void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
    {
    	W25Q_Read_Bytes_MultiPage((address24 - MEMORY_START_ADDRESS), length, buffer);
    }
