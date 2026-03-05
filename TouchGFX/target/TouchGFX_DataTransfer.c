/* Functions called by the TouchGFX HAL to invoke the actual data transfer to ILI9341.
 * Pero, 2021
 */

#include "ili9341.h"
#include "TouchGFX_DataTransfer.h"

extern void DisplayDriver_TransferCompleteCallback();

static volatile uint8_t isTransmittingData = 0;

uint32_t touchgfxDisplayDriverTransmitActive(void)
{
	return isTransmittingData;
}

void touchgfxDisplayDriverTransmitBlock(uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	isTransmittingData = 1;
	ILI9341_SetWindow(x, y, x+w-1, y+h-1);
	ILI9341_DrawBitmap(w, h, pixels);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
		// Wait for the SPI hardware BSY flag to clear before signaling TouchGFX
		// This prevents HAL_BUSY errors in Ili9341_SetWindow's polling HAL_SPI_Transmit
		while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET) {}
		
		ILI9341_EndOfDrawBitmap();
		isTransmittingData = 0;
		DisplayDriver_TransferCompleteCallback();
	}
}
