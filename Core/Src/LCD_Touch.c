/*
 * LCD_Touch.c
 *
 *  Created on: Jan 5, 2025
 *      Author: jainp
 */


#include "LCD_Touch.h"

extern I2C_HandleTypeDef hi2c1;

void TS_IO_Init(void)
{
	HAL_GPIO_WritePin(Touch_reset_GPIO_Port, Touch_reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Touch_reset_GPIO_Port, Touch_reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_I2C_Mem_Write(&hi2c1, Addr, Reg, 1, &Value, 1, 100);
}

uint8_t  TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, &data, 1, 100);
	return data;
}

uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
	return (HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, Buffer, Length, 1000));
}

void TS_IO_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

