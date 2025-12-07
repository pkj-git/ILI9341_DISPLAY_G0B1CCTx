/*
 * LCD_Touch.h
 *
 *  Created on: Jan 5, 2025
 *      Author: jainp
 */

#ifndef INC_LCD_TOUCH_H_
#define INC_LCD_TOUCH_H_

#include "main.h"

/* TouchScreen (TS) external IO functions */
 void     TS_IO_Init(void);
 void     TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
 uint8_t  TS_IO_Read(uint8_t Addr, uint8_t Reg);
 uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
 void     TS_IO_Delay(uint32_t Delay);

#endif /* INC_LCD_TOUCH_H_ */
