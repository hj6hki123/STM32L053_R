/*
 * IIC_1602.h
 *
 *  Created on: Mar 9, 2020
 *      Author: User
 */
#include "stm32l0xx_hal.h"
#ifndef IIC_1602_H_
#define IIC_1602_H_
#define PCF8574T (0x27<<1)
#define PCF8574A (0x3f<<1)
#define RS (1<<0)
#define E	(1<<2)
#define BLACKLIGHT	(1<<3)

void writed(unsigned char data);
void writec(unsigned char command);
void writes(unsigned char command,char *string);
void clear_LCD(void);
void Initialize_LCD(I2C_HandleTypeDef i2c);
void Delay_us( uint32_t n );
#endif /* IIC_1602_H_ */
