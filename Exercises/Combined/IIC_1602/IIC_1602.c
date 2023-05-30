/*
 * IIC_1602.c
 *
 *  Created on: Mar 9, 2020
 *      Author: User
 */
#include "IIC_1602.h"
I2C_HandleTypeDef i2c_port;
void Delay_us( uint32_t n ) //撱嗆���凝蝘�撠梯撓�憭���
{
	uint32_t temp1 = SysTick->LOAD, temp2 = SysTick->CTRL;
	SysTick->LOAD = HAL_RCC_GetHCLKFreq() / 8000000 * n;
	SysTick->VAL = SysTick->LOAD;
	SysTick->CTRL = 1;     //等待計數到0
	while (!( SysTick->CTRL & (1 << 16))); //SysTick ->CTRL&SysTick_CTRL_COUNTFLAG_Msk
	SysTick->LOAD = temp1;SysTick->CTRL=temp2;
}
void writec(uint8_t cmd)
{
	uint8_t txbuf[4],up=cmd&0xf0,lo=(cmd<<4)&0xf0;
	txbuf[0]=up|BLACKLIGHT|E;
	txbuf[1]=up|BLACKLIGHT;
	txbuf[2]=lo|BLACKLIGHT|E;
	txbuf[3]=lo|BLACKLIGHT;
	HAL_I2C_Master_Transmit(&i2c_port, PCF8574T, txbuf, 4, 10);
	//Delay_us(100);
}
void writed(uint8_t cmd)
{
	uint8_t txbuf[4],up=cmd&0xf0,lo=(cmd<<4)&0xf0;
	txbuf[0]=up|BLACKLIGHT|RS|E;
	txbuf[1]=up|BLACKLIGHT|RS;
	txbuf[2]=lo|BLACKLIGHT|RS|E;
	txbuf[3]=lo|BLACKLIGHT|RS;
	HAL_I2C_Master_Transmit(&i2c_port, PCF8574T, txbuf, 4, 10);
	//Delay_us(100);
}
void writes(unsigned char command,char *string)
{
	writec(command);
	while(*string!='\0'){
		writed(*string);
		string++;
	}
}
void Initialize_LCD(I2C_HandleTypeDef i2c)
{
	i2c_port=i2c;
	HAL_Delay(50);
	writec(0x3);HAL_Delay(5);
	writec(0x3);writec(0x3);
	writec(0x2);
	writec(0x28);
	writec(0x8);
	writec(0x1);
	HAL_Delay(3);
	writec(0x6);
	writec(0xc);
}
void clear_LCD(void)
{
	char str[]="                ";
	writes(0x80,str);
	writes(0xc0,str);
	writec(0x80);
}
