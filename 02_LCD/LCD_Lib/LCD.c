#include "LCD.h"

LCD myLCD;

void LCD_init(void) {
	HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.RW, 0);
	HAL_Delay(50);
	LCD_Write_cmd(0x38);	// Function set (8-bit interface, 2 lines, 5*7 Pixels)
	HAL_Delay(5);
	LCD_Write_cmd(0x0C);	// Make cursor invisible
	HAL_Delay(5);
	LCD_Write_cmd(0x01);	// Clear Screen
	HAL_Delay(5);
	LCD_Write_cmd(0x06);	// Text L->R
	HAL_Delay(5);
};

void LCD_Print(const char *str) {
	while(*str) {
		LCD_Write_data(*str++);
	}
}

void LCD_Write_Byte(uint8_t data, uint8_t mode) {
	if (mode == 0x01) {
		//data
		HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.RS, 1);	// RS=1
	} else if (mode == 0x00) {
		//cm
		HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.RS, 0);	// RS=0
	}
	
	// D0-D7
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D0, (data & 0x01)?1:0);	// 0000 0001
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D1, (data & 0x02)?1:0);	// 0000 0010
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D2, (data & 0x04)?1:0);	// 0000 0100
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D3, (data & 0x08)?1:0);	// 0000 1000
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D4, (data & 0x10)?1:0);	// 0001 0000
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D5, (data & 0x20)?1:0);	// 0010 0000
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D6, (data & 0x40)?1:0);	// 0100 0000
	HAL_GPIO_WritePin(myLCD.PORT_DATA, myLCD.D7, (data & 0x80)?1:0);	// 1000 0000
	
	// Enable
	HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.E, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.E, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(myLCD.PORT_R, myLCD.E, 0);
	HAL_Delay(1);
}

void LCD_Write_data(uint8_t data) {
	LCD_Write_Byte(data, 1);
}
 
void LCD_Write_cmd(uint8_t data) {
	LCD_Write_Byte(data, 0);
}