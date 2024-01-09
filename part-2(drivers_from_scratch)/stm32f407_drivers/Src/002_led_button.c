/*
 * 002_led_button.c
 *
 *  Created on: 09-Oct-2023
 *      Author: suraj.sd
 */

#include <stm32f407xx.h>

#define HIGH        1
#define BTN_PRESSED HIGH
void delay(void)
{
	for(uint32_t i = 0;i<500000/2;i++);
}


int main(void)
{
	GPIO_Handle_t GPIO_led,GPIO_btn;
	GPIO_led.pGPIOx=GPIOD;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;    //push pull configuration
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;     //if we are not using any external push pull up or down resistors.

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIO_led);

	GPIO_led.pGPIOx=GPIOA;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIOA_PCLK_EN();

	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_btn);



	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
}

