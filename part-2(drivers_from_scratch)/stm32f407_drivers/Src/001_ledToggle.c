/*
 * 001_ledToggle.c
 *
 *  Created on: 06-Oct-2023
 *      Author: suraj.sd
 */

#include <stm32f407xx.h>

void delay(void)
{
	for(uint32_t i = 0;i<1000000;i++);
}


int main(void)
{
	GPIO_Handle_t GPIO_led;
	GPIO_led.pGPIOx=GPIOD;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIOD_PCLK_EN();
	GPIO_Init(&GPIO_led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
}
