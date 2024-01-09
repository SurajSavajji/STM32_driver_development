/*
 * 003_led_button_ext.c
 *
 *  Created on: 10-Oct-2023
 *      Author: suraj.sd
 */


#include <stm32f407xx.h>

#define HIGH        1
#define LOW         0
#define BTN_PRESSED LOW
void delay(void)
{
	for(uint32_t i = 0;i<500000;i++);
}


int main(void)
{
	GPIO_Handle_t GPIO_led,GPIO_btn;
	GPIO_led.pGPIOx=GPIOA;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;    //push pull configuration
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;     //if we are not using any external push pull up or down resistors.

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIO_led);

	GPIO_led.pGPIOx=GPIOB;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
    //GPIO_led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;   //while doing circuit connection we connected external pull up resistor but here we are
	                                                          //enabling internal pull up resistor
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIOB_PCLK_EN();

	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_btn);



	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}
}

