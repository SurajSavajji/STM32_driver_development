/*
 * 005_button_Pressed.c
 *
 *  Created on: 13-Oct-2023
 *      Author: suraj.sd
 */


#include <stm32f407xx.h>
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0;i<500000;i++);
}

int main()
{
	GPIO_Handle_t GPIO_led,GPIO_btn;


	memset(&GPIO_led,0,sizeof(GPIO_led));
	memset(&GPIO_btn,0,sizeof(GPIO_btn));



	GPIO_led.pGPIOx=GPIOD;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;    //push pull configuration
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;     //if we are not using any internal pull up or pull down resistors.

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIO_led);

	GPIO_btn.pGPIOx=GPIOD;
	GPIO_btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIO_btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIO_btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;


	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_btn);

	//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12, GPIO_PIN_RESET);
    GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5 ,NVIC_IRQ_PRI15);

	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
	/*{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}*/


}

void EXTI9_5_IRQHandler()
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

