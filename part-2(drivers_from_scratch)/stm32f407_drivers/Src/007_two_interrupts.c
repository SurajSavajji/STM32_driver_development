/*
 * 007_two_interrupts.c
 *
 *  Created on: 30-Oct-2023
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
	GPIO_Handle_t GPIO_btn,GPIO_btn2;


	memset(&GPIO_btn,0,sizeof(GPIO_btn));
	memset(&GPIO_btn2,0,sizeof(GPIO_btn2));


	GPIO_btn.pGPIOx=GPIOD;
	GPIO_btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIO_btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIO_btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_btn);

	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5 ,NVIC_IRQ_PRI15);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5, ENABLE);

	GPIO_btn2.pGPIOx=GPIOD;
	GPIO_btn2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_1;
	GPIO_btn2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIO_btn2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_btn2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;



	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_btn2);

	//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12, GPIO_PIN_RESET);



	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI1 ,NVIC_IRQ_PRI1);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI1, ENABLE);



	while(1);
	/*{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}*/


}

void EXTI9_5_IRQHandler()
{
    delay();
    int a=10;
    GPIO_IRQHandling(GPIO_PIN_NO_5);
}

void EXTI1_IRQHandler()
{
    delay();
    int b=10;
    GPIO_IRQHandling(GPIO_PIN_NO_1);
}
