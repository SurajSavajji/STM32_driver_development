#include <string.h>
#include "stm32f407xx.h"

#define LOW 0
#define BTN_PRESSED LOW


void delay(void)
{
	// this statement will introduce ~200ms delay when sysclk is 16MHz (RCC Oscillator)
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	// Led set up
	GPIO_Handle_t GpioLed, GpioButton;

 	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioButton));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	// Button set up

	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioButton);


	// IRQ configuration.
	GPIO_IRQPRConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOuputPin(GPIOD, GPIO_PIN_NO_12);
}
