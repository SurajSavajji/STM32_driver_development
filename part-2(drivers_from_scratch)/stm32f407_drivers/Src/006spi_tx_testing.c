/*
 * 006spi_tx_testing.c
 *
 *  Created on: 27-Oct-2023
 *      Author: suraj.sd
 */
//ALternate fun mode - 5
//PB15 -- MOSI
//PB14 -- MISO
//PB13 -- SCLK
//PB12 -- NSS

#include <stm32f407xx.h>
#include <string.h>

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    memset(&SPIPins,0,sizeof(SPIPins));

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //SPIPins->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;            //NSS
    //GPIO_Init(&SPIPins);

    SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;            //SCLK
    GPIO_Init(&SPIPins);

    //SPIPins->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;            //MISO
    //GPIO_Init(&SPIPins);

    SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;            //MOSI
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	memset(&SPI2handle,0,sizeof(SPI2handle));

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;            //generates sclk of 8MZ
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;               //software slave management is enabled for nss pins

	SPI_Init(&SPI2handle);


}

int main()
{
/*
 * it is recommended to set spi peripheral after configuring all the control bits
 * after configuring all the control bits only we should enable the spi peripheral as it is mainly used for communication only
 * after enabling spi peripheral it will be busy in data communication
 */
	char user_data[] = "Hello World";
	//this function is used to initialize the gpio pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the spi peripheral parameters
	SPI2_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the spi2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	//lets confirm SPI busy flag is not set
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	//disable the spi2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}
