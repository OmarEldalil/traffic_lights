#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "basic_io.h"

#define tgn_tgs 5000
#define tge_tgw 2500
#define tcross 10000
#define tsafety 30000
#define North GPIO_PIN_0
#define South GPIO_PIN_1
#define East GPIO_PIN_2
#define West GPIO_PIN_3
#define FourCrossingsLEDsPort GPIO_PORTA_BASE
#define FourCrossingsPedestrianPort GPIO_PORTB_BASE

#define SesorLeft GPIO_PIN_0
#define SesorRight GPIO_PIN_1
#define GateLEDAndSiren GPIO_PIN_2
#define TrainPort GPIO_PORTF_BASE

#define GREEN_LED 0x08
#define RED_LED 0x02

xSemaphoreHandle xPedestrianSemaphore;
xSemaphoreHandle xPedestrianToNormalSemaphore;
xSemaphoreHandle xTrainSemaphore;
xSemaphoreHandle xTrainToNormalSemaphore;

/*
 *RED light is 0
 *Green light is 1
*/

void TrainCrossingHandler()
{
	//give semaphore to run the handler with NOOO context switching
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xTrainSemaphore, &xHigherPriorityTaskWoken);

	//clear the interupt
	uint32_t status;
	status = GPIOIntStatus(TrainPort, true);
	GPIOIntClear(TrainPort, status);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void PedestrianCrossingHandler()
{
	//give semaphore to run the handler with NOOO context switching
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xPedestrianSemaphore, &xHigherPriorityTaskWoken);

	//clear the interupt
	uint32_t status;
	status = GPIOIntStatus(FourCrossingsPedestrianPort, true);
	GPIOIntClear(FourCrossingsPedestrianPort, status);
}

void setNorthAndSouth(int state)
{
	GPIOPinWrite(FourCrossingsLEDsPort, North, state);
	GPIOPinWrite(FourCrossingsLEDsPort, South, state);
}

void setEastAndWest(int state)
{
	GPIOPinWrite(FourCrossingsLEDsPort, East, state);
	GPIOPinWrite(FourCrossingsLEDsPort, West, state);
}
void normalMode(void *pvParameters)
{
	while (xSemaphoreTake(xPedestrianToNormalSemaphore, 100) != pdTRUE)
		;
	xSemaphoreGive(xPedestrianToNormalSemaphore);

	while (xSemaphoreTake(xTrainToNormalSemaphore, 100) != pdTRUE)
		;
	xSemaphoreGive(xTrainToNormalSemaphore);

	setNorthAndSouth(1);
	setEastAndWest(0);
	vTaskDelay(tgn_tgs / portTICK_RATE_MS);
	setNorthAndSouth(0);
	setEastAndWest(1);
	vTaskDelay(tge_tgw / portTICK_RATE_MS);
}

void pedestrianMode(void *pvParameters)
{
	xSemaphoreTake(xPedestrianSemaphore, portMAX_DELAY);

	while (xSemaphoreTake(xTrainToNormalSemaphore, 100) != pdTRUE)
		;
	xSemaphoreGive(xTrainToNormalSemaphore);

	setNorthAndSouth(0);
	setEastAndWest(0);
	xSemaphoreTake(xPedestrianToNormalSemaphore, 0);
	vTaskDelay(tcross / portTICK_RATE_MS);
	xSemaphoreGive(xPedestrianToNormalSemaphore);
}

void trainMode(void *pvParameters)
{
	xSemaphoreTake(xTrainSemaphore, portMAX_DELAY);

	setNorthAndSouth(0);
	setEastAndWest(0);

	//turn on red LED and run the siren
	GPIOPinWrite(TrainPort, GateLEDAndSiren, 1);

	xSemaphoreTake(xTrainToNormalSemaphore, 0);
	vTaskDelay(tsafety / portTICK_RATE_MS);
	xSemaphoreGive(xTrainToNormalSemaphore);

	//turn off red LED and run the siren
	GPIOPinWrite(TrainPort, GateLEDAndSiren, 0);
}

void setupInputPins()
{
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, North);
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, South);
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, East);
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, West);
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, South);

	GPIOPinTypeGPIOInput(TrainPort, SesorLeft);
	GPIOPinTypeGPIOInput(TrainPort, SesorRight);
}

void setupOutputPins()
{

	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPort, North);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPort, South);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPort, East);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPort, West);

	GPIOPinTypeGPIOOutput(TrainPort, GateLEDAndSiren);
}

void sysInit()
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}
int main(void)
{
	sysInit();

	setupOutputPins();

	setupInputPins();

	vSemaphoreCreateBinary(xPedestrianSemaphore);

	//this is to make busy wait when pedestrians cross streets
	vSemaphoreCreateBinary(xPedestrianToNormalSemaphore);

	if (xPedestrianSemaphore != NULL && xPedestrianToNormalSemaphore != NULL)
	{
		xTaskCreate(normalMode, "normal", 256, NULL, 1, NULL);
		xTaskCreate(pedestrianMode, "pedestrian", 256, NULL, 1, NULL);
		xTaskCreate(trainMode, "train", 256, NULL, 1, NULL);

		vTaskStartScheduler();
	}
	for (;;)
		;
}