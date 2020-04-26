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
xSemaphoreHandle xTrainSemaphore;

/*
 *RED light is 0
 *Green light is 1
*/

int globalStateOfNorthAndSouth = 0;
int globalStateOfEastAndWest = 0;

void delayMS(int delayTimeMs)
{
	// 1 clock cycle = (1 / 120000000) second
	// 1 SysCtlDelay = 3 clock cycle (from the docs) = (3 / 120000000) second
	// 1 second = (120000000 / 3)
	// 1 ms = (120000000 / 3 / 1000)
	SysCtlDelay(delayTimeMs * (120000000 / 3 / 1000));
}

void TrainCrossingHandler()
{
	//give semaphore to run the handler with context switching
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
	globalStateOfNorthAndSouth = state;
}

void setEastAndWest(int state)
{
	GPIOPinWrite(FourCrossingsLEDsPort, East, state);
	GPIOPinWrite(FourCrossingsLEDsPort, West, state);
	globalStateOfEastAndWest = state;
}
void normalMode(void *pvParameters)
{
	setNorthAndSouth(1);
	setEastAndWest(0);

	//busy waiting
	delayMS(tgn_tgs);

	//to make this task blocked to check if the pedestrians' buttons are clicked to run pedestrianMode
	taskYIELD();

	setNorthAndSouth(0);
	setEastAndWest(1);

	delayMS(tge_tgw);

	//to make this task blocked to check if the pedestrians' buttons are clicked to run pedestrianMode
	taskYIELD();
}

void pedestrianMode(void *pvParameters)
{
	xSemaphoreTake(xPedestrianSemaphore, portMAX_DELAY);

	setNorthAndSouth(0);
	setEastAndWest(0);

	delayMS(tcross);
}

void trainMode(void *pvParameters)
{
	xSemaphoreTake(xTrainSemaphore, portMAX_DELAY);

	int tempNS = globalStateOfNorthAndSouth;
	int tempES = globalStateOfEastAndWest;

	setNorthAndSouth(0);
	setEastAndWest(0);

	//turn on red LED and run the siren
	GPIOPinWrite(TrainPort, GateLEDAndSiren, 1);

	delayMS(tsafety);

	//turn off red LED and run the siren
	GPIOPinWrite(TrainPort, GateLEDAndSiren, 0);

	setNorthAndSouth(tempNS);
	setEastAndWest(tempES);
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
	SysCtlClockFreqSet((SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN), 120000000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}

void enableInterrupts()
{

	IntMasterEnable();

	GPIOIntEnable(FourCrossingsPedestrianPort, (GPIO_INT_PIN_0 | GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | GPIO_INT_PIN_3));

	GPIOIntEnable(TrainPort, (GPIO_INT_PIN_0 | GPIO_INT_PIN_1));
}

int main(void)
{
	sysInit();

	enableInterrupts();

	setupOutputPins();

	setupInputPins();

	vSemaphoreCreateBinary(xPedestrianSemaphore);

	if (xPedestrianSemaphore != NULL)
	{
		xTaskCreate(normalMode, "normal", 256, NULL, 1, NULL);
		xTaskCreate(pedestrianMode, "pedestrian", 256, NULL, 2, NULL);
		xTaskCreate(trainMode, "train", 256, NULL, 3, NULL);

		vTaskStartScheduler();
	}
	for (;;)
		;
}