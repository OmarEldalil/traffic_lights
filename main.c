/****************************************************************************/
/*************************		standard includes 	***************************/
/****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/****************************************************************************/
/*************************		TivaWare includes 	***************************/
/****************************************************************************/
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"

#define PART_TM4C1294NCPDT
#include "driverlib/pin_map.h"

/****************************************************************************/
/*************************		FreeRTOS includes 	***************************/
/****************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "basic_io.h"

#define TGN_TGS 5000
#define TGE_TGW 2500
#define TCROSS 10000
#define tsafety 30000
#define TSAFETY 30

#define North GPIO_PIN_0
#define South GPIO_PIN_1
#define East GPIO_PIN_2
#define West GPIO_PIN_3
#define FourCrossingsLEDsPortRED GPIO_PORTL_BASE
#define FourCrossingsLEDsPortGREEN GPIO_PORTK_BASE
#define FourCrossingsPedestrianPort GPIO_PORTF_BASE
#define TrainGatePORT GPIO_PORTF_BASE

#define SesorLeft GPIO_PIN_0
#define SesorRight GPIO_PIN_1
#define GateLEDAndSiren GPIO_PIN_2
#define TrainPort GPIO_PORTJ_BASE

#define GREEN_LED 0x08
#define RED_LED 0x02

/****************************************************************************/
/****************semaphores to synchronize between tasks*********************/
/****************************************************************************/
xSemaphoreHandle xNormalSemaphore;
xSemaphoreHandle xPedestrianSemaphore;
xSemaphoreHandle xTrainSemaphore;

/****************************************************************************/
/*******************************System clock rate in Hz.*********************/
/****************************************************************************/
uint32_t g_ui32SysClock;


uint8_t globalStateOfNorthAndSouth = 0;
uint8_t globalStateOfEastAndWest = 0;

/****************************************************************************/
/******************					train handler flags					*********************/
/****************************************************************************/
uint32_t trainFlag = 0;
uint32_t trainSource = 0;

/****************************************************************************/
/******************					busy waiting delay					*********************/
/****************************************************************************/
void delayMS(int delayTimeMs)
{
	// 1 clock cycle = (1 / 120000000) second
	// 1 SysCtlDelay = 3 clock cycle (from the docs) = (3 / 120000000) second
	// 1 second = (120000000 / 3)
	// 1 ms = (120000000 / 3 / 1000)
	SysCtlDelay(delayTimeMs * (120000000 / 3 / 1000));
}

/****************************************************************************/
/******************					control red LEDS						*********************/
/****************************************************************************/
void setRED(int state)
{
	IntMasterDisable();
	GPIOPinWrite(FourCrossingsLEDsPortRED, North, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, South, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, East, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, West, state);
	IntMasterEnable();
}

/****************************************************************************/
/******************					control Green LEDS					*********************/
/****************************************************************************/
void setGREEN(uint8_t state)
{
	IntMasterDisable();
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, North, state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, South, state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, East, state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, West, state);
	IntMasterEnable();
}

/****************************************************************************/
/**********		control NorthAndSouth traffic LEDS				*********************/
/****************************************************************************/
void setNorthAndSouth(uint8_t state)
{
	IntMasterDisable();
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, North, state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, South, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, East, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, West, state);	
	
	GPIOPinWrite(FourCrossingsLEDsPortRED, North, !state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, South, !state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, East, !state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, West, !state);
	
	globalStateOfNorthAndSouth = state;
	IntMasterEnable();
}

/****************************************************************************/
/**********			control EastAndWest traffic LEDS				*********************/
/****************************************************************************/
void setEastAndWest(int state)
{
	IntMasterDisable();
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, East, state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, West, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, North, state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, South, state);	
	
	GPIOPinWrite(FourCrossingsLEDsPortRED, East, !state);
	GPIOPinWrite(FourCrossingsLEDsPortRED, West, !state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, North, !state);
	GPIOPinWrite(FourCrossingsLEDsPortGREEN, South, !state);
	
	globalStateOfEastAndWest = state;
	IntMasterEnable();
}

void setupInputPins()
{
	GPIOPinTypeGPIOInput(FourCrossingsPedestrianPort, GPIO_PIN_3);
	GPIOPadConfigSet(FourCrossingsPedestrianPort,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	GPIOPinTypeGPIOInput(TrainPort, SesorLeft);
	GPIOPinTypeGPIOInput(TrainPort, SesorRight);
	GPIOPadConfigSet(TrainPort ,SesorLeft,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(TrainPort ,SesorRight,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void setupOutputPins()
{

	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortRED, North);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortRED, South);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortRED, East);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortRED, West);
	
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortGREEN, North);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortGREEN, South);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortGREEN, East);
	GPIOPinTypeGPIOOutput(FourCrossingsLEDsPortGREEN, West);
	GPIOPinTypeGPIOOutput(TrainGatePORT, GateLEDAndSiren);

	//GPIOPinTypeGPIOOutput(TrainPort, GateLEDAndSiren);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
}

void sysInit()
{
	/****************************************************************************/
	/********Set the clocking to run directly from the crystal at 120MHz.********/
	/****************************************************************************/
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
											 SYSCTL_OSC_MAIN |
											 SYSCTL_USE_PLL |
											 SYSCTL_CFG_VCO_480), 120000000);
	
	/****************************************************************************/
	/******************			Enabling Peripherals clock									*********/
	/****************************************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	/****************************************************************************/
	/******************			Enabling Modules clock											*********/
	/****************************************************************************/
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);							/*devide PWM Clk Src by 64*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	/****************************************************************************/
	/******************			waiting till clocks are ready								*********/
	/****************************************************************************/	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
	/****************************************************************************/
	/**************		Loop while there are more characters to send			*********/
	/****************************************************************************/
	while(ui32Count--)
	{
		/**************************************************************************/
		/************				Write the next character to the UART.					*********/
		/**************************************************************************/
		UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
	}
}

void PWM_Init (void) 
{
	GPIOPinConfigure(GPIO_PF1_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 37500 ); //load value -> 120000000/64/50 -> clk/clk_div/hz , hz for servo = 50
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2400); // this number / load value = % of on , for servo -> between .05% and .1%-> 1875 , 3750
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void UART_Init(void)
{
	/****************************************************************************/
	/******************			Set GPIO A0 and A1 as UART pins.						*********/
	/****************************************************************************/
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	/****************************************************************************/
	/************			Configure the UART for 115,200, 8-N-1 operation.	*********/
	/****************************************************************************/
	UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
													(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
													 UART_CONFIG_PAR_NONE));

	/****************************************************************************/
	/******************			Enable the UART interrupt.									*********/
	/****************************************************************************/
	IntEnable(INT_UART0_TM4C129);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	/****************************************************************************/
	/******************					start text over UART										*********/
	/****************************************************************************/
	UARTSend((uint8_t *)"UART start, ", 12);
}
void timerStart(uint32_t seconds)
{
			TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT);
			TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock * seconds);
			TimerValueGet(TIMER0_BASE, TIMER_A);
			TimerEnable(TIMER0_BASE, TIMER_A);
}

void TrainCrossingHandler()
{
	//give semaphore to run the handler with context switching
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	uint32_t status=0;

	status = GPIOIntStatus(GPIO_PORTJ_BASE,true);
	GPIOIntClear(GPIO_PORTJ_BASE,status);

	if( (status & GPIO_INT_PIN_0) == GPIO_INT_PIN_0) //Then there was a pin0 interrupt
	{
		if(0 == trainFlag) //no train yet 
		{
			trainFlag = 1; //train mode is on
			trainSource = 0; // record the angle the train came from
			xSemaphoreGiveFromISR(xTrainSemaphore, &xHigherPriorityTaskWoken);
			timerStart(TSAFETY);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else //there is a train
		{
			if(0 == trainSource) //came from the same angle
			{
				//do nothing
			}
			else
			{
				if(1==(*(volatile uint32_t *)(TIMER0_BASE + 0x00C) && 1)) //tsafty isn't passed yet
				{
					//do nithing
				}
				else
				{
					trainFlag = 0; // train has passed
				}
			}
		}
	} 
	if( (status & GPIO_INT_PIN_1) == GPIO_INT_PIN_1) ///Then there was a pin1 interrupt
	{
		if(0 == trainFlag) //no train yet 
		{
			trainFlag = 1; // train mode on
			trainSource = 1; // record the angle the train came from
			xSemaphoreGiveFromISR(xTrainSemaphore, &xHigherPriorityTaskWoken);
			timerStart(TSAFETY);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else //there is a train
		{
			if(1 == trainSource) //came from the same angle
			{
				//do nothing
			}
			else
			{
				if(1==(*(volatile uint32_t *)(TIMER0_BASE + 0x00C) && 1)) //tsafty isn't passed yet
				{
					//do nithing
				}
				else
				{
					trainFlag = 0; // train has passed
				}
			}
		}	
	}
}

void PedestrianCrossingHandler()
{
	//clear the interupt
	uint32_t status;
	status = GPIOIntStatus(FourCrossingsPedestrianPort, true);
	GPIOIntClear(FourCrossingsPedestrianPort, status);
	//give semaphore to run the handler with context switching
	if (0 == trainFlag)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xPedestrianSemaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		//do nothing, there is a train passing, lights are already red
	}

}

void enableInterrupts()
{
	GPIOIntEnable(FourCrossingsPedestrianPort, GPIO_INT_PIN_3);
	GPIOIntTypeSet(FourCrossingsPedestrianPort,GPIO_PIN_3,GPIO_FALLING_EDGE);
	IntPrioritySet(INT_GPIOF_TM4C129 ,0xE0);
	IntRegister(INT_GPIOF_TM4C129,PedestrianCrossingHandler);
	IntEnable(INT_GPIOF_TM4C129);
	
	GPIOIntEnable(TrainPort, (GPIO_INT_PIN_0 | GPIO_INT_PIN_1));
	GPIOIntTypeSet(TrainPort,GPIO_PIN_0,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(TrainPort,GPIO_PIN_1,GPIO_FALLING_EDGE);
	IntPrioritySet(INT_GPIOJ_TM4C129 ,0xA0);
	IntRegister(INT_GPIOJ_TM4C129,TrainCrossingHandler);
	IntEnable(INT_GPIOJ_TM4C129);
	IntMasterEnable();
}

void eastWest(void *pvParameters)
{
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		xSemaphoreTake(xNormalSemaphore, portMAX_DELAY);
		UARTSend((uint8_t *)"eastWest start, ", 16);
//		vPrintString("east and west road \n");
		//nl3ab fe el lomad
		setEastAndWest(0xff);
		//vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( TGE_TGW ) );
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0xFF);
		delayMS(TGE_TGW);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x0);
		xSemaphoreGive(xNormalSemaphore);
		taskYIELD();
	}
}
void northSouth(void *pvParameters)
{
	//TickType_t xLastWakeTime;
	//xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		xSemaphoreTake(xNormalSemaphore, portMAX_DELAY);
		UARTSend((uint8_t *)"northSouth start, ", 18);
//		vPrintString("norht and south road \n");
		//nl3ab fe el lomad
		setNorthAndSouth(0xff);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x1);
		delayMS(TGN_TGS);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
		//vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( TGN_TGS ) );
		
		xSemaphoreGive(xNormalSemaphore);
		taskYIELD();
	}
}

void pedestrianMode(void *pvParameters)
{
	for (;;)
	{
		xSemaphoreTake(xPedestrianSemaphore, portMAX_DELAY);
		xSemaphoreTake(xNormalSemaphore, portMAX_DELAY);
		UARTSend((uint8_t *)"pedestrian start, ", 18);
//		vPrintString("pedestrians passing \n");
		//nl3ab fe el lomad
		setGREEN(0x00);
		setRED(0xff);
		vTaskDelay(pdMS_TO_TICKS(TCROSS));
		
		xSemaphoreGive(xNormalSemaphore);
	}
}

void trainMode(void *pvParameters)
{
	for (;;)
	{
		xSemaphoreTake(xTrainSemaphore, portMAX_DELAY);
//		vPrintString("train is passing \n");
		UARTSend((uint8_t *)"train start, ", 13);
		uint8_t tempNS = globalStateOfNorthAndSouth;
		uint8_t tempES = globalStateOfEastAndWest;
		setGREEN(0x00);
		setRED(0xFF);
		//turn on red LED and run the siren
		GPIOPinWrite(TrainGatePORT, GateLEDAndSiren, 0xff);
		
		// this number / load value = % of on , for servo -> between .05% and .1%-> 1875 , 3750 , for clk = 120MHZ
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2900);

			// will be false train passes
			while( 1 == trainFlag) 
			{
				//red leds toggle
				setRED(0xFF);
				delayMS(500);
				setRED(0x00);
				delayMS(500);
			}
		//turn off red LED and run the siren
		GPIOPinWrite(TrainGatePORT, GateLEDAndSiren, 0);
		// this number / load value = % of on , for servo -> between .05% and .1%-> 1875 , 3750 , for clk = 120MHZ
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2400);
//		vPrintString("train passed \n");
	setNorthAndSouth(tempNS);
	setEastAndWest(tempES);
	}
}

int main(void)
{
	sysInit();
	PWM_Init();
	UART_Init();
	setupOutputPins();
	setupInputPins();
	enableInterrupts();
	xNormalSemaphore = xSemaphoreCreateBinary();
	xPedestrianSemaphore = xSemaphoreCreateBinary();
	xTrainSemaphore = xSemaphoreCreateBinary();

	if ( xNormalSemaphore != NULL && xPedestrianSemaphore != NULL && xTrainSemaphore != NULL)
	{
		xTaskCreate(northSouth, "northSouth", 256, NULL, 1, NULL);
		xTaskCreate(eastWest, "eastWest", 256, NULL, 1, NULL);
		xTaskCreate(pedestrianMode, "pedestrian", 256, NULL, 2, NULL);
		xTaskCreate(trainMode, "train", 256, NULL, 3, NULL);
		xSemaphoreGive(xNormalSemaphore);
		vTaskStartScheduler();
	}

	for (;;);
}
