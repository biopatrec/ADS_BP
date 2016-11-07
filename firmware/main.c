/* -------------------- Copyright notice ---------------------
 *
 * This function is part of "ADS_BP" embedded software.
 *
 */

/** \mainpage ADS_BP project
 *
 * ADS_BP provides a low-cost open-source solution for signal conditioning of bioelectric signals.
 * This can be used conjointly with a micro-controller unit and a software as a human-machine interfacing system.
 * This PCB provides the Analog Front End(AFE) and the communication for a 2 part signal acquisition system.
 * This project was motivated by the need for hardware for the acquisition of the bioelectric signals to be used with BioPatRec.
 *
 */

/** \file main.c
 *
 * Main file and module of the ADS_BP project.
 *
 * <pre>
 *  ------------------- Function Description ------------------
 * This module includes the main function, all ISRs as well
 * as the initialization of the device.
 *  ------------------------- Updates -------------------------
 *
 *  2016-06-17 	/ 	Enzo Mastinu 		/ Creation ADS_BP project.
 * </pre>
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "ADS1299.h"
#include "process.h"


/* defines */
// Finite State Machine
#define IDLE			  		 0
#define ACQUIRE_EMG      	     1


/* structures */


/* prototypes */
void DeviceInit(void);


/* global variables */
unsigned char firmwareV[13] = {'1','.','5','-','1','1','/','1','0','/','1','6','~'};
unsigned char serialN[6] = {'A','D','S','B','P','~'};
uint32_t clockFreq = 0;
volatile unsigned char alcdState = IDLE;
volatile unsigned char timer;
volatile uint32_t sF = 1000;

extern volatile unsigned char nChannels;
extern volatile unsigned char filterEnable;
extern const float32_t lsbVolt;
extern volatile unsigned char btStatus;
extern volatile float32_t temperatureMCU;
extern volatile float32_t batteryVoltage;
extern volatile float32_t batteryTemperature;

// Only for test
int32_t	priorityCOMM = 100;
int32_t	priorityTMR0 = 100;
int32_t	priorityTMR3 = 100;
int32_t	priorityTMR2 = 100;


/* ISR */
/**
 * \brief UART reception ISR for decoding and executing commands given by external master devices.
 * @param[in] none
 * @return none.
 */
void COMM_IntHandler(void) {

    unsigned long ulStatus, i;
    uint32_t received = 0;
	unsigned char reg[25];
	float32_t tempFloatValue;

	// Get the interrupt status.
	ulStatus = UARTIntStatus(COMM_UARTPORT, true);

	// Clear the asserted interrupts.
	UARTIntClear(COMM_UARTPORT, ulStatus);

    received = UARTCharGet(COMM_UARTPORT);

	// Decode and Execute
	switch (received)
	{
		case 'A': // Test connection, it echos the received 'C' after the 'A'
			UARTSendByte(UARTCharGet(COMM_UARTPORT));
		break;
		case 0xA0: // Firmware Version & Device number
			UARTSendByte(received);
			for(i=0;i<13;i++)
				UARTSendByte(firmwareV[i]);
			for(i=0;i<6;i++)
				UARTSendByte(serialN[i]);
			UARTSendByte(0xA0);
		break;
		case 'B': // battery voltage check
			UARTSendByte(received);
			tempFloatValue = 5.0f; // to fix!!!!!!!!!
			UARTSend4Bytes((unsigned char *)&tempFloatValue);
			UARTSendByte(received);
		break;
		case 'S': // Read internal Registers
			UARTSendByte(received);
			ADS1299ReadAllReg(&reg[0]);
			for(i=0; i<24; i++)
				UARTSendByte(reg[i+1]);
			UARTSendByte('S');
		break;
		case 'H': // Enable/Disable Filters
			UARTSendByte(received);
			received = UARTCharGet(COMM_UARTPORT);
			filterEnable = received;
			UARTSendByte('H');
		break;
		case 'K': // Set Gain
			UARTSendByte(received);
			received = UARTCharGet(COMM_UARTPORT);
			ADS1299SetInputModeSetGain(0x00,received);
			UARTSendByte('K');
		break;
		case 'F': // Set the output data rate of ADS1299
			UARTSendByte(received);
			received = UARTCharGet(COMM_UARTPORT);
			ADS1299SetDataRate(received);
			UARTSendByte('F');
		break;
		case 'r': // Set the sampling frequency
			UARTSendByte(received);
            uint32_t sFTmp;
			UARTReceive4Bytes(&sFTmp);
			sF = sFTmp;
			if(sF!=500 && sF!=1000 && sF!= 2000)
				sF = 1000;
			UARTSendByte('r');
		break;
		case 'P': // Set internal test signal
			UARTSendByte(received);
			received = UARTCharGet(COMM_UARTPORT);
			ADS1299TestSignal(received);
			UARTSendByte('P');
		break;
		case 'G': // Start EMG continuous acquisition
			alcdState = ACQUIRE_EMG;
			nChannels = UARTCharGet(COMM_UARTPORT);
			if(nChannels>8)
				nChannels = 8;
			ResetFilterArrays(nChannels);
			TimerDisable(TIMER2_BASE, TIMER_A);
			TimerLoadSet(TIMER2_BASE, TIMER_A, clockFreq/10);
			TimerEnable(TIMER2_BASE, TIMER_A);
			ADS1299ReadContinuousMode();
			TimerDisable(TIMER0_BASE, TIMER_A);
			TimerLoadSet(TIMER0_BASE, TIMER_A, clockFreq/sF);
			TimerEnable(TIMER0_BASE, TIMER_A);
			UARTSendByte(received);
		break;
		case 'T': // Stop EMG continuous acquisition
			alcdState = IDLE;
			TimerDisable(TIMER0_BASE, TIMER_A);
			TimerDisable(TIMER2_BASE, TIMER_A);
			TimerLoadSet(TIMER2_BASE, TIMER_A, clockFreq);
			TimerEnable(TIMER2_BASE, TIMER_A);
			ADS1299StopContinuousMode();
			TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		break;
	}

}

/**
 * \brief Timer ISR for ADS1299 samples acquisition.
 * @param[in] none
 * @return none.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 357 us
 */
void Timer0IntHandler(void)
{

	int32_t channels[8];
	float32_t newSample[8];
	unsigned char iCh = 0;
	float32_t tempFloatValue;
    uint32_t dSamp;

	if(TimerIntStatus(TIMER0_BASE, TIMER_TIMA_TIMEOUT)) {

		/* Since the ADS1299 sampling can be faster than TIMER0 triggering 
         * (2K vs 1K or 0.5k) we throw away last available samples from ADS1299 
         * on the SPI line based on the sampling frequency.
		 * This can be done simply by sending dummy data on SPI in way that:
		 * - ADS1299 sends out all 27 samples bytes
		 * - ADS1299 sets NDRDY high again */
        for(dSamp=sF;dSamp<=2000;dSamp*=2) {
            SSIDataPutNonBlocking(ADS_SSI_BASE, 0x00);
            // Wait for a new sample to be ready on the SPI line
            while(GPIOPinRead(ADS_NDRDY_PORT,ADS_NDRDY_PIN) && ADS_NDRDY_PIN);
            // Acquire new sample: read it from SPI
            ADS1299AcquireSample(nChannels, &channels[0]);
        }

		// Convert new sample into voltage
		for(iCh=0; iCh<nChannels; iCh++) {
			// convert data into volt
			tempFloatValue = lsbVolt * (float32_t)channels[iCh];
			// process data with IIR filters
			newSample[iCh] = FilterSample(tempFloatValue, iCh);
		}

		// Do whatever needs to be done accordingly to the state machine
		switch(alcdState)
		{
			// EMG SIGNAL ACQUISITION: ACQUIRE EMG
			case ACQUIRE_EMG:
				for(iCh=0; iCh<nChannels; iCh++) {
					// send via UART
					UARTSend4Bytes((unsigned char *)&newSample[iCh]);
				}
			break;
			default:
			break;
		}
		// Clear the timer interrupt.
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	}

}

/**
 * \brief Timer ISR for LED blinking.
 * @param[in] none
 * @return none.
 *
 * In the common setup LEDs are driven in this way:
 * - LED1 (usually green) blinks at 2 Hz in stand-by, communication and control mode
 * - LED1 blinks at 10 Hz during EMG or features recording
 * - LED2 (usually blue):
 *  + is off when Bluetooth module not found
 *  + blinks at 2 Hz when the Bluetooth module is present but not connected
 *  + is on when the Bluetooth module is connected
 */
void Timer2IntHandler(void)
{

	/* Red LED blink to show acquisition or idle state */
	if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1))
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

    // Clear the timer interrupt.
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

}

/**
 * \brief Timer ISR for accessory operation.
 * @param[in] none
 * @return none.
 *
 * This has been used for UART and SPI timeout of reading.
 */
void Timer3IntHandler(void)
{
	// Set Timer flag because timer has expired
 	timer = true;
	// Clear the timer interrupt.
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}


/* MAIN */
/**
 *  \brief Main function
 *
 * <pre>
 * ISR priorities
 *  | Timer3IntHandler    		   -> 000 = 0x00 --> Accessory timer
 * 		| Timer0IntHandler         -> 001 = 0x20 --> Acquisition timer
 *      | COMM_IntHandler 	  	   -> 001 = 0x20 --> UART communication
 *          	| Timer2IntHandler -> 011 = 0x40 --> LED blinking
 * 	<---------------------- higher priority -------------------------
 * </pre>
 */
int main(void) {

	DeviceInit();


	while(1) {

		switch(alcdState)
		{
			// STAND-BY STATE
			case IDLE:
			break;

			// EMG SIGNAL ACQUISITION: ACQUIRE EMG AND SEND OUT
			case ACQUIRE_EMG:
				// GPIOc ISR
			break;

			default:
				alcdState = IDLE;
			break;
		}
	}

}

/**
 * \brief Function in charge of initializing the ALC-D system.
 * @param[in] none
 * @return none.
 *
 * The function initializes the following peripherals and devices:
 * - System Clock at 80 Mhz
 * - All needed GPIOs
 * - LEDs output pins
 * - Analog pin for battery voltage/temperature reading
 * - UART for external communication
 * - SPI for ADS1299
 * - ADS1299
 * - TIMERs
 *
 *  Measured Execution Time @80MHz:
 *  14/07/2016 -> 1.26 s
 */
void DeviceInit(void) {

	/* =================CLOCK Initialization======================= */
	// SYSCTL_SYSDIV_2_5 = 80MHz
	// SYSCTL_SYSDIV_3   = 66MHz
	// SYSCTL_SYSDIV_4   = 50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	clockFreq = SysCtlClockGet();	// SysCtlClockGet() can fail with values upon 50MHz!

	/* =================GPIO Initialization======================= */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* =================LED Initialization======================= */
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	//* =================UART3 Initialization======================= */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	//GPIOPinConfigure(GPIO_PF0_U1RTS);
	//GPIOPinConfigure(GPIO_PF1_U1CTS);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(COMM_UARTPORT, clockFreq, 460800, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	IntPrioritySet(COMM_UARTINT, 0x20);
	priorityCOMM = IntPriorityGet(COMM_UARTINT);
	IntEnable(COMM_UARTINT);
	UARTIntEnable(COMM_UARTPORT, UART_INT_RX | UART_INT_RT);

	/* =================SPI Initialization======================= */
	SysCtlPeripheralEnable(ADS_SSI_PHERIPH);		// SPI for ADS1299
	GPIOPinConfigure(ADS_SSISCLK_PIN);
	GPIOPinConfigure(ADS_SSIRX_PIN);
    GPIOPinConfigure(ADS_SSITX_PIN);
	//GPIOPinTypeSSI(ADS_SSI_PORT, ADS_SSISCLK_PIN | ADS_SSIRX_PIN | ADS_SSITX_PIN);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_2 | GPIO_PIN_5);
	SSIConfigSetExpClk(ADS_SSI_BASE, clockFreq, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 18000000, 8);
	SSIEnable(ADS_SSI_BASE);
	uint32_t ulDataRx;
	while(SSIDataGetNonBlocking(ADS_SSI_BASE, &ulDataRx)){}

	/* =================TIMER Initialization======================= */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	// ADS1299 samples reading timeout
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, clockFreq/sF -1);
	IntPrioritySet(INT_TIMER0A_TM4C123, 0x20);
	priorityTMR0 = IntPriorityGet(INT_TIMER0A_TM4C123);
	IntEnable(INT_TIMER0A_TM4C123);
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);	// LED blinking timer
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER2_BASE, TIMER_A, clockFreq -1);
	IntPrioritySet(INT_TIMER2A_TM4C123, 0x40);
	priorityTMR2 = IntPriorityGet(INT_TIMER2A_TM4C123);
	IntEnable(INT_TIMER2A_TM4C123);
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER2_BASE, TIMER_A);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);	// Accessory reading timeout
	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER3_BASE, TIMER_A, clockFreq/100 -1);
	IntPrioritySet(INT_TIMER3A_TM4C123, 0x00);
	priorityTMR3 = IntPriorityGet(INT_TIMER3A_TM4C123);
	IntEnable(INT_TIMER3A_TM4C123);
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

	/* =================ADS1299 Initialization======================= */
	GPIOPinTypeGPIOOutput(ADS_NRST_PORT, ADS_NRST_PIN);
	GPIOPinTypeGPIOOutput(ADS_START_PORT, ADS_START_PIN);
	GPIOPinTypeGPIOOutput(ADS_NCS_PORT, ADS_NCS_PIN);
	GPIOPinTypeGPIOInput(ADS_NDRDY_PORT, ADS_NDRDY_PIN);
	ADS1299Init();

	// Enable processor interrupts.
	IntMasterEnable();

}
