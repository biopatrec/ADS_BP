/** \file process.h
 *
 * Header file of process module of the ALC-D project.
 *
 * <pre>
 *  ------------------- Function Description ------------------
 * The process.h file contains all defines, structures and
 * prototypes needed for the process.c file.
 *  ------------------------- Updates -------------------------
 *
*  2015-10-01 	/ 	Enzo Mastinu 		/ Creation
 * </pre>
 */

#ifndef ALCD_PROCESS_H_
#define ALCD_PROCESS_H_


/* libraries */
#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"

#include "ADS1299.h"


/* defines */
#define COMM_UARTPORT			UART1_BASE
#define COMM_UARTINT			INT_UART1

// BT module
#define WT12_NOTPRESENT    0x00
#define WT12_PRESENT       0x01
#define WT12_CONNECTED     0x02

// Battery analog pins
#define AN_INPUT				ADC_CTL_CH0
#define R1_DIVIDER	      		10000 	// R29
#define R2_DIVIDER		  		4750 	// ALC-D1.0 R32, ALC-D1.1 R30
#define K_DIVIDER		  		(R1_DIVIDER+R2_DIVIDER)/R2_DIVIDER
#define AN_INPUT_NTC			ADC_CTL_CH5
#define R_DIVIDER_NTC     		5600 	// LatchPB5 R8
#define R_PARALLEL_NTC     		10000 	// LatchPB5 R7
#define NTC_25_RESISTANCE  		10000
#define NTC_B_PARAMETER			3435
#define NTC_T0_PARAMETER		25
#define NTC_R_PARAMETER			(NTC_25_RESISTANCE*exp(-NTC_B_PARAMETER/NTC_T0_PARAMETER)))

/* structures */


/* prototypes */
void ResetFilterArrays(unsigned char nCh);
float32_t FilterSample(float32_t inData, unsigned char filterIndex);
void UARTReceive4Bytes(uint32_t *ptr);
void UARTSend4Bytes(unsigned char *ptr);
void UARTSendByte(unsigned char byte);


/* global variables */


#endif /* ALCD_PROCESS_H_ */

