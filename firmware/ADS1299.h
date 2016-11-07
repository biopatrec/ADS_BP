/** \file ADS1299.h
 *
 * Source file for the ADS1299 device.
 *
 * <pre>
 *  ------------------- Function Description ------------------
 * This h file includes all functions and defines relative
 * to the ADS1299 Analog Front-End that are supposed to be
 * called from external modules.
 *  ------------------------- Updates -------------------------
 *
 *  2015-05-29 	/ 	Enzo Mastinu 		/ Creation
 * </pre>
 */

#ifndef ADS1299_H_
#define ADS1299_H_


/* libraries */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"


/* defines */
// MCU Pheripherals and Pins
#define ADS_NPWDN_PORT   	    0
#define ADS_NPWDN_PIN   	    0
#define ADS_NRST_PORT   	    GPIO_PORTD_BASE
#define ADS_NRST_PIN   	      	GPIO_PIN_2			// check: PD7 needs to be unlocked first
#define ADS_START_PORT   	    GPIO_PORTD_BASE
#define ADS_START_PIN   	    GPIO_PIN_1
#define ADS_NCS_PORT   	      	GPIO_PORTA_BASE
#define ADS_NCS_PIN   	      	GPIO_PIN_0
#define ADS_SSI_PHERIPH			SYSCTL_PERIPH_SSI0
#define ADS_SSI_BASE			SSI0_BASE
#define ADS_SSI_PORT 	      	GPIO_PORTA_BASE
#define ADS_SSIRX_PIN 	      	GPIO_PA4_SSI0RX
#define ADS_SSITX_PIN   	    GPIO_PA5_SSI0TX
#define ADS_SSISCLK_PIN	      	GPIO_PA2_SSI0CLK
#define ADS_NDRDY_PORT   	   	GPIO_PORTC_BASE
#define ADS_NDRDY_PIN   	    GPIO_INT_PIN_5
#define ADS_NDRDY_INT			INT_GPIOA_TM4C123

// ADS1299 registers
#define ADSregID   	      		0x00
#define ADSregCONFIG1     		0x01
#define ADSregCONFIG2     		0x02
#define ADSregCONFIG3     		0x03
#define ADSregCH1SET      		0x05

// ADS1299 SPI commands
#define WREG		      		0x40
#define RREG		      		0x20
#define SDATAC	          		0x11
#define RDATAC	          		0x10
#define RDATA		      		0x12

/* Before sending a new command to the ADS1299, we must wait at least 
 * t_ADS_decode = 4*t_ADS_clk = 1.96us. So, we wait 3us. */
#define ADS_CMD_DELAY           (clockFreq/(3*300000))

/* structures */


/* prototypes */
void ADS1299Init(void);
void ADS1299ReadContinuousMode(void);
void ADS1299StopContinuousMode(void);
void ADS1299ReadAllReg(unsigned char *registers);
void ADS1299SetInputModeSetGain(unsigned char mode, unsigned char gain);
void ADS1299SetDataRate(unsigned char mode);
unsigned char ADS1299TestSignal(unsigned char enable);
void ADS1299AcquireSample(unsigned char nCh, int *chValues);


/* global variables */


#endif /* ADS1299_H_ */
