/** \file ADS1299.c
 *
 * Source file for the ADS1299 device.
 *
 * <pre>
 *  ------------------- Function Description ------------------
 * This c file includes all functions relative to the
 * ADS1299 Analog Front-End.
 *  ------------------------- Updates -------------------------
 *
 *  2015-05-29 	/ 	Enzo Mastinu 		/ Creation
 * </pre>
 */


/* libraries */
#include "ADS1299.h"


/* defines */


/* structures */


/* prototypes */
unsigned char ADS1299ReadReg(unsigned char addr);
void ADS1299WriteReg(unsigned char addr, unsigned char value);
int Interpret24bitAsInt32(unsigned char* byteArray);


/* global variables */
volatile unsigned char adsGain = 1;

extern unsigned int clockFreq;


/* functions */
/**
 * \brief This function executes the ADS1299 power-on procedure and
 * initializes all needed config registers.
 * @param[in] none
 * @return none.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 1.22 s
 */
void ADS1299Init(void) {

    // ADS1299 POWER-UP SEQUENCE

    // Enable reset ADS1299 (nRESET = 0)
    GPIOPinWrite(ADS_NRST_PORT, ADS_NRST_PIN, 0);
    // Disable ADS1299 start (START = 0)
    GPIOPinWrite(ADS_START_PORT, ADS_START_PIN, 0);
    // Disable SPI comunication (nCS = 1),
    GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);
	// Enable ADS1299 (nPWDN = 1) , already done by ADS1299 board pullup
	//GPIOPinWrite(ADS_NPWDN_PORT, ADS_NPWDN_PIN, ADS_NPWDN_PIN);

	// wait 100us
	SysCtlDelay((clockFreq/(3*10000)));
	// ROM_SysCtlDelay() is more precise than SysCtlDelay

    // Disable reset ADS1299 (nRESET = 1)
	GPIOPinWrite(ADS_NRST_PORT, ADS_NRST_PIN, ADS_NRST_PIN);

    // Wait for at least t_por = 0.032s
    SysCtlDelay((clockFreq/(3*10)));	// wait 0.1s

    // Reset ADS1299 (nRESET = 0)
    GPIOPinWrite(ADS_NRST_PORT, ADS_NRST_PIN, 0);

    // Wait at least t_rst = (2*t_ADS_clk) = 1us
    SysCtlDelay((clockFreq/(3*100000)));     // wait 10us

    // Disable reset ADS1299 (nRESET = 1)
    GPIOPinWrite(ADS_NRST_PORT, ADS_NRST_PIN, ADS_NRST_PIN);

    // Wait at least t_rst = (18*t_ADS_clk) = 9us
    SysCtlDelay((clockFreq/(3*10000)));     // wait 100us

    // Wait 1s
    SysCtlDelay((clockFreq/6));

    // SDATAC
    // send SDATAC (Stop Read Continuosly mode) to set ADS1299 CONFIG registers
    ADS1299StopContinuousMode();

    // CONFIG3
    // 1110 1000, Internal reference buffer enabled, Bias signal generator and Bias buffer disabled
    ADS1299WriteReg(0x03, 0xE0);

    // 1110 1000, Internal reference buffer enabled, Bias signal generator and Bias buffer enabled, Reference on GND applied externally
    //ADS1299_WriteReg(0x0D, 0x01);
    //ADS1299_WriteReg(0x0E, 0x01);
    //ADS1299_WriteReg(0x03, 0xEC);

    // THIS IS TO TEST USING ALSO THE REF ELECTRODE, SRB1 connected to all INPN
    //ADS1299_WriteReg(0x15, 0x20);

    // CONFIG1
    // 1000 0110, DR = f / 4096  -->  250 SPS   (BW = 65Hz)
    // 1000 0101, DR = f / 2048  -->  500 SPS   (BW = 131Hz)
    // 1000 0100, DR = f / 1024  -->  1000 SPS  (BW = 262Hz)
    // 1000 0011, DR = f / 512   -->  2000 SPS  (BW = 524Hz)
    // 1000 0010, DR = f / 256   -->  4000 SPS  (BW = 1048Hz)
    // 1000 0001, DR = f / 128   -->  8000 SPS  (BW = 2096Hz)
    // 1000 0000, DR = f / 64    -->  16000 SPS (BW = 4193Hz)
    ADS1299WriteReg(0x01, 0x93);

    // TEST DAISY-CHAIN
    //ADS1299_WriteReg(0x01, 0xF6);

    // CONFIG2
    // 1101 0001 Send WREG CONFIG2 D1, Internal test signal generator disabled
    ADS1299WriteReg(0x02, 0xC0);

    // TEST signal: Internal test signal generator enabled
    //ADS1299_WriteReg(0x02, 0xD0);

    // CHnSET
    // 0100 0000 Send WREG CHnSET, Gain 24, MUX normal input mode
    ADS1299SetInputModeSetGain(0,1);

}

/**
 * \brief Routine to read a value from a ADS1299 register passing its address.
 * @param[in] addr is the address of the register
 * @return the value of the register.
 */
unsigned char ADS1299ReadReg(unsigned char addr) {

    uint32_t byte2TX, byte2RX;

    // Enable SPI comunication
    GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

	// Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	uint32_t ulDataRx;
	while(SSIDataGetNonBlocking(ADS_SSI_BASE, &ulDataRx));

	// 2 byte opcode RREG, Read From Register
	byte2TX = RREG + addr;

	// First opcode byte: 001r rrrr, where r rrrr is the starting register address
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	// Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	// Second opcode byte: 000n nnnn, where n nnnn is the number of registers to read – 1
	byte2TX = 0x00;
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	// Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataGet(ADS_SSI_BASE, &byte2RX);

	// After the serial communication is finished, always wait four or more tCLK cycles
	SysCtlDelay(ADS_CMD_DELAY);

	// Disable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

	return byte2RX;

}

/**
 * \brief Routine to read all values from ADS1299 registers. It receives a
 * pointer to a 25 bytes chars array.
 * @param[out] registers points to an array where store the values
 * @return none.
 */
void ADS1299ReadAllReg(unsigned char *registers) {

    unsigned char iReg;

    for(iReg=0; iReg<25; iReg++) {
		registers[iReg] = ADS1299ReadReg(iReg);
	}

}

/**
 * \brief Routine to write a value into a ADS1299 register passing its
 * address and the value.
 * @param[in] addr is the address of the register
 * @param[in] value is the value to write
 * @return none.
 */
void ADS1299WriteReg(unsigned char addr, unsigned char value) {

  unsigned char byte2TX;

  // Enable SPI comunication
  GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

  // Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
  SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

  byte2TX = WREG; 	                // 2 byte opcode RREG, Read From Register
  byte2TX |= addr;
  // First opcode byte: 001r rrrr, where r rrrr is the starting register address
  SSIDataPut(ADS_SSI_BASE, byte2TX);
  while(SSIBusy(ADS_SSI_BASE));

  // Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
  SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

  // Second opcode byte: 000n nnnn, where n nnnn is the number of registers to read
  byte2TX = 0x00;
  SSIDataPut(ADS_SSI_BASE, byte2TX);
  while(SSIBusy(ADS_SSI_BASE));

  // Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
  SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

  // Value of the register to write
  byte2TX = value;
  SSIDataPut(ADS_SSI_BASE, byte2TX);
  while(SSIBusy(ADS_SSI_BASE));

  // After the serial communication is finished, always wait four or more tCLK cycles
  SysCtlDelay(ADS_CMD_DELAY);

  // Disable SPI comunication
  GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

}

/**
 * \brief Routine to start the continuous acquisition mode.
 * @param[in] none
 * @return none.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 145 us
 */
void ADS1299ReadContinuousMode(void) {

	unsigned char byte2TX;

	// Enable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

	// Wait after take low the nCS line
	SysCtlDelay(ADS_CMD_DELAY);

	// Enable ADS1299 start (START = 0)
	GPIOPinWrite(ADS_START_PORT, ADS_START_PIN, ADS_START_PIN);

	// RDATAC
	// send RDATAC (Read Continuosly mode) to start the acquisition
	byte2TX = RDATAC;
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	// Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);

	// Disable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

}

/**
 * \brief Routine to stop the continuous acquisition mode.
 * @param[in] none
 * @return none.
 */
void ADS1299StopContinuousMode(void) {

	unsigned char byte2TX;

	// Enable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

	// Wait after take low the nCS line
	SysCtlDelay(ADS_CMD_DELAY);

	// Disable ADS1299 start (START = 0)
	GPIOPinWrite(ADS_START_PORT, ADS_START_PIN, 0);

	// SDATAC
	// send SDATAC (Stop Read Continuosly mode) to exit from the acquisition mode
	byte2TX = SDATAC;
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	// Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);

	// Disable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

}

/**
 * \brief This function writes the ADS1299 channel config registers
 * in way to set the output mode of the multiplexer and the gain.
 * @param[in] mode is the output mode
 * @param[in] gain is the gain
 * @return none.
 *
 * Each acquisition channel can be independently configured with its gain and input mode.
 * There are in total 7 possible configuration for the gain parameter:
 * - 1 --> x1
 * - 2 --> x2
 * - 3 --> x4
 * - 4 --> x6
 * - 5 --> x8
 * - 6 --> x12
 * - 7 --> x24
 *
 * There are in total 8 possible configuration for the mode parameter:
 * - 0 --> 0000 0000 Normal electrode input
 * - 1 --> 0000 0000 Normal electrode input
 * - 2 --> 0000 0010 Used in conjunction with BIAS_MEAS bit for BIAS measurements
 * - 3 --> 0000 0011, MVDD for supply measurement
 * - 4 --> 0000 0100, Temperature sensor
 * - 5 --> 0000 0101, Test signal
 * - 6 --> 0000 0110, BIAS_DRP (positive electrode is the driver)
 * - 7 --> 0000 0111, BIAS_DRN (negative electrode is the driver)
 */
void ADS1299SetInputModeSetGain(unsigned char mode, unsigned char gain) {

	unsigned char newvValue, byte2TX;
	newvValue = 0x00;

	switch(gain) {
		case 1:
			newvValue = mode;
			adsGain = 1;
		break;
		case 2:
			newvValue = 0x10 + mode;
			adsGain = 2;
		break;
		case 3:
			newvValue = 0x20 + mode;
			adsGain = 4;
		break;
		case 4:
			newvValue = 0x30 + mode;
			adsGain = 6;
		break;
		case 5:
			newvValue = 0x40 + mode;
			adsGain = 8;
		break;
		case 6:
			newvValue = 0x50 + mode;
			adsGain = 12;
		break;
		case 7:
			newvValue = 0x60 + mode;
			adsGain = 24;
		break;
	}

	// Enable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	// CHnSET
	// Send WREG CHnSET 01hSet, TEST mode
	// 0100(WREGopcode) + 0101(CH1SETaddr)
	byte2TX = 0x45;
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	byte2TX = 0x07;               // 0000 0111 number reg to write - 1
	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	// MUXn[2:0]: Channel input
	//  byte2TX = 0x00; 0000 0000 Normal electrode input
	//  byte2TX = 0x01; 0000 0001 Input shorted (for offset or noise measurements)
	//  byte2TX = 0x02; 0000 0010 Used in conjunction with BIAS_MEAS bit for BIAS measurements
	//  byte2TX = 0x03; 0000 0011, MVDD for supply measurement
	//  byte2TX = 0x04; 0000 0100, Temperature sensor
	//  byte2TX = 0x05; 0000 0101, Test signal
	//  byte2TX = 0x06; 0000 0110, BIAS_DRP (positive electrode is the driver)
	//  byte2TX = 0x07; 0000 0111, BIAS_DRN (negative electrode is the driver)
	byte2TX = newvValue;

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 1

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 2

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 3

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 4

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 5

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 6

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 7

	//Before sending new command must wait at least t_ADS_decode = 4*t_ADS_clk = 1.96us
	SysCtlDelay(ADS_CMD_DELAY);     // wait 3us

	SSIDataPut(ADS_SSI_BASE, byte2TX);
	while(SSIBusy(ADS_SSI_BASE));     // ch 8

	// After the serial communication is finished, always wait four or more tCLK cycles
	SysCtlDelay(ADS_CMD_DELAY);

	// Disable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

}

/**
 * \brief This function writes the ADS1299 CONFIG1 register
 * in way to set the output data rate.
 * @param[in] mode specifies the output data rate mode
 * @return none.
 *
 * There are in total 7 possible values for the mode parameter:
 * - 0000 0110, DR = f / 4096  -->  250 SPS   (BW = 65Hz)
 * - 0000 0101, DR = f / 2048  -->  500 SPS   (BW = 131Hz)
 * - 0000 0100, DR = f / 1024  -->  1000 SPS  (BW = 262Hz)
 * - 0000 0011, DR = f / 512   -->  2000 SPS  (BW = 524Hz)
 * - 0000 0010, DR = f / 256   -->  4000 SPS  (BW = 1048Hz)
 * - 0000 0001, DR = f / 128   -->  8000 SPS  (BW = 2096Hz)
 * - 0000 0000, DR = f / 64    -->  16000 SPS (BW = 4193Hz)
 */
void ADS1299SetDataRate(unsigned char mode) {

	unsigned char oldValue, newValue;

	oldValue = ADS1299ReadReg(0x01);
	newValue = oldValue|(mode&0x07);
	ADS1299WriteReg(0x01, newValue);

}

/**
 * \brief This function reads the 27 bytes coming from ADS1299, processes
 * them converting from complement-of-two form into integer value.
 * @param[in] nCh specifies the number of channels
 * @param[out] *chValues points to the array where store the values
 * @return none.
 *
 *  Measured Execution Time @80MHz:
 *  14/07/2016 -> 22 us
 */
void ADS1299AcquireSample(unsigned char nCh, int *chValues) {

	unsigned char iByte, iCh;
	uint32_t byte2RX;
	unsigned char bytes_from_ADS[24];
	unsigned char temp[3];

	// Enable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, 0);

	// sync the Tiva SPI with the ADS
	do {
		SSIDataPutNonBlocking(ADS_SSI_BASE, 0x00);
		SSIDataGet(ADS_SSI_BASE, &byte2RX);
	} while(byte2RX != 0xC0);
	// discard 0xC00000 bytes
	SSIDataPutNonBlocking(ADS_SSI_BASE, 0x00);
	SSIDataGet(ADS_SSI_BASE, &byte2RX);
	SSIDataPutNonBlocking(ADS_SSI_BASE, 0x00);
	SSIDataGet(ADS_SSI_BASE, &byte2RX);
	// reading ADS bytes from SPI
	for(iByte=0; iByte<24; iByte++) {
		SSIDataPutNonBlocking(ADS_SSI_BASE, 0x00);
		SSIDataGet(ADS_SSI_BASE, &byte2RX);
		bytes_from_ADS[iByte] = byte2RX;
	}
	for(iCh=0; iCh<nCh; iCh++) {
		// translate bytes into channels data (integer format)
		temp[0] = bytes_from_ADS[iCh*3];
		temp[1] = bytes_from_ADS[1+iCh*3];
		temp[2] = bytes_from_ADS[2+iCh*3];
		chValues[iCh] = Interpret24bitAsInt32(temp);
	}

	// Disable SPI comunication
	GPIOPinWrite(ADS_NCS_PORT, ADS_NCS_PIN, ADS_NCS_PIN);

}

/**
 * \brief This function translates 24 bits information (3 bytes) from
 * complement of 2 format into an integer value.
 * @param[in] *byteArray points to the array where store the values
 * @return result of the conversion.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 0.2 us
 */
int Interpret24bitAsInt32(unsigned char* byteArray) {

	int newInt = 0;

	newInt = (
		((0xFF & byteArray[0]) << 16) |
		((0xFF & byteArray[1]) << 8) |
		((0xFF & byteArray[2]))
	);
	if ((newInt & 0x00800000) > 0) {
	   newInt |= 0xFF000000;
	 } else {
	   newInt &= 0x00FFFFFF;
	 }
	return newInt;

}

/**
 * \brief This function enables or disables the generation of a test signal
 * built in the ADS1299. It also modifies the input multiplexer
 * configuration shorting the input channels to the test signal.
 * @param[in] enable can be either 0 (disable) or 1 (enable)
 * @return echo of the enable parameter.
 */
unsigned char ADS1299TestSignal(unsigned char enable) {

	if(enable) {
		// WREG CONFIG2: Internal test signal generator enabled
		ADS1299WriteReg(0x02, 0xD0);
		// WREG CHnSET: Gain 24, MUX normal input mode
		ADS1299SetInputModeSetGain(0x05,1);
		return 1;
	} else {
		// WREG CONFIG2: Internal test signal generator disabled
		ADS1299WriteReg(0x02, 0xC0);
		// WREG CHnSET: Gain 24, MUX normal input mode
		ADS1299SetInputModeSetGain(0x00,1);
		return 0;
	}

}

