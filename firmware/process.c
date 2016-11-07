/** \file process.c
 *
 * Source file of process module of the NEUROMOTUS project.
 *
 * <pre>
 *  ------------------- Function Description ------------------
 * The process.c file contains all routines needed to properly
 * process the EMG signals acquired from the ADS1299. It includes
 * also helpful routines for communication.
 *
 *  ------------------------- Updates -------------------------
 *
*  2015-10-01 	/ 	Enzo Mastinu 		/ Creation
 * </pre>
 */


/* libraries */
#include "process.h"


/* defines */
// filters parameter
#define NZEROS 			  		2
#define NPOLES 			  		2
#define GAIN_HP_2000	  		1.092933031f
#define GAIN_N_2000            	1.002243999f
#define GAIN_HP_1000			1.045431062f
#define GAIN_N_1000             1.004488020f
#define GAIN_HP_500			  	1.194615832f
#define GAIN_N_500            	1.008976220f


/* structures */


/* prototypes */
float32_t FilterHP(float32_t inData, unsigned char ch, uint32_t frequency);
float32_t FilterNOTCH(float32_t inData, unsigned char ch, uint32_t frequency);


/* global variables */
const float32_t lsbVolt = 0.000000536442f;
float32_t xv[NZEROS+1][8];
float32_t yv[NZEROS+1][8];
float32_t xvN[NPOLES+1][8];
float32_t yvN[NPOLES+1][8];
volatile unsigned char nChannels = 8;
volatile unsigned char filterEnable = 1;
volatile float32_t temperatureMCU = 0;
volatile float32_t batteryVoltage = 0;
volatile float32_t batteryTemperature = 0;
extern volatile uint32_t sF;


/* functions */
/**
 * \brief It filters a new sample applying a IIR high-pass filter in real time.
 * @param[in] inData float input value
 * @param[in] ch index of the channel
 * @param[in] frequency is the sampling frequency
 * @return float filtered value.
 *
 * Butterworth HP 2nd order @ 20Hz IIR Filter
 * this function takes a float input and applies a real time filter
 * process. The filter needs information about the previous states
 * of input and output values, they are stored into
 * xv[order+1][nChannels] and yv[order+1][nChannels] global arrays.
 * The filters used are IIR:
 * - from current input, past inputs and past outputs
 *   we generate the current value. In FIR you use only the current
 *   and past inputs.
 * IIR parameters are calculated via Matlab:
 * Fc = x;
 * Wn = 20 / (Fc/2);
 * Order = 2;
 * [num den] = butter(Order, Wn, 'high');
 * we normalize the coefficients
 * num = num/num(1)
 * den = den/den(1)
 * then, the coefficients the multiply the inputs are
 * x0 * num(1), x1 * num(2), x2 * num(3)
 * while for the outputs is
 * y2 * den(1), y1 * den(2), y0 * den(3)
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 1.31 us
 */
float32_t FilterHP(float32_t inData, unsigned char ch, uint32_t frequency) {

	switch(frequency)
	{
		case 500:
			xv[0][ch] = xv[1][ch]; xv[1][ch] = xv[2][ch];
			xv[2][ch] = inData / GAIN_HP_500;
			yv[0][ch] = yv[1][ch]; yv[1][ch] = yv[2][ch];
			yv[2][ch] =   (xv[0][ch] + xv[2][ch]) - 2 * xv[1][ch] + ( -0.7008967812f * yv[0][ch]) + (  1.6474599811f * yv[1][ch]);
		break;
		case 1000:
			xv[0][ch] = xv[1][ch]; xv[1][ch] = xv[2][ch];
			xv[2][ch] = inData / GAIN_HP_1000;
			yv[0][ch] = yv[1][ch]; yv[1][ch] = yv[2][ch];
			yv[2][ch] =   (xv[0][ch] + xv[2][ch]) - 2 * xv[1][ch] + ( -0.8371816513f * yv[0][ch]) + (  1.8226949252f * yv[1][ch]);
		break;
		case 2000:
			xv[0][ch] = xv[1][ch]; xv[1][ch] = xv[2][ch];
			xv[2][ch] = inData / GAIN_HP_2000;
			yv[0][ch] = yv[1][ch]; yv[1][ch] = yv[2][ch];
			yv[2][ch] =   (xv[0][ch] + xv[2][ch]) - 2 * xv[1][ch] + ( -0.9149758348f * yv[0][ch]) + (  1.9111970674f * yv[1][ch]);
		break;
		default:
			return inData;
	}
	return yv[2][ch];

}

/**
 * \brief It filters a new sample applying a IIR notch filter in real time.
 * @param[in] inData float input value
 * @param[in] ch index of the channel
 * @param[in] frequency is the sampling frequency
 * @return float filtered value.
 *
 * Butterworth NOTCH 1st order @ 45-55 Hz IIR Filter
 * this function takes a float input and applies a real time filter
 * process. The filter needs information about the previous states
 * of input and output values, they are stored into
 * xvN[order+1][nChannels] and yvN[order+1][nChannels] global arrays.
 * IIR parameters are calculated via Matlab:
 * Fc = x;
 * W0 = 50 / (Fc/2);
 * Q = 35;
 * Bw = W0/Q;
 * [num den] = iirnotch(W0, Bw);
 * we normalize the coefficients
 * num = num/num(1)
 * den = den/den(1)
 * then, the coefficients the multiply the inputs are
 * x0 * num(1), x1 * num(2), x2 * num(3)
 * while for the outputs is
 * y2 * den(1), y1 * den(2), y0 * den(3)
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 1.38 us
 */
float32_t FilterNOTCH(float32_t inData, unsigned char ch, uint32_t frequency) {

	switch(frequency)
	{
		case 500:
			xvN[0][ch] = xvN[1][ch]; xvN[1][ch] = xvN[2][ch];
			xvN[2][ch] = inData / GAIN_N_500;
			yvN[0][ch] = yvN[1][ch]; yvN[1][ch] = yvN[2][ch];
			yvN[2][ch] = (xvN[0][ch] + xvN[2][ch]) - 1.6180339887f * xvN[1][ch] + ( -0.9822072713f * yvN[0][ch]) + (  1.6036393689f * yvN[1][ch]);
		break;
		case 1000:
			xvN[0][ch] = xvN[1][ch]; xvN[1][ch] = xvN[2][ch];
			xvN[2][ch] = inData / GAIN_N_1000;
			yvN[0][ch] = yvN[1][ch]; yvN[1][ch] = yvN[2][ch];
			yvN[2][ch] = (xvN[0][ch] + xvN[2][ch]) - 1.9021130326f * xvN[1][ch] + ( -0.9910640654f * yvN[0][ch]) + (  1.8936144537f * yvN[1][ch]);
		break;
		case 2000:
			xvN[0][ch] = xvN[1][ch]; xvN[1][ch] = xvN[2][ch];
			xvN[2][ch] = inData / GAIN_N_2000;
			yvN[0][ch] = yvN[1][ch]; yvN[1][ch] = yvN[2][ch];
			yvN[2][ch] = (xvN[0][ch] + xvN[2][ch]) - 1.9753766812f * xvN[1][ch] + ( -0.9955220515f * yvN[0][ch]) + (  1.9709538636f * yvN[1][ch]);
		break;
		default:
			return inData;
	}
	return yvN[2][ch];

}

/**
 * \brief Needed to reset the history of the filters and delete the
 * previous states.
 * @param[in] nCh number of channels
 * @return none.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 16 us
 */
void ResetFilterArrays(unsigned char nCh) {

	unsigned char iCh, k;

	for(iCh=0; iCh<nCh; iCh++) {
		for(k=0; k<3; k++) {
			xv[k][iCh] = 0;
			yv[k][iCh] = 0;
			xvN[k][iCh] = 0;
			yvN[k][iCh] = 0;
		}
	}

}

/**
 * \brief It calls the routines for filtering a new sample.
 * @param[in] inData float input value
 * @param[in] filterIndex index of the channel which the sample belongs
 * @return float output value.
 *
 * This function contains the routines needed to apply digital
 * filters to the data in a on-fly technique. This means that the
 * latest acquired sample is passed through the filters and the
 * filtered version is returned by this function.
 *
 *  Measured Execution Time @80MHz:
 *  13/07/2016 -> 3 us
 */
float32_t FilterSample(float32_t inData, unsigned char filterIndex) {

	float32_t tempFloatValue;
	float32_t tempFloatValue2 = inData;

	if(filterEnable) {
		// filter data
		tempFloatValue = FilterNOTCH(inData, filterIndex, sF);
		tempFloatValue2 = FilterHP(tempFloatValue, filterIndex, sF);
	}
	return tempFloatValue2;

}

/**
 * \brief This function can be used to easily handle the reception on the
 * UART of 4bytes sized variables, e. g. float.
 * @param[in] *ptr points the variable to receive
 * @return none.
 */
void UARTReceive4Bytes(uint32_t *ptr) {

	*ptr = ((0xFF & UARTCharGet(COMM_UARTPORT)) << 24) | ((0xFF & UARTCharGet(COMM_UARTPORT)) << 16) | ((0xFF & UARTCharGet(COMM_UARTPORT)) << 8) | (0xFF & UARTCharGet(COMM_UARTPORT));

}

/**
 * \brief This function can be used to easily handle the transmit on the
 * UART of 4bytes sized variables, e. g. float.
 * @param[in] *ptr points the variable to send
 * @return none.
 */
void UARTSend4Bytes(unsigned char *ptr) {

	unsigned char k, byte;

	ptr += 3;
	for(k=0; k<sizeof(float32_t); k++) {
		byte = *ptr;
		UARTSendByte(byte);
		ptr--;
	}

}

/**
 * \brief This function can be used to easily handle the transmit on the
 * UART of 4bytes sized variables, e. g. int16.
 * @param[in] *ptr points the variable to send
 * @return none.
 */
void UARTSend2Bytes(unsigned char *ptr) {

	unsigned char k, byte;

	ptr += 2;
	for(k=0; k<sizeof(int16_t); k++) {
		byte = *ptr;
		UARTSendByte(byte);
		ptr--;
	}

}

/**
 * \brief This function can be used to easily handle the transmission on the
 * UART of one byte. It is meant for the application of flow control.
 * @param[in] byte to send
 * @return none.
 */
void UARTSendByte(unsigned char byte) {

	UARTCharPut(COMM_UARTPORT,byte);

}
