#include <iostream>
#include <cassert>
#include "pigpio.h"
using namespace std;

#define	GPIO_7				(7)
#define	GPIO_8				(8)
#define	GPIO_9				(9)
#define	GPIO_10				(10)
#define	GPIO_11				(11)
#define	SPI_0_MOSI			(GPIO_10)
#define	SPI_0_MISO			(GPIO_9)
#define	SPI_0_SCLK			(GPIO_11)
#define	SPI_CE0_N			(GPIO_8)
#define	SPI_CE1_N			(GPIO_7)
#define	SPI_MODE_0	(0)
#define	SPI_MODE_1	(1)
#define	SPI_MODE_2	(2)
#define	SPI_MODE_3	(3)
#define	SPI_CE0_ACTIVE_LOW	(0)
#define	SPI_CE0_ACTIVE_HIGH	(1 << 2)
#define	SPI_CE1_ACTIVE_LOW	(0)
#define	SPI_CE1_ACTIVE_HIGH	(1 << 3)
#define	SPI_USE_AUX_SPI		(1 << 8)
#define	SPI_USE_AUX_MAIN	(0)
#define	SPI_CHANNEL_0		(0)
#define	SPI_CHANNEL_1		(1)
#define	SPI_CHANNEL_MAIN	(SPI_CHANNEL_0)
#define	SPI_CHANNEL_SUB		(SPI_CHANNEL_1)
#define	SPI_CLOCK_125K		(125 * 1000)
#define	SPI_CLOCK_250K		(250 * 1000)
#define	SPI_CLOCK_500K		(500 * 1000)
#define	SPI_CLOCK_1M		(1 * 1000 * 1000)
#define	SPI_CLOCK_4M		(4 * 1000 * 1000)
#define	SPI_CLOCK_8M		(8 * 1000 * 1000)
#define	SPI_CLOCK_10M		(10 * 1000 * 1000)
#define	SPI_CLOCK_USE		SPI_CLOCK_4M

//SPI commands macros of MPL115A1
#define	MPL115A1_COMMAND_START_CONVERSION		(0x24)
#define	MPL115A1_COMMAND_READ_PRESS_MSB			(0x80)
#define	MPL115A1_COMMAND_READ_PRESS_LSB			(0x82)
#define	MPL115A1_COMMAND_READ_TEMP_MSB			(0x84)
#define	MPL115A1_COMMAND_READ_TEMP_LSB			(0x86)
#define	MPL115A1_COMMAND_READ_COEFF_A0_MSB		(0x88)
#define	MPL115A1_COMMAND_READ_COEFF_A0_LSB		(0x8A)
#define	MPL115A1_COMMAND_READ_COEFF_B1_MSB		(0x8C)
#define	MPL115A1_COMMAND_READ_COEFF_B1_LSB		(0x8E)
#define	MPL115A1_COMMAND_READ_COEFF_B2_MSB		(0x90)
#define	MPL115A1_COMMAND_READ_COEFF_B2_LSB		(0x92)
#define	MPL115A1_COMMAND_READ_COEFF_C12_MSB		(0x94)
#define	MPL115A1_COMMAND_READ_COEFF_C12_LSB		(0x96)
#define	MPL115A1_COMMAND_OUTPUT_DATA_BYTE		(0x00)
#define	MPL115A1_COEFF_A0_LSB					(1.0 / 8.0)
#define	MPL115A1_COEFF_B1_LSB					(1.0 / 8192.0)
#define	MPL115A1_COEFF_B2_LSB					(1.0 / 16384.0)
#define	MPL115A1_COEFF_C12_LSB					(1.0 / 4194304.0)
#define	MPL115A1_PRESS_ADC_LSB					(1.0)
#define	MPL115A1_TEMP_ADC_LSB					(1.0)

enum {
	MPL115A1_DATA_INDEX_COEFF_A0_MSB = 0,
	MPL115A1_DATA_INDEX_COEFF_A0_LSB,
	MPL115A1_DATA_INDEX_COEFF_B1_MSB,
	MPL115A1_DATA_INDEX_COEFF_B1_LSB,
	MPL115A1_DATA_INDEX_COEFF_B2_MSB,
	MPL115A1_DATA_INDEX_COEFF_B2_LSB,
	MPL115A1_DATA_INDEX_COEFF_C12_MSB,
	MPL115A1_DATA_INDEX_COEFF_C12_LSB,
	MPL115A1_DATA_INDEX_COEFF_MAX,
};
enum {
	MPL115A1_DATA_INDEX_PRESS_MSB = 0,
	MPL115A1_DATA_INDEX_PRESS_LSB,
	MPL115A1_DATA_INDEX_TEMP_MSB,
	MPL115A1_DATA_INDEX_TEMP_LSB,
	MPL115A1_DATA_INDEX_MAX,
};


const uint8_t READ_COEFF_SEQ_COMMAND[MPL115A1_DATA_INDEX_COEFF_MAX] = {
		MPL115A1_COMMAND_READ_COEFF_A0_MSB,
		MPL115A1_COMMAND_READ_COEFF_A0_LSB,
		MPL115A1_COMMAND_READ_COEFF_B1_MSB,
		MPL115A1_COMMAND_READ_COEFF_B1_LSB,
		MPL115A1_COMMAND_READ_COEFF_B2_MSB,
		MPL115A1_COMMAND_READ_COEFF_B2_LSB,
		MPL115A1_COMMAND_READ_COEFF_C12_MSB,
		MPL115A1_COMMAND_READ_COEFF_C12_LSB,
};

const uint8_t READ_PRESS_TEMP_SEQ_COMMAND[MPL115A1_DATA_INDEX_MAX] = {
		MPL115A1_COMMAND_READ_PRESS_MSB,
		MPL115A1_COMMAND_READ_PRESS_LSB,
		MPL115A1_COMMAND_READ_TEMP_MSB,
		MPL115A1_COMMAND_READ_TEMP_LSB,
};
#define MPL115A1_PRESS_ADC_BIT_MASK		(0xFFC0)
#define MPL115A1_TEMP_ADC_BIT_MASK			(0xFFC0)


uint8_t coefficientBuff[MPL115A1_DATA_INDEX_COEFF_MAX];
uint8_t dataBuff[MPL115A1_DATA_INDEX_MAX];

float MPL_115A1_a0 = 0.0;
float MPL_115A1_b1 = 0.0;
float MPL_115A1_b2 = 0.0;
float MPL_115A1_c12 = 0.0;
float MPL_115A1_padc = 0.0;
float MPL_115A1_tadc = 0.0;
float MPL_115A1_press = 0.0;

void StartSequence()
{
	gpioWrite(SPI_CE0_N, PI_LOW);
	gpioSleep(0, 0, 3 * 1000);
}
void StopSequence()
{
	gpioWrite(SPI_CE0_N, PI_HIGH);
}

/**
 * Send command and receive response from MPL115A1 each 1 byte.
 *
 * @param	spiHandle		SPI handle.
 * @param	command			Command code to send to MPL115A1.
 * @param[out]	readData	Pointer to buffer to store response from the sensor.
 */
void SendAndRecvCommand(
		const unsigned int spiHandle,
		const uint8_t command,
		uint8_t* readData)
{
	assert(NULL != readData);

	//Send command.
	spiWrite(spiHandle, (char*)(&command), 1);
	spiRead(spiHandle, (char*)(readData), 1);

	printf("Cmd = 0x%02x Res = 0x%02x(%4d)\n", command, *readData, *readData);
}

/**
 * Send extra [00h] command to MPL115A1 to output the last data byte on the slave
 * side of the SPI.
 *
 * @param	spiHandle	SPI handle.
 */
void SendExtraCommand(const unsigned int spiHandle) {
	uint8_t tempReadData = 0;

	for (int extraIndex = 0; extraIndex < 2; extraIndex++) {
		//Send command.
		uint8_t outputDataByteCommand = MPL115A1_COMMAND_OUTPUT_DATA_BYTE;
		spiWrite(spiHandle, (char*)(&outputDataByteCommand), 1);
		spiRead(spiHandle, (char*)(&tempReadData), 1);
	}
}

/**
 * Convert buffer data, data type is uint8_t, into float.
 *
 * @param[in]	buffer	Pointer to buffer conataining the uint8_t data.
 * @param	msbIndex	Index of buffer of MSB byte.
 * @param	lsbIndex	Index of buffer of LSB byte.
 * @param	floatLsb	LSB of float value.
 * @param	mask		Bit array to mask buffer. Default value is 0xFFFF.
 */
float Buff2Float(
		uint8_t* buffer,
		int msbIndex, int lsbIndex,
		float floatLsb,
		uint16_t mask = 0xFFFF)
{

	float converted = 0.0;

	uint16_t rawData = ((uint16_t)(buffer[msbIndex] << 8) + (uint16_t)(buffer[lsbIndex]));
	rawData &= mask;
	converted = (float)((int16_t)rawData * floatLsb);

	printf("rawData = 0x%04x floatData = %f(lsb = %f)\n", rawData, converted, floatLsb);

	return converted;
}

/**
 * Send command to start reading the datas used to calcurate pressure.
 *
 * @param	spiHandle	Handle of SPI.
 */
void StartDataConversion(const unsigned int spiHandle) {

	uint8_t tempBuff = 0;

	StartSequence();
	SendAndRecvCommand(spiHandle, MPL115A1_COMMAND_START_CONVERSION, &tempBuff);
	StopSequence();
}

/**
 * Read coefficient value..
 *
 * @param	spiHandle	SPI handle.
 */
void ReadCoefficient(const unsigned int spiHandle) {

	StartSequence();
	for (int buffIndex = 0; buffIndex < MPL115A1_DATA_INDEX_COEFF_MAX; buffIndex++) {
		SendAndRecvCommand(spiHandle, READ_COEFF_SEQ_COMMAND[buffIndex], &coefficientBuff[buffIndex]);
	}
	SendExtraCommand(spiHandle);
	StopSequence();

	//Setup coefficient from receive data.
	MPL_115A1_a0 = Buff2Float(
			coefficientBuff,
			MPL115A1_DATA_INDEX_COEFF_A0_MSB, MPL115A1_DATA_INDEX_COEFF_A0_LSB,
			MPL115A1_COEFF_A0_LSB);
	MPL_115A1_b1 = Buff2Float(
			coefficientBuff,
			MPL115A1_DATA_INDEX_COEFF_B1_MSB, MPL115A1_DATA_INDEX_COEFF_B1_LSB,
			MPL115A1_COEFF_B1_LSB);
	MPL_115A1_b2 = Buff2Float(
			coefficientBuff,
			MPL115A1_DATA_INDEX_COEFF_B2_MSB, MPL115A1_DATA_INDEX_COEFF_B2_LSB,
			MPL115A1_COEFF_B2_LSB);
	MPL_115A1_c12 = Buff2Float(
			coefficientBuff,
			MPL115A1_DATA_INDEX_COEFF_C12_MSB, MPL115A1_DATA_INDEX_COEFF_C12_LSB,
			MPL115A1_COEFF_C12_LSB);
	uint16_t rawData_c12 =
			((uint16_t)(coefficientBuff[MPL115A1_DATA_INDEX_COEFF_C12_MSB] << 8) +
			(uint16_t)(coefficientBuff[MPL115A1_DATA_INDEX_COEFF_C12_LSB]));
	rawData_c12 >>= 2;
	rawData_c12 &= 0x3FFF;
	MPL_115A1_c12 = (float)((int16_t)rawData_c12 * MPL115A1_COEFF_C12_LSB);

	printf("MPL_115A1_a0 =  %f\n", MPL_115A1_a0);
	printf("MPL_115A1_b1 =  %f\n", MPL_115A1_b1);
	printf("MPL_115A1_b2 =  %f\n", MPL_115A1_b2);
	printf("MPL_115A1_c12 = %f\n", MPL_115A1_c12);
}

/**
 * Read pressure and temperature value.
 */
void ReadTempAndPress(const unsigned int spiHandle) {

	StartSequence();
	for (int buffIndex = 0; buffIndex < MPL115A1_DATA_INDEX_MAX; buffIndex++) {
		SendAndRecvCommand(spiHandle,
				READ_PRESS_TEMP_SEQ_COMMAND[buffIndex],
				(uint8_t*)(&dataBuff[buffIndex]));
	}
	SendExtraCommand(spiHandle);
	StopSequence();

	//Get Padc and Tadc value by buffer.
	{
		uint16_t rawData_padc =
				(((uint16_t)dataBuff[MPL115A1_DATA_INDEX_PRESS_MSB]) << 8) +
				dataBuff[MPL115A1_DATA_INDEX_PRESS_LSB];
		rawData_padc >>= 6;
		MPL_115A1_padc = (float)((int16_t)rawData_padc);
		printf("MPL_115A1_padc =  %.0f\n", MPL_115A1_padc);
	}
	{
		uint16_t rawData_tadc =
				(((uint16_t)dataBuff[MPL115A1_DATA_INDEX_TEMP_MSB]) << 8) +
				dataBuff[MPL115A1_DATA_INDEX_TEMP_LSB];
		rawData_tadc >>= 6;
		MPL_115A1_tadc = (float)((int16_t)rawData_tadc);
		printf("MPL_115A1_tadc =  %.0f\n", MPL_115A1_tadc);
	}
}

/**
 * Get pressure by reading data from the sensor.
 *
 * @return	Pressure specified by kPa.
 */
float GetPress() {
	float MPL_115A1_press_comp =
			MPL_115A1_a0 +
			(MPL_115A1_b1 + MPL_115A1_c12 * MPL_115A1_tadc) * MPL_115A1_padc +
			MPL_115A1_b2 * MPL_115A1_tadc;
	MPL_115A1_press = (MPL_115A1_press_comp * ((115.0 - 50.0) / 1023.0)) + 50.0;

	return MPL_115A1_press;
}

/**
 * Main function.
 */
int main() {
	int spiHandle = 0;
	uint32_t spiMode =
			SPI_MODE_0 |
			SPI_CE0_ACTIVE_LOW |
			SPI_CE1_ACTIVE_LOW |
			SPI_USE_AUX_MAIN;

	printf("Start MPL115A1 sample applicaton.\n");

	if (gpioInitialise() < 0) {
		printf("gpioInitialise() failed\n");

		return 0;
	}

	//Initialize CE0, slave select disable.
	gpioSetMode(SPI_CE0_N, PI_OUTPUT);
	gpioWrite(SPI_CE0_N, PI_HIGH);

	spiHandle = spiOpen(0, SPI_CLOCK_USE, spiMode);
	if (spiHandle < 0) {
		printf("spiOpen() failed - %d\n", spiHandle);

		gpioTerminate();

		return 0;
	}

	ReadCoefficient(spiHandle);

	while (1) {
		gpioSleep(0, 1, 0);

		StartDataConversion(spiHandle);
		gpioDelay(3 * 1000);
		ReadTempAndPress(spiHandle);
		float press = GetPress();
		printf("Press = %.2f[kPa](%.2f[hPa])\n", press, press * 10.0);
	}

	spiClose(spiHandle);
	gpioTerminate();

	return 0;
}
