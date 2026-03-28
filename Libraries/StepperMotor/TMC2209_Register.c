/*
 * TMC2209_Register.c
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#include "TMC2209_Register.h"

extern UART_HandleTypeDef huart1;

uint8_t CRC8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++)
	{
		// We create a local COPY of the byte so the original array is safe
		uint8_t current_byte = data[i];

		for (uint8_t j = 0; j < 8; j++)
		{
			// TMC2209 logic: (CRC MSB) XOR (Data LSB)
			if ((crc >> 7) ^ (current_byte & 0x01))
			{
				crc = (crc << 1) ^ 0x07; // Polynomial 0x07
			}
			else
			{
				crc = (crc << 1);
			}
			current_byte >>= 1; // Shift the local copy, not the original array
		}
	}
	return crc;
}

// Write to TMC2209 Register
void writeRegisterUART(uint8_t icID, uint8_t reg, int32_t val)
{

	uint8_t msg[8];
	msg[0] = 0x05;		 // Sync
	msg[1] = 0x00;		 // Slave Address 0
	msg[2] = reg | 0x80; // Write bit
	msg[3] = (val >> 24) & 0xFF;
	msg[4] = (val >> 16) & 0xFF;
	msg[5] = (val >> 8) & 0xFF;
	msg[6] = (val) & 0xFF;
	msg[7] = CRC8(msg, 7);
	// 1. Switch to Transmit Mode
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	HAL_UART_Transmit(&huart1, msg, 8, 50);

	// 2. Switch back to Receiver Mode to keep the line ready
	HAL_HalfDuplex_EnableReceiver(&huart1);
}

int32_t readRegisterUART(uint8_t icID, uint8_t address)
{
	uint8_t tx[4];
	uint8_t rx[8];
	int32_t value = 0;

	tx[0] = 0x05;
	tx[1] = 0x00;
	tx[2] = address & 0x7F; // Read bit
	tx[3] = CRC8(tx, 3);

	__HAL_UART_CLEAR_OREFLAG(&huart1);

	// 1. Talk: Send request
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	HAL_UART_Transmit(&huart1, tx, 4, 10);

	// 2. Listen: Release the line and wait for reply
	HAL_HalfDuplex_EnableReceiver(&huart1);
	if (HAL_UART_Receive(&huart1, rx, 8, 50) == HAL_OK)
	{
		if (rx[0] == 0x05 && rx[2] == address)
		{
			value = (rx[3] << 24) | (rx[4] << 16) | (rx[5] << 8) | rx[6];
		}
	}
	return value;
}
// For a single motor setup, this is usually 0.
uint8_t tmc2209_getNodeAddress(uint8_t icID)
{
	// If your icID 0 is set to MS1/MS2 = GND, return 0.
	// If you add more motors, you can use a switch case or return icID.
	return (uint8_t)icID;
}
void tmc2209_writeRegister(uint8_t icID, uint8_t address, int32_t value)
{
	writeRegisterUART(icID, (uint8_t)address, value);
}

int32_t tmc2209_readRegister(uint8_t icID, uint8_t address)
{
	return readRegisterUART(icID, (uint8_t)address);
}

// // FIXED Bridge Function in TMC2209_Register.c
// bool tmc2209_readWriteUART(uint8_t icID, uint8_t *data, size_t writeLength, size_t readLength)
// {
// 	__HAL_UART_CLEAR_OREFLAG(&huart1); // Clear any Overrun errors

// 	// 1. Send the Request
// 	if (HAL_UART_Transmit(&huart1, data, writeLength, HAL_MAX_DELAY) != HAL_OK)
// 		return false;

// 	// 2. ALWAYS DISCARD ECHO (Required for both Reads and Writes in Half-Duplex)
// 	uint8_t dummy[16];
// 	if (HAL_UART_Receive(&huart1, dummy, writeLength, 50) != HAL_OK)
// 	{
// 		return false; // Error: We didn't even hear our own echo
// 	}

// 	// 3. Handle the Reply (Only for Read operations)
// 	if (readLength > 0)
// 	{
// 		if (HAL_UART_Receive(&huart1, data, readLength, 100) != HAL_OK)
// 		{
// 			return false; // Error: No response from driver
// 		}
// 	}
// 	return true;
// }
