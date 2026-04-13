/*
 * SerialUART.c
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#include "SerialUART.h"


extern UART_HandleTypeDef huart2;

/*
    _write is a "System Call" implementation.
    In the STM32 toolchain (specifically arm-none-eabi-gcc), the library needs a way to interface with "files."

    + Top Level: You call printf("Hello").
    + Middle Level (_write): The C library (Newlib) calls _write. It hands over a buffer (the whole string "Hello") and tells it how many characters to send.

*/
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
