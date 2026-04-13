/*
 * TMC2209_Register.h
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#ifndef STEPPERMOTOR_TMC2209_REGISTER_H_
#define STEPPERMOTOR_TMC2209_REGISTER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct
{
    uint32_t mask;
    uint8_t shift;
    uint8_t address;
    bool isSigned;
} RegisterField;

uint8_t tmc2209_getNodeAddress(uint8_t icID);
// bool tmc2209_readWriteUART(uint8_t icID, uint8_t *data, size_t writeLength, size_t readLength);
int32_t tmc2209_readRegister(uint8_t icID, uint8_t address);
void tmc2209_writeRegister(uint8_t icID, uint8_t address, int32_t value);

uint8_t CRC8(const uint8_t *data, uint8_t len);
void writeRegisterUART(uint8_t icID, uint8_t reg, int32_t val);
int32_t readRegisterUART(uint8_t icID, uint8_t address);

static inline int32_t tmc2209_fieldExtract(int32_t data, RegisterField field)
{
    int32_t value = (data & field.mask) >> field.shift;

    if (field.isSigned)
    {
        // Apply signedness conversion
        uint32_t baseMask = field.mask >> field.shift;
        uint32_t signMask = baseMask & (~baseMask >> 1);
        value = (value ^ signMask) - signMask;
    }

    return value;
}

static inline int32_t tmc2209_fieldRead(uint8_t icID, RegisterField field)
{
    int32_t value = tmc2209_readRegister(icID, field.address);

    return tmc2209_fieldExtract(value, field);
}

static inline int32_t tmc2209_fieldUpdate(int32_t data, RegisterField field, uint32_t value)
{
    return (data & (~field.mask)) | ((value << field.shift) & field.mask);
}

static inline void tmc2209_fieldWrite(uint8_t icID, RegisterField field, int32_t value)
{
    int32_t regValue = tmc2209_readRegister(icID, field.address);

    regValue = tmc2209_fieldUpdate(regValue, field, value);

    tmc2209_writeRegister(icID, field.address, regValue);
}

#endif /* STEPPERMOTOR_TMC2209_REGISTER_H_ */
