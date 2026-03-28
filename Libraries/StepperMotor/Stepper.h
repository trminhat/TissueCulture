/*
 * TMC2209.h
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#ifndef STEPPERMOTOR_STEPPER_H_
#define STEPPERMOTOR_STEPPER_H_

#include "main.h"
#include "TMC2209_Register.h"
#include <TMC2209_HW_Abstraction.h>
#include <TMC2209.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

static const float Rsense = 0.11;                // Sense resistor value in Ohm (adjust based on your hardware)
static const float holdMultiplier = 0.5;         // Hold current as a fraction of run current (0.5 = 50%)
static const float VFS_Default_mA = 0.325;       // Default voltage for current scaling (adjust based on your setup)
static const float VFS_HighSensitivy_mA = 0.180; // Voltage for vsense current scaling (adjust based on your setup)

void init(uint8_t motor_id);
bool who_am_i(uint8_t motor_id);
void rms_current(uint8_t motor_id, uint16_t mA);
void hold_current(uint8_t motor_id, uint8_t ihold_value);
void microsteps(uint8_t motor_id, uint8_t mres_value);
void silent_mode(bool enable);

#endif /* STEPPERMOTOR_STEPPER_H_ */
