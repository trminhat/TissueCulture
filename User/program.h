/*
 * program.h
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#ifndef PROGRAM_H_
#define PROGRAM_H_

#include <Stepper.h>
#include "SerialUART.h"
#include "PWMStepper.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MOTOR_ID_0 0x00
#define MOTOR_ID_1 0x01
#define MOTOR_ID_2 0x02
#define MOTOR_ID_3 0x03

#define MICRO_STEPPING_256 0
#define MICRO_STEPPING_128 1
#define MICRO_STEPPING_64 2
#define MICRO_STEPPING_32 3
#define MICRO_STEPPING_16 4
#define MICRO_STEPPING_8 5
#define MICRO_STEPPING_4 6
#define MICRO_STEPPING_2 7
#define MICRO_STEPPING_FULLSTEP 8

void motor_begin(uint8_t motor_id);
void motor_microstep(uint8_t motor_id, uint8_t mres_value);
void motor_rms_current(uint8_t motor_id, uint16_t mA);
void motor_hold_current(uint8_t motor_id, uint16_t mA_hold);
void check_motor_addr(uint8_t motor_id);
void motor_silent_mode(uint8_t motor_id, bool enable);

#endif /* PROGRAM_H_ */
