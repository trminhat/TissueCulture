/*
 * PWMStepper.h
 *
 *  Created on: Mar 23, 2026
 *      Author: tranminhnhat
 */

#ifndef STEPPERMOTOR_PWMSTEPPER_H_
#define STEPPERMOTOR_PWMSTEPPER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


void pwm_set_freq(uint32_t freq);
void set_acceleration(uint32_t accel);
void set_deceleration(uint32_t decel);
void set_speed(uint32_t speed);
void set_max_speed(uint32_t max_speed);
void set_direction(bool dir);
void stop_motor();


#endif /* STEPPERMOTOR_PWMSTEPPER_H_ */
