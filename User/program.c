/*
 * program.c
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#include "program.h"

void motor_begin(uint8_t motor_id)
{
    init(motor_id);
}
void motor_microstep(uint8_t motor_id, uint8_t mres_value)
{
    microsteps(motor_id, mres_value);
}
void motor_rms_current(uint8_t motor_id, uint16_t mA)
{
    rms_current(motor_id, mA);
}
void motor_hold_current(uint8_t motor_id, uint16_t mA_hold)
{
    hold_current(motor_id, mA_hold);
}

void motor_silent_mode(uint8_t motor_id, bool enable)
{
    silent_mode(motor_id, enable);
}

void check_motor_addr(uint8_t motor_id)
{
    if (who_am_i(motor_id))
    {
        printf("Motor with ID 0x%02X is detected and responding.\n", motor_id);
    }
    else
    {
        printf("Motor with ID 0x%02X is NOT responding. Check connections.\n", motor_id);
        while (1)
            ; // Stop execution if communication fails
    }
}
