/*
 * TMC2209.c
 *
 *  Created on: Jan 14, 2026
 *      Author: tranminhnhat
 */

#include <Stepper.h>

// extern TIM_HandleTypeDef htim3;
//
void init(uint8_t motor_id)
{

    tmc2209_init(motor_id);

    HAL_GPIO_WritePin(ENA_DRV_X_GPIO_Port, ENA_DRV_X_Pin, GPIO_PIN_RESET);
}

void rms_current(uint8_t motor_id, uint16_t mA)
{
    uint8_t cs; // current scale value for TMC2209
    // Convert mA to the appropriate CS value based on TMC2209 specifications
    cs = 32.0 * 1.41421 * mA / 1000.0 * (Rsense + 0.02) / VFS_Default_mA - 1;
    if (cs < 16)
    {
        tmc2209_vsense(true); // Enable vsense for lower current settings
        cs = 32.0 * 1.41421 * mA / 1000.0 * (Rsense + 0.02) / VFS_HighSensitivy_mA - 1;
    }
    else
    {
        tmc2209_vsense(false); // Disable vsense for higher current settings
    }
    if (cs > 31)
        cs = 31; // Cap CS value at 31

    tmc2209_IRUN(motor_id, cs);                              // Set run current scale
    tmc2209_IHOLD(motor_id, (uint8_t)(cs * holdMultiplier)); // Set hold current to a fraction of run current
}

void hold_current(uint8_t motor_id, uint8_t ihold_value)
{
    tmc2209_IHOLD(motor_id, ihold_value);
}

void microsteps(uint8_t motor_id, uint8_t mres_value)
{
    tmc2209_mres(motor_id, mres_value);
}

void silent_mode(bool enable)
{
    tmc2209_en_SpreadCycle(!enable); // SpreadCycle is the default mode, so we invert the logic here
}

bool who_am_i(uint8_t motor_id)
{
    return tmc2209_who_am_i(motor_id);
}
