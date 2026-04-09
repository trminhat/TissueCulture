/*******************************************************************************
 * Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices Inc.),
 *
 * Copyright © 2024 Analog Devices Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 *******************************************************************************/

#include "TMC2209.h"

void tmc2209_init(uint8_t motor_id)
{

    tmc2209_fieldWrite(motor_id, TMC2209_PDN_DISABLE_FIELD, 0); // Ensure UART is enabled (pdn_disable = 0)
    tmc2209_fieldWrite(0, TMC2209_INTERNAL_RSENSE_FIELD, 0);

    tmc2209_fieldWrite(motor_id, TMC2209_EN_SPREADCYCLE_FIELD, 0);   // Disable spreadCycle
    tmc2209_fieldWrite(motor_id, TMC2209_MSTEP_REG_SELECT_FIELD, 1); // Use MSTEP register for microstepping configuration

    // How long to wait before dropping from IRUN to IHOLD (0-15)
    tmc2209_fieldWrite(motor_id, TMC2209_IHOLDDELAY_FIELD, 6);

    // --- STEP 3: STANDBY TIMING ---
    // TPOWERDOWN: Time after motor stops before IHOLD is applied.
    // Setting this to ~10 (approx 0.5s to 1s depending on clock).
    tmc2209_fieldWrite(motor_id, TMC2209_TPOWERDOWN_FIELD, 10);

    // 1. Enable Automatic Amplitude Scaling
    tmc2209_fieldWrite(motor_id, TMC2209_PWM_AUTOSCALE_FIELD, 1);

    // 2. Enable Automatic Gradient Adaptation
    tmc2209_fieldWrite(motor_id, TMC2209_PWM_AUTOGRAD_FIELD, 1);

    // 3. Set standard PWM frequency (2/1024 fclk is standard for 16MHz)
    tmc2209_fieldWrite(motor_id, TMC2209_PWM_FREQ_FIELD, 1);

    // 4. Set baseline starting values (Standard for NEMA17)
    tmc2209_fieldWrite(motor_id, TMC2209_PWM_OFS_FIELD, 36);
    tmc2209_fieldWrite(motor_id, TMC2209_PWM_GRAD_FIELD, 14);

    HAL_Delay(500);
}

bool tmc2209_who_am_i(uint8_t motor_id)
{
    uint32_t version = tmc2209_fieldRead(motor_id, TMC2209_VERSION_FIELD);

    if (version >= 0x21)
        return true;
    else
        return false;
}

void tmc2209_mres(uint8_t motor_id, uint8_t mres_value)
{
    tmc2209_fieldWrite(motor_id, TMC2209_MRES_FIELD, mres_value);
}
void tmc2209_IRUN(uint8_t motor_id, uint8_t irun_value)
{
    tmc2209_fieldWrite(motor_id, TMC2209_IRUN_FIELD, irun_value);
}
void tmc2209_IHOLD(uint8_t motor_id, uint8_t ihold_value)
{
    tmc2209_fieldWrite(motor_id, TMC2209_IHOLD_FIELD, ihold_value);
}

void tmc2209_vsense(uint8_t motor_id, bool mode)
{
    tmc2209_fieldWrite(motor_id, TMC2209_VSENSE_FIELD, mode ? 1 : 0);
}

void tmc2209_externalRsense(uint8_t motor_id, bool mode)
{
    // This function allows you to switch between using the internal sense resistor (if available) or an external one.
    tmc2209_fieldWrite(motor_id, TMC2209_INTERNAL_RSENSE_FIELD, mode ? 0 : 1);
}

void tmc2209_en_SpreadCycle(uint8_t motor_id, bool enable)
{
    tmc2209_fieldWrite(motor_id, TMC2209_EN_SPREADCYCLE_FIELD, enable ? 1 : 0);
    if (enable)
    {
        // 1. Set TOFF to 3 (This enables the chopper)
        tmc2209_fieldWrite(motor_id, TMC2209_TOFF_FIELD, 3);

        // 2. Set Blank Time (TBL) to 2 (This ignores switching noise)
        tmc2209_fieldWrite(motor_id, TMC2209_TBL_FIELD, 2);

        // 3. Set Hysteresis to standard values
        tmc2209_fieldWrite(motor_id, TMC2209_HSTRT_FIELD, 5);
        tmc2209_fieldWrite(motor_id, TMC2209_HEND_FIELD, 0);
        tmc2209_fieldWrite(motor_id, TMC2209_EN_SPREADCYCLE_FIELD, 0);
    }
}
