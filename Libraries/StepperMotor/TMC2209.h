/*******************************************************************************
 * Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices Inc.),
 *
 * Copyright © 2024 Analog Devices Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 *******************************************************************************/

#ifndef TMC_IC_TMC2209_H_
#define TMC_IC_TMC2209_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "TMC2209_HW_Abstraction.h"
#include "TMC2209_Register.h"

void tmc2209_init(uint8_t motor_id);
bool tmc2209_who_am_i(uint8_t motor_id);
void tmc2209_mres(uint8_t motor_id, uint8_t mres_value);
void tmc2209_IRUN(uint8_t motor_id, uint8_t irun_value);
void tmc2209_IHOLD(uint8_t motor_id, uint8_t ihold_value);
void tmc2209_vsense(uint8_t motor_id, bool mode);
void tmc2209_externalRsense(uint8_t motor_id, bool mode);
void tmc2209_en_SpreadCycle(uint8_t motor_id, bool enable);
#endif /* TMC_IC_TMC2209_H_ */
