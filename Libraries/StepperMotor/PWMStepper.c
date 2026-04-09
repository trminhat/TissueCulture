/*
 * PWMStepper.c
 *
 *  Created on: Mar 23, 2026
 *      Author: tranminhnhat
 */

#include "PWMStepper.h"

void pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t freq)
{
    if (freq == 0)
        return; // Avoid division by zero

    uint32_t timer_clk;
    uint32_t ppre; // APB Prescaler value
    if (htim->Instance == TIM1 || htim->Instance == TIM9 ||
        htim->Instance == TIM10 || htim->Instance == TIM11)
    {
        timer_clk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    }
    else
    {
        timer_clk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    }

    /* * STM32 Clock Tree Rule:
     * If the APB prescaler is 1 (PPRE value 0, 1, 2, or 3 are reserved/div1), multiplier is 1.
     * In STM32F4, PPRE bit 2 (the MSB) tells us if division is happening.
     * If PPRE < 4 (binary 100), the divider is 1.
     */
    // If APB prescaler is not 1, timer clock is multiplied by 2
    if (ppre >= 4)
        timer_clk *= 2;

    uint32_t total_div = timer_clk / freq;
    uint32_t psc = 0;
    uint32_t arr = total_div - 1;

    // If ARR is too big for a 16-bit timer ( > 65535)
    if (arr > 65535)
    {
        // Increase PSC until ARR fits
        psc = (total_div / 65536);
    }
    arr = (total_div / (psc + 1)) - 1;
    htim->Instance->PSC = psc;
    htim->Instance->ARR = arr;
    htim->Instance->CCR1 = (arr + 1) / 2; // Set duty cycle to 50% by default

    // Generate an update event to apply PSC immediately
    htim->Instance->EGR = TIM_EGR_UG;
}

void set_acceleration(uint32_t accel)
{
    // Placeholder for acceleration control logic
    // This would typically involve ramping the PWM frequency up or down over time
}

void set_deceleration(uint32_t decel)
{
    // Placeholder for deceleration control logic
    // This would typically involve ramping the PWM frequency down over time
}

void set_speed(uint32_t speed)
{
    // Placeholder for speed control logic
    // This could involve directly setting the PWM frequency or implementing a speed ramp
}

void set_max_speed(uint32_t max_speed)
{
    // Placeholder for max speed control logic
    // This could involve setting a maximum PWM frequency or implementing a speed ramp
}

void set_direction(bool dir)
{
    // Placeholder for direction control logic
    // This would typically involve setting a GPIO pin high or low to control motor direction
}

void stop_motor(TIM_HandleTypeDef *htim)
{
    // Placeholder for motor stop logic
    // This could involve setting the PWM duty cycle to 0 or disabling the timer output
    htim->Instance->CCR1 = 0; // Set duty cycle to 0 to stop the motor
}
