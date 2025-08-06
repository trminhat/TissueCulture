#ifndef __MAIN_H__
#define __MAIN_H__


#include <Arduino.h>
#include "PWMStepper.h"
#include <AccelStepper.h>
#include <math.h>
#include <TMCStepper.h>


/* NOTICE: DISCONNECT GPIO 0/2/5/12/15/RST FROM ENA PIN. IT WLL MAKE ESP32 CANNOT WORK AFTER FIRST TIME UPLOAD

    + The GPIO 0/2/5/12/15/RST pin is Strapping Pin, which is used to select the boot mode of the ESP32.
    + The ENA pin on the CNC Shield V3 is pull-up to 3v3 normally. This pin need to pull-down to GND to enable the driver.
    + So that, if connect the GPIO12 to ENA pin, it will cause the ESP32 to enter the boot mode and cannot work normally.
    + So, we need to disconnect the GPIO12 from ENA pin.
    + If you want to use the ENA pin, you can use any other GPIO pin to control the ENA pin.
    + The GPIO 12 is used to select the boot mode of the ESP32, so it should not be used as a normal GPIO pin.

*/
// #define ENABLE_DRV_DEFAULT 12  ~> 
#define X_AXIS_STEP 26
#define Y_AXIS_STEP 25
#define Z_AXIS_STEP 23
#define X_AXIS_DIR 18
#define Y_AXIS_DIR 19
#define Z_AXIS_DIR 14

/* Limit Switch */
#define X_AXIS_LIMIT 13
#define Y_AXIS_LIMIT 27
// #define Z_AXIS_LIMIT 23

#define CW 1
#define CCW 0



 


#endif // __MAIN_H__