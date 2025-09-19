#pragma once

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
#define Z_AXIS_LIMIT 32

#define CW 1
#define CCW 0

#define DRIVER_X_ADDRESS 0b00  // MS1 and MS2 pins are LOW, so address is 0b00
#define DRIVER_Y1_ADDRESS 0b01 // MS1 is HIGH, MS2 is LOW, so address is 0b01
#define DRIVER_Y2_ADDRESS 0b10 // MS1 is LOW, MS2 is HIGH
#define DRIVER_Z_ADDRESS 0b11  // MS1 and MS2 are HIGH, so address is 0b11

// #define MAX_SPEED 200000   // Maximum speed for the steppers
// #define ACCELERATION 100000 // Acceleration for the steppers

#define MAX_SPEED 100000   // Maximum speed for the steppers
#define ACCELERATION 80000 // Acceleration for the steppers

// TMC2209 UART settings
#define R_SENSE 0.11f // typical sense resistor

#define MICRO_STEPPING 32        // Microstepping setting for the TMC2209 drivers
#define PULLEY_TEETH 20          // Number of teeth on the pulley
#define BELL_PITCH 2.0f          // Pitch of the GT2 belt in mm
#define STEPS_PER_REVOLUTION 200 // Steps per revolution for the stepper motor
#define LED_SCREW_PITCH 8.0f       // Lead screw pitch in mm/rev for Z axis

typedef struct FeedBags{
    uint8_t column;
    uint8_t row; // Number of rows of feed bags
    uint16_t radius; // Radius of the feed bag in mm
    uint16_t height; // Height of the feed bag in mm
    uint16_t clearanceX; // Clearance of X direction for the feed bags in mm
    uint16_t clearanceY; // Clearance of Y direction for the feed bags in mm
} FeedBags;

typedef struct Foils{
    uint16_t qty;
    uint16_t radius; // Radius of the feed bag in mm
    uint16_t height; // Height of the foil in mm
    uint16_t clearanceX; // Clearance of X directoion for the feed bags in mm
    uint16_t clearanceY; // Clearance of Y direction for the feed bags in mm

} Foils;

typedef struct WorkArea{
    uint16_t length; // Length of the work area in mm
    uint16_t width;  // Width of the work area in mm
    uint16_t height; // Height of the work area in mm

} WorkArea;
typedef struct FoilHolder{
    uint16_t qtyFoils;
    uint16_t length; // Length of the foil holder in mm
    uint16_t width;  // Width of the foil holder in mm
}FoilsHolder;

class Control
{
public:
    Control(uint16_t workLength, uint16_t workWidth, uint16_t workHeight);
    // Access to shared stepper instances
    void setup(FeedBags _feedBags, Foils _foils, FoilsHolder _holder); // Setup the control system with feed bags and foils
    void setSpreadCycle();
    void setStealthChop();
    void checkConnection(TMC2209Stepper &driver, const char *axisName);
    void Homing();
    void XHoming();
    void YHoming();
    void ZHoming();
    void GoBackToHome();
    void XBackToHome();
    void YBackToHome();
    void ZBackToHome();
    
    long getHomeX() const { return HomeX; }
    long getHomeY() const { return HomeY; }
    long getHomeZ() const { return HomeZ; }
    
    uint16_t getCurrentBags()  const { return currentBag;  }
    uint16_t getCurrentFoils() const { return currentFoil; }

    /* Sequency Functions */
    void goToFeedBags();
    void goToFoils();
    void goToHolder();
    void nextColumnBags();
    void nextRowBags();
    void runSequence();
    
    long distanceMM(int16_t mm);
    long distanceMM_ZAxis(int16_t mm);
    long currentMM(AccelStepper &stepper);

    static AccelStepper &getStepperX();
    static AccelStepper &getStepperY();
    static AccelStepper &getStepperZ();
    ~Control() = default;

private:
    void setupMaterial(FeedBags _feedBags, Foils _foils, FoilsHolder _holder); // Setup material properties
   

    TMC2209Stepper driverX = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_X_ADDRESS);
    TMC2209Stepper driverY1 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y1_ADDRESS);
    TMC2209Stepper driverY2 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y2_ADDRESS);
    TMC2209Stepper driverZ = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Z_ADDRESS);

    // Static shared stepper instances
    static AccelStepper stepperX;
    static AccelStepper stepperY;
    static AccelStepper stepperZ;

    long HomeX;
    long HomeY;
    long HomeZ;


    // Work area dimensions
    WorkArea workArea;
    // Feed bags and foils
    FeedBags feedBags;
    Foils foils;
    FoilsHolder holder;

    
    uint16_t currentBag = 0; // Current bag index
    uint16_t currentFoil = 0; // Current foil index
    bool isLastColumnBags = false; // Flag to check if it's the last column of feed bags
    bool isLastRowBags = false; // Flag to check if it's the last row
    bool isFoilsDoneProcess = false; // Flag to check if foils processing is done
   
    long currentPositionX = 0; // Current position of the X axis in mm
    long currentPositionY = 0; // Current position of the Y axis in mm
    long currentPositionZ = 0; // Current position of the Z axis in mm

    long firstBagPositionX = 0; // Position of the first bag in X axis
    long firstBagPositionY = 0; // Position of the first bag in Y axis
    long firstBagPositionZ = 0; // Position of the first bag in Y axis

};