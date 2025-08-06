#include "main.h"


#define DRIVER_X_ADDRESS 0b00  // MS1 and MS2 pins are LOW, so address is 0b00
#define DRIVER_Y1_ADDRESS 0b01  // MS1 is HIGH, MS2 is LOW, so address is 0b01
#define DRIVER_Y2_ADDRESS 0b10  // MS1 is LOW, MS2 is HIGH
#define DRIVER_Z_ADDRESS 0b11  // MS1 and MS2 are HIGH, so address is 0b11

#define MAX_SPEED 100000  // Maximum speed for the steppers
#define ACCELERATION 80000  // Acceleration for the steppers

// TMC2209 UART settings
#define R_SENSE 0.11f  // typical sense resistor


#define MICRO_STEPPING 32  // Microstepping setting for the TMC2209 drivers
#define PULLEY_TEETH 20  // Number of teeth on the pulley
#define BELL_PITCH 2.0f  // Pitch of the GT2 belt in mm
#define STEPS_PER_REVOLUTION 200  // Steps per revolution for the stepper motor


TMC2209Stepper driverX = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_X_ADDRESS);
TMC2209Stepper driverY1 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y1_ADDRESS);
TMC2209Stepper driverY2 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y2_ADDRESS);
TMC2209Stepper driverZ = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Z_ADDRESS); 


AccelStepper stepperX(AccelStepper::DRIVER, X_AXIS_STEP, X_AXIS_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_AXIS_STEP, Y_AXIS_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_AXIS_STEP, Z_AXIS_DIR);

float stepsToMM(long mm){
  long steps = (STEPS_PER_REVOLUTION * MICRO_STEPPING) / (PULLEY_TEETH * BELL_PITCH); // 200 steps per revolution, 32 microsteps, 20 teeth pulley, 2 mm pitch
  float stepsNeeded = mm * steps;     // Convert mm to steps

  return stepsNeeded; // Return the number of steps needed to move the specified distance in mm
}

/**
 * @brief Homes a single axis using a two-pass method for high accuracy.
 * @param axis The AccelStepper object for the axis to be homed (passed by reference).
 * @param limitSwitchPin The GPIO pin for the axis's limit switch.
 */
void home2Pass(AccelStepper &axis, int limitSwitchPin) {
  Serial.println("Homing axis...");

  // --- Save original settings ---
  float originalMaxSpeed = axis.maxSpeed();
  float originalAcceleration = axis.acceleration();

  // --- Pass 1: Fast move to the limit switch ---
  axis.setMaxSpeed(2000.0);      // Use a moderate speed for the first pass
  axis.setAcceleration(1000.0);
  
  // Set a target far in the negative direction
  axis.moveTo(-999999);

  while (digitalRead(limitSwitchPin) == HIGH) {
    axis.run();
  }
  
  // Stop motor and reset position temporarily
  axis.stop();
  axis.setCurrentPosition(0);
  Serial.println("First pass complete.");

  // --- Back off the switch ---
  axis.moveTo(stepsToMM(10)); // Move 5mm away from the switch
  axis.runToPosition();      // Blocking call to ensure it finishes
  Serial.println("Backed off switch.");

  // --- Pass 2: Slow move to the limit switch for accuracy ---
  axis.setMaxSpeed(500.0); // Very slow for precision
  axis.setAcceleration(250.0);
  axis.moveTo(-999999); // Move back towards the switch

  while (digitalRead(limitSwitchPin) == HIGH) {
    axis.run();
  }
  axis.stop();

  // This is the true home position. Set it to 0.
  axis.setCurrentPosition(0);
  Serial.println("Homing complete. Position set to 0.");

  // --- Restore original settings ---
  axis.setMaxSpeed(originalMaxSpeed);
  axis.setAcceleration(originalAcceleration);
}


void checkUARTConnection(TMC2209Stepper &driver, const char *name) {
    Serial.print("Testing UART connection to ");
    Serial.print(name);
    Serial.print("...");
    uint8_t version = driver.version();
    if (version == 0x21) {
        Serial.println(" successful!");
        Serial.print("Version: 0x");
        Serial.println(version, HEX);
    } else {
        Serial.println(" FAILED!");
        Serial.print("Received: 0x");
        Serial.println(version, HEX);
        Serial.println("Check wiring, driver power, and address pins. Halting.");
        while(1); // Stop execution if communication fails
    }
}
void setup(){
  Serial.begin(115200); // Initialize Serial for debugging
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Initialize Serial2 for TMC2209

  pinMode(X_AXIS_LIMIT, INPUT_PULLUP);
  pinMode(Y_AXIS_LIMIT, INPUT_PULLUP);
  // pinMode(Z_AXIS_LIMIT, INPUT_PULLUP);

  /* Setup Driver */
  driverX.begin(); // Initialize TMC2209 driver for X axis
  driverX.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverX.microsteps(32);        // optional
  driverX.en_spreadCycle(true); // use StealthChop
  driverX.pwm_autoscale(true);   // adjust PWM

  driverY1.begin(); // Initialize TMC2209 driver for X axis
  driverY1.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverY1.microsteps(32);        // optional
  driverY1.en_spreadCycle(true); // use StealthChop
  driverY1.pwm_autoscale(true);   // adjust PWM
  
  driverY2.begin(); // Initialize TMC2209 driver for X axis
  driverY2.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverY2.microsteps(32);        // optional
  driverY2.en_spreadCycle(true); // use StealthChop
  // driverY2.pwm_autoscale(true);   // adjust PWM
  
  // --- UART Connection Test ---
  checkUARTConnection(driverX, "X Axis");
  checkUARTConnection(driverY1, "Y1 Axis");
  checkUARTConnection(driverY2, "Y2 Axis");
  // checkUARTConnection(driverZ, "Z Axis");

  home2Pass(stepperX, X_AXIS_LIMIT); // Home X axis
  delay(100);
  home2Pass(stepperY, Y_AXIS_LIMIT); // Home Y axis
  delay(100);
  
  /* Setup Stepper */
  stepperX.setMaxSpeed(MAX_SPEED);     // Set maximum speed for X axis
  stepperX.setAcceleration(ACCELERATION); // Set acceleration for X axis

  stepperY.setMaxSpeed(MAX_SPEED);     // Set maximum speed for Y axis
  stepperY.setAcceleration(ACCELERATION); // Set acceleration for Y axis

}

void loop(){

  stepperX.moveTo(stepsToMM(0)); // Move X axis to 0 mm
  stepperX.runToPosition(); // Wait until the move is complete

  stepperY.moveTo(stepsToMM(0)); // Move Y axis to 0 mm
  stepperY.runToPosition(); // Wait until the move is complete
  delay(2000);

  stepperX.moveTo(stepsToMM(100)); 
  stepperX.runToPosition(); 

  stepperY.moveTo(stepsToMM(100)); 
  stepperY.runToPosition(); 

  /* 150 */
  stepperX.moveTo(stepsToMM(150)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 

  /* 200 */
  stepperX.moveTo(stepsToMM(200)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 
  
  /* 300 */
  stepperX.moveTo(stepsToMM(300)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 
  
  /* 400 */
  stepperX.moveTo(stepsToMM(400)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 

  stepperX.moveTo(stepsToMM(500)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(150)); 
  stepperY.runToPosition(); 
  delay(1000); // Wait for a second before next iteration

  stepperX.moveTo(0);  // go back to home
  stepperY.moveTo(0);  // go back to home
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }  
  
}
