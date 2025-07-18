#include "main.h"


#define DRIVER_X_ADDRESS 0b00  // MS1 and MS2 pins are LOW, so address is 0b00
#define DRIVER_Y1_ADDRESS 0b01  // MS1 is HIGH, MS2 is LOW, so address is 0b01
#define DRIVER_Y2_ADDRESS 0b10  // MS1 is LOW, MS2 is HIGH
#define DRIVER_Z_ADDRESS 0b11  // MS1 and MS2 are HIGH, so address is 0b11


// TMC2209 UART settings
#define R_SENSE 0.11f  // typical sense resistor

TMC2209Stepper driverX = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_X_ADDRESS);
TMC2209Stepper driverY1 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y1_ADDRESS);
TMC2209Stepper driverY2 = TMC2209Stepper(&Serial2, R_SENSE, DRIVER_Y2_ADDRESS);
// TMC2209Stepper driverZ = TMC2209Stepper(&Serial, 0.11f, 0.11f); 


AccelStepper stepperX(AccelStepper::DRIVER, X_AXIS_STEP, X_AXIS_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_AXIS_STEP, Y_AXIS_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_AXIS_STEP, Z_AXIS_DIR);

float stepsToMM(long mm){
  long steps = (200 * 32) / (20 * 2); // 200 steps per revolution, 32 microsteps, 20 teeth pulley, 2 mm pitch
  float stepsNeeded = mm * steps;     // Convert mm to steps

  return stepsNeeded; // Return the number of steps needed to move the specified distance in mm
}

void homeAxis(AccelStepper axis, int ls) {
  Serial.println("Homing...");

  axis.setMaxSpeed(1000);  // Slow speed for homing
  axis.setAcceleration(500);

  // Move in negative direction until switch is hit
  while (digitalRead(ls) == HIGH) {  // HIGH = not pressed
    axis.moveTo(stepsToMM(axis.currentPosition() - 10));  // Keep moving slowly
    axis.run();
  }

  // Stop motion
  axis.stop();
  delay(100);

  // Optionally back off
  axis.moveTo(stepsToMM(-1));  // Back off 1cm
  while (axis.distanceToGo() != 0) {
    axis.run();
  }

  // Set home coordinate
  axis.setCurrentPosition(0);  // Now we're at position 0
  Serial.println("Homing done!");
}

void setup(){
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Initialize Serial2 for TMC2209


  pinMode(X_AXIS_LIMIT, INPUT_PULLUP);
  pinMode(Y_AXIS_LIMIT, INPUT_PULLUP);
  pinMode(Z_AXIS_LIMIT, INPUT_PULLUP);

  /* Setup Driver */
  driverX.begin(); // Initialize TMC2209 driver for X axis
  driverX.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverX.microsteps(32);        // optional
  driverX.en_spreadCycle(false); // use StealthChop
  driverX.pwm_autoscale(true);   // adjust PWM

  driverY1.begin(); // Initialize TMC2209 driver for X axis
  driverY1.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverY1.microsteps(32);        // optional
  driverY1.en_spreadCycle(false); // use StealthChop
  driverY1.pwm_autoscale(true);   // adjust PWM

  driverY2.begin(); // Initialize TMC2209 driver for X axis
  driverY2.rms_current(450);      // Setting 80% of current rate limit of stepper motor in mA
  driverY2.microsteps(32);        // optional
  driverY2.en_spreadCycle(false); // use StealthChop
  driverY2.pwm_autoscale(true);   // adjust PWM



  
  homeAxis(stepperX, X_AXIS_LIMIT); // Home X axis
  delay(100);
  homeAxis(stepperY, Y_AXIS_LIMIT); // Home Y axis
  delay(100);
  
  /* Setup Stepper */
  stepperX.setMaxSpeed(100000);     // Set maximum speed for X axis
  stepperX.setAcceleration(15000); // Set acceleration for X axis

  stepperY.setMaxSpeed(100000);     // Set maximum speed for Y axis
  stepperY.setAcceleration(15000); // Set acceleration for Y axis
  //  stepperX.setCurrentPosition(0);  // Now we're at position 0
  //  stepperY.setCurrentPosition(0);  // Now we're at position 0
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

  // /* 200 */
  stepperX.moveTo(stepsToMM(200)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 
  
  // /* 300 */
  stepperX.moveTo(stepsToMM(300)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 
  
  // /* 400 */
  stepperX.moveTo(stepsToMM(400)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(50)); 
  stepperY.runToPosition(); 

  stepperX.moveTo(stepsToMM(500)); 
  stepperX.runToPosition(); 
  stepperY.move(stepsToMM(200)); 
  stepperY.runToPosition(); 
  delay(1000); // Wait for a second before next iteration

  stepperX.moveTo(0);  // go back to home
  stepperY.moveTo(0);  // go back to home
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }  
  
}
