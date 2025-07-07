#include "main.h"


// PWMStepper axisX(X_AXIS_PIN_DEFAULT, X_AXIS_DIR_DEFAULT, PWM_CHANNEL0, PWM_RESOLUTION_8);
// PWMStepper axisY(Y_AXIS_PIN_DEFAULT, Y_AXIS_DIR_DEFAULT, PWM_CHANNEL2, PWM_RESOLUTION_8);
// PWMStepper axisZ(Z_AXIS_PIN_DEFAULT, Z_AXIS_DIR, PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);

AccelStepper axisX(AccelStepper::DRIVER, X_AXIS_STEP, X_AXIS_DIR);
AccelStepper axisY(AccelStepper::DRIVER, Y_AXIS_STEP, Y_AXIS_DIR);
AccelStepper axisZ(AccelStepper::DRIVER, Z_AXIS_STEP, Z_AXIS_DIR);

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
  Serial.begin(115200);

  pinMode(X_AXIS_LIMIT, INPUT_PULLUP);
  pinMode(Y_AXIS_LIMIT, INPUT_PULLUP);
  pinMode(Z_AXIS_LIMIT, INPUT_PULLUP);


 

  
  homeAxis(axisY, Y_AXIS_LIMIT); // Home Y axis
  delay(100);
  homeAxis(axisX, X_AXIS_LIMIT); // Home X axis
  delay(100);

  axisX.setMaxSpeed(100000);     // Set maximum speed for X axis
  axisX.setAcceleration(50000); // Set acceleration for X axis

  axisY.setMaxSpeed(100000);     // Set maximum speed for Y axis
  axisY.setAcceleration(50000); // Set acceleration for Y axis


  // digitalWrite(ENABLE_DRV_DEFAULT, LOW);
  
}

void loop(){

  axisX.moveTo(stepsToMM(0)); // Move X axis to 0 mm
  axisX.runToPosition(); // Wait until the move is complete

  axisY.moveTo(stepsToMM(0)); // Move Y axis to 0 mm
  axisY.runToPosition(); // Wait until the move is complete
  delay(2000);

  axisX.moveTo(stepsToMM(100)); 
  axisX.runToPosition(); 

  axisY.moveTo(stepsToMM(100)); 
  axisY.runToPosition(); 

  /* 150 */
  axisX.moveTo(stepsToMM(150)); 
  axisX.runToPosition(); 
  axisY.move(stepsToMM(50)); 
  axisY.runToPosition(); 

  /* 200 */
  axisX.moveTo(stepsToMM(200)); 
  axisX.runToPosition(); 
  axisY.move(stepsToMM(50)); 
  axisY.runToPosition(); 
  
  /* 300 */
  axisX.moveTo(stepsToMM(300)); 
  axisX.runToPosition(); 
  axisY.move(stepsToMM(50)); 
  axisY.runToPosition(); 
  
  /* 400 */
  axisX.moveTo(stepsToMM(400)); 
  axisX.runToPosition(); 
  axisY.move(stepsToMM(50)); 
  axisY.runToPosition(); 

  axisX.moveTo(stepsToMM(500)); 
  axisX.runToPosition(); 
  axisY.move(stepsToMM(100)); 
  axisY.runToPosition(); 
  delay(1000); // Wait for a second before next iteration

  axisX.moveTo(0);  // go back to home
  axisY.moveTo(0);  // go back to home
  while (axisX.distanceToGo() != 0 || axisY.distanceToGo() != 0) {
    axisX.run();
    axisY.run();
  }  
}
