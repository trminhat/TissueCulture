#include "main.h"

Control control(600, 600, 300); // Initialize Control with work area dimensions
FeedBags feedBags = {2, 2, 60, 30, 300, 300}; // Initialize feed bags
Foils foils = {2, 60, 3, 10, 10}; // Initialize foils
FoilsHolder holder = {100, 60, 30}; // Initialize foil holder (if needed)

uint16_t qtyBags = feedBags.column * feedBags.row; // Calculate total quantity of feed bags

void setup(){
  Serial.begin(115200); // Initialize Serial for debugging
  control.setup(feedBags, foils, holder); // Setup the control system
  control.setSpreadCycle(); // Set SpreadCycle mode for all drivers
  // control.setStealthChop(); // Set StealthChop mode for all drivers
  Serial.println("Setup complete. All axes homed.");
  
  Control::getStepperX().setMaxSpeed(MAX_SPEED); // Set max speed for X stepper
  Control::getStepperY().setMaxSpeed(MAX_SPEED); // Set max speed for
  Control::getStepperX().setAcceleration(ACCELERATION); // Set acceleration for X stepper
  Control::getStepperY().setAcceleration(ACCELERATION); // Set acceleration for Y
  Control::getStepperZ().setMaxSpeed(MAX_SPEED); // Set max speed for Z stepper
  Control::getStepperZ().setAcceleration(ACCELERATION); // Set acceleration for Z

  // Control::getStepperZ().setCurrentPosition(0); // Reset current position for X stepper
 
  control.Homing(); // Perform homing for all axes
  delay(1000); // Wait for a second after homing
}

void loop(){
  control.goToHolder();
  delay(1000); // Wait for a second before next operation
  while(control.getCurrentBags() <= qtyBags){
    control.goToFeedBags(); // Move to feed bags
    delay(10);
  }

  // Control::getStepperZ().moveTo(control.distanceMM_ZAxis(50)); // Move Z axis to 100mm position
  // Control::getStepperZ().runToPosition(); // Blocking call to ensure it finishes
  // // while(Control::getStepperZ().distanceToGo() != 0){
  // //   Control::getStepperZ().run();
  // // }
  // delay(2000); // Wait for 2 seconds
  // control.GoBackToHome(); // Move all axes back to home position
  // delay(2000); // Wait for 2 seconds

}
