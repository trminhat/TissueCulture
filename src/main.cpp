#include "main.h"

Control control(600, 600, 300); // Initialize Control with work area dimensions
FeedBags feedBags = {10, 60, 60, 30}; // Initialize feed bags
Foils foils = {100, 6, 6, 3}; // Initialize foils

void setup(){
  Serial.begin(115200); // Initialize Serial for debugging
  control.setup(); // Setup the control system
  control.setupMaterial(feedBags, foils); // Setup material properties
  control.Homing(); // Perform homing for all axes
  Serial.println("Setup complete. All axes homed.");

  // Control::getStepperX().setMaxSpeed(MAX_SPEED); // Set max speed for X stepper
  // Control::getStepperY().setMaxSpeed(MAX_SPEED); // Set max speed for
  // Control::getStepperX().setAcceleration(ACCELERATION); // Set acceleration for X stepper
  // Control::getStepperY().setAcceleration(ACCELERATION); // Set acceleration for Y

  // Control::getStepperX().moveTo(100000);
  // Control::getStepperY().moveTo(100000); // Move both steppers to a position
  // Serial.println("Steppers are moving to position 100000.");
}

void loop(){
  // Control::getStepperX().run(); // Continuously run the X stepper
  // Control::getStepperY().run(); // Continuously run the Y stepper
 
}
