#include "main.h"

Control control(600, 600, 300); // Initialize Control with work area dimensions
FeedBags feedBags = {10, 60, 60, 30, 100}; // Initialize feed bags
Foils foils = {100, 6, 6, 3, 10}; // Initialize foils

void setup(){
  Serial.begin(115200); // Initialize Serial for debugging
  control.setup(feedBags, foils); // Setup the control system
  control.setSpreadCycle(); // Set SpreadCycle mode for all drivers
  control.Homing(); // Perform homing for all axes
  Serial.println("Setup complete. All axes homed.");

  Control::getStepperX().setMaxSpeed(MAX_SPEED); // Set max speed for X stepper
  Control::getStepperY().setMaxSpeed(MAX_SPEED); // Set max speed for
  Control::getStepperX().setAcceleration(ACCELERATION); // Set acceleration for X stepper
  Control::getStepperY().setAcceleration(ACCELERATION); // Set acceleration for Y


  // Control::getStepperX().moveTo(1000000);
  // Control::getStepperY().moveTo(1000000); // Move both steppers to a position

}

void loop(){
  control.goToFeedBags(); // Move to feed bags position
}
