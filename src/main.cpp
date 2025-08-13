#include "main.h"

Control control(600, 600, 300); // Initialize Control with work area dimensions
FeedBags feedBags = {10, 60, 60, 30}; // Initialize feed bags
Foils foils = {100, 6, 6, 3}; // Initialize foils

void setup(){
  Serial.begin(115200); // Initialize Serial for debugging
  control.setup(); // Setup the control system
  control.setSpreadCycle(); // Set SpreadCycle mode for all drivers
  control.setupMaterial(feedBags, foils); // Setup material properties
  control.Homing(); // Perform homing for all axes
  Serial.println("Setup complete. All axes homed.");

  Control::getStepperX().setMaxSpeed(MAX_SPEED); // Set max speed for X stepper
  Control::getStepperY().setMaxSpeed(MAX_SPEED); // Set max speed for
  Control::getStepperX().setAcceleration(ACCELERATION); // Set acceleration for X stepper
  Control::getStepperY().setAcceleration(ACCELERATION); // Set acceleration for Y

  Control::getStepperX().moveTo(1000000);
  Control::getStepperY().moveTo(1000000); // Move both steppers to a position

}

void loop(){
  Control::getStepperX().run(); // Continuously run the X stepper
  Control::getStepperX().runToPosition(); // Run to the target position for X stepper
  Control::getStepperY().run(); // Continuously run the Y stepper
  Control::getStepperY().runToPosition(); // Run to the target position for
  long backHomeX = control.getHomeX(); // Get home position for X axis
  long backHomeY = control.getHomeY(); // Get home position for Y axis
  Serial.printf("X Position: %ld, Y Position: %ld\n", 
                backHomeX, 
                backHomeY); // Print current positions of X and Y steppers 
  delay(100); // Delay for stability

  Control::getStepperX().moveTo(backHomeX);
  Control::getStepperY().moveTo(backHomeX); // Move both steppers to a position
  Control::getStepperX().run(); // Run to the target position for X stepper
  Control::getStepperY().run(); // Run to the target position for Y

}
