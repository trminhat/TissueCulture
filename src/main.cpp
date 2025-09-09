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
  // Serial.printf("Finished Sequency\n");
  // delay(10000); // Wait for a second before next operation
}
