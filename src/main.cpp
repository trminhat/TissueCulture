#include "main.h"

Control control(600, 600, 300); // Initialize Control with work area dimensions
FeedBags feedBags = {2, 2, 60, 30, 50, 50}; // Initialize feed bags
Foils foils = {2, 60, 3, 10, 10}; // Initialize foils
FoilsHolder holder = {100, 60, 30}; // Initialize foil holder (if needed)

uint16_t qtyBags = feedBags.column * feedBags.row; // Calculate total quantity of feed bags

void setup(){
  
  Serial.begin(115200); // Initialize Serial for debugging


  control.setup(feedBags, foils, holder); // Setup the control system
  // control.setSpreadCycle(); // Set SpreadCycle mode for all drivers
  // control.setStealthChop(); // Set StealthChop mode for all drivers
  control.setStealthChop('X'); // Set StealthChop mode for X driver
  control.setStealthChop('Y'); // Set SpreadCycle mode for Y drivers
  control.setSpreadCycle('Z'); // Set StealthChop mode for Z
  
  control.setMaxSpeed('X', MAX_SPEED_100KHZ); // Set max speed for X axis
  control.setMaxSpeed('Y', MAX_SPEED_100KHZ); // Set max speed for Y axis
  control.setMaxSpeed('Z', MAX_SPEED_100KHZ);  // Set max speed for Z

  control.setAcceleration('X', ACCELERATION_50KHZ); // Set acceleration for X axis
  control.setAcceleration('Y', ACCELERATION_50KHZ); // Set acceleration for Y axis
  control.setAcceleration('Z', ACCELERATION_80KHZ); // Set acceleration for Z axis 
  
  Serial.println("Setup complete. All axes homed.");

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

}
