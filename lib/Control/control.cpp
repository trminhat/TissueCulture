#include "control.h"

// Static shared stepper instances (members of Control)
AccelStepper Control::stepperX(AccelStepper::DRIVER, X_AXIS_STEP, X_AXIS_DIR);
AccelStepper Control::stepperY(AccelStepper::DRIVER, Y_AXIS_STEP, Y_AXIS_DIR);
AccelStepper Control::stepperZ(AccelStepper::DRIVER, Z_AXIS_STEP, Z_AXIS_DIR);

Control::Control(uint16_t workLength, uint16_t workWidth, uint16_t workHeight)
{
    
    workArea.length = workLength;
    workArea.width = workWidth;
    workArea.height = workHeight;

    feedBags.qty = 0; // Initialize feed bags quantity
    foils.qty = 0;    // Initialize foils quantity
}


void Control::setup(FeedBags _feedBags, Foils _foils)
{
    Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Initialize Serial2 for TMC2209
    setupMaterial(_feedBags, _foils); // Setup material properties
    pinMode(limitX, INPUT_PULLUP); // Set limit switch pins as input with pull-up
    pinMode(limitY, INPUT_PULLUP);
    // pinMode(limitZ, INPUT_PULLUP); // Not used, but defined for consistency

    // Setup Driver
    driverX.begin();                    // Initialize TMC2209 driver for X axis
    driverX.rms_current(450);           // Setting 80% of current rate limit of stepper motor in mA
    driverX.microsteps(MICRO_STEPPING); // Set microstepping to 32

    driverY1.begin(); // Initialize TMC2209 driver for Y1 axis
    driverY1.rms_current(450);
    driverY1.microsteps(MICRO_STEPPING);

    driverY2.begin(); // Initialize TMC2209 driver for Y2 axis
    driverY2.rms_current(450);
    driverY2.microsteps(MICRO_STEPPING);
    driverY2.pwm_autoscale(true);

    driverZ.begin(); // Initialize TMC2209 driver for Z axis
    driverZ.rms_current(450);
    driverZ.microsteps(MICRO_STEPPING);

    // Check UART connection for each driver
    checkConnection(driverX, "X Axis");
    checkConnection(driverY1, "Y1 Axis");
    checkConnection(driverY2, "Y2 Axis");
    checkConnection(driverZ, "Z Axis");
}
void Control::setSpreadCycle()
{
    // Set all drivers to SpreadCycle mode
    driverX.en_spreadCycle(true);
    driverY1.en_spreadCycle(true);
    driverY2.en_spreadCycle(true);
    driverZ.en_spreadCycle(true);

    Serial.println("SpreadCycle mode set for all drivers.");
}

void Control::setStealthChop()
{
    // Set all drivers to StealthChop mode
    driverX.en_spreadCycle(false);
    driverY1.en_spreadCycle(false);
    driverY2.en_spreadCycle(false);
    driverZ.en_spreadCycle(false);

    driverX.pwm_autoscale(true);        // Enable automatic scaling of PWM frequency
    driverY1.pwm_autoscale(true);       // Enable automatic scaling of PWM frequency
    driverY2.pwm_autoscale(true);       // Enable automatic scaling of PWM frequency
    driverZ.pwm_autoscale(true);        // Enable automatic scaling of PWM frequency

    Serial.println("StealthChop mode set for all drivers.");
}

void Control::setupMaterial(FeedBags _feedBags, Foils _foils)
{
    // Setup material properties
    this->feedBags = _feedBags; // Store feed bags information
    this->foils = _foils;       // Store foils information

    Serial.print("Feed Bags: ");
    Serial.print(feedBags.qty);
    Serial.print("pcs, Length: ");
    Serial.print(feedBags.length);
    Serial.print("mm, Width: ");
    Serial.print(feedBags.width);
    Serial.print("mm, Height: ");
    Serial.println(feedBags.height);
    Serial.print("Clearance: ");
    Serial.println(feedBags.clearance);

    Serial.print("Foils: ");
    Serial.print(foils.qty);
    Serial.print("pcs, Length: ");
    Serial.print(foils.length);
    Serial.print("mm, Width: ");
    Serial.print(foils.width);
    Serial.print("mm, Height: ");
    Serial.println(foils.height);
    Serial.print("Clearance: ");
    Serial.println(foils.clearance);

}

void Control::checkConnection(TMC2209Stepper &driver, const char *axisName)
{
    Serial.print("Testing UART connection to ");
    Serial.print(axisName);
    Serial.print("...");
    uint8_t version = driver.version();
    if (version == 0x21)
    {
        Serial.println(" successful!");
        Serial.print("Version: 0x");
        Serial.println(version, HEX);
    }
    else
    {
        Serial.println(" FAILED!");
        Serial.print("Received: 0x");
        Serial.println(version, HEX);
        Serial.println("Check wiring, driver power, and address pins. Halting.");
        while (1)
            ; // Stop execution if communication fails
    }
}

float Control::distanceMM(long mm)
{
    long steps = (STEPS_PER_REVOLUTION * MICRO_STEPPING) / (PULLEY_TEETH * BELL_PITCH); // 200 steps per revolution, 32 microsteps, 20 teeth pulley, 2 mm pitch
    float stepsNeeded = mm * steps;                                                     // Convert mm to steps
    return stepsNeeded;
}

long Control::currentMM(AccelStepper &stepper)
{
    long currentPositionSteps = stepper.currentPosition(); // Get current position in steps
    long currentPositionMM  = currentPositionSteps * (PULLEY_TEETH * BELL_PITCH) / (STEPS_PER_REVOLUTION * MICRO_STEPPING); // Convert steps to mm
    return currentPositionMM;
  
}

void Control::Homing()
{
    // Perform homing for all axes
    XHoming();
    YHoming();
    // ZHoming();
}

void Control::XHoming()
{
    Serial.println("Homing X axis...");

    // --- Save original settings ---
    float originalMaxSpeed = stepperX.maxSpeed();
    float originalAcceleration = stepperX.acceleration();

    // --- Pass 1: Fast move to the limit switch ---
    stepperX.setMaxSpeed(2000.0); // Use a moderate speed for the first pass
    stepperX.setAcceleration(1000.0);

    // Set a target far in the negative direction
    stepperX.moveTo(-999999);

    while (digitalRead(limitX) == HIGH)
    {
        stepperX.run();
    }

    // Stop motor and reset position temporarily
    stepperX.stop();
    stepperX.setCurrentPosition(0);
    Serial.println("First pass complete.");

    // --- Back off the switch ---
    stepperX.moveTo(distanceMM(10)); // Move 5mm away from the switch
    stepperX.runToPosition();       // Blocking call to ensure it finishes
    Serial.println("Backed off switch.");

    // --- Pass 2: Slow move to the limit switch for accuracy ---
    stepperX.setMaxSpeed(500.0); // Very slow for precision
    stepperX.setAcceleration(250.0);
    stepperX.moveTo(-999999); // Move back towards the switch

    while (digitalRead(limitX) == HIGH)
    {
        stepperX.run();
    }
    stepperX.stop();

    // This is the true home position. Set it to 0.
    stepperX.setCurrentPosition(0);
    HomeX = stepperX.currentPosition(); // Store the home position for X axis
    Serial.println("Homing X complete. Position set to 0.");
    Serial.printf("Home X: %ld\n", HomeX);


    // --- Restore original settings ---
    stepperX.setMaxSpeed(originalMaxSpeed);
    stepperX.setAcceleration(originalAcceleration);
}

void Control::YHoming()
{
    Serial.println("Homing Y axis...");

    // --- Save original settings ---
    float originalMaxSpeed = stepperY.maxSpeed();
    float originalAcceleration = stepperY.acceleration();

    // --- Pass 1: Fast move to the limit switch ---
    stepperY.setMaxSpeed(2000.0); // Use a moderate speed for the first pass
    stepperY.setAcceleration(1000.0);

    // Set a target far in the negative direction
    stepperY.moveTo(-999999);

    while (digitalRead(limitY) == HIGH)
    {
        stepperY.run();
    }

    // Stop motor and reset position temporarily
    stepperY.stop();
    stepperY.setCurrentPosition(0);
    Serial.println("First pass complete.");

    // --- Back off the switch ---
    stepperY.moveTo(distanceMM(10)); // Move 5mm away from the switch
    stepperY.runToPosition();       // Blocking call to ensure it finishes
    Serial.println("Backed off switch.");

    // --- Pass 2: Slow move to the limit switch for accuracy ---
    stepperY.setMaxSpeed(500.0); // Very slow for precision
    stepperY.setAcceleration(250.0);
    stepperY.moveTo(-999999); // Move back towards the switch

    while (digitalRead(limitY) == HIGH)
    {
        stepperY.run();
    }
    stepperY.stop();

    // This is the true home position. Set it to 0.
    stepperY.setCurrentPosition(0);
    HomeY = stepperY.currentPosition(); // Store the home position for Y axis
    Serial.println("Homing Y complete. Position set to 0.");
    Serial.printf("Home Y: %ld", HomeY);

    // --- Restore original settings ---
    stepperY.setMaxSpeed(originalMaxSpeed);
    stepperY.setAcceleration(originalAcceleration);
}

// void Control::ZHoming() {
//     Serial.println("Homing Z axis...");

//     // --- Save original settings ---
//     float originalMaxSpeed = stepperZ.maxSpeed();
//     float originalAcceleration = stepperZ.acceleration();

//     // --- Pass 1: Fast move to the limit switch ---
//     stepperZ.setMaxSpeed(2000.0);      // Use a moderate speed for the first pass
//     stepperZ.setAcceleration(1000.0);

//     // Set a target far in the negative direction
//     stepperZ.moveTo(-999999);

//     while (digitalRead(limitZ) == HIGH) {
//         stepperZ.run();
//     }

//     // Stop motor and reset position temporarily
//     stepperZ.stop();
//     stepperZ.setCurrentPosition(0);
//     Serial.println("First pass complete.");

//     // --- Back off the switch ---
//     stepperZ.moveTo(stepsToMM(10)); // Move 5mm away from the switch
//     stepperZ.runToPosition();      // Blocking call to ensure it finishes
//     Serial.println("Backed off switch.");

//     // --- Pass 2: Slow move to the limit switch for accuracy ---
//     stepperZ.setMaxSpeed(500.0); // Very slow for precision
//     stepperZ.setAcceleration(250.0);
//     stepperZ.moveTo(-999999); // Move back towards the switch

//     while (digitalRead(limitZ) == HIGH) {
//         stepperZ.run();
//     }
//     stepperZ.stop();

//     // This is the true home position. Set it to 0.
//     stepperZ.setCurrentPosition(0);
//     HomeZ = stepperZ.currentPosition(); // Store the home position for Z axis
//     Serial.println("Homing Z complete. Position set to 0.");

//     // --- Restore original settings ---
//     stepperZ.setMaxSpeed(originalMaxSpeed);
//     stepperZ.setAcceleration(originalAcceleration);
// }

void Control::GoBackToHome()
{
    // Move all axes back to their home positions
    XBackToHome();
    YBackToHome();
    ZBackToHome();
}

void Control::XBackToHome()
{
    Serial.println("Moving X axis back to home position...");
    stepperX.moveTo(HomeX); // Move to the stored home position
    stepperX.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("X axis back to home position.");
}
void Control::YBackToHome()
{
    Serial.println("Moving Y axis back to home position...");
    stepperY.moveTo(HomeY); // Move to the stored home position
    stepperY.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("Y axis back to home position.");
}
void Control::ZBackToHome()
{
    Serial.println("Moving Z axis back to home position...");
    stepperZ.moveTo(HomeZ); // Move to the stored home position
    stepperZ.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("Z axis back to home position.");
}

void Control::goToFeedBags()
{
    stepperX.moveTo(distanceMM(100)); // Move X stepper to the specified distance in mm
    stepperX.runToPosition(); // Blocking call to ensure it finishes
    stepperY.moveTo(distanceMM(300)); // Move Y stepper to the specified distance
    stepperY.runToPosition(); // Blocking call to ensure it finishes

    Serial.printf("Stepper X is moved: %ld mm\t", currentMM(stepperX));
    Serial.printf("Stepper Y is moved: %ld mm\n", currentMM(stepperY));
    delay(2000);

     stepperX.move(distanceMM(400)); // Move X stepper to the specified distance in mm
    stepperX.runToPosition(); // Blocking call to ensure it finishes
    stepperY.move(distanceMM(200)); // Move Y stepper to the specified distance
    stepperY.runToPosition(); // Blocking call to ensure it finishes

    Serial.printf("Stepper X is moved: %ld mm\t", currentMM(stepperX));
    Serial.printf("Stepper Y is moved: %ld mm\n", currentMM(stepperY));
    delay(2000);

    // while(currentBag < feedBags.qty) {
    //     Serial.print("Moving to feed bag ");
        

    //     currentBag++;
    // }
}

void Control::goToFoils()
{
    while(currentFoil < foils.qty) {
        Serial.print("Moving to foil ");
        // Implement movement logic here
        currentFoil++;
    }
}

// Getter implementations for external access
AccelStepper &Control::getStepperX() { return stepperX; }
AccelStepper &Control::getStepperY() { return stepperY; }
AccelStepper &Control::getStepperZ() { return stepperZ; }
