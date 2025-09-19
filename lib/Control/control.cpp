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

    feedBags.column = 0;     // Initialize feed bags quantity
    feedBags.row = 0;        // Initialize feed bags quantity
    feedBags.radius = 0;     // Initialize feed bag radius
    feedBags.height = 0;     // Initialize feed bag height
    feedBags.clearanceX = 0; // Initialize clearance in X direction
    feedBags.clearanceY = 0; // Initialize clearance in Y direction

    foils.qty = 0;        // Initialize foils quantity
    foils.radius = 0;     // Initialize foil radius
    foils.height = 0;     // Initialize foil height
    foils.clearanceX = 0; // Initialize clearance in X direction
    foils.clearanceY = 0; // Initialize clearance in Y direction
}

void Control::setup(FeedBags _feedBags, Foils _foils, FoilsHolder _holder)
{
    Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Initialize Serial2 for TMC2209
    setupMaterial(_feedBags, _foils, _holder);   // Setup material properties
    pinMode(X_AXIS_LIMIT, INPUT_PULLUP);               // Set limit switch pins as input with pull-up
    pinMode(Y_AXIS_LIMIT, INPUT_PULLUP);
    pinMode(Z_AXIS_LIMIT, INPUT_PULLUP); // Not used, but defined for consistency

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


    driverZ.begin(); // Initialize TMC2209 driver for Z axis
    driverZ.rms_current(1300); // Setting 80% of current rate limit of stepper motor in mA
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

    driverX.pwm_autoscale(true);  // Enable automatic scaling of PWM frequency
    driverY1.pwm_autoscale(true); // Enable automatic scaling of PWM frequency
    driverY2.pwm_autoscale(true); // Enable automatic scaling of PWM frequency
    driverZ.pwm_autoscale(true);  // Enable automatic scaling of PWM frequency

    Serial.println("StealthChop mode set for all drivers.");
}

void Control::setupMaterial(FeedBags _feedBags, Foils _foils, FoilsHolder _holder)
{
    // Setup material properties
    this->feedBags = _feedBags; // Store feed bags information
    this->foils = _foils;       // Store foils information
    this->holder = _holder;     // Store foil holder information

    Serial.print("Feed Bags: ");
    Serial.print(feedBags.column * feedBags.row); // Calculate total quantity of feed bags
    Serial.print("pcs, Radius: ");
    Serial.print(feedBags.radius);
    Serial.print("mm, Height: ");
    Serial.println(feedBags.height);
    Serial.print("Clearance of : ");
    Serial.println(feedBags.clearanceX);
    Serial.print("Clearance of Y: ");
    Serial.println(feedBags.clearanceY);

    Serial.print("Foils: ");
    Serial.print(foils.qty); // Calculate total quantity of foils
    Serial.print("pcs, Radius: ");
    Serial.print(foils.radius);
    Serial.print("mm, Height: ");
    Serial.println(foils.height);
    Serial.print("Clearance of X: ");
    Serial.println(foils.clearanceX);
    Serial.print("Clearance of Y: ");
    Serial.println(foils.clearanceY);

    Serial.print("Foil Holder: ");
    Serial.print(holder.qtyFoils);
    Serial.print("pcs, Length: ");
    Serial.print(holder.length);
    Serial.print("mm, Width: ");
    Serial.println(holder.width);

    Serial.println("Material properties set.");
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

long Control::distanceMM(int16_t mm)
{
    // 200 steps per revolution, 32 microsteps, 20 teeth pulley, 2 mm pitch
    long steps = (STEPS_PER_REVOLUTION * MICRO_STEPPING) / (PULLEY_TEETH * BELL_PITCH);

    // Convert mm to steps
    long stepsNeeded = mm * steps;

    return stepsNeeded;
}

long Control::distanceMM_ZAxis(int16_t mm)
{
    // 200 steps per revolution, 32 microsteps, 20 teeth pulley, 8 mm pitch
    long steps = (STEPS_PER_REVOLUTION * MICRO_STEPPING) / (LED_SCREW_PITCH);

    if(mm > 50) mm = 50; // Limit maximum Z travel to 55mm to avoid collision with the work area
    if(mm < 0) mm = 0;   // Limit minimum Z travel to 0mm

    // Convert mm to steps
    long stepsNeeded = mm * steps;

    return stepsNeeded;
}

long Control::currentMM(AccelStepper &stepper)
{
    long currentPositionSteps = stepper.currentPosition();                                                                 // Get current position in steps
    long currentPositionMM = currentPositionSteps * (PULLEY_TEETH * BELL_PITCH) / (STEPS_PER_REVOLUTION * MICRO_STEPPING); // Convert steps to mm
    return currentPositionMM;
}

void Control::Homing()
{
    // Perform homing for all axes
    XHoming();
    YHoming();
    ZHoming();
}

void Control::XHoming()
{
    Serial.println("Homing X axis...");
    if (digitalRead(X_AXIS_LIMIT) == LOW)
    {
        Serial.println("X axis limit switch is already triggered. Skipping homing.");
        stepperX.setCurrentPosition(0); // Set current position to 0 if already at home
        HomeX = 0;                      // Store home position
        return;
    }
    else
    {
        // --- Save original settings ---
        float originalMaxSpeed = stepperX.maxSpeed();
        float originalAcceleration = stepperX.acceleration();

        // --- Pass 1: Fast move to the limit switch ---
        stepperX.setMaxSpeed(2000.0); // Use a moderate speed for the first pass
        stepperX.setAcceleration(1000.0);

        // Set a target far in the negative direction
        stepperX.moveTo(-999999);

        while (digitalRead(X_AXIS_LIMIT) == HIGH)
        {
            stepperX.run();
        }

        // Stop motor and reset position temporarily
        stepperX.stop();
        stepperX.setCurrentPosition(0);
        Serial.println("First pass complete.");

        // --- Back off the switch ---
        stepperX.moveTo(distanceMM(10)); // Move 5mm away from the switch
        stepperX.runToPosition();        // Blocking call to ensure it finishes
        Serial.println("Backed off switch.");

        // --- Pass 2: Slow move to the limit switch for accuracy ---
        stepperX.setMaxSpeed(500.0); // Very slow for precision
        stepperX.setAcceleration(250.0);
        stepperX.moveTo(-999999); // Move back towards the switch

        while (digitalRead(X_AXIS_LIMIT) == HIGH)
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
}

void Control::YHoming()
{
    Serial.println("Homing Y axis...");
    if (digitalRead(Y_AXIS_LIMIT) == LOW)
    {
        Serial.println("Y axis limit switch is already triggered. Skipping homing.");
        stepperY.setCurrentPosition(0); // Set current position to 0 if already at home
        HomeY = 0;                      // Store home position
        return;
    }
    else
    {
        // --- Save original settings ---
        float originalMaxSpeed = stepperY.maxSpeed();
        float originalAcceleration = stepperY.acceleration();

        // --- Pass 1: Fast move to the limit switch ---
        stepperY.setMaxSpeed(2000.0); // Use a moderate speed for the first pass
        stepperY.setAcceleration(1000.0);

        // Set a target far in the negative direction
        stepperY.moveTo(-999999);

        while (digitalRead(Y_AXIS_LIMIT) == HIGH)
        {
            stepperY.run();
        }

        // Stop motor and reset position temporarily
        stepperY.stop();
        stepperY.setCurrentPosition(0);
        Serial.println("First pass complete.");

        // --- Back off the switch ---
        stepperY.moveTo(distanceMM(10)); // Move 5mm away from the switch
        stepperY.runToPosition();        // Blocking call to ensure it finishes
        Serial.println("Backed off switch.");

        // --- Pass 2: Slow move to the limit switch for accuracy ---
        stepperY.setMaxSpeed(500.0); // Very slow for precision
        stepperY.setAcceleration(250.0);
        stepperY.moveTo(-999999); // Move back towards the switch

        while (digitalRead(Y_AXIS_LIMIT) == HIGH)
        {
            stepperY.run();
        }
        stepperY.stop();

        // This is the true home position. Set it to 0.
        stepperY.setCurrentPosition(0);
        HomeY = stepperY.currentPosition(); // Store the home position for Y axis
        Serial.printf("Homing Y complete. Position set to 0.\n");
        Serial.printf("Home Y: %ld\n", HomeY);

        // --- Restore original settings ---
        stepperY.setMaxSpeed(originalMaxSpeed);
        stepperY.setAcceleration(originalAcceleration);
    }
}

void Control::ZHoming() {
    Serial.println("Homing Z axis...");
    if (digitalRead(Z_AXIS_LIMIT) == LOW)
    {
        Serial.println("Z axis limit switch is already triggered. Skipping homing.");
        stepperX.setCurrentPosition(0); // Set current position to 0 if already at home
        HomeZ = 0;                      // Store home position
        return;
    }
    else
    {
        // --- Save original settings ---
        float originalMaxSpeed = stepperX.maxSpeed();
        float originalAcceleration = stepperX.acceleration();

        // --- Pass 1: Fast move to the limit switch ---
        stepperZ.setMaxSpeed(2000.0); // Use a moderate speed for the first pass
        stepperZ.setAcceleration(1000.0);

        // Set a target far in the negative direction
        stepperZ.moveTo(-999999);

        while (digitalRead(Z_AXIS_LIMIT) == HIGH)
        {
            stepperZ.run();
        }

        // Stop motor and reset position temporarily
        stepperZ.stop();
        stepperZ.setCurrentPosition(0);
        Serial.println("First pass complete.");

        // --- Back off the switch ---
        stepperZ.moveTo(distanceMM(10)); // Move 5mm away from the switch
        stepperZ.runToPosition();        // Blocking call to ensure it finishes
        Serial.println("Backed off switch.");

        // --- Pass 2: Slow move to the limit switch for accuracy ---
        stepperZ.setMaxSpeed(500.0); // Very slow for precision
        stepperZ.setAcceleration(250.0);
        stepperZ.moveTo(-999999); // Move back towards the switch

        while (digitalRead(Z_AXIS_LIMIT) == HIGH)
        {
            stepperZ.run();
        }
        stepperZ.stop();

        // This is the true home position. Set it to 0.
        stepperZ.setCurrentPosition(0);
        HomeZ = stepperZ.currentPosition(); // Store the home position for X axis
        Serial.println("Homing X complete. Position set to 0.");
        Serial.printf("Home X: %ld\n", HomeX);

        // --- Restore original settings ---
        stepperZ.setMaxSpeed(originalMaxSpeed);
        stepperZ.setAcceleration(originalAcceleration);
    }
}

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
    stepperX.moveTo(HomeX);   // Move to the stored home position
    stepperX.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("X axis back to home position.");
}
void Control::YBackToHome()
{
    Serial.println("Moving Y axis back to home position...");
    stepperY.moveTo(HomeY);   // Move to the stored home position
    stepperY.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("Y axis back to home position.");
}
void Control::ZBackToHome()
{
    Serial.println("Moving Z axis back to home position...");
    stepperZ.moveTo(HomeZ);   // Move to the stored home position
    stepperZ.runToPosition(); // Blocking call to ensure it finishes
    Serial.println("Z axis back to home position.");
}

void Control::goToFeedBags()
{
    uint16_t borderX = 100;
    uint16_t borderY = 100;

    // Serial.print("Moving to feed bag ");
    Serial.print(currentBag);
    Serial.print(" of ");
    Serial.print(feedBags.column * feedBags.row);
    Serial.println("...");
    
    Serial.printf("Current Position X: %ld mm\t", currentPositionX);
    Serial.printf("Current Position Y: %ld mm\n", currentPositionY);
    
    if (currentBag < 1)
    {
        // Serial.println("Moving to first feed bag...");
        stepperX.moveTo(distanceMM(borderX + feedBags.radius));     // Move to the first feed bag position
        stepperY.moveTo(distanceMM(borderY + feedBags.height / 2)); // Center Y position for the first bag
        while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
        {
            stepperX.run();
            stepperY.run();
        }
        // Serial.println("Reached first feed bag.");
        firstBagPositionX = currentMM(stepperX);
        firstBagPositionY = currentMM(stepperY);
        currentPositionX = firstBagPositionX;
        currentPositionY = firstBagPositionY;
        Serial.printf("Fist bag position X: %ld mm\n", firstBagPositionX);
        Serial.printf("Fist bag position Y: %ld mm\n", firstBagPositionY);

        goToFoils(); // Move to foils after reaching the first feed bag
        if (isFoilsDoneProcess)
        {
            currentBag++; // Increment the bag index
        }
    }
    else
    {
        // Serial.println("Moving to next feed bag...");

        if (isFoilsDoneProcess)
        {
            nextColumnBags(); // Move to the next column if not at the last column
        }
        if (isLastColumnBags)
        {
            if (isFoilsDoneProcess)
            { 
                stepperX.moveTo(distanceMM(firstBagPositionX)); // Reset X to first column
                stepperX.runToPosition();                       // Blocking call to ensure it finishes
                currentPositionX = currentMM(stepperX);         // Update current bag position in X
            }
            nextRowBags(); // Move to the next row if at the last column
            // isLastColumnBags = false;
        }

        goToFoils(); // Move to foils after reaching the feed bag
        if (isFoilsDoneProcess)
            currentBag++; // Increment the bag index
    }
    delay(1000); // Wait for a second before next operation
}

void Control::goToFoils()
{
    if (currentFoil <= foils.qty)
    {
        goToHolder(); // Move to the foil holder first
        // Serial.print("Moving to foil ");
        // Serial.print(currentFoil);
        // Serial.print(" of ");
        // Serial.print(foils.qty);
        // Serial.println("...");
       
        /* Pick the foils here */
        stepperZ.moveTo(distanceMM_ZAxis(50)); // Move Z axis up to avoid collision
        stepperZ.runToPosition();              // Blocking call to ensure it finishes
        stepperZ.moveTo(distanceMM_ZAxis(0));  // Move Z axis down to pick foil
        stepperZ.runToPosition();              // Blocking call to ensure it finishes
        Serial.println("Picked foil.");
        
        // /* back to current Bag */
        if (currentBag < 1)
        {
            // Serial.println("Moving to first foil...");
            stepperX.moveTo(distanceMM(firstBagPositionX)); // Move to the first feed bag position
            stepperY.moveTo(distanceMM(firstBagPositionY)); // Center Y position for the first bag
            while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
            {
                stepperX.run();
                stepperY.run();
            }
            if (currentFoil < foils.qty)
            {
                
                stepperX.move(distanceMM(2)); // Move to the next foil position
                stepperY.move(distanceMM(2)); // Adjust Y position based on column
                while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
                {
                    stepperX.run();
                    stepperY.run();
                }
                
                /* Place foil into FeedBag here */
                stepperZ.moveTo(distanceMM_ZAxis(50)); // Move Z axis up to avoid collision
                stepperZ.runToPosition();              // Blocking call to ensure it finishes
                stepperZ.moveTo(distanceMM_ZAxis(0));  // Move Z axis down to pick foil
                stepperZ.runToPosition();              // Blocking call to ensure it finishes
                Serial.println("Placed foil.");

                currentFoil++; // Increment the foil index
            }
            else
            {
                isFoilsDoneProcess = true; // Set flag to indicate foils processing is done
                currentFoil = 0;           // Reset foil index
                currentBag++;
            }
        }
        else
        {
            if (currentFoil < foils.qty)
                isFoilsDoneProcess = false; // Reset flag for foils processing

            stepperX.moveTo(distanceMM(currentPositionX)); // Move to the current bag position
            stepperY.moveTo(distanceMM(currentPositionY));

            // if(isLastColumnBags)  stepperY.moveTo(distanceMM(currentPositionY));
            // else                  stepperY.moveTo(distanceMM(firstBagPositionY)); // If last column, reset Y to first row

            while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
            {
                stepperX.run();
                stepperY.run();
            }
            // Serial.println("Moving to next foil...");
            if (currentFoil < foils.qty)
            {
                
                // float angleInRadians = 90 * PI / 180.0;
                // float targetX_mm = currentMM(stepperX) + foils.radius * cos(angleInRadians);
                // float targetY_mm = currentMM(stepperY) + foils.radius * sin(angleInRadians);
                stepperX.move(distanceMM(2)); // Move to the next foil position
                stepperY.move(distanceMM(2)); // Adjust Y position based on column
                while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
                {
                    stepperX.run();
                    stepperY.run();
                } // Blocking call to ensure it finishes
                Serial.println("Reached next foil.");

                /* Place the Foils into FeedBags here */
                 stepperZ.moveTo(distanceMM_ZAxis(50)); // Move Z axis up to avoid collision
                stepperZ.runToPosition();              // Blocking call to ensure it finishes
                stepperZ.moveTo(distanceMM_ZAxis(0));  // Move Z axis down to pick foil
                stepperZ.runToPosition();              // Blocking call to ensure it finishes


                currentFoil++; // Increment the foil index
            }
            else
            {
                isFoilsDoneProcess = true; // Set flag to indicate foils processing is done
                currentFoil = 0;           // Reset foil index
                // Serial.println("No more foils to pick.");
            }
        }
    }
    /* NOTE: REMEMBER RESET POSITION OF AXIS TO CURRENT POSITION AFTER PLACE THE FOILS */
    // currentPositionX = firstBagPositionX;
    // currentPositionY = firstBagPositionY;
}

void Control::goToHolder()
{
    // Serial.println("Moving to foil holder...");

    if (stepperX.currentPosition() != holder.length && stepperY.currentPosition() != holder.width)
    {
        stepperX.moveTo(distanceMM(holder.length / 2)); // Example movement, adjust as needed
        stepperY.moveTo(distanceMM(holder.width / 2));  // Example movement, adjust as needed
        while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
        {
            stepperX.run();
            stepperY.run();
        }
    }
    // Serial.println("Reached foil holder.");
}

void Control::nextColumnBags()
{
    /* Column - 1: without first bags */
    if (currentMM(stepperX) < ((feedBags.column - 1) * feedBags.clearanceX + firstBagPositionX))
    {
        stepperX.move(distanceMM(feedBags.clearanceX)); // Move to the next column position
        stepperX.runToPosition();                       // Blocking call to ensure it finishes
        currentPositionX = currentMM(stepperX); // Update current bag position in X
        isLastColumnBags = false;               // Reset flag as we are not at the last column yet
    }
    else
    {
        Serial.println("Reached end of column holder.");
        isLastColumnBags = true; // Set flag to indicate last column
    }
}

void Control::nextRowBags()
{
    /* Row - 1: without first bags */
    if (currentMM(stepperY) < ((feedBags.row - 1) * feedBags.clearanceY + firstBagPositionY))
    {
        stepperY.move(distanceMM(feedBags.clearanceY)); // Move to the next row position
        stepperY.runToPosition();                       // Blocking call to ensure it finishes
        currentPositionY = currentMM(stepperY);         // Update current bag position in X
        isLastRowBags = false;                          // Reset flag as we are not at the last row yet
    }
    else
    {
        Serial.println("Reached end of row holder.");
        isLastRowBags = true; // Set flag to indicate last row
    }
}

void Control::runSequence()
{
}

// Getter implementations for external access
AccelStepper &Control::getStepperX() { return stepperX; }
AccelStepper &Control::getStepperY() { return stepperY; }
AccelStepper &Control::getStepperZ() { return stepperZ; }
