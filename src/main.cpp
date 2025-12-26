#include "main.h"

void sequenceMotor()
{
  control.goToHolder();
  delay(1000); // Wait for a second before next operation
  if (control.getCurrentBags() <= qtyBags)
  {
    control.goToNutriBags(); // Move to feed bags

    delay(10);
  }
  vTaskDelay(10); // Yield to other tasks
}
void sendPackge()
{
  if (xQueueReceive(xSendQueue, &sendPackage, 0) == pdTRUE)
  {

    Serial1.write(send_headder_1st);
    Serial1.write(send_headder_2nd);
    Serial1.write((uint8_t *)&sendPackage, sizeof(sendPackage_t));

    Serial.println("Data is Send Successfully.");
  }
  else
  {
    // Serial.println("No data to Send.");
  }
  delay(10);
}

void recvPackage()
{
  static recvPackage_t incomingPkg;
  static uint8_t buffer[sizeof(recvPackage_t)];
  static uint8_t dataLength = 0;
  static int8_t buff_index = 0;

  /* Check if data is sending */
  while (Serial1.available())
  {
    char c = Serial1.read(); // Read a byte from Serial1
    Serial.printf("Ras Data Receiving: 0x%02X\n", c);
    switch (recvState)
    {
    case WATING_START_1ST:
      if (c == START_NEW_PACKAGE_1ST)
      {
        recvState = WATING_START_2ND;
        // Serial.print("RX BYTE(HEX): ");
        Serial.println(c, HEX);
      }
      break;

    case WATING_START_2ND:
      if (c == START_NEW_PACKAGE_2ND)
      {
        recvState = READ_REQUEST_PAYLOAD;
        buff_index = 0; // Reset buffer index
        // Serial.print("RX BYTE(HEX): ");
        Serial.println(c, HEX);
      }
      else
      {
        recvState = WATING_START_1ST; // Reset state if not matched
      }
      break;

    case READ_REQUEST_PAYLOAD:
      incomingPkg.request = c;
      switch (incomingPkg.request)
      {
      case MOVE_XYZ:
      case ASK_CURRENT_POSITION:
      case CONTROL_GRIPPER:
        dataLength = sizeof(Payload_State_t);
        break;
      case SETUP_MATERIAL:
        dataLength = sizeof(Payload_System_t);
        break;

      case HOMING:
      case RUN_SEQUENCE:
        dataLength = 0;
        break;

      default:
        recvState = WATING_START_1ST; // Reset state for unknown request
        break;
      }
      buff_index = 0; // Reset buffer index

      if (dataLength == 0)
      {
        xQueueSend(xReceiveQueue, &incomingPkg, 0); // Send received package to the queue
        recvState = WATING_START_1ST;               // Reset state for next package
      }
      else
      {
        recvState = READ_PAYLOAD;
      }

      break;

    case READ_PAYLOAD:
      buffer[buff_index++] = c;
      /* Payload complete. Copy buffer into the Union. */
      if (buff_index >= dataLength)
      {
        if (dataLength == sizeof(Payload_State_t))
        {
          memcpy(&incomingPkg.payload.stateData, buffer, sizeof(Payload_State_t));
        }

        else if (dataLength == sizeof(Payload_System_t))
        {
          memcpy(&incomingPkg.payload.systemData, buffer, sizeof(Payload_System_t));
        }

        xQueueSend(xReceiveQueue, &incomingPkg, 0); // Send received package to the queue

        // Reset state for next package
        recvState = WATING_START_1ST;
      }
      break;
    }
  }
  delay(100);
}
void vTask_UARTComm(void *parameter)
{
  while (true)
  {
    // Serial.printf("Checking for incoming packages...\n");
    recvPackage(); // Check and process incoming packages
    sendPackge();
    vTaskDelay(10);
  }
}

void vTask_ControlMotor(void *parameter)
{
  recvPackage_t incomingPack;
  sendPackage_t outgoingPack;

  while (true)
  {
    // Serial.printf("Enter to Task Motor.\n");
    if (xQueueReceive(xReceiveQueue, &incomingPack, 0) == pdTRUE)
    {

      switch (incomingPack.request)
      {
      case ASK_CURRENT_POSITION:
        Serial.println("Requesting Current Position...");
        outgoingPack.request = MOTION_COMPLETED;
        outgoingPack.current_X_mm = control.getCurrentPositionX_mm();
        outgoingPack.current_Y_mm = control.getCurrentPositionY_mm();
        outgoingPack.current_Z_mm = control.getCurrentPositionZ_mm();
        outgoingPack.current_gripper_deg = control.getCurrentGripper();

        xQueueSend(xSendQueue, &outgoingPack, 0);

        break;
      case RUN_SEQUENCE:
        Serial.println("Running predefined motor sequence...");
        // Execute the motor sequence
        sequenceMotor();
        // After sequence completion, send back the current position
        outgoingPack.request = MOTION_COMPLETED;
        outgoingPack.current_X_mm = control.getCurrentPositionX_mm();
        outgoingPack.current_Y_mm = control.getCurrentPositionY_mm();
        outgoingPack.current_Z_mm = control.getCurrentPositionZ_mm();
        outgoingPack.current_Foils = control.getCurrentFoils();
        outgoingPack.current_Bags = control.getCurrentBags();

        xQueueSend(xSendQueue, &outgoingPack, 0);

        break;

      case MOVE_XYZ:
        Serial.println("Move");
        // Serial.printf("Moving X Axis... %d mm\n", incomingPack.x_pos_mm);
        control.getStepperX().moveTo(control.distanceMM(incomingPack.payload.stateData.x_pos_mm));
        control.getStepperX().runToPosition(); // Blocking call to ensure it finishes
        control.getStepperY().moveTo(control.distanceMM(incomingPack.payload.stateData.y_pos_mm));
        control.getStepperY().runToPosition(); // Blocking call to ensure it finishes
        control.getStepperZ().moveTo(control.distanceMM_ZAxis(incomingPack.payload.stateData.z_pos_mm));
        control.getStepperZ().runToPosition(); // Blocking call to ensure it finishes
        outgoingPack.request = MOTION_COMPLETED;
        break;
      case CONTROL_GRIPPER:
        Serial.println("Received OPEN_GRIPPER command.");
        control.getGripper().write(incomingPack.payload.stateData.gripper_deg); // Assuming z_pos_mm holds the gripper position
        outgoingPack.request = MOTION_COMPLETED;
        break;

      case HOMING:
        Serial.println("Performing Homing Sequence...");
        control.Homing(); // Issue: Homing IN TASK MAY BLOCK OTHER TASKS CAUSED FROM WHILE LOOP WAITING FOR LIMIT SWITCH !!!!
        outgoingPack.request = MOTION_COMPLETED;
        break;

      case SETUP_MATERIAL:
        Serial.println("Setting up material properties...");
        control.resetCurrentBag();
        control.resetCurrentFoil();

        bags = incomingPack.payload.systemData.bags;
        foils = incomingPack.payload.systemData.foils;
        holder = incomingPack.payload.systemData.holder;
        control.setupMaterial(bags, foils, holder);
        outgoingPack.request = MOTION_COMPLETED;

        Serial.printf("Material Setup Complete: Bags(%d cols, %d rows), Foils(%d qty), Holder( %d Foils in Holder, %d length, %d width)\n",
                      bags.column, bags.row,
                      foils.qty,
                      holder.qtyFoils, holder.length, holder.width);
        break;

      default:
        // Serial.println("Unknown command received.");
        break;
      }
    }

    vTaskDelay(100);
  }
}

void setup()
{

  Serial.begin(115200);                                                // Initialize Serial for debugging
  Serial1.begin(115200, SERIAL_8N1, UART_JETSON_RX1, UART_JETSON_TX1); // Initialize Serial1(UART1) for communicate Jetson
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);                         // Initialize Serial2(UART2) for TMC2209

  control.setup(bags, foils, holder); // Setup the control system
  // control.setSpreadCycle(); // Set SpreadCycle mode for all drivers
  // control.setStealthChop(); // Set StealthChop mode for all drivers
  control.setStealthChop('X'); // Set StealthChop mode for X driver
  control.setStealthChop('Y'); // Set SpreadCycle mode for Y drivers
  control.setSpreadCycle('Z'); // Set StealthChop mode for Z

  control.setMaxSpeed('X', MAX_SPEED_100KHZ); // Set max speed for X axis
  control.setMaxSpeed('Y', MAX_SPEED_100KHZ); // Set max speed for Y axis
  control.setMaxSpeed('Z', MAX_SPEED_100KHZ); // Set max speed for Z

  control.setAcceleration('X', ACCELERATION_50KHZ); // Set acceleration for X axis
  control.setAcceleration('Y', ACCELERATION_50KHZ); // Set acceleration for Y axis
  control.setAcceleration('Z', ACCELERATION_80KHZ); // Set acceleration for Z axis

  control.Homing(); // Perform homing for all axes
  Serial.printf("Size of recvPackage_t: %d bytes\n", sizeof(recvPackage_t));
  Serial.printf("Size of sendPackage_t: %d bytes\n", sizeof(sendPackage_t));

  Serial.println("Setup complete. All axes homed.");
  xReceiveQueue = xQueueCreate(5, sizeof(recvPackage_t)); // Create queue for receiving data
  xSendQueue = xQueueCreate(5, sizeof(sendPackage_t));    // Create queue for sending data

  delay(1000); // Wait for a second after homing

  xTaskCreate(vTask_ControlMotor, "ControlTask", 8192, NULL, 1, &MotorTaskHandle);
  xTaskCreate(vTask_UARTComm, "CommunicationTask", 8192, NULL, 2, &UARTCommTaskHandle);
  vTaskDelete(NULL);

  vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

void loop()
{
  // control.goToHolder();
  // delay(1000); // Wait for a second before next operation
  // while(control.getCurrentBags() <= qtyBags){
  //   control.goToNutriBags(); // Move to feed bags
  //   delay(10);
  // }
}
