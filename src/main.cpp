#include "main.h"


void sendPackge()
{
  sendPackage.request = MOVE_X;
  sendPackage.x_pos_mm ++; 
  sendPackage.y_pos_mm ++; 
  sendPackage.z_pos_mm ++; 
  
  // sendPackage.x_pos_mm = control.getCurrentPositionX_mm();
  // sendPackage.y_pos_mm = control.getCurrentPositionY_mm();
  // sendPackage.z_pos_mm = control.getCurrentPositionZ_mm();

  Serial.printf("Sending Package - Request: %d, X: %d, Y: %d, Z: %d\n",
                sendPackage.request,
                sendPackage.x_pos_mm,
                sendPackage.y_pos_mm,
                sendPackage.z_pos_mm);

  Serial1.write(send_headder_1st);
  Serial1.write(send_headder_2nd);

  Serial1.write((uint8_t *)&sendPackage, sizeof(sendPackage_t));
  // Serial.printf("Size of sendPackage_t: %d Size of Request_Package_t: %d Size of sendPackage: %d\n",
  //               sizeof(sendPackage_t), sizeof(sendPackage.request), sizeof(sendPackage));
  
}

void recvPackage(){
/* Check if data is sending */
    while (Serial1.available())
    {
      char c = Serial1.read(); // Read a byte from Serial1
      // Serial1.write(c); // Echo back the received character
      switch (recvState)
      {
      case WATING_START_1ST:
        if (c == START_NEW_PACKAGE_1ST)
        {
          recvState = WATING_START_2ND;
          Serial.print("RX BYTE(HEX): ");
          Serial.println(c, HEX);
        }
        break;

      case WATING_START_2ND:
        if (c == START_NEW_PACKAGE_2ND)
        {
          recvState = READ_PAYLOAD;
          buff_index = 0; // Reset buffer index
          Serial.print("RX BYTE(HEX): ");
          Serial.println(c, HEX);
        }
        else
        {
          recvState = WATING_START_1ST; // Reset state if not matched
        }
        break;

      case READ_PAYLOAD:
        buffer[buff_index++] = c;
        if (buff_index == sizeof(newPackage_t))
        {
          Serial.printf("RX BYTE(HEX): ");
          for (int i = 0; i < sizeof(newPackage_t); i++)
          {
            Serial.printf("%02X ", buffer[i]);
          }
          Serial.println();
          // Copy data to struct
          memcpy(&receivedPackage, buffer, sizeof(newPackage_t));
          // Process received package
          Serial.print("Received Request: ");
          Serial.println(receivedPackage.request);
          Serial.print("X Pos: ");
          Serial.println(receivedPackage.x_pos_mm);
          Serial.print("Y Pos: ");
          Serial.println(receivedPackage.y_pos_mm);
          Serial.print("Z Pos: ");
          Serial.println(receivedPackage.z_pos_mm);

          // Reset state for next package
          recvState = WATING_START_1ST;
        }
        break;
      }
    }
}


void controlMotor(void *parameter)
{
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

  Serial.println("Setup complete. All axes homed.");

  control.Homing(); // Perform homing for all axes
  delay(1000);      // Wait for a second after homing
  while (true)
  {
    control.goToHolder();
    delay(1000); // Wait for a second before next operation
    while (control.getCurrentBags() <= qtyBags)
    {
      control.goToNutriBags(); // Move to feed bags
      delay(10);
    }
    vTaskDelay(10); // Yield to other tasks
  }
}

void commuTask(void *parameter)
{
  while (true)
  {
    // sendPackge();
    recvPackage();
    vTaskDelay(50);
  }
}

// void testWithTerminalMac(){
//   if(Serial1.available()){
//       char c = Serial1.read();
//       if(c == '\n' || c == '\r'){
//         Serial1.write('\n');
//         Serial1.write('\r');

//         Serial.print("Message from Jetson:");
//         Serial.println(incomMess);
//         incomMess = " ";
//       }
//       else{
//         // Serial1.write(c); // This sends the character right back
//         incomMess += c;
//       }
//     }
// }
/* demo */

void setup()
{

  Serial.begin(115200);                                                // Initialize Serial for debugging
  Serial1.begin(115200, SERIAL_8N1, UART_JETSON_RX1, UART_JETSON_TX1); // Initialize Serial1(UART1) for communicate Jetson
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);                         // Initialize Serial2(UART2) for TMC2209

  sendPackage.x_pos_mm = 0; 
  sendPackage.y_pos_mm = 0; 
  sendPackage.z_pos_mm = 0; 

  // xTaskCreate(controlMotor, "ControlTask", 8192, NULL, 1, &controlTaskHandle);
  xTaskCreate(commuTask, "CommunicationTask", 8192, NULL, 2, &commuTaskHandle);
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
