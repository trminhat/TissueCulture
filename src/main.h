#pragma once

#include "control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
     What's Mean The Value of START_NEW_PACKAGE_1ST = 0xAA and
                              START_NEW_PACKAGE_2ND = 0x55 ?

     https://docs.google.com/document/d/1GpMNVmQ4YFp2cJAGa4HW1h8lhP90mtHFp8KaZajN2fg/edit?usp=sharing 
*/
#define START_NEW_PACKAGE_1ST 0xAA // 1010 1010 - Start with alternating high/low bits 
#define START_NEW_PACKAGE_2ND 0x55 // 0101 0101 - Continue the inverse alternating pattern
 
Control control(600, 600, 300); // Initialize Control with work area dimensions
NutriBags_t bags = {2, 2, 60, 30, 50, 30}; // Initialize feed bags
Foils_t foils = {2, 60, 3, 10, 10}; // Initialize foils
FoilHolder_t holder = {100, 60, 30}; // Initialize foil holder (if needed)

uint16_t qtyBags = bags.column * bags.row; // Calculate total quantity of feed bags

/*FreeRTOS variable handle */
TaskHandle_t MotorTaskHandle;
TaskHandle_t UARTCommTaskHandle;
QueueHandle_t xReceiveQueue; // Queue for receiving data (Jetson to ESP32)
QueueHandle_t xSendQueue;    // Queue for sending data (ESP32 to Jetson)


const uint8_t send_headder_1st = START_NEW_PACKAGE_1ST;
const uint8_t send_headder_2nd = START_NEW_PACKAGE_2ND;

typedef enum : uint8_t {
    ASK_CURRENT_POSITION,
    RUN_SEQUENCE,
    MOVE_XYZ,
    CONTROL_GRIPPER,
    SETUP_MATERIAL,
    HOMING,
    MOTION_COMPLETED
}Request_Package_t;

#pragma pack(push, 1) // Ensure no padding is added by the compiler
typedef struct{
    NutriBags_t bags;
    Foils_t foils;
    FoilHolder_t holder;
}Payload_System_t;

typedef struct{
    uint16_t x_pos_mm;
    uint16_t y_pos_mm;
    uint16_t z_pos_mm;
    uint16_t gripper_deg;
}Payload_State_t;

/*
    Structure for receiving package from Jetson to ESP32
*/
typedef struct{
    uint8_t request;
    union{
        Payload_State_t stateData;
        Payload_System_t systemData;
    }payload;
}recvPackage_t;

/*
    Structure for sending package from ESP32 to Jetson
*/
typedef struct{
    Request_Package_t request;
    uint16_t current_X_mm;
    uint16_t current_Y_mm;
    uint16_t current_Z_mm;
    uint16_t current_gripper_deg;
    uint16_t current_Foils;
    uint16_t current_Bags;   
}sendPackage_t;
#pragma pack(pop) // Restore default packing




/*
    What's Purpose Of RecvState :
         https://docs.google.com/document/d/1GpMNVmQ4YFp2cJAGa4HW1h8lhP90mtHFp8KaZajN2fg/edit?usp=sharing
*/
enum RecvState{
    WATING_START_1ST,
    WATING_START_2ND,
    READ_REQUEST_PAYLOAD,
    READ_PAYLOAD,
};
RecvState recvState = WATING_START_1ST;

recvPackage_t receivedPackage;
sendPackage_t sendPackage;

