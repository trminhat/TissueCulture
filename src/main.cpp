#include <Arduino.h>
// #include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "PWMStepper.h"

#define X_AXIS_LIMIT 13
#define Y_AXIS_LIMIT 5
#define Z_AXIS_LIMIT 23

PWMStepper axisX(X_AXIS_PIN_DEFAULT, X_AXIS_DIR_DEFAULT, PWM_CHANNEL0, PWM_RESOLUTION_8);
PWMStepper axisY(Y_AXIS_PIN_DEFAULT, Y_AXIS_DIR_DEFAULT, PWM_CHANNEL2, PWM_RESOLUTION_8);
// PWMStepper axisZ(Z_AXIS_PIN_DEFAULT, Z_AXIS_DIR, PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);


bool dirX = 0;
bool dirY = 0;


void setup()
{
    Serial.begin(115200);
    axisX.begin();
    axisY.begin();

    // pinMode(ENABLE_DRV_DEFAULT, OUTPUT);
    // pinMode(X_AXIS_DIR_DEFAULT, OUTPUT);
    // pinMode(Y_AXIS_DIR_DEFAULT, OUTPUT);
    // pinMode(Z_AXIS_DIR_DEFAULT, OUTPUT);

    pinMode(X_AXIS_LIMIT, INPUT_PULLUP);
    pinMode(Y_AXIS_LIMIT, INPUT_PULLUP);
    pinMode(Z_AXIS_LIMIT, INPUT_PULLUP);

    // digitalWrite(ENABLE_DRV_DEFAULT, LOW);

    // ledcSetup(PWM_CHANNEL0, 32000, PWM_RESOLUTION_8);
    // ledcAttachPin(X_AXIS_PIN_DEFAULT, PWM_CHANNEL0); // Attach the X_AXIS to PWM channel 0

    // ledcSetup(PWM_CHANNEL2, 32000, PWM_RESOLUTION_8);
    // ledcAttachPin(Y_AXIS_PIN_DEFAULT, PWM_CHANNEL2); // Attach the Y_AXIS to PWM channel 0
  axisY.moveDistanceCm(10.0, 160, 2000);  // Move 1 cm
}

void loop()
{

    // if(digitalRead(X_AXIS_LIMIT) == LOW)
    // {
    //     Serial.printf("Stepper is Reached Limit \n");
    //     axisX.stop();
    //     dirX = !dirX;
    //     delay(100);
    // }
    
    // if(digitalRead(Y_AXIS_LIMIT) == LOW)
    // {
    //     Serial.printf("Stepper is Reached Limit \n");
    //     axisY.stop();
    //     dirY = !dirY;
    //     delay(1000);
    // }
    // axisX.move(dirX, 12800);

    
    // axisY.move(dirY, 800);
    
   
    

    // axisY.moveToLimit(dir, 12800, Y_AXIS_LIMIT);

    delay(100);
}


