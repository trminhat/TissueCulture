#include "PWMStepper.h"

PWMStepper::PWMStepper(uint8_t axis, uint8_t dirPin, uint8_t channel, uint8_t resolution)
{
    // _enaPin = ENABLE_DRV_DEFAULT;
    _axisPin = axis;
    _dirPin = dirPin;
    _channel = channel;
    _resolution = resolution;

    _freq = (SOURCE_CLOCK_80MHZ / (pow(2, _resolution))); // Maximum Frequency of PWM signal
    duty50 = (pow(2, _resolution)) / 2; // 50% duty cycle

    pinMode(_dirPin, OUTPUT);
    // pinMode(_enaPin, OUTPUT);


}

void PWMStepper::begin()
{
    // ledcSetup(_channel, _freq, duty50); // Set up the PWM channel
    ledcAttachPin(_axisPin, _channel); // Attach the pin to the PWM channel
    // digitalWrite(_enaPin, LOW);        // Enable the driver

}

void PWMStepper::move(uint8_t dir, uint32_t freq){
    // if (digitalRead(_enaPin) == HIGH)
    // {
    //     digitalWrite(_enaPin, LOW); // Enable the driver
    // } 
    // if(!isAttached){
    //     ledcAttachPin(_axisPin, _channel); // Detach the pin from the PWM channel
    //     isAttached = true;
    // }
    digitalWrite(_dirPin, dir);     // Set the direction
    ledcWriteTone(_channel, freq); // Set the pulse width
    
}



void PWMStepper::moveDistanceCm(float cm, float spmm, uint32_t freq) {
  int steps = cm * 10.0 * spmm; // Convert cm to steps (assuming 10 steps per mm)
  float duration_s = (float)steps / 1000; // Calculate duration in seconds
  unsigned long duration_ms = duration_s * 1000;

  // Start pulse train
  ledcWriteTone(PWM_CHANNEL2, freq);
  delay(duration_ms);
  ledcWriteTone(PWM_CHANNEL2, 0);  // Stop PWM
}


void PWMStepper::stop()
{
    ledcWriteTone(_channel, 0); // Set the pulse width
    ledcDetachPin(_axisPin); // Detach the pin from the PWM channel
    isAttached = false;
    Serial.printf("Stepper is Stop \n");
}

PWMStepper::~PWMStepper() {}
