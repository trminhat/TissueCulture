#ifndef __PWMSTEPPER_H__
#define __PWMSTEPPER_H__

#include <Arduino.h>
#include <math.h>


/*
 * This group/channel/timmer mapping is for information only;
 * the details are handled by lower-level code
 *
 * Group/Channel/Timer Mapping:
 ** Group: 0, Channel: 0, Timer: 0
 ** Group: 0, Channel: 1, Timer: 0
 ** Group: 0, Channel: 2, Timer: 1
 ** Group: 0, Channel: 3, Timer: 1
 ** Group: 0, Channel: 4, Timer: 2
 ** Group: 0, Channel: 5, Timer: 2
 ** Group: 0, Channel: 6, Timer: 3
 ** Group: 0, Channel: 7, Timer: 3

 ** Group: 1, Channel: 0, Timer: 0
 ** Group: 1, Channel: 1, Timer: 0
 ** Group: 1, Channel: 2, Timer: 1
 ** Group: 1, Channel: 3, Timer: 1
 ** Group: 1, Channel: 4, Timer: 2
 ** Group: 1, Channel: 5, Timer: 2
 ** Group: 1, Channel: 6, Timer: 3
 ** Group: 1, Channel: 7, Timer: 3

 */
/* ************
Max Frequency posibles:
    Fmax= 80Mhz / PWM_RESOLUTION 

    PWM_RESOLUTION:
    Resolution of PWM signal.
    8 bit = 256 steps
    10 bit = 1024 steps
    12 bit = 4096 steps
    14 bit = 16384 steps
    16 bit = 65536 steps
    20 bit = 1048576 steps

************ */



/* The Default pin configured according to Arduindo Uno platform connnect to CNC Shield V3 */
#define SOURCE_CLOCK_80MHZ 80000000 // ESP32 SOURCE CLOCK 80MHZ
#define PWM_CHANNEL0 0   // Use PWM channel 0 - Timer 0
#define PWM_CHANNEL1 1   // Use PWM channel 1 - Timer 0
#define PWM_CHANNEL2 2   // Use PWM channel 2 - Timer 1
#define PWM_CHANNEL3 3   // Use PWM channel 3 - Timer 1
#define PWM_CHANNEL4 4   // Use PWM channel 4 - Timer 2
#define PWM_CHANNEL5 5   // Use PWM channel 5 - Timer 2
#define PWM_CHANNEL6 6   // Use PWM channel 6 - Timer 3
#define PWM_CHANNEL7 7   // Use PWM channel 7 - Timer 3

#define PWM_FREQ 100    // Frequency of PWM signal


#define PWM_RESOLUTION_8 8 // Resolution 8 bit. Max Speed Frequency 312.5KHz
#define PWM_RESOLUTION_10 10 // Resolution 10 bit. Max Speed Frequency 78.1KHz
#define PWM_RESOLUTION_12 12 // Resolution 12 bit. Max Speed Frequency 19.5KHz
#define PWM_RESOLUTION_14 14 // Resolution 14 bit. Max Speed Frequency 4.9KHz
#define PWM_RESOLUTION_16 16 // Resolution 16 bit. Max Speed Frequency 1.2KHz
#define PWM_RESOLUTION_20 20 // Resolution 20 bit. Max Speed Frequency 0.3KHz


class PWMStepper
{
private:
    uint8_t _enaPin;
    uint8_t _axisPin;
    uint8_t _dirPin;

    uint32_t _pulse;
    uint8_t _channel;
    uint32_t _freq;
    uint8_t _resolution;
    uint32_t duty50 = 0; // 50% duty cycle
    bool isAttached = false;
    bool isReachedLimit = false;
    

public:
    PWMStepper(uint8_t pin, uint8_t dir, uint8_t channel, uint8_t resolution);
    ~PWMStepper();
    void begin();

    void moveFree(uint8_t dir, uint32_t freq);
    void moveToLimit(uint8_t dir, uint32_t pulse, uint8_t limit);
    void move(uint8_t dir, uint32_t rounds);
    void stop();
    void moveDistanceCm(float cm, float spmm, uint32_t freq) ;
};



#endif