#ifndef Motor_h
#define Motor_h
#include <Arduino.h>
#define FREQUENCY 30000
#define RESOLUTION 8
#define MIN_POWER 0
#define MAX_POWER 255

class Motor {
  public:
    Motor(int pinA, int pinB, int encoderPinA, int encoderPinB, int enablePin, int channel);
    void rotate(int power);
    void stop();
    int pinA;
    int pinB;
    int encoderPinA;
    int encoderPinB;
    int enablePin;
    int channel;
    bool reverse;
};

#endif
