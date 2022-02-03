
#include "Motor.h"

Motor::Motor(int pinA, int pinB, int encoderPinA, int encoderPinB, int enablePin, int channel){
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
 

  ledcSetup(channel, FREQUENCY, RESOLUTION);
  ledcAttachPin(enablePin, channel);
  Motor::pinA = pinA;
  Motor::pinB = pinB;
  Motor::encoderPinA = encoderPinA;
  Motor::encoderPinB = encoderPinB;
  Motor::enablePin = enablePin;
  Motor::channel = channel;
  Motor::reverse = false;
}

void Motor::rotate(int power){
//  Serial.print("Power: ");
//  Serial.println(power);
  int absPower = abs(power);

  if (power >=0) {
      digitalWrite(Motor::pinA, LOW);
      digitalWrite(Motor::pinB, HIGH); 
  } else {
      digitalWrite(Motor::pinA, HIGH);
      digitalWrite(Motor::pinB, LOW);
  }
  
  ledcWrite(Motor::channel, absPower);
}

void Motor::stop(){
  digitalWrite(Motor::pinA, LOW);
  digitalWrite(Motor::pinB, LOW);  
}
