#include"motors.h"

Motor::Motor(byte enablePin, byte forwardPin, byte backPin) {
  this->enablePin = enablePin;
  this->forwardPin = forwardPin;
  this->backPin = backPin;

  pinMode(enablePin, OUTPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(backPin, OUTPUT);
}

void Motor::forward(uint8_t speed) {
  analogWrite(enablePin, speed);
  digitalWrite(forwardPin, HIGH);
  digitalWrite(backPin, LOW);
}

void Motor::back(uint8_t speed) {
  analogWrite(enablePin, speed);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backPin, HIGH);
}

void Motor::stop() {
  analogWrite(enablePin, 0);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backPin, LOW);
}

void Motor::brake() {
  analogWrite(enablePin, 255);
  digitalWrite(forwardPin, LOW);
  digitalWrite(forwardPin, LOW);
}

void Motor::rotateRight(){
  
}

void Motor::rotateRight180(){
  
}

void Motor::rotateLeft(){
  
}

void Motor::rotateLeft180(){
  
}
