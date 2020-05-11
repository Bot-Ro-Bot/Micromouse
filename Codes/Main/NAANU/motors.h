#ifndef motors_H
#define motors_H
#include<Arduino.h>

class Motor {
  protected:
    byte enablePin, forwardPin, backPin;
  public:
    Motor(byte enablePin, byte forwardPin, byte backPin);
    static uint8_t speed;
    void forward(uint8_t speed);
    void back(uint8_t speed);
    void brake();
    void stop();
    void rotateRight();
    void rotateLeft();
    void rotateRight180();
    void rotateLeft180();
};

#endif
