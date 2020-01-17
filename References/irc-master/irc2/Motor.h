#ifndef MOTOR_H
#define MOTOR_H
#include<Arduino.h>

#define max_control_speed 180
#define base_control_speed 160
#define min_control_speed 30


#define PWM_L 10
#define PWM_R 9


//offset value forward 2cm due to momentum
//32 strips = 23 cm
//1 cm = 32/23 strip
//2cm = 2.78 = 3 strip
//30 cm = 30 * 32/23 = 41.73 = 42
//after offset 42 - 3 strips = 39strips for forward
//finding wall offset needed?

//rotation 18 steps
//fullrotating 36 steps

#define forward_offset 3;
#define rotate_offset 3;

typedef enum {
  FORWARD,
  BACKWARD,
  ROTATING_LEFT,
  ROTATING_RIGHT,
  STOP,
  NULL_STATE,
  FORCE_STOP_LEFT,
  FORCE_STOP_RIGHT,
  FORCE_STOP
}
MotorState;

void countLeft();

class Motor{
  private:
    int l_pwm, r_pwm, control_val;
    unsigned int rotate_time = 500;
    unsigned int momentum_wait_time = 1000;


  public:
    Motor(unsigned int control_val = 0);
    
    void updateEncoder(int, MotorState);

    void updatePWM(int);
    MotorState getState();
    void setState(MotorState);
    
    void setPWM(int);
    void moveForward(int steps = 0);
    void moveBackward(int steps = 0);
    void stopMovingForward();
    void stopMovingBackward();
    
    void rotateRight(int steps = 0);
    void rotateLeft(int steps = 0);
    void completeRotate(int steps = 0);

    void rotateLeftStop();
    void rotateRightStop();
    void stopMovingBackSlow();
    void stopMoving();
    
    
    void moveRight();
    void moveLeft();
};

#endif
