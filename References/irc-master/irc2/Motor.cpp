#include "Motor.h"
//00000000
//00001010  0x0A    pin no 25 24 23 22
//00000101  0x05
//00000010  0X02
//00001000  0X08
//00000110  0x06
//00001001  0x09
//00001111  0x0f
//http://i49.tinypic.com/1h7zp2.png

//unsigned long count ,timeISR = 0;
//unsigned int steps_count= 0;
////MotorState motorState;


volatile unsigned long count ,timeISR =0;
volatile unsigned int steps_count =0;
unsigned int rotate_time_wait =0;

MotorState motorState;

Motor::Motor(unsigned int control_val){
  pinMode(PWM_L,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(25,OUTPUT);
  
  updatePWM(control_val);
  motorState = NULL_STATE;
  pinMode(2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), countLeft, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(3), countRight, CHANGE);
}

MotorState Motor::getState(){
  return motorState;
}

void Motor::setState(MotorState state){
  motorState = state;
}

void countLeft(){
  if(millis() - timeISR > 3){
    count++;
    if(count == steps_count){
      switch (motorState){
        case ROTATING_LEFT:
            motorState = FORCE_STOP_LEFT;
            break;
        case ROTATING_RIGHT:
            motorState = FORCE_STOP_RIGHT;
            break;
        case FORWARD:
            motorState = FORCE_STOP;
            break;
      }
    }
    timeISR = millis();
  }
}

void countRight(){
  
}

void Motor::updatePWM(int pid_control_val){
    control_val = pid_control_val;
    
    r_pwm = base_control_speed + control_val;
    l_pwm = base_control_speed - control_val;

    if(l_pwm> max_control_speed)
      l_pwm = max_control_speed;
    else if(l_pwm < 130)
      l_pwm = 0;

    if(r_pwm> max_control_speed)
      r_pwm = max_control_speed;
    else if(r_pwm < 130)
      r_pwm = 0;

    analogWrite(PWM_L, l_pwm);
    analogWrite(PWM_R, r_pwm);
}

//void Motor::moveForward(int steps){
//  updateEncoder(steps, FORWARD);
//  PORTA &= 0;
//  PORTA |=0x0A;   //pin 22 23 24 25
//  analogWrite(PWM_L, l_pwm);
//  analogWrite(PWM_R, r_pwm);
//}

 void Motor::moveForward(int steps){
  updateEncoder(steps, FORWARD);
  PORTA &= 0;
  PORTA |=0x0A;   //pin 22 23 24 25
  analogWrite(PWM_L, l_pwm);
  analogWrite(PWM_R, r_pwm);  
 }

void Motor::moveBackward(int steps){
  updateEncoder(steps, BACKWARD);
  PORTA &= 0;
  PORTA |= 0x05;
  analogWrite(PWM_L, l_pwm);
  analogWrite(PWM_R, r_pwm);
}

void Motor::setPWM(int pwm){
  l_pwm = pwm;
  r_pwm = pwm;
  analogWrite(PWM_L , pwm);
  analogWrite(PWM_R, pwm);
}

void Motor::rotateLeft(int steps){
  updateEncoder(steps, ROTATING_LEFT);
  PORTA &= 0;
  PORTA |=0x06;
  setPWM(100);
  rotate_time_wait = millis();
  while(1){
    if(millis() - rotate_time_wait > 1){
      if(motorState == FORCE_STOP_LEFT){
        rotateLeftStop();
        delay(200);
        break;
      }
    }
  }
}

void Motor::rotateRight(int steps){
  updateEncoder(steps, ROTATING_RIGHT);

  PORTA &=0;
  PORTA |=0x09;
  setPWM(100);


 rotate_time_wait = millis();
  while(1){
    if(millis() - rotate_time_wait > 1){
      if(motorState == FORCE_STOP_RIGHT){
        rotateRightStop();
        delay(200);
        break;
      }
    }
  }
}

void Motor::completeRotate(int steps){
  updateEncoder(steps, ROTATING_RIGHT);
  PORTA &=0;
  PORTA |=0x09;
  updatePWM(0);

  rotate_time_wait = millis();
  while(1){
    if(millis() - rotate_time_wait > 1){
      if(motorState == FORCE_STOP_RIGHT){
        rotateRightStop();
        break;
      }
    }
  }
}

void Motor::moveLeft(){
  PORTA &= 0;
  PORTA |=0x02;
}
void Motor::moveRight(){
  PORTA &=0;
  PORTA |= 0x08;
}


void Motor::stopMovingForward(){

  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(10);
  PORTA |= 0x05;
  delay(20);
  analogWrite(PWM_R, 30);
  analogWrite(PWM_L, 30);
//  
//  analogWrite(PWM_R, 255);
//  analogWrite(PWM_L, 255);
}

void Motor::stopMovingBackward(){

  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(10);
  PORTA |=0x0A;
  delay(20);
  analogWrite(PWM_R, 30);
  analogWrite(PWM_L, 30);
//  
//  analogWrite(PWM_R, 255);
//  analogWrite(PWM_L, 255);
}

//void Motor::stopMoving(){
//
//  PORTA &= 0x00;
//  motorState = STOP;
//  analogWrite(PWM_L, 255);
//  analogWrite(PWM_R, 255);
//}

void Motor::rotateLeftStop(){
  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(10);
  PORTA |=0x09;
  delay(30);
  analogWrite(PWM_R, 20);
  analogWrite(PWM_L, 20);
  PORTA &= 0;
}

void Motor::rotateRightStop(){
  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(10);
  PORTA |=0x06;
  delay(30);
  analogWrite(PWM_R, 20);
  analogWrite(PWM_L, 20);
  PORTA &= 0;
}

void Motor::stopMoving(){
  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
}

void Motor::stopMovingBackSlow(){
  PORTA &= 0x00;
  motorState = STOP;
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(10);
  PORTA |=0x0A;
  delay(20);
  analogWrite(PWM_R, 18);
  analogWrite(PWM_L, 18);
  delay(20);
}

void Motor::updateEncoder(int steps, MotorState state){
  count = 0;
  steps_count = steps;
  motorState = state;
  timeISR = millis();
}
