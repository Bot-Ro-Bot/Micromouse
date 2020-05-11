#pragma once
#include"wirish.h"

volatile  int countLeft = 0;

volatile  int tempCountRight = 0;

volatile  int countRight = 0;
volatile uint16_t rpmLeft = 0;
volatile uint16_t rpmRight = 0;
volatile long previousLeft = 0;
volatile long previousRight = 0;

enum motion {
  Forward = 0, Back, Stop, Brake, rotateLeft, rotateRight
};

class Motor {
  public:
    Motor();
    static void Motion(motion input, unsigned int SPEED1 = baseSpeed , unsigned int SPEED2 = baseSpeed);
};




/*
  for rising pulse only..7 ppr
  42mm*3.141=131.922
  131.922/7=18.84
  18.846/50=0.3769
  180÷0.3769

  0.75×90
  67.5÷0.3769

*/


/*
  for changing pulse..14 ppr
  42mm*3.141=131.922
  131.922/14=9.423
  9.432/50=0.18846
  180÷0.18846=955.109

*/

void rMotorCount() {
  //  countRight++;
  countRight += (digitalRead(rEncoderA) == digitalRead(rEncoderB)) ? +1 : -1;
  //  countRight = countRight < 0 ? 0 : countRight;
  if (countRight % 14 == 0) {
    rpmRight = micros() - previousRight;
    rpmRight = (60 * 1000000 / rpmRight) / 50;
    previousRight = micros();
  }
  //  if (countRight > 1000) {
  //    countRight = 0;
  //  }
}

void lMotorCount() {
  //  countLeft++;

  countLeft += (digitalRead(lEncoderA) == digitalRead(lEncoderB)) ? +1 : -1;
  //  countLeft = countLeft < 0 ? 0 : countLeft;
  tempCountRight++;
  if (tempCountRight > 14) {
    rpmLeft = micros() - previousLeft;
    previousLeft = micros();
    tempCountRight = 0;
  }
  //  if (countLeft > 1000) {
  //    countLeft = 0;
  //  }

}

Motor::Motor() {
  pinMode(lMotorEnable, PWM);
  pinMode(lMotorForward, OUTPUT);
  pinMode(lMotorBack, OUTPUT);
  pinMode(rMotorEnable, PWM);
  pinMode(rMotorForward, OUTPUT);
  pinMode(rMotorBack, OUTPUT);

  pinMode(rEncoderA, INPUT_PULLDOWN);
  pinMode(rEncoderB, INPUT_PULLDOWN);
  pinMode(lEncoderA, INPUT_PULLDOWN);
  pinMode(lEncoderB, INPUT_PULLDOWN);

  attachInterrupt(rEncoderA, rMotorCount, CHANGE);
  attachInterrupt(lEncoderA, lMotorCount, CHANGE);
}

void Motor::Motion(motion input, unsigned int SPEED1, unsigned int SPEED2) {
  analogWrite(lMotorEnable, SPEED1);
  analogWrite(rMotorEnable, SPEED2);

  switch (input) {
    case Forward:
      //left motor
      GPIOA_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 16);
      //right motor
      GPIOA_BASE->BSRR |= (1 << 4);
      GPIOA_BASE->BSRR |= (1 << 21);
      break;

    case Back:
      GPIOA_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 0);
      GPIOA_BASE->BSRR |= (1 << 20);
      GPIOA_BASE->BSRR |= (1 << 5);
      break;

    case Stop:
      GPIOA_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 16);
      GPIOA_BASE->BSRR |= (1 << 20);
      GPIOA_BASE->BSRR |= (1 << 21);
      break;

    case Brake:
      pwmWrite(lMotorEnable, 0);
      pwmWrite(rMotorEnable, 0);
      GPIOA_BASE->BSRR |= (1 << 0);
      GPIOA_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;

    case rotateRight:
      GPIOA_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 16);
      GPIOA_BASE->BSRR |= (1 << 20);
      GPIOA_BASE->BSRR |= (1 << 5);
      break;

    case rotateLeft:
      GPIOA_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 0);
      GPIOA_BASE->BSRR |= (1 << 4);
      GPIOA_BASE->BSRR |= (1 << 21);
      break;
  }
}
