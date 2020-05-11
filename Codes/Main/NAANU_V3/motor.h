#ifndef motor_H
#define motor_H

enum motion {
  Forward = 0, Back, Stop, Brake, rotateLeft, rotateRight, rotate360
};

class Motor {
  public:
    Motor();
    static void Motion(motion input, uint8_t SPEED1 = baseSpeed , uint8_t SPEED2 = baseSpeed);
    static void updatePWM();
    static void Right();
    static void Left();
    static void _360();
};


Motor::Motor() {
  pinMode(lMotorEnable, OUTPUT);
  pinMode(lMotorForward, OUTPUT);
  pinMode(lMotorBack, OUTPUT);
  pinMode(rMotorEnable, OUTPUT);
  pinMode(rMotorForward, OUTPUT);
  pinMode(rMotorBack, OUTPUT);
}

void Motor::Motion(motion input, uint8_t SPEED1, uint8_t SPEED2) {
  analogWrite(lMotorEnable, SPEED1);
  analogWrite(rMotorEnable, SPEED2);

  switch (input) {
    case Forward:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 17);

      GPIOA_BASE->BSRR |= (1 << 21);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;

    case Back:
      GPIOB_BASE->BSRR |= (1 << 16);
      GPIOB_BASE->BSRR |= (1 << 1);

      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 20);
      break;

    case Stop:
      GPIOB_BASE->BSRR |= (1 << 16);
      GPIOB_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 21);
      GPIOA_BASE->BSRR |= (1 << 20);
      break;

    case Brake:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;

    case rotateLeft:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 17);

      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 20);
      break;

    case rotateRight:
      GPIOB_BASE->BSRR |= (1 << 16);
      GPIOB_BASE->BSRR |= (1 << 1);

      GPIOA_BASE->BSRR |= (1 << 21);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;

    case rotate360:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 20);
      break;
  }
}

void Motor::Right() {

}

void Motor::Left() {

}

void Motor::_360() {

}

void Motor::updatePWM(){
  
}
#endif
