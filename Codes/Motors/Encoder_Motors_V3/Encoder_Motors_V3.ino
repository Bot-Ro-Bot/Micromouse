#include<PID_v1.h>

#define LED_PIN PB3
#define BLUE_HIGH   (GPIOB_BASE->BSRR |= (1 << 12))
#define BLUE_LOW    (GPIOB_BASE->BSRR |= (1 << 28))

#define lMotorEnable PA2
#define lMotorForward PA1
#define lMotorBack PA0

#define rMotorEnable PA3
#define rMotorForward PA4
#define rMotorBack PA5


#define baseSpeed 60

enum motion {
  Forward = 0, Back, Stop, Brake, rotateLeft, rotateRight, rotate360
};


class Motor {
  private:
  public:
    Motor();
    static void Motion(motion input, int SPEED1 = baseSpeed, int SPEED2 = baseSpeed);
    void forward();
    void back();
};

Motor::Motor() {
  pinMode(lMotorEnable, OUTPUT);
  pinMode(lMotorForward, OUTPUT);
  pinMode(lMotorBack, OUTPUT);
  pinMode(rMotorEnable, OUTPUT);
  pinMode(rMotorForward, OUTPUT);
  pinMode(rMotorBack, OUTPUT);
}

void Motor::forward() {
  analogWrite(lMotorEnable, 30);
  analogWrite(rMotorEnable, 30);
  digitalWrite(lMotorForward, HIGH);
  digitalWrite(lMotorBack, LOW);
  digitalWrite(rMotorForward, HIGH);
  digitalWrite(rMotorBack, LOW);
}


void Motor::back() {
  analogWrite(lMotorEnable, 30);
  analogWrite(rMotorEnable, 30);
  digitalWrite(lMotorForward, LOW);
  digitalWrite(lMotorBack, HIGH);
  digitalWrite(rMotorForward, LOW);
  digitalWrite(rMotorBack, HIGH);
}

void Motor::Motion(motion input, int SPEED1, int SPEED2 ) {
  analogWrite(lMotorEnable, SPEED1);
  analogWrite(rMotorEnable, SPEED2);

  switch (input) {
    case Forward:
      GPIOA_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 16);

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
      GPIOA_BASE->BSRR |= (1 << 0);
      GPIOA_BASE->BSRR |= (1 << 1);

      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;
  }
}

Motor motor;

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  disableDebugPorts();
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

}

void loop() {
    motor.Motion(Forward);
  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));

}
