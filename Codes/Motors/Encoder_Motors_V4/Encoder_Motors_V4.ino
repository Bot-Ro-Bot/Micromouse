#include<PID_v1.h>

//HardwareTimer timer(2);

#define LED_PIN PB12
#define BLUE_HIGH   (GPIOB_BASE->BSRR |= (1 << 12))
#define BLUE_LOW    (GPIOB_BASE->BSRR |= (1 << 28))

#define lMotorEnable PA6
#define lMotorForward PB1
#define lMotorBack PB0

#define rMotorEnable PA7
#define rMotorForward PA5
#define rMotorBack PA4

#define lEncoderA PB8
#define lEncoderB PB9
#define rEncoderA PB10
#define rEncoderB PB11

#define baseSpeed 50

int32_t channel_1_start, channel_1_stop, channel_1;


#define kp 2.0f
#define ki 0
#define kd 0

double Input, Output;

PID *pidEncoder;

void setupPID() {
  pidEncoder = new PID(&Input, &Output, 0, kp, ki, kd, REVERSE);
  pidEncoder->SetMode(AUTOMATIC);
  pidEncoder->SetSampleTime(5);
  pidEncoder->SetOutputLimits(-40, 40);
}


void setupTimer() {
  Timer4.attachCompare3Interrupt(encoderLeft);
  //  Timer4.attachCompare2Interrupt(encoderRight);

  TIMER4_BASE->CR1 = TIMER_CR1_CEN;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = TIMER_DIER_CC3IE;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = 0b100000001;
  TIMER4_BASE->CCMR2 = 0b100000001;
  TIMER4_BASE->CCER = TIMER_CCER_CC3E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 0xFFFF;
  TIMER4_BASE->DCR = 0;
}

void encoderLeft(void) {
  if (0b1 & GPIOB_BASE->IDR >> 8) {
    channel_1_start = TIMER4_BASE->CCR3;
    TIMER4_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else {
    channel_1 = TIMER4_BASE->CCR3 - channel_1_start;
    if (channel_1 < 0)channel_1 += 0xFFFF;
    TIMER4_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}


//void encoderRight(void) {
//  if (0b1 & GPIOA_BASE->IDR) {
//    channel_1_start = TIMER4_BASE->CCR1;
//    TIMER4_BASE->CCER |= TIMER_CCER_CC1P;
//  }
//  else {
//    channel_1 = TIMER4_BASE->CCR1 - channel_1_start;
//    if (channel_1 < 0)channel_1 += 0xFFFF;
//    TIMER4_BASE->CCER &= ~TIMER_CCER_CC1P;
//  }
//}

enum motion {
  Forward = 0, Back, Stop, Brake, rotateLeft, rotateRight, rotate360
};

class Motor {
  private:
  public:
    Motor();
    static void Motion(motion input, int SPEED1 = baseSpeed, int SPEED2 = baseSpeed);
};

Motor::Motor() {

}

void Motor::Motion(motion input, int SPEED1, int SPEED2 ) {
  pwmWrite(lMotorEnable, SPEED1);
  pwmWrite(rMotorEnable, SPEED2);

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

    case rotateRight:
      GPIOB_BASE->BSRR |= (1 << 16);
      GPIOB_BASE->BSRR |= (1 << 1);
      GPIOA_BASE->BSRR |= (1 << 5);
      GPIOA_BASE->BSRR |= (1 << 20);
      break;

    case rotateLeft:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 21);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;

    case rotate360:
      GPIOB_BASE->BSRR |= (1 << 0);
      GPIOB_BASE->BSRR |= (1 << 17);
      GPIOA_BASE->BSRR |= (1 << 21);
      GPIOA_BASE->BSRR |= (1 << 4);
      break;
  }
}
//
//Motor motor;

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  pinMode(lMotorEnable, PWM);
  pinMode(lMotorForward, OUTPUT);
  pinMode(lMotorBack, OUTPUT);
  pinMode(rMotorEnable, PWM);
  pinMode(rMotorForward, OUTPUT);
  pinMode(rMotorBack, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(rEncoderA, INPUT_PULLDOWN);
  pinMode(rEncoderB, INPUT_PULLDOWN);
  setupTimer();
  Serial.begin(115200);
}

void loop() {
  Serial.print("Left :  "  ); Serial.println(channel_1); //Serial.print(" Right :  "  ); Serial.println(countRight);

}
