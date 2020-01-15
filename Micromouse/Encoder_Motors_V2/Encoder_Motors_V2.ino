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

bool temp = 0;
volatile  int countLeft = 0;
volatile  int countRight = 0;


volatile unsigned long previous = 0;
volatile unsigned long current = 0;

volatile unsigned int rpm = 0;

volatile unsigned int countsLeft;
volatile unsigned int countsRight;
volatile bool rPreviousState = 0;
volatile bool rCurrentState = 0;

volatile bool lPreviousState = 0;
volatile bool lCurrentState = 0;

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

void handler_loop() {
  //  rCurrentState = digitalRead(rEncoderB);
  //  if (rCurrentState == 1 && rPreviousState == 0) {
  //    countRight++;
  //  }
  //  rPreviousState = rCurrentState;
  //
  //  lCurrentState = digitalRead(lEncoderB);
  //  if (lCurrentState == 1 && lPreviousState == 0) {
  //    countLeft++;
  //  }
  //  lPreviousState = lCurrentState;
  //
  //  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));

}
void setupTimer() {
  //  timer.pause();
  //  timer.setPeriod(30000);
  //  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  //  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  //  timer.attachCompare1Interrupt(encoderRight);
  //  timer.refresh();
  //  timer.resume();

  //  Timer2.attachCompare1Interrupt(encoderLeft);
  //  Timer2.attachCompare2Interrupt(encoderRight);

}

enum motion {
  Forward = 0, Back, Stop, Brake, rotateLeft, rotateRight, rotate360
};

void rMotorCount() {
  countRight += (digitalRead(rEncoderA) == digitalRead(rEncoderB)) ? +1 : -1;
  countRight = countRight < 0 ? 0 : countRight;

  if (countRight >= 7) {
    current = micros();
    rpm = current - previous;
    previous = current;
    countRight = 0;
  }

}

void lMotorCount() {
  //  detachInterrupt(lEncoderB);
  //  lCurrentState = digitalRead(lEncoderB);
  //  if (lCurrentState == 1 && lPreviousState == 0) {
  //    countLeft++;
  //  }
  //  lPreviousState = lCurrentState;
  //  attachInterrupt(lEncoderB, lMotorCount, RISING);
  countLeft++;
}

class Motor {
  private:
  public:
    Motor();
    static void Motion(motion input, int SPEED1 = baseSpeed, int SPEED2 = baseSpeed);
};

Motor::Motor() {
  pinMode(lMotorEnable, PWM);
  pinMode(lMotorForward, OUTPUT);
  pinMode(lMotorBack, OUTPUT);
  pinMode(rMotorEnable, PWM);
  pinMode(rMotorForward, OUTPUT);
  pinMode(rMotorBack, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(rEncoderA, INPUT_PULLDOWN);
  pinMode(rEncoderB, INPUT_PULLDOWN);

  pinMode(lEncoderA, INPUT_PULLDOWN);
  pinMode(lEncoderB, INPUT_PULLDOWN);
  attachInterrupt(rEncoderA, rMotorCount, RISING);
  //attachInterrupt(rEncoderB, rMotorCount, RISING);

  attachInterrupt(lEncoderA, lMotorCount, RISING);
  //  attachInterrupt(lEncoderB, lMotorCount, RISING);
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

Motor MicroGear;

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  Serial.begin(115200);

}

void loop() {
  //  Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);

    MicroGear.Motion(Back, 30, 30);

  Serial.println((60 * 1000000 / rpm) / 50);
  //  Input = countLeft - countRight;
  //  Input = countLeft - countRight;
  //  pidEncoder->Compute();
  //
  //   MicroGear.Motion(Forward, baseSpeed +Output, baseSpeed - Output);
  //  if (countRight <= 350) {
  //    MicroGear.Motion(Back, 0, 5800);
  //  } else {
  //    if (!temp) {
  //      temp = 1;
  //      MicroGear.Motion(Forward, 255, 255);
  //    }
  //
  //    MicroGear.Motion(Stop);
  //  }

  //  delay(1000);
  //  MicroGear.Motion(Stop);
  //  delay(1000);
}
