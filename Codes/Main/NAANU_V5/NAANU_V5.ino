#include"config.h"
#include"motor.h"
#include<Wire.h>
#include<PID_v1.h>

#define EAST -90
#define WEST 90
#define NORTH 0
#define SOUTH 180

#define FRONT 0
#define RIGHT 1
#define LEFT 2


//mpu pid ko gains
#define kp_mpu 2.0f
#define ki_mpu 0.1
#define kd_mpu 0.5

//vlx pid ko gains
#define kp_vlx 0.55//1.625//1.5//0.5
#define ki_vlx 0.15 //0.016 //0.01
#define kd_vlx 5000//0.018//0

//front sensors pid ko gains
#define kp_front 1.5
#define ki_front 0
#define kd_front 0

double Input, Output;
//vlx variables
uint16_t distance[3];
uint16_t distanceOld[3];

//mpu variables
volatile int16_t  gyZ, mgX, mgY, mgZ;
volatile float Gz, Mx, My, Mz;
volatile float gzOffset;
volatile float gyroYaw = 0;
volatile float yaw = 0;
volatile float magYaw = 0;
volatile float magNormal;
volatile float magInitialYaw = 0;
volatile uint32_t loopTime = 0;

//instance of motor class
Motor motor;
PID *pidMpu, *pidVlx, *pidFront;

HardwareTimer timer(2);

void handler_loop() {
  //interrupt bolayo bhane majjaley na adki aauxa data..i2c ley ni synchronize garna interrput garxa timer 1/2 kai registers utilize garera
  //takes 531us for calculations and 741us for readDistance
  /// interrupts();
  calculations();
  readDistance();
}

void setupTimer() {
  timer.pause();
  timer.setPeriod(4000);
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler_loop);
  timer.refresh();
  timer.resume();
}

void setupPID() {
  pidMpu = new PID(&Input, &Output, &heading, kp_mpu, ki_mpu, kd_mpu, REVERSE);
  pidMpu->SetMode(AUTOMATIC);
  pidMpu->SetSampleTime(5);
  pidMpu->SetOutputLimits(-40, 40);

  pidVlx = new PID(&Input, &Output, 0, kp_vlx, ki_vlx, kd_vlx, REVERSE);
  pidVlx->SetMode(AUTOMATIC);
  pidVlx->SetSampleTime(5);
  pidVlx->SetOutputLimits(-40, 40);

  pidFront = new PID(&Input, &Output, 0, kp_front, ki_front, kd_front, REVERSE);
  pidFront->SetMode(AUTOMATIC);
  pidFront->SetSampleTime(5);
  pidFront->SetOutputLimits(-40, 40);
}

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  //  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PB3, OUTPUT);
  //  pinMode(BATTERY_PIN, INPUT);
  //  BUZZER_ON;
  //  delay(100);
  //  BUZZER_OFF;
  //  delay(100);
  //
  //  float volts = readBatteryVoltage();
  //  Serial.println(volts);
  //  if (volts < 10.5) {
  //    BUZZER_ON;
  //    BLUE_HIGH;
  //    delay(100);
  //    BUZZER_OFF;
  //    BLUE_LOW;
  //    delay(100);
  //  }
  //  if (!initializeVlx()) {
  //    //    BUZZER_ON;
  //    while (1);
  //  }
  //  BUZZER_ON;
  initializeMPU();
  //  BUZZER_OFF;
  //setupTimer();

  //  setupPID();
}


//supposed linearity
/*200=255=12.6v
  v=rw ,r=42mm,w=200/255=62.4
  v=274.36mm/s
  w=dtheta /dt
  90 degree turn implied
  v=rw,r=6cm,v=274.36mm/s
  w=4.573rad/s
  therefore, dt=0.34516s=345ms*/
void right() {
  int16_t angle = abs(calculations());
  //int16_t angle = abs(yaw);

  motor.Motion(rotateRight, 70, 70);
  long count = millis();
  while (1) {

    if (( abs(abs(calculations()) - angle)  > 80) || ((millis() - count) > 350 * (12.2 / readBatteryVoltage())) ) {
      motor.Motion(rotateLeft, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch ((int)heading) {
        case NORTH:
          heading = EAST;
          break;
        case EAST:
          heading = SOUTH;
          break;
        case SOUTH:
          heading = WEST;
          break;
        case WEST:
          heading = NORTH;
          break;
      }
      break;
    }
    delay(20);
  }
  //  pidMpu->resetValue();
  //  pidVlx->resetValue();

}

void left() {
  int16_t angle = abs(calculations());
  //int16_t angle = abs(yaw);
  motor.Motion(rotateLeft, 75, 75);
  long count = millis();
  while (1) {
    if (( abs(abs(calculations()) - angle)  > 80) || ((millis() - count) > 350 * (12.2 / readBatteryVoltage())) ) {
      motor.Motion(rotateRight, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch ((int)heading) {
        case NORTH:
          heading = WEST;
          break;
        case EAST:
          heading = NORTH;
          break;
        case SOUTH:
          heading = EAST;
          break;
        case WEST:
          heading = SOUTH;
          break;
      }
      delay(1000);
      break;
    }
    delay(20);
  }
  //  pidMpu->resetValue();
  //  pidVlx->resetValue();
}

void _180() {
  motor.Motion(rotateLeft, 75, 75);
  calculations();
  int16_t angle = abs(yaw);
  long count = millis();
  while (1) {
    if (( abs(abs(yaw) - angle)  > 165) || ((millis() - count) > 1000) ) {
      motor.Motion(rotateRight, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      break;
    }
    delay(20);
  }
  // pidMpu->resetValue();
  //  pidVlx->resetValue();
}



void straight(int16_t setPoint) {
  //pidVlx->resetValue();
  //pidMpu->resetValue();

  while (1) {
    readDistance();
    int16_t pwm;
    // pwm = pidVlx->calculatePID( distance[LEFT], 3) * 0.9864f  + ((pidMpu->calculatePID( setPoint, calculations()))) * 0.3;
    motor.Motion(Forward, baseSpeed - pwm, baseSpeed + pwm);
    if (distance[LEFT] > 8 || distance[RIGHT] > 8) {
      motor.Motion(Stop);
      return;
    }
  }
}

int STRAIGHT() {
  //  Input =  (int)heading - yaw;
  //  Serial.println(Input);
  //  pidMpu->Compute();
  int angle = calculations();
  while (1) {
    readDistance();
    if (distance[FRONT] < 95) {
      motor.Motion(Back);
      motor.Motion(Stop);
      BUZZER_ON;
      delay(100);
      BUZZER_OFF;
      delay(500);
      return 0;
    }
    else if (distance[RIGHT] < 60 && distance[LEFT] < 60) {
      Input = distance[RIGHT] - distance[LEFT];
      pidVlx->Compute();
      motor.Motion(Forward, baseSpeed - Output, baseSpeed + Output);
    }
    else if (distance[RIGHT] < 60) {
      Input = ( distance[RIGHT] - 30);
      pidVlx->Compute();
      motor.Motion(Forward, baseSpeed - Output, baseSpeed + Output);
    }
    else if (distance[LEFT] < 60) {
      Input = (30 - distance[LEFT]);
      pidVlx->Compute();
      motor.Motion(Forward, baseSpeed - Output, baseSpeed + Output);
    }
    else if (distance[RIGHT] > 60 && distance[LEFT] > 60) {
      //
      //Input = angle - calculations();
      //pidMpu->Compute();
      // motor.Motion(Forward, baseSpeed - Output, baseSpeed + Output);
      motor.Motion(Forward, baseSpeed , baseSpeed);
      //motor.Motion(Back);
      //motor.Motion(Stop);
      //BUZZER_ON;
      //delay(100);
      //BUZZER_OFF;
      //return 0;
    }
  }


}

void _right() {
  calculations();
  int16_t dummy = gyroYaw;
  gyroYaw = 0;
  int16_t angle = abs(calculations());

  motor.Motion(rotateRight, 80, 80);
  long count = millis();
  while (1) {
    if (( abs(abs(calculations()) - angle)  > 73) || ((millis() - count) > 500) ) {
      motor.Motion(rotateLeft, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch ((int)heading) {
        case NORTH:
          heading = EAST;
          break;
        case EAST:
          heading = SOUTH;
          break;
        case SOUTH:
          heading = WEST;
          break;
        case WEST:
          heading = NORTH;
          break;
      }
      gyroYaw = dummy - 90;
      break;
    }
    delay(20);
  }
}

void _left() {
  calculations();
  int16_t dummy = gyroYaw;
  gyroYaw = 0;
  int16_t angle = abs(calculations());
  motor.Motion(rotateLeft, 70, 70);
  long count = millis();
  while (1) {
    if (( abs(abs(calculations()) - angle)  > 73) || ((millis() - count) > 500) ) {
      motor.Motion(rotateRight, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch ((int)heading) {
        case NORTH:
          heading = WEST;
          break;
        case EAST:
          heading = NORTH;
          break;
        case SOUTH:
          heading = EAST;
          break;
        case WEST:
          heading = SOUTH;
          break;
      }
      gyroYaw = dummy + 90;
      break;
    }
    delay(20);
  }
}


void __right() {
  resetMPU();
  int16_t angle = abs(calculations());

  motor.Motion(rotateRight, 80, 80);
  long count = millis();
  while (1) {
    if (( abs(abs(calculations()) - angle)  > 73) || ((millis() - count) > 500) ) {
      motor.Motion(rotateLeft, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch ((int)heading) {
        case NORTH:
          heading = EAST;
          break;
        case EAST:
          heading = SOUTH;
          break;
        case SOUTH:
          heading = WEST;
          break;
        case WEST:
          heading = NORTH;
          break;
      }
      break;
    }
    delay(20);
  }
}
void constantDistance() {
  readDistance();
  Input = 500 - distance[FRONT];
  pidFront->Compute();
  motor.Motion(Forward);
}


void loop() {

  //motor.Motion(Forward,255,255);
  //
  //  if ( STRAIGHT() == 0) {
  //    readDistance();
  //    if (distance[LEFT] > 80) {
  //      right(); //left ho
  //    }
  //    else if (distance[RIGHT] > 80) {
  //      left(); //right ho
  //    } else if (distance[RIGHT] < 60 && distance[LEFT] < 60) {
  //
  //    }
  //    BUZZER_ON;
  //    delay(100);
  //    BUZZER_OFF;
  //    delay(100);
  //  }


  //  readDistance();
  //  printDistance();
  calculations();
  printData();

  digitalWrite(PB3, 1 ^ digitalRead(PB3));
}
