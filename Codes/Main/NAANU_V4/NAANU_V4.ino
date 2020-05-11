#include"config.h"
#include"motor.h"
#include<Wire.h>

//absolute heading in degrees
#define EAST 90
#define WEST -90
#define NORTH 0
#define SOUTH 180

#define FRONT 0
#define RIGHT 1
#define LEFT 2

#define END(dist)  ((dist<36)?((dist>18)?1:0):0)
//#define endRight(distanceRight) ()
//#define endLeft(distanceLeft) ()

//angle waaala pid ko gains
#define kp 2.0f
#define ki 0.1
#define kd 0.5

void calculatePID(int16_t setPoint = 0);

//pid variables
int16_t input = 0, output = 0;
long currentTime = 0, previousTime = 0;
int16_t previousError = 0, error = 0;
float integral = 0, derivative = 0;

int16_t heading = NORTH;

//enum heading {
//  East = 0, West, North, South
//};
//
//heading myDirection = North;

//#define LeftOf(Dir) ((Dir+1)%4)

//#define LeftOf(Dir) ((Dir+1)%4)
//#define RightOf(Dir) (Dir==North?East:(Dir)-1)
//
//#define RelativeX(Dir) ((Dir==West)?(-1):((Dir==East)?1:0))
//#define RelativeY(Dir) (Dir==South?(-1):(Dir==North?1:0))

//vlx variables
uint16_t distance[3];
uint16_t distanceOld[3];

//battery variable
float volts = 0;

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

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  BUZZER_ON;
  delay(100);
  BUZZER_OFF;
  delay(100);
  if (!initializeVlx()) {
    BUZZER_ON;
    while (1);
  }
  BUZZER_ON;
  delay(50);
  initializeMPU();
  BUZZER_OFF;
  if (readBatteryVoltage() < 10.2) {
    BUZZER_ON;
    while (1);
  }
}

void constantDistance() {
  readDistance();
  if (distance[0] < 21) {
    motor.Motion(Back);
  } else if (distance[0] > 21) {
    motor.Motion(Forward);
  }
  if (distance[0] == 20) {
    motor.Motion(Stop);
  }
}


void right() {
  motor.Motion(rotateRight, 80, 80);
  calculations();
  int16_t angle = abs(calculations());
  long count = millis();
  while (1) {
    if (( (abs(calculations()) - angle) > 75) || ((millis() - count) > 500) ) {
      motor.Motion(rotateLeft, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch (heading) {
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

void left() {
  motor.Motion(rotateLeft, 80, 80);
  calculations();
  int16_t angle = abs(calculations());
  long count = millis();
  while (1) {
    if (( (abs(calculations()) - angle) > 72) || ((millis() - count) > 500) ) {
      motor.Motion(rotateRight, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      switch (heading) {
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
      break;
    }
    delay(20);
  }
}

void _180() {
  motor.Motion(rotateLeft, 80, 80);
  calculations();
  int16_t angle = abs(calculations());
  long count = millis();
  while (1) {
    if (( (abs(calculations()) - angle) > 165 ) || ((millis() - count) > 1000) ) {
      motor.Motion(rotateRight, 255, 255);
      delay(10);
      motor.Motion(Brake, 0, 0);
      //update heading
      break;
    }
    delay(20);
  }
}

void straight(int16_t setPoint) {
  calculatePID(setPoint);
  motor.Motion(Forward, baseSpeed + output, baseSpeed - output );
}

void loop() {
  //code lekhne for simple left wall follower
  readDistance();
  //check end condition
  if ((END(distance[FRONT]) && END(distance[RIGHT])) || (END(distance[FRONT]) && END(distance[LEFT]))  ) {
    motor.Motion(Stop);
    digitalWrite(LED_PIN, HIGH);
    BUZZER_ON;
    delay(300);
    BUZZER_OFF;
  }

  if (distance[LEFT] > 8) {
    left();
  }
  while (distance[FRONT] < 5) {
    right();
  }
  straight(heading);


  //printDistance();
  //  if (distance[0] > 3) {
  //    straight();
  //    calculatePID(0);
  //  }else motor.Motion(Stop);

  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
}

void calculatePID(int16_t setPoint) {
  currentTime = millis();
  error = setPoint - calculations();
  if (error > 180) {
    error = error - 360;
  }
  integral =  integral + (error * (currentTime - previousTime) / 1000) ;
  derivative = (error - previousError) / ((currentTime - previousTime) / 1000);
  output = kp * error + ki * integral + kd * derivative;
  output = (output > 30) ? 30 : (output < -30) ? -30 : output ;
  previousError = error;
  previousTime = currentTime;
}
