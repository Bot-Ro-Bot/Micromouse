#include"config.h"
#include"motor.h"
#include<Wire.h>
#include<PID_v1.h>

#define EAST -90
#define WEST 90
#define NORTH 0
#define SOUTH 180

#define kp 2.0f
#define ki 0.1
#define kd 0.5

double Input, Output;
PID *pid;
void calculatePID(int16_t setPoint = 0);

//pid variables
int16_t input = 0, output = 0;
long currentTime = 0, previousTime = 0;
int16_t previousError = 0, error = 0;
float integral = 0, derivative = 0;


enum heading {
  East = 0, West, North, South
};

heading myDirection = North;

#define LeftOf(Dir) ((Dir+1)%4)

//#define LeftOf(Dir) ((Dir+1)%4)
#define RightOf(Dir) (Dir==North?East:(Dir)-1)

#define RelativeX(Dir) ((Dir==West)?(-1):((Dir==East)?1:0))
#define RelativeY(Dir) (Dir==South?(-1):(Dir==North?1:0))

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

Motor motor;

void setupPID() {
  pid = new PID(&Input, &Output, 0, kp, ki, kd, REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetSampleTime(5);
  pid->SetOutputLimits(-30, 30);
}


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
  setupPID();
  // right();
  //delay(5000);
  //left();
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
      break;
    }
    delay(20);
  }
}

void straight() {
  //  Input = (double)calculations();
  //  pid->Compute();
  //
  //  int pwmRight = Output + baseSpeed;
  //  int pwmLeft = Output - baseSpeed;
  //  if (pwmRight > 80) {
  //    pwmRight = 80;
  //  } else if (pwmRight < 50 ) {
  //    pwmRight = 0;
  //  }
  //  if (pwmLeft > 80) {
  //    pwmLeft = 80;
  //  } else if (pwmLeft < 50 ) {
  //    pwmRight = 0;
  //  }
  //  motor.Motion(Forward, pwmLeft, pwmRight);
  motor.Motion(Forward, baseSpeed + output, baseSpeed - output );
}

void loop() {
  readDistance();
  straight();
  calculatePID();
  if (millis() > 5000) {
    //right();
  }
  Serial.print(output); Serial.print("\t");
  Serial.println(error);
  //Serial.println(output);
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
