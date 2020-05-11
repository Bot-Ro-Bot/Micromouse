#include"config.h"
#include"motor.h"
#include"eeprom.h"
//#include"Node.h"
#include<Wire.h>
#include<PID_v1.h>

//angles according to mpu9250
#define EAST -90
#define WEST 90
#define NORTH 0
#define SOUTH 180

//sensor ko data array maa bhayera lina sajilo banauna lai matra ho
#define FRONT_R 0
#define FRONT_L 1
#define RIGHT 2
#define LEFT 3

//mpu pid ko gains
#define kp_mpu 1
#define ki_mpu 0.5
#define kd_mpu 5


//vlx pid ko gains
#define kp_vlx 1.25//0.125//0.4//1.625//1.5//0.5
#define ki_vlx 0.1//0.075 //0.1 //0.016 //0.01
#define kd_vlx 12500//2000//5000//0.018//0

#define kp_vlx_right 1.25
#define ki_vlx_right 0.1
#define kd_vlx_right 12500

#define kp_vlx_left 1.25
#define ki_vlx_left 0.1
#define kd_vlx_left 12500

#define kp_Encoder 10
#define ki_Encoder 0.001
#define kd_Encoder 2500

#define kp_Encoder_Rotation 3
#define ki_Encoder_Rotation 0
#define kd_Encoder_Rotation 3000
//pid variables
double Input, Output;

//vlx variables
uint16_t distance[4];
uint16_t distanceOld[4];

//mpu variables
int16_t  gyZ, mgX, mgY, mgZ;
float Gz, Mx, My, Mz;
float gzOffset;
float gyroYaw = 0;
float yaw = 0;
float magYaw = 0;
float magNormal;
float magInitialYaw = 0;
uint32_t loopTime = 0;

//node variables
uint8_t posX = 0, posY = 0;
//eeprom variables
bool endFlag = 0;
char eepromData;
char shortPath[256];
//instance of motor class
Motor motor;
//instance of PID class
PID  *pidVlx, *pidEncoder, *pidMpu, *pidRight, *pidLeft;

//instance of flash class
flash eeprom;



void setupPID() {
  pidVlx = new PID(&Input, &Output, 0, kp_vlx, ki_vlx, kd_vlx, REVERSE);
  pidVlx->SetMode(AUTOMATIC);
  pidVlx->SetSampleTime(5);
  pidVlx->SetOutputLimits(-50, 50);

  pidVlx = new PID(&Input, &Output, 0, kp_vlx_left, ki_vlx_left, kd_vlx_left, REVERSE);
  pidVlx->SetMode(AUTOMATIC);
  pidVlx->SetSampleTime(5);
  pidVlx->SetOutputLimits(-50, 50);

  pidVlx = new PID(&Input, &Output, 0, kp_vlx_right, ki_vlx_right, kd_vlx_right, REVERSE);
  pidVlx->SetMode(AUTOMATIC);
  pidVlx->SetSampleTime(5);
  pidVlx->SetOutputLimits(-50, 50);

  pidEncoder = new PID(&Input, &Output, 0, kp_Encoder, ki_Encoder, kd_Encoder, REVERSE);
  pidEncoder->SetMode(AUTOMATIC);
  pidEncoder->SetSampleTime(5);
  pidEncoder->SetOutputLimits(-50, 50);

  pidMpu = new PID(&Input, &Output, 0, kp_mpu, ki_mpu, kd_mpu, REVERSE);
  pidMpu->SetMode(AUTOMATIC);
  pidMpu->SetSampleTime(5);
  pidMpu->SetOutputLimits(-50, 50);
}

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  //  disableDebugPorts();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  BUZZER_ON;
  initializeVlx();
  BUZZER_ON;
  initializeMPU();
  BUZZER_OFF;
  setupPID();
  //    printPotential();
  //    Serial.println("''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''");
  potential();

  //    printPotential();
  //  Serial.println(readBatteryVoltage());
  //  printDirection();
  //  updateDirection();
  //  printDirection();
  //  printPosition();
  //  updatePosition();
  //  printPosition();

  //    moveForward();
  //  delay(500);
  //  moveForward();
  //  align();
  //
  //  align();
  //  turnRight();
  //    turnRight();

  //  _right();
  //pi d
  /*
    //  while (countRight < 500 && countLeft < 500 ) {
    //    Input = countLeft - countRight - 0.5;
    //    pidEncoder->Compute();
    //    motor.Motion(Forward, 50 - Output, 50 + Output);
    ////    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
    //  }

    motor.Motion(Back, 255, 255);
    motor.Motion(Brake);
  */

  endFlag =  * (bool *) flagAddress ;
if(endFlag){
  
}
  //  turnLeft();
  //  for (int i = 0; i < 4; i++) {
  //    pid_turnLeft();
  //    //    turnRight();
  //    delay(200);
  //  }
  //  delay(5000);
  //  for (int i = 0; i < 4; i++) {
  //    //    turnLeft();
  //      turnRight();
  //    delay(200);
  //  }
  //    newMotion();
  //  turnLeft();

  //  motor.Motion(Forward, 33, 33);
  //  delay(2000);
  //  motor.Motion(Brake);

//  pid_turnLeft();
}

void leftWall() {
  moveForward();
  if (!wallLeft() && wallRight()) {
    turnLeft();
    return;
  } else if (!wallRight()) {
    turnRight();
    return;
  } else if (wallRight() && wallLeft()) {
    turnRight();
    turnRight();
    return;
  }
}

void loop() {
  //  Serial.println(calculateAngle());
  //    leftWall();
  //  TremauxAlgorithm();
  //  moveForward();
  //  printNode();
  //  delay(100);
    readDistance();
    printDistance();
  //  align();
  //  calculations();
  //  printData();
  //  delay(4);

  //  pid_turnLeft();
  //  motor.Motion(Forward, 30, 30);
  //
  //  //  moveForward();
  //  delay(2000);
  //  //  motor.Motion(Back, 30, 30);
  //
  //  delay(200);
  //  Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
  //
  //  Serial.print( "Left:    "); Serial.print((60 * 1000000 / rpmLeft) / 50); Serial.print( " Right:    "); Serial.println(rpmRight);

  //  Serial.println(readBatteryVoltage());
  //    readDistance();
  //
  //  if (((distance[FRONT_L] - distance[FRONT_R])) > 0) {
  //    motor.Motion(Forward, baseSpeed, 0);
  //  } else if (((distance[FRONT_L] - distance[FRONT_R])) < 0) {
  //    motor.Motion(Forward, 0, baseSpeed);
  //  } else motor.Motion(Brake);
  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
}
