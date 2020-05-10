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
#define kp_vlx 0.5//1.25//0.125//0.4//1.625//1.5//0.5
#define ki_vlx 0//0.1//0.075 //0.1 //0.016 //0.01
#define kd_vlx 8//12500//2000//5000//0.018//0

#define kp_vlx_right 1.25
#define ki_vlx_right 0.1
#define kd_vlx_right 12500

#define kp_vlx_left 1.25
#define ki_vlx_left 0.1
#define kd_vlx_left 12500

#define kp_Encoder 1
#define ki_Encoder 0.001
#define kd_Encoder 250

#define kp_Rotation 1
#define ki_Rotation 0
#define kd_Rotation 0
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
  //  turnRight();
  //  delay(500);
  //  turnRight();
  //  delay(500);

  //      pid_turnRight();
  //  turnAngle(-calculateAngle());
  //  readDistance();
  //  printDistance();
  //  pid_moveForward();
  //  turnAngle(calculateAngle());
  //  turnAngle(-90);
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
  //  Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
  //    readDistance();
  //    printDistance();
  //  printNode();
  TremauxAlgorithm();
  if (node[7][7].map && node[7][8].map && node[8][7].map && node[8][8].map > 0) {
    Serial.print("EXPLORED");
    break;
  }
  //  BLUE_ON

  //  calculations();
  //  printData();//  printPosition();
  //  moveForward();
  //  delay(50);
  //  motor.Motion(Forward, 255, 255);
  //  Serial.println((60 * 1000000 / rpmLeft) / 50);
  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
  //    delay(50);
}
while (true) {
  returnNow();
  if (X == 0 && Y == 0) {
    break;
  }
}
myDirection = OppositeOf(myDirection);
motion.turnRight();
motion.turnRight();
X += RelativeX(myDirection);
Y += RelativeY(myDirection);

while (true) {
  wetRun();
  if (X >= 7 && X <= 8 && Y >= 7 && Y <= 8)
  {
    Serial.print("MAZE SOLVED");
    break;
  }
