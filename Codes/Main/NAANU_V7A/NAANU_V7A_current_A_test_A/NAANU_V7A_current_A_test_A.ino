#include"config.h"
#include"motor.h"
#include<Wire.h>
#include<PID_v1.h>
#include"flash_stm32.h"

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

/*******************************************************************************************************************************************/
#define SPEED 50
/*******************************************************************************************************************************************/


//speed run parameters
/*******************************************************************************************************************************************/
#define _FORWARD 88
#define _RIGHT 66
#define _LEFT  44
#define _END 55

/*******************************************************************************************************************************************/



//pid variables
/*******************************************************************************************************************************************/
double Input, Output;
/*******************************************************************************************************************************************/


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


//eeprom variables int naanu = 0;
#define flagAddress 0x0801F400
#define pathAddress 0x0801F404
uint16_t endFlag = 0;
uint16_t shortPath[256];
uint16_t writeOffset;
uint16_t readOffset;
uint8_t path = 0;

//instance of motor class
Motor motor;

//instance of PID class
PID  *pidVlx, *pidEncoder, *pidMpu, *pidRight, *pidLeft;

bool SPEED_RUN = 0;
bool SEARCH_RUN = 1;

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
  initializeMPU();
  BUZZER_OFF;
  setupPID();
  potential();

  endFlag =  * (uint16_t *) flagAddress ;

//  if (endFlag == 31) {
//    path =  * (uint16_t *) pathAddress ;
//    for (int i = 0; i < path; i++) {
//      shortPath[path - i - 1] = eepromRead();
//      Serial.println(shortPath[path - i - 1]);
//      if (shortPath[i] == 31) {
//        break;
//      }
//    }
//    BUZZER_ON;
//    delay(1000);
//    BUZZER_OFF;
//    speedRun();
//    while (1);
//  }
  //  testEeprom();
}
//ulto read garne code

void loop() {
  //  Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
//    readDistance();
//    printDistance();
  //  printNode();
  TremauxAlgorithm();
  //  calculations();
  //  printData();
  //  printPosition();
  //  motor.Motion(Forward, 255, 255);
  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
}
