#include "VL53L0X.h"
#include"motors.h"
//#include "Battery.h"
#include<Wire.h>

#define MPU_Address 0x68

#define frontAddress 0x30
#define rightAddress 0x31
#define leftAddress 0x32

#define xshutLeft PB4
#define xshutRight PB3
#define xshutFront PB5

#define lMotorEnable PA7
#define lMotorForward PB0
#define lMotorBack PB1

#define rMotorEnable PA6
#define rMotorForward PA4
#define rMotorBack PA5

#define BUZZER_ON   (GPIOA_BASE->BSRR |= (1 << 0 ))
#define BUZZER_OFF  (GPIOA_BASE->BSRR |= (1 << 16))

#define kp 2.0f
#define kd 0
#define ki 0

HardwareTimer timer(2);

VL53L0X sensorFront, sensorRight, sensorLeft;

Motor *leftMotor, *rightMotor;

//mpu variables
int16_t gyX, gyY, gyZ, mgX, mgY, mgZ;
float Gx, Gy, Gz, Mx, My, Mz;
float gxOffset, gyOffset, gzOffset;
float gyroYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float magYaw = 0;
float magNormal;
float magInitialYaw = 0;


//vlx variables
uint16_t distance[3];

//pid variables
int16_t input = 0, output = 0;
uint32_t currentTime = 0, previousTime = 0;
int16_t previousError = 0, error = 0;
float integral, derivative;

//mpu ko functions
void mpuSetup();
void calibrateMPU();
void readData();
void calculations();


//VLX ko functions
void initializeVLX();
void setId();
void checkDevice();

//pid ko functions
void calculatePID(int16_t setPoint);
int16_t setPoint = 0;

//timer ko functions
void setupTimer() {
  timer.pause();
  timer.setPeriod(4000);
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler_loop);
  timer.refresh();
  timer.resume();
}

void handler_loop() {
  calculations();
  calculatePID(setPoint);
  //Serial.println(".");
}

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);
  disableDebugPorts();

  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  //initializeVLX();

  pinMode(PA0, OUTPUT);
  checkDevice();
  mpuSetup();

  BUZZER_ON;
  delay(100);
  BUZZER_OFF;

  calibrateMPU();

  BUZZER_ON;
  delay(200);
  BUZZER_OFF;

  leftMotor  = new Motor(lMotorEnable, lMotorForward, lMotorBack);
  rightMotor = new Motor(rMotorEnable, rMotorForward, rMotorBack);

  setupTimer();


  delay(1000);
  BUZZER_ON;
  delay(500);
  BUZZER_OFF;
  delay(500);
}


void right() {
  while ((int)gyroYaw != -90) {
    if ( (int)gyroYaw < -90) {
      leftMotor->stop();
      rightMotor->stop();
      break;
    }
    leftMotor->back(50);
    rightMotor->forward(50);
  }
}

void left() {
  while ((int)yaw != 90) {
    if ( (int)yaw > 90) {
      leftMotor->stop();
      rightMotor->stop();
      break;
    }
    leftMotor->forward(50);
    rightMotor->back(50);

  }

}
void straight(uint16_t angle) {
  //  setPoint = angle;
  //  int16_t  adjust = output;
  //  leftMotor->forward(50 - adjust);
  //  rightMotor->forward(50 + adjust);
  leftMotor->forward(50 );
  rightMotor->forward(50);

}



void loop() {

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////VLX_FUNCTIONS//////////////////////////////////////////////////////////////////////////////////
void initializeVLX() {
  pinMode(xshutFront, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  setId();

  checkDevice();
  //Serial.println("checkdevice ended");

  sensorFront.init();
  sensorFront.setTimeout(500);
  sensorFront.startContinuous();
  Serial.println("front initialized");

  checkDevice();

  sensorRight.init();
  sensorRight.setTimeout(500);
  sensorRight.startContinuous();
  Serial.println("right initialized");

  sensorLeft.init();
  sensorLeft.setTimeout(500);
  sensorLeft.startContinuous();
  Serial.println("left initialized");

}


void setId() {
  //all sensors in reset/(OFF)
  digitalWrite(xshutFront, LOW);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  delay(10);

  // all sensors is ON
  digitalWrite(xshutFront, HIGH);
  digitalWrite(xshutRight, HIGH);
  digitalWrite(xshutLeft, HIGH);
  delay(10);

  // keeping front sensor ON while turning OFF sensors on the LEFT and RIGHT
  digitalWrite(xshutFront, HIGH);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  sensorFront.setAddress(frontAddress); //setting the address (API mai bhako function raixa yo)
  delay(10);
  Serial.println("front done");    //debugging purpose ko laagi print matra haaneyko

  // keeping front sensor and Left sensor ON while turning OFF sensor on the RIGHT
  digitalWrite(xshutLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  delay(10);
  sensorLeft.setAddress(leftAddress);
  delay(10);
  Serial.println("left done");

  digitalWrite(xshutRight, HIGH);
  delay(10);
  sensorRight.setAddress(rightAddress);
  delay(10);
  Serial.println("right done");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////MPU_FUNCTIONS//////////////////////////////////////////////////////////////////////////////////

void mpuSetup() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x00);                 //Disable sleep mode                                         Put bit 3 to 1 if you want to disable temperature sensor
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1B);
  Wire.write(0b00011000);                         //Select degrees per second for gyro to read data accordingly   +-500dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1C);
  Wire.write(0b00011000);                        ////Select range for accl to read data accordingly              +-8g
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x37);
  Wire.write(0x02);                           //enable slave device 1 (magnetometer) inside the sensor itself
  Wire.endTransmission();

  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
  Wire.endTransmission();

  Serial.println("MPU Setup Done");
}


void calibrateMPU() {
  // GYRO CALIBRATION
  Serial.println("Calibrating gyro...");
  delay(100);                               //Ignore data for half of a second for better accuracy

  for (int32_t i = 0; i < 2000; i++) {
    readData();
    gxOffset = gxOffset + gyX;
    gyOffset = gyOffset + gyY;
    gzOffset = gzOffset + gyZ;
    delay(1);
    if (i % 200 == 0) Serial.println("...");
  }
  gxOffset = gxOffset / 2000;
  gyOffset = gyOffset / 2000;
  gzOffset = gzOffset / 2000;
  Serial.println("Gyroscope Calibrated.");

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //hard iron calibration

  readData();
  Mx = mgX * 0.15 * 1.19f;
  My = mgY * 0.15 * 1.20f;
  Mz = mgZ * 0.15 * 1.15f;

  // normalize magnetometer data
  magNormal = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx = Mx / magNormal;
  My = My / magNormal;
  Mz = Mz / magNormal;

  magInitialYaw = atan2(My, Mx) * 57.29f;
}


void readData() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_Address, 6);
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 7); //7 kina leko maathi bhaneko xa
  mgX = Wire.read() << 8 | Wire.read();
  mgY = Wire.read() << 8 | Wire.read();
  mgZ = Wire.read() << 8 | Wire.read();
  uint8_t ST2 = Wire.read();
}

void calculations(void) {
  readData();
  gyX -= gxOffset;
  gyY -= gyOffset;
  gyZ -= gzOffset;

  //in degrees per second
  Gx = gyX / 16.4f;
  Gy = gyY / 16.4f;
  Gz = gyZ / 16.4f;

  //in uT
  Mx = mgX * 0.15f * 1.19f;
  My = mgY * 0.15f * 1.20f;
  Mz = mgZ * 0.15f * 1.15f;

  // normalize magnetometer data
  magNormal = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx = Mx / magNormal;
  My = My / magNormal;
  Mz = Mz / magNormal;

  pitch = pitch + Gx * 0.004;
  roll = roll + Gy * 0.004;
  gyroYaw = gyroYaw + Gz * 0.004;

  magYaw = atan2(My, Mx) * 57.29f - magInitialYaw;

  yaw = gyroYaw * 0.97 + magYaw * 0.03;
}


void checkDevice() {
  Serial.println("STARTING");

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(3000);           // wait 5 seconds for next scan*/
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////PID_FUNCTIONS//////////////////////////////////////////////////////////////////////////////////

void calculatePID(int16_t setPoint) {
  currentTime = millis();
  error = setPoint - (int)yaw;
  integral =  integral + error * (currentTime - previousTime);
  derivative = (error - previousError) / (currentTime - previousTime);
  output = kp * error + ki * integral + kd * derivative;
  output = (output > 450) ? 450 : (output < -450) ? -450 : output ;
  previousError = error;
  previousTime = currentTime;
  // Serial.println(output);
  // constrain(output, -360, 360);
  output = map((int)output, -450, 450, -50, 50);
}
