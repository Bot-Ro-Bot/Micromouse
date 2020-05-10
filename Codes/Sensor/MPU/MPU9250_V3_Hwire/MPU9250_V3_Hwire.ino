
#include <SoftWire.h>
SoftWire SWire(PB6, PB7, SOFT_FAST);

#define MPU_Address 0x68

int16_t acX, acY, acZ , gyX, gyY, gyZ, mgX, mgY, mgZ;
int16_t temperature;
float Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz;
uint8_t T;
float gxOffset, gyOffset, gzOffset, axOffset, ayOffset, azOffset, mxOffset, myOffset, mzOffset ;
float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
//float accRoll, accPitch;
float roll = 0, pitch = 0, yaw = 0, heading = 0;
float loopTime = 0;
float magYaw = 0;
float magNormal;
float magInitialYaw = 0;
float magCalibrationData[3];

void setup() {
//  __HAL_AFIO_REMAP_SWJ_DISABLE();
  disableDebugPorts();
  SWire.begin();
  Serial.begin(9600);
  mpuSetup();
  delay(1000);
  checkDevice();
  // calibrateMPU();
  //Serial.print("Mag Adjustments X::  "); Serial.print(magCalibrationData[0]);
  //Serial.print("Mag Adjustments y::  "); Serial.print(magCalibrationData[1]);
  //Serial.print("Mag Adjustments z::  "); Serial.print(magCalibrationData[2]);
  // Timer1.initialize(400000);
  //Timer1.attachInterrupt(calculations);
  loopTime = micros();
  delay(1000);
}

void loop() {
  // checkDevice();
  calculations();
  //Serial.print("  Yaw from gyro  ");   Serial.print(gyroYaw);
  //Serial.print("        Yaw from mag:");  Serial.println(magYaw);

  Serial.print("        Yaw from filter:");  Serial.println(yaw);
  delay(300);

  while ((micros() - loopTime) < 4000);
  loopTime = micros();
}


void mpuSetup() {
  SWire.beginTransmission(MPU_Address);
  SWire.write(0x6B);
  SWire.write(0x00);                 //Disable sleep mode                                         Put bit 3 to 1 if you want to disable temperature sensor
  SWire.endTransmission();

  SWire.beginTransmission(MPU_Address);
  SWire.write(0x1B);
  SWire.write(0b00011000);                         //Select degrees per second for gyro to read data accordingly   +-500dps
  SWire.endTransmission();

  SWire.beginTransmission(MPU_Address);
  SWire.write(0x1C);
  SWire.write(0b00011000);                        ////Select range for accl to read data accordingly              +-8g
  SWire.endTransmission();

  SWire.beginTransmission(MPU_Address);
  SWire.write(0x37);
  SWire.write(0x02);                           //enable slave device 1 (magnetometer) inside the sensor itself
  SWire.endTransmission();

  SWire.beginTransmission(0x0C);
  SWire.write(0x0A);
  SWire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
  SWire.endTransmission();

  Serial.println("MPU Setup Done");


}


void calibrateMPU() {
  //GYRO CALIBRATION
  Serial.println("Calibrating gyro...");
  delay(500);                               //Ignore data for half of a second for better accuracy
  for (int i = 0; i < 2000; i++) {
    readData();
    gxOffset = gxOffset + gyX;
    gyOffset = gyOffset + gyY;
    gzOffset = gzOffset + gyZ;
    delay(5);
    if (i % 200 == 0) Serial.println("...");
  }
  gxOffset = gxOffset / 2000;
  gyOffset = gyOffset / 2000;
  gzOffset = gzOffset / 2000;
  Serial.println("Gyroscope Calibrated.");


  //MAGNETOMETER CALIBRATION
  //magCalibration();
  /*
    //offset calibration of raw Data
    /////////////////////////////////////////////////////////////////////////////////////////
    //company bata aako offset ko calibration
    //yo code run garera aaako values  x 1.19 y 1.20 z 1.15
    SWire.beginTransmission(0x0C);
    SWire.write(0x0A);
    SWire.write(0x00);
    SWire.endTransmission();

    SWire.beginTransmission(0x0C);
    SWire.write(0x0A);
    SWire.write(0x0F);
    SWire.endTransmission();

    SWire.beginTransmission(0x0C);
    SWire.write(0x10);
    SWire.endTransmission();
    SWire.requestFrom(0x0C, 3);
    magCalibrationData[0] = SWire.read();
    magCalibrationData[1] = SWire.read();
    magCalibrationData[2] = SWire.read();

    //data sheet maa xa yo calculation...mag ko sensitivity company mai calibrate bhako ani rom maa store garera pathako
    magCalibrationData[0] = (float) ( magCalibrationData[0] - 128 ) / 256.0f + 1.0f;
    magCalibrationData[1] = (float) ( magCalibrationData[1] - 128 ) / 256.0f + 1.0f;
    magCalibrationData[2] = (float) ( magCalibrationData[2] - 128 ) / 256.0f + 1.0f;


    SWire.beginTransmission(0x0C);
    SWire.write(0x0A);
    SWire.write(0x00);
    SWire.endTransmission();

    SWire.beginTransmission(MPU_Address);
    SWire.write(0x6B);
    SWire.write(0x01);
    SWire.endTransmission();
    delay(10);

    //yesmaa doubt xaina aba haha..
    SWire.beginTransmission(0x0C);
    SWire.write(0x0A);
    SWire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
    SWire.endTransmission();
  */

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //hard iron calibration

  readData();
  Mx = mgX * 0.15;
  My = mgY * 0.15;
  Mz = mgZ * 0.15;

  // normalize magnetometer data
  magNormal = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx = Mx / magNormal;
  My = My / magNormal;
  Mz = Mz / magNormal;

  magInitialYaw = atan(My / Mx) * 57.29f;


}


void readData() {
  SWire.beginTransmission(MPU_Address);
  SWire.write(0x3B);
  SWire.endTransmission();
  SWire.requestFrom(MPU_Address, 14);
  while (SWire.available() < 14);
  acX = SWire.read() << 8 | SWire.read();
  acY = SWire.read() << 8 | SWire.read();
  acZ = SWire.read() << 8 | SWire.read();
  temperature = SWire.read() << 8 | SWire.read();
  gyX = SWire.read() << 8 | SWire.read();
  gyY = SWire.read() << 8 | SWire.read();
  gyZ = SWire.read() << 8 | SWire.read();

  SWire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
  SWire.write(0x03);
  SWire.endTransmission();
  SWire.requestFrom(0x0C, 7);
  mgX = SWire.read() << 8 | SWire.read();
  mgY = SWire.read() << 8 | SWire.read();
  mgZ = SWire.read() << 8 | SWire.read();
}

void calculations() {
  readData();
  gyX -= gxOffset;
  gyY -= gyOffset;
  gyZ -= gzOffset;

  //in g
  Ax = acX / 2048.0f;
  Ay = acY / 2048.0f;
  Az = acZ / 2048.0f;

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

  //tilt compensate gareko xaina
  magYaw = atan(My / Mx) * 57.29f - magInitialYaw;

  //tilt compensate bhayeko chai
  //magYaw = atan2((-My * cos(roll) + Mz * sin(roll)), Mx * cos(pitch) + My * sin(pitch) * sin(roll) + Mz * sin(pitch) * cos(roll));
  //  magYaw = magYaw * 57.29f - magInitialYaw;

  T = temperature / 333.87f + 21.53f;

  //complementary filter....gyro ko drift compensate gareko..take large portion of gyro and small portion of mag
  //tyo values chai hit and trial bata garne ho
  yaw = gyroYaw * 0.9864 + magYaw * 0.0136;

  //kathamdu ko magnetic declination 0 degrees 35 minutes ....not so significant for yaw calculations
  // yaw = yaw - 0.56f;
}

void checkDevice() {
  Serial.println("STARTING");

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    SWire.beginTransmission(address);
    error = SWire.endTransmission();

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

  delay(100);           // wait 5 seconds for next scan*/
}

void debug() {
  //Serial.print("  Yaw from gyro  ");   Serial.print(gyroYaw);
  //Serial.print("        Yaw from mag:");  Serial.println(magYaw);

  //Serial.print("        Yaw from filter:");  Serial.println(yaw);


  //  Serial.print("  Ax:");   Serial.print(acX);
  //  Serial.print("  Ay:");   Serial.print(acY);
  //  Serial.print("  Az:");   Serial.print(acZ);
  //  Serial.print("  Gx:");   Serial.print(gyX);
  //  Serial.print("  Gy:");   Serial.print(gyY);
  //  Serial.print("  Gz:");   Serial.println(gyZ);

  //  Serial.print("  Mx:");   Serial.print(mgX);
  //  Serial.print("  My:");   Serial.print(mgY);
  //  Serial.print("  Mz:");   Serial.println(mgZ);

  //  Serial.print("  Mx:");   Serial.print(Mx);
  //  Serial.print("  My:");   Serial.print(My);
  //  Serial.print("  Mz:");   Serial.println(Mz);
  //  Serial.print("  T:");    Serial.println(temperature);

  //  Serial.print("  Ax:");   Serial.print(Ax);
  //  Serial.print("  Ay:");   Serial.print(Ay);
  //  Serial.print("  Az:");   Serial.print(Az);
  //  Serial.print("  Gx:");   Serial.print(Gx);
  //  Serial.print("  Gy:");   Serial.print(Gy);
  //  Serial.print("  Gz:");   Serial.print(Gz);
  //  Serial.print("  T:");    Serial.println(T);
}
