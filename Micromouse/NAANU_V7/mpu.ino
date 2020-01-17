#include<Wire.h>

#define MPU_Address 0x68

#define frameSize 10

uint16_t magData[frameSize];

void initializeMPU() {
  mpuSetup();
  calibrateMPU();

}

void resetMPU() {
  //  gyZ = mgX = mgY = mgZ = 0;
  //  Gz = Mx = My = Mz = 0;
  gzOffset = 0;

  magYaw = 0;
  magNormal = 0;
  magInitialYaw = 0;
  initializeMPU();
  gyroYaw = 0;
  yaw = 0;
  loopTime = micros();
}


void mpuSetup() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x00);                 //Disable sleep mode                                         Put bit 3 to 1 if you want to disable temperature sensor
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1B);
  Wire.write(0b00011000);                         //Select degrees per second for gyro to read data accordingly   +-2000dps
  Wire.endTransmission();

  //  Wire.beginTransmission(MPU_Address);
  //  Wire.write(0x1C);
  //  Wire.write(0b00011000);                        ////Select range for accl to read data accordingly              +-8g
  //  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x37);
  Wire.write(0x02);                           //enable slave device 1 (magnetometer) inside the sensor itself
  Wire.endTransmission();

  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
  Wire.endTransmission();
  //  Serial.println("Setup done");
}


void calibrateMPU() {
  //GYRO CALIBRATION
  //  Serial.println("Calibrating gyro...");

  for (int32_t i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU_Address);
    Wire.write(0x47);
    Wire.endTransmission();
    Wire.requestFrom(MPU_Address, 2);
    gyZ = Wire.read() << 8 | Wire.read();
    gzOffset = gzOffset + gyZ;
    delay(5);
    //    if (i % 20 == 0) Serial.println("..."); //remove this line in final code
  }
  gzOffset = (gzOffset / 1000);

  readData();
  Mx = mgX * 0.15 * 1.19f;
  My = mgY * 0.15 * 1.20f;
  Mz = mgZ * 0.15 * 1.15f;

  // normalize magnetometer data
  magNormal = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx = Mx / magNormal;
  My = My / magNormal;
  Mz = Mz / magNormal;

  magInitialYaw = atan2(My , Mx) * 57.29f;

}


void readData() {
  //  Serial.println("reading");
  //  delay(400);
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x47);
  Wire.endTransmission();
  Wire.requestFrom(MPU_Address, 2);
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


int calculations() {
  readData();
  gyZ -= gzOffset;

  //in degrees per second
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

  if (digitalRead(lMotorForward) || digitalRead(rMotorForward) || digitalRead(lMotorBack) || digitalRead(rMotorBack) ) {

  }
  gyroYaw = gyroYaw + Gz * (micros() - loopTime) / 1000000 ;

  //  if (gyroYaw < -181) {
  //    gyroYaw = gyr_RotationoYaw + 360;
  //  } else if (gyroYaw > 181) {
  //    gyroYaw = gyroYaw - 360;
  //  }


  magYaw = atan2(My , Mx) * 57.29f - magInitialYaw;

  ////////////////////////rolling avg//////////////////////////////////////////////////
  for (int8_t i = 0; i < frameSize; i++) {
    magData[i + 1] = magData[i];
  }

  magData[0] = magYaw;

  for (int8_t i = 0; i < frameSize; i++) {
    magYaw += magData[i];
  }

  magYaw = magYaw / frameSize;

  ////////////////////////////////////////////////////////////////////////////////////

  yaw = gyroYaw * 0.959f + magYaw * 0.041f ;

  loopTime = micros();
  return (int)yaw;
}

void printData() {
  Serial.print("  Yaw from filter:"  );   Serial.print((int)yaw);  Serial.print("  Yaw from mag:"); Serial.print(magYaw);  Serial.print("  Yaw from gyro:");  Serial.println(gyroYaw);

}
