#include<Wire.h>
#define MPU_Address 0x68

int16_t gyX, gyY, gyZ, mgX, mgY, mgZ;
float Gx, Gy, Gz, Mx, My, Mz;
float gxOffset, gyOffset, gzOffset;
float gyroYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float magYaw = 0;
float magNormal;
float magInitialYaw = 0;
uint32_t loopTime = 0;

void setup() {
  delay(3000);
  Serial.begin(115200);
  Wire.begin();
  checkDevice();
  mpuSetup();
  calibrateMPU();
  loopTime = micros();
}

void loop() {
  calculations();
  Serial.print("        Yaw from filter:");  Serial.println(yaw);
  while ((micros() - loopTime) < 4000);
  loopTime = micros();
}


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
  //GYRO CALIBRATION
  Serial.println("Calibrating gyro...");
  delay(500);                               //Ignore data for half of a second for better accuracy

  for (int32_t i = 0; i < 2000; i++) {
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
  Wire.requestFrom(0x0C, 6);
  mgX = Wire.read() << 8 | Wire.read();
  mgY = Wire.read() << 8 | Wire.read();
  mgZ = Wire.read() << 8 | Wire.read();
}

void calculations() {
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


  //tilt compensate bhayeko chai
  magYaw = atan2((-My * cos(roll) + Mz * sin(roll)), Mx * cos(pitch) + My * sin(pitch) * sin(roll) + Mz * sin(pitch) * cos(roll));
  magYaw = magYaw * 57.29f - magInitialYaw;

  yaw = gyroYaw * 0.97 + magYaw * 0.03;

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

  delay(1000);           // wait 5 seconds for next scan*/
}
