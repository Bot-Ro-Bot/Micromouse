#include "MadgwickAHRS.h"
#include<Wire.h>


#define MPU_Address 0x68

int16_t acX, acY, acZ , gyX, gyY, gyZ, mgX, mgY, mgZ;
int16_t temperature;
float Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, netAcc;
uint8_t T;
float gxOffset, gyOffset, gzOffset, axOffset, ayOffset, azOffset, mxOffset, myOffset, mzOffset ;
float gyroRoll, gyroPitch, gyroYaw;
float accRoll, accPitch;
float roll, pitch, yaw, heading;


/*Magnetometer bata etikai data lina mildainda raixa kina bhane tyo chip bhitra ni magnetometer lai xuttai device jasto  raixa
  tesaile AK8975 bhanne euta xuttai magnetometer raixa bhitra jasko address 0x0C raixa tara yo herna ni etikai na paune raixa
  tyo herna lai BYPASS_EN bit in register 0x37 lai 1 lekhyo bhane matra mag lai access garna milne raixa
  Datasheet ko page no. 23 maa xa auxiliary i2c mode ko master mode maa...
  I2C Master Mode: Allows the MPU-9250 to directly access the data registers of external digital
  sensors, such as a magnetometer. In this mode, the MPU-9250 directly obtains data from auxiliary
  sensors without intervention from the system applications processor.
  For example, In I2C Master mode, the MPU-9250 can be configured to perform burst reads, returning
  the following data from a magnetometer:
   X magnetometer data (2 bytes)
   Y magnetometer data (2 bytes)
   Z magnetometer data (2 bytes)
  The I2C Master can be configured to read up to 24 bytes from u
*/

void setup() {
  Wire.begin();
  Serial.begin(115200);
  checkDevice();
  mpuSetup();
  // calibrateMPU();
}

void loop() {

  readData();
  calculations();
  Serial.print("  Roll");   Serial.print(roll);
  Serial.print("  Pitch");   Serial.print(pitch);
  Serial.print("  Yaw");   Serial.println(yaw);

  //  Serial.print("  Ax:");   Serial.print(acX);
  //  Serial.print("  Ay:");   Serial.print(acY);
  //  Serial.print("  Az:");   Serial.print(acZ);
  //  Serial.print("  Gx:");   Serial.print(gyX);
  //  Serial.print("  Gy:");   Serial.print(gyY);
  //  Serial.print("  Gz:");   Serial.print(gyZ);

  //Serial.println(heading*100);

  //  Serial.print("  Mx:");   Serial.print(mgX);
  //  Serial.print("  My:");   Serial.print(mgY);
  //  Serial.print("  Mz:");   Serial.println(mgZ);
  //  Serial.print("  T:");    Serial.println(temperature);

  //  Serial.print("  Ax:");   Serial.print(Ax);
  //  Serial.print("  Ay:");   Serial.print(Ay);
  //  Serial.print("  Az:");   Serial.print(Az);
  //  Serial.print("  Gx:");   Serial.print(Gx);
  //  Serial.print("  Gy:");   Serial.print(Gy);
  //  Serial.print("  Gz:");   Serial.print(Gz);
  //  Serial.print("  T:");    Serial.println(T);
}


void mpuSetup() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x00);                 //Disable sleep mode                                         Put bit 3 to 1 if you want to disable temperature sensor
  Wire.endTransmission();

  ////filter at 5hz??
  //  Wire.beginTransmission(MPU_Address);
  //  Wire.write(0x1A);
  //  Wire.write(0x06);
  //  Wire.endTransmission();
  //
  ////filter at 5hz??
  //  Wire.beginTransmission(MPU_Address);
  //  Wire.write(0x1D);
  //  Wire.write(0x06);
  //  Wire.endTransmission();


  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1B);
  Wire.write(0x08);                         //Select degrees per second for gyro to read data accordingly   +-500dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1C);
  Wire.write(0x10);                        ////Select range for accl to read data accordingly              +-8g
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

  //ACCELEROMETER CALIBRATION
  Serial.println("Calibrating Accelerometer...");
  delay(500);
  for (int i = 0; i < 2000; i++) {
    readData();
    axOffset = axOffset + acX;
    ayOffset = ayOffset + acY;
    azOffset = azOffset + acZ;
    delay(5);
    if (i % 200 == 0) Serial.println("...");
  }
  axOffset = axOffset / 2000;
  ayOffset = ayOffset / 2000;
  azOffset = azOffset / 2000;
  Serial.println("Accelerometer Calibrated.");


  //MAGNETOMETER CALIBRATION
magCalibration();


}

void magcalMPU9250(float * dest1, float * dest2) {
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

  Serial.println(“Mag Calibration: Wave device in a figure eight until done!”);
  sample_count = 128;

  for (ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] – mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] – mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] – mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);
  Serial.println(“Mag Calibration done!”);
}

void readData() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_Address, 14);
  while (Wire.available() < 14);
  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 7);
  mgX = Wire.read() << 8 | Wire.read();
  mgY = Wire.read() << 8 | Wire.read();
  mgZ = Wire.read() << 8 | Wire.read();
  // heading = atan2(mgY, mgX);
}

void calculations() {
  gyX -= gxOffset;
  gyY -= gyOffset;
  gyZ -= gzOffset;

  acX -= axOffset;
  acY -= ayOffset;
  acZ -= azOffset;

  mgX -= mxOffset;
  mgY -= myOffset;
  mgZ -= mzOffset;

  Ax = acX / 4096;
  Ay = acY / 4096;
  Az = acZ / 4096;
  netAcc = sqrt(Ax * Ax + Ay * Ay + Az * Az);

  Gx = gyX / 65.5;
  Gy = gyY / 65.5;
  Gz = gyZ / 65.5;

  Gx = Gx * 0.0174;
  Gy = Gy * 0.0174;
  Gz = Gz * 0.0174;

  Mx = (float)mgX;
  My = (float)mgY;
  Mz = (float)mgZ;


  T = temperature / 340 + 36.53;

  MadgwickAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);


  roll = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  pitch = asin(-2.0 * (q1 * q3 - q0 * q2));
  yaw = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

  yaw *= 57.29;
  pitch *= 57.29;
  roll *= 57.29;
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

  delay(5000);           // wait 5 seconds for next scan*/
}
