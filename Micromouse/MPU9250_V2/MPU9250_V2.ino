#include "MadgwickAHRS.h"
#include<Wire.h>


#define MPU_Address 0x68
#define Mag_Address 0x0C
#define MagScale 0.15 //milliGauss per LSB 16bit ko laagi
#define Mmode 0x06  //Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR 
#define LSB_A 8.0/32768.0 //8g ko laagi
#define LSB_G 2000.0/32768.0 //for 2000 degrees per seconds
#define LSB_M 10.0*4912.0/32760.0 // Proper scale to return milliGauss
#define PI 3.141592654

int16_t acX, acY, acZ , gyX, gyY, gyZ, mgX, mgY, mgZ;
float magCalibrationData[3] = {0, 0, 0};
float gyroOffset[3] = {0, 0, 0}, accelOffset[3] = {0, 0, 0}, magOffset[3] = {0, 0, 0};
float gxOffset, gyOffset, gzOffset, axOffset, ayOffset, azOffset, mxOffset, myOffset, mzOffset ;


int16_t temperature;

float Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz;
uint8_t T;
float gyroRoll, gyroPitch, gyroYaw;
float accRoll, accPitch;
float roll, pitch, yaw, heading;

float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


bool ignoreMagData = false;



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
  The I2C Master can be configured to read up to 24 bytes from ...

  Mag maa 7 ota bit kina bhanda kheri .... // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
*/

void resetMPU();
void setupMPU();
void calibrateMPU();
void magCalibration();
void readData(bool flag = 1);
void calculations();
void checkDevice();
void readMPU(byte address, byte regAddress, byte dataNo);
void writeByteMPU(byte address, byte regAddress, byte data);

void readMPU(byte address, byte regAddress, byte dataNo) {
  byte dataArray[dataNo];
  byte i = 0;
  Wire.beginTransmission(address);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(address, dataNo);
  while (Wire.available() < dataNo) {
    dataArray[i] = Wire.read();
    i++;
  }

}

void writeByteMPU(byte address, byte regAddress, byte data) {
  Wire.beginTransmission(address);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();

}



void setup() {
  Wire.begin();
  Serial.begin(115200);
  setupMPU();
  checkDevice();
  calibrateMPU();
}

void loop() {
  readData();
  calculations();
  Serial.print("  Roll");   Serial.print(roll);
  Serial.print("  Pitch");   Serial.print(pitch);
  Serial.print("  Yaw");   Serial.println(yaw);
}


//chahiyena bhane paxi hataula
void resetMPU() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(10);      //start huna lai time
}

void setupMPU() {
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x00);                 //Disable sleep mode                                         Put bit 3 to 1 if you want to disable temperature sensor
  Wire.endTransmission();
  delay(100);  //Delay 100 ms for PLL to get established on x-axis gyro

  //clock select
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();

  // Configure Gyro and Accelerometer...filter ra ext_sync ko kura
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  //sample divide rate ko settings
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x19);
  Wire.write(0x02);
  Wire.endTransmission();

  //Select degrees per second for gyro to read data accordingly   +-2000dps
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission();

  //Select range for accl to read data accordingly              +-8g
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //config accelerometer for data rates
  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz //datasheet ko pageno. 15 maa xa
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1D);
  Wire.write(0b00000011);
  Wire.endTransmission();



  //enable slave device 1 (magnetometer) inside the sensor itself
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  //yesmaa doubt xa
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
  Wire.endTransmission();

  Serial.println("MPU Setup Done");


}


void calibrateMPU() {
  //NOTE: Bot should be at rest for this calibration process to work
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
  //actions of gravity lai ettikai xodna parxa hola
  Serial.println("Calibrating Accelerometer...");
  delay(200);
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

void magCalibration() {
  /////////////////////////////////////////////////////////////////////////////////////////
  //company bata aako offset ko calibration
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.beginTransmission(0x0C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 3);
  magCalibrationData[0] = Wire.read();
  magCalibrationData[1] = Wire.read();
  magCalibrationData[2] = Wire.read();

  //data sheet maa xa yo calculation...mag ko sensitivity company mai calibrate bhako ani rom maa store garera pathako
  magCalibrationData[0] = (float) ( magCalibrationData[0] - 128 ) / 256.0f + 1.0f;
  magCalibrationData[1] = (float) ( magCalibrationData[1] - 128 ) / 256.0f + 1.0f;
  magCalibrationData[2] = (float) ( magCalibrationData[2] - 128 ) / 256.0f + 1.0f;

  //  mxOffset = magCalibration[0];
  //  myOffset = magCalibration[1];
  //  mzOffset = magCalibration[2];

  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);

  //yesmaa doubt xa
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x16);                           //Request continuous magnetometer measurements in 16 bits
  Wire.endTransmission();

  ///////////////////////////////////////////////////////////////////////////////////////////

  //environmental calibration mgOffset maa rakhne

  //8 maa ghumaune waala calibration yo chai
  int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF};
  int16_t mag_temp[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  for (byte i = 0; i < 128; i++) {
    readData(0);
    for (byte j = 0; j < 3; j++) {
      if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
      if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
    }
    delay(135);
  }
  // Get hard iron correction
  mxOffset  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  myOffset  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mzOffset  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  mxOffset = (float)mxOffset * LSB_M * magCalibrationData[0] ;
  myOffset = (float)myOffset * LSB_M * magCalibrationData[1] ;
  mzOffset = (float)mzOffset * LSB_M * magCalibrationData[2];

  //  // Get soft iron correction estimate
  //  mag_scale[0]  = (mag_max[0] – mag_min[0]) / 2; // get average x axis max chord length in counts
  //  mag_scale[1]  = (mag_max[1] – mag_min[1]) / 2; // get average y axis max chord length in counts
  //  mag_scale[2]  = (mag_max[2] – mag_min[2]) / 2; // get average z axis max chord length in counts
  //
  //  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  //  avg_rad /= 3.0;
  //  dest2[0] = avg_rad / ((float)mag_scale[0]);
  //  dest2[1] = avg_rad / ((float)mag_scale[1]);
  //  dest2[2] = avg_rad / ((float)mag_scale[2]);
  Serial.println("Mag Calibration done!");
}

void readData(bool flag = 1) {
  if (flag) {
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

  }

  /*mag ko data chai alik ramro sanga lina parxa for khatra accuracy
     Mag maa 7 ota bit kina bhanda kheri .... // x/y/z gyro register data, ST2
     register stored here, must read ST2 at end of data acquisition..
     pahila data ready bit set bhayo ki nai herne ani matra data read garne...na bhaye samma wait garne ...
     ani read garne ani ST2 bit read garey paxi chai overflow bhako xa ki nai check garne...bhako
     raixa bhane tyo data ignore garne...improves accuracy ekdam dherai
  */

  Wire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 1); //7 kina leko maathi bhaneko xa
  uint8_t ST1 = Wire.read();

  if (ST1 & 0x01) {
    Wire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(0x0C, 7); //7 kina leko maathi bhaneko xa
    mgX = Wire.read() << 8 | Wire.read();
    mgY = Wire.read() << 8 | Wire.read();
    mgZ = Wire.read() << 8 | Wire.read();
    uint8_t ST2 = Wire.read();

    if (!(ST2 & 0x08)) {
      ignoreMagData = true;
    }
  }

  /*uint16_t endWait = millis();
    while ( !(ST1 & 0x01) ) {
    //wait ra delay waala code baaki xa
    if ( millis() - endWait) > 1500 { //1.5 second ko laagi matra wait garxa
      ignoreMagData = true;
      break;
    }
    }*/
  // heading = atan2(mgY, mgX);
}



void calculations() {
  gyX -= gxOffset;
  gyY -= gyOffset;
  gyZ -= gzOffset;

  acX -= axOffset;
  acY -= ayOffset;
  // acZ -= azOffset;    //gravity lai na hataune

  Ax = acX * LSB_A / 4096;
  Ay = acY * LSB_A / 4096;
  Az = acZ * LSB_A / 4096;

  Gx = gyX * LSB_G / 32.8 ;
  Gy = gyY * LSB_G / 32.8 ;
  Gz = gyZ * LSB_G / 32.8 ;

  Mx = mgX * LSB_M * magCalibrationData[0] ;
  My = mgY * LSB_M * magCalibrationData[1] ;
  Mz = mgZ * LSB_M * magCalibrationData[2];

  Mx -= mxOffset;
  My -= myOffset;
  Mz -= mzOffset;

  //convert to radian per second from dps
  Gx = Gx * 0.0174;
  Gy = Gy * 0.0174;
  Gz = Gz * 0.0174;



  T = temperature / 333.87f + 21.53f;

  MadgwickAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);


  roll = atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
  yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

  //to degress again
  yaw *= 57.29f;
  pitch *= 57.29f;
  roll *= 57.29f;

  //kathmandu ko magnetic declination
  yaw = yaw - 0.56f;
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
