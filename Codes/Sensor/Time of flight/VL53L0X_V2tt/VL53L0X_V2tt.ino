#include<Wire.h>
#include "VL53L0X.h"

#define MPU_Address 0x68
//user defined addresses.....j haalda ni hunxa tara 0x7F samma
#define frontAddress 0x35
#define rightAddress 0x36
#define leftAddress 0x37

#define xshutLeft PB4
#define xshutRight PB3
#define xshutFront PB5
//digitalpins arduino ko laagi test handa ..connected to respective xshut pins

#define LED_PIN PB12

//VL53L0X sensorFront, sensorRight, sensorLeft;

VL53L0X sensor[3];

uint16_t distance[3];

uint16_t distanceOld[3];

#define frameSize 10
uint16_t rollAverage[3][frameSize];
HardwareTimer timer(2);


int16_t gyX, gyY, gyZ, mgX, mgY, mgZ;
float Gx, Gy, Gz, Mx, My, Mz;
float gxOffset, gyOffset, gzOffset;
float gyroYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float magYaw = 0;
float magNormal;
float magInitialYaw = 0;
bool magInitial = false;
long unsigned int loopTime = 0;
bool ignoreMPU = false;

uint16_t magData[frameSize];
uint16_t gyroData[frameSize];

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
  // readData();
  //  interrupts();
  calculations();
  //  distance[0] = sensorFront.readRangeSingleMillimeters();
  //  distance[1] = sensorRight.readRangeSingleMillimeters();
  //  distance[2] = sensorLeft.readRangeSingleMillimeters();
  // Serial.println(".");  Serial.println(".");  Serial.println(".");  Serial.println(".");  Serial.println(".");
}



/* I am looking into this, but I want to point out right away that you
  shouldn't drive the XSHUT pins high (they are not 5V tolerant). Instead,
  you can set the Arduino pin back to an input (pinMode(4, INPUT);)
  to allow the sensor board to pull XSHUT up to 2.8 V.
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them

  https://forum.pololu.com/t/vl53l0x-maximum-sensors-on-i2c-arduino-bus/10845/7?u=oleglyan
  https://forum.pololu.com/t/vl53l0x-address-programming/11395
  https://github.com/pololu/vl53l0x-arduino/issues/1
*/

void setup() {
  delay(3000);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  pinMode(LED_PIN, OUTPUT);
  pinMode(xshutFront, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  checkDevice();
  setId();

  //  Serial.println(sensorRight.getAddress(), HEX);
  //  Serial.println(sensorFront.getAddress(), HEX);
  //  Serial.println(sensorLeft.getAddress(), HEX);

  //  Serial.println( sensorFront.init());
  //  Serial.println("abc");
  //  Serial.println( sensorLeft.init());
  //  Serial.println("abc");
  //  Serial.println( sensorRight.init());
  //  Serial.println("abc");

  //  sensorFront.setTimeout(400);
  //  sensorRight.setTimeout(400);
  //  sensorLeft.setTimeout(400);

  Serial.println( sensor[0].init());
  Serial.println("abc");
  Serial.println( sensor[1].init());
  Serial.println("abc");
  Serial.println( sensor[2].init());
  Serial.println("abc");

  sensor[0].setTimeout(400);
  sensor[1].setTimeout(400);
  sensor[2].setTimeout(400);

  //  sensorFront.setMeasurementTimingBudget(20000);
  //  sensorLeft.setMeasurementTimingBudget(20000);
  //  sensorRight.setMeasurementTimingBudget(20000);
  //
  //
  ////single maa data ramro aako xa tara code chalda chaldai adkiraako xa...data type ko kei problem jasto dekhinxa...tara continous maa ramro sanga chali raako xa tei kura...api
  sensor[0].startContinuous();
  sensor[2].startContinuous();
  sensor[1].startContinuous();
  mpuSetup();
  calibrateMPU();
  setupTimer();
  Serial.println(sensor[0].getAddress(), HEX);
  Serial.println(sensor[2].getAddress(), HEX);
  Serial.println(sensor[1].getAddress(), HEX);


}


void loop() {

  // calculations();
  //Serial.println(Wire.endTransmission());
  //  calculations();

  //  distance[0] = sensorFront.readRangeContinuousMillimeters();
  //  Serial.println(sensorFront.last_status);
  //  distance[1] = sensorRight.readRangeContinuousMillimeters();
  //  Serial.println(sensorRight.last_status);
  //  distance[2] = sensorLeft.readRangeContinuousMillimeters();
  //  Serial.println(sensorLeft.last_status);

  distance[0] = sensor[0].readRangeContinuousMillimeters();
  Serial.println(sensor[0].last_status);
  distance[1] = sensor[2].readRangeContinuousMillimeters();
  Serial.println(sensor[2].last_status);
  distance[2] = sensor[1].readRangeContinuousMillimeters();
  Serial.println(sensor[1].last_status);

  distance[0] = 0.97 * distance[0] + 0.03 * distanceOld[0];
  distance[1] = 0.97 * distance[1] + 0.03 * distanceOld[1];
  distance[2] = 0.97 * distance[2] + 0.03 * distanceOld[2];

  //  distance[0] = rollingAverage(distance[0], 0);
  //  distance[1] = rollingAverage(distance[1], 1);
  //  distance[2] = rollingAverage(distance[2], 2);

  distance[0] = ( 0.96f * distance[0] + 0.96f * distance[0]) / 20 ;
  distance[1] = ( 0.96f * distance[1] + 0.96f * distance[1]) / 20 ;
  distance[2] = ( 0.96f * distance[2] + 0.96f * distance[2]) / 20 ;

  distanceOld[0] = distance[0];
  distanceOld[1] = distance[1];
  distanceOld[2] = distance[2];


  if (ignoreMPU) {

  }
  else {
    //    distance[0] = sensorFront.readRangeSingleMillimeters();
    //    distance[1] = sensorRight.readRangeSingleMillimeters();
    //    distance[2] = sensorLeft.readRangeSingleMillimeters();


  }

  Serial.print(distance[0]);  Serial.print("\t");
  Serial.print(distance[1]);  Serial.print("\t"); //euta sensor maa bigreko ho ki dhulo paseko xa tara 80 mm value badi dirako xa ...off set milauna matra lekheko aile
  Serial.println(distance[2]);  //Serial.print("\t");
  //  Serial.print("\t"); Serial.print("\t");  Serial.print("\t");   Serial.print("\t");
  //  Serial.print(rollingAverage(distance[0], 0));  Serial.print("\t");
  //  Serial.print(rollingAverage(distance[1]-80, 1));  Serial.print("\t");
  //  Serial.println(rollingAverage(distance[2], 2));
  Serial.print("  Yaw from filter:"  );   Serial.print((int)yaw);  Serial.print("  Yaw from mag:"); Serial.print(magYaw);  Serial.print("  Yaw from gyro:");  Serial.println(gyroYaw);

  ignoreMPU ^= 1;
  digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
  //  checkDevice();

}

void setId() {
  //  //all sensors in reset/(OFF)
  //  digitalWrite(xshutFront, LOW);
  //  digitalWrite(xshutRight, LOW);
  //  digitalWrite(xshutLeft, LOW);
  //  delay(10);
  //
  //  // all sensors is ON
  //  digitalWrite(xshutFront, HIGH);
  //  digitalWrite(xshutRight, HIGH);
  //  digitalWrite(xshutLeft, HIGH);
  //  delay(10);
  //
  //  // keeping front sensor ON while turning OFF sensors on the LEFT and RIGHT
  //  digitalWrite(xshutFront, HIGH);
  //  digitalWrite(xshutRight, LOW);
  //  digitalWrite(xshutLeft, LOW);
  //  sensorFront.setAddress(frontAddress); //setting the address (API mai bhako function raixa yo)
  //  delay(10);
  //  Serial.println("front done");    //debugging purpose ko laagi print matra haaneyko
  //
  //  // keeping front sensor and Left sensor ON while turning OFF sensor on the RIGHT
  //  digitalWrite(xshutLeft, HIGH);
  //  digitalWrite(xshutRight, LOW);
  //  delay(10);
  //  sensorLeft.setAddress(leftAddress);
  //  delay(10);
  //  Serial.println("left done");
  //
  //  digitalWrite(xshutRight, HIGH);
  //  delay(10);
  //  sensorRight.setAddress(rightAddress);
  //  delay(10);
  //  Serial.println("right done");

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
  sensor[0].setAddress(frontAddress); //setting the address (API mai bhako function raixa yo)
  delay(10);
  Serial.println("front done");    //debugging purpose ko laagi print matra haaneyko

  // keeping front sensor and Left sensor ON while turning OFF sensor on the RIGHT
  digitalWrite(xshutLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  delay(10);
  sensor[1].setAddress(leftAddress);
  delay(10);
  Serial.println("left done");

  digitalWrite(xshutRight, HIGH);
  delay(10);
  sensor[2].setAddress(rightAddress);
  delay(10);
  Serial.println("right done");
}


uint16_t rollingAverage(uint16_t newData, bool sensorNo) {

  for (int8_t i = 0; i < frameSize; i++) {
    rollAverage[sensorNo][i + 1] = rollAverage[sensorNo][i];
  }

  rollAverage[sensorNo][0] = newData;

  for (int8_t i = 0; i < frameSize; i++) {
    newData += rollAverage[sensorNo][i];
  }
  return uint16_t(newData / frameSize);

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
  delay(100);                               //Ignore data for half of a second for better accuracy

  for (int32_t i = 0; i < 1000; i++) {
    readData();
    gzOffset = gzOffset + gyZ;
    delay(2);
    if (i % 200 == 0) Serial.println("...");
  }
  gzOffset = gzOffset / 1000;
  Serial.println("Gyroscope Calibrated.");
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
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x47);
  Wire.endTransmission();
  Wire.requestFrom(MPU_Address, 2);
  gyZ = Wire.read() << 8 | Wire.read();

  //  Serial.println("reeaeding mag");
  Wire.beginTransmission(0x0C);     //Reading data from magnetometer measurement registers
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x0C, 7); //7 kina leko maathi bhaneko xa
  mgX = Wire.read() << 8 | Wire.read();
  mgY = Wire.read() << 8 | Wire.read();
  mgZ = Wire.read() << 8 | Wire.read();
  uint8_t ST2 = Wire.read();

}


void calculations() {
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

  gyroYaw = gyroYaw + Gz * (micros() - loopTime) / 1000000 ;

  //  if (gyroYaw < 0) {
  //    gyroYaw = gyroYaw + 360;
  //  }

  //tilt compensate bhayeko chai
  // magYaw = atan2((-My * cos(roll) + Mz * sin(roll)), Mx * cos(pitch) + My * sin(pitch) * sin(roll) + Mz * sin(pitch) * cos(roll));

  // magYaw = atan(My / Mx) * 57.29f -magInitialYaw;
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

  // magYaw = magYaw * 57.29f;

  //  if (magYaw < 0) {
  //    magYaw = magYaw + 360;
  //  }

  yaw = gyroYaw * 0.97f + magYaw * 0.03f;

  //kathamdu ko magnetic declination 0 degrees 35 minutes ....not so significant for yaw calculations
  // yaw = yaw - 0.56f;
  loopTime = micros();
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
