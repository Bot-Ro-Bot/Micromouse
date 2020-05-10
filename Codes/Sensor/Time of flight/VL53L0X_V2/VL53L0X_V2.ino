#include<Wire.h>
#include "VL53L0X.h"

//user defined addresses.....j haalda ni hunxa tara 0x7F samma
#define frontAddress 0x30
#define rightAddress 0x31
#define leftAddress 0x32

#define xshutLeft PB4
#define xshutRight PB8
#define xshutFront PB5
//digitalpins arduino ko laagi test handa ..connected to respective xshut pins


#define frameSize 10

VL53L0X sensorFront, sensorRight, sensorLeft;
uint16_t distance[3];

uint16_t distanceOld[3];

uint16_t rollAverage[3][frameSize];


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
  delay(2000);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  pinMode(xshutFront, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  checkDevice();
  setId();

  Serial.println(sensorRight.getAddress(), HEX);
  Serial.println(sensorFront.getAddress(), HEX);
  Serial.println(sensorLeft.getAddress(), HEX);

  sensorFront.init();
  Serial.println("abc");
  sensorLeft.init();
  Serial.println("abc");
  sensorRight.init();
  Serial.println("abc");

  sensorFront.setTimeout(400);
  sensorLeft.setTimeout(400);
  sensorRight.setTimeout(400);
  //
  //
  ////single maa data ramro aako xa tara code chalda chaldai adkiraako xa...data type ko kei problem jasto dekhinxa...tara continous maa ramro sanga chali raako xa tei kura...api
  sensorFront.startContinuous();
  sensorRight.startContinuous();
  sensorLeft.startContinuous();
}


void loop() {
  //checkDevice();
  // Serial.println("abc");
  distance[0] = sensorFront.readRangeContinuousMillimeters();
  distance[1] = sensorRight.readRangeContinuousMillimeters();
  distance[2] = sensorLeft.readRangeContinuousMillimeters();

  distance[0] = 0.97 * distance[0] + 0.03 * distanceOld[0];
  distance[1] = 0.97 * distance[1] + 0.03 * distanceOld[1];
  distance[2] = 0.97 * distance[2] + 0.03 * distanceOld[2];

  //  distance[0] = rollingAverage(distance[0], 0);
  //  distance[1] = rollingAverage(distance[1], 1);
  //  distance[2] = rollingAverage(distance[2], 2);

  distance[0] = ( 0.96f * distance[0] + 0.96f * distance[0]) / 20 ;
  distance[1] = ( 0.96f * distance[1] + 0.96f * distance[1]) / 20 ;
  distance[2] = ( 0.96f * distance[2] + 0.96f * distance[2]) / 20 ;
  Serial.print(distance[0]);  Serial.print("\t");
  Serial.print(distance[1]);  Serial.print("\t"); //euta sensor maa bigreko ho ki dhulo paseko xa tara 80 mm value badi dirako xa ...off set milauna matra lekheko aile
  Serial.println(distance[2]);  //Serial.print("\t");
  //  Serial.print("\t"); Serial.print("\t");  Serial.print("\t");   Serial.print("\t");
  //  Serial.print(rollingAverage(distance[0], 0));  Serial.print("\t");
  //  Serial.print(rollingAverage(distance[1]-80, 1));  Serial.print("\t");
  //  Serial.println(rollingAverage(distance[2], 2));

  distanceOld[0] = distance[0];
  distanceOld[1] = distance[1];
  distanceOld[2] = distance[2];
  //  checkDevice();

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

uint16_t distanceFilter() {

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
