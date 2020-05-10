#include<Wire.h>
#include "VL53L0X.h"

//user defined addresses.....j haalda ni hunxa tara 0x7F samma
#define frontAddressLeft 0x30
#define frontAddressRight 0x31
#define rightAddress 0x32
#define leftAddress 0x33

#define xshutLeft PB4
#define xshutRight PB8
#define xshutFrontRight PB5
#define xshutFrontLeft PB9


#define frontRight 0
#define frontLeft 1
#define right 2
#define left 3

VL53L0X sensor[4];

uint16_t distance[4];

uint16_t distanceOld[4];


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
  // delay(5000);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  pinMode(xshutFrontRight, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  pinMode(xshutFrontLeft, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  checkDevice();
  setId();

  Serial.println(sensor[frontRight].getAddress(), HEX);
  Serial.println(sensor[frontLeft].getAddress(), HEX);
  Serial.println(sensor[right].getAddress(), HEX);
  Serial.println(sensor[left].getAddress(), HEX);

  sensor[frontRight].init();
  Serial.println("abc");
  sensor[frontLeft].init();
  Serial.println("abc");
  sensor[left].init();
  Serial.println("abc");
  sensor[right].init();
  Serial.println("abc");

  sensor[frontRight].setTimeout(400);
  sensor[frontLeft].setTimeout(400);
  sensor[left].setTimeout(400);
  sensor[right].setTimeout(400);
  //
  //
  ////single maa data ramro aako xa tara code chalda chaldai adkiraako xa...data type ko kei problem jasto dekhinxa...tara continous maa ramro sanga chali raako xa tei kura...api
  sensor[frontRight].startContinuous();
  sensor[frontLeft].startContinuous();
  sensor[right].startContinuous();
  sensor[left].startContinuous();
}


void loop() {
  //checkDevice();
  // Serial.println("abc");
  distance[0] = sensor[frontRight].readRangeContinuousMillimeters();
  distance[1] = sensor[right].readRangeContinuousMillimeters();
  distance[2] = sensor[left].readRangeContinuousMillimeters();
  distance[3] = sensor[frontLeft].readRangeContinuousMillimeters();


  distance[0] = 0.97 * distance[0] + 0.03 * distanceOld[0];
  distance[1] = 0.97 * distance[1] + 0.03 * distanceOld[1];
  distance[2] = 0.97 * distance[2] + 0.03 * distanceOld[2];
  distance[3] = 0.97 * distance[3] + 0.03 * distanceOld[3];


  distanceOld[0] = distance[0];
  distanceOld[1] = distance[1];
  distanceOld[2] = distance[2];
  distanceOld[4] = distance[4];


}

void setId() {
  //all sensors in reset/(OFF)
  digitalWrite(xshutFrontRight, LOW);
  digitalWrite(xshutFrontLeft, LOW);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  delay(10);

  // all sensors is ON
  digitalWrite(xshutFrontRight, HIGH);
  digitalWrite(xshutFrontLeft, HIGH);
  digitalWrite(xshutRight, HIGH);
  digitalWrite(xshutLeft, HIGH);

  delay(10);

  // keeping front sensor ON while turning OFF sensors on the LEFT and RIGHT
  digitalWrite(xshutFrontRight, HIGH);
  digitalWrite(xshutFrontLeft, LOW);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  sensor[frontRight].setAddress(frontAddressRight); //setting the address (API mai bhako function raixa yo)
  delay(10);
  Serial.println("front right done");    //debugging purpose ko laagi print matra haaneyko

  digitalWrite(xshutFrontLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  sensor[frontLeft].setAddress(frontAddressLeft); //setting the address (API mai bhako function raixa yo)
  delay(10);
  Serial.println("front left done");    //debugging purpose ko laagi print matra haaneyko


  // keeping front sensor and Left sensor ON while turning OFF sensor on the RIGHT
  digitalWrite(xshutLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  delay(10);
  sensor[left].setAddress(leftAddress);
  delay(10);
  Serial.println("left done");

  digitalWrite(xshutRight, HIGH);
  delay(10);
  sensor[right].setAddress(rightAddress);
  delay(10);
  Serial.println("right done");
}

void printDistance() {
  Serial.print(distance[0]);  Serial.print("\t");
  Serial.print(distance[1]);  Serial.print("\t"); //euta sensor maa bigreko ho ki dhulo paseko xa tara 80 mm value badi dirako xa ...off set milauna matra lekheko aile
  Serial.print(distance[2]);  Serial.print("\t");
  Serial.println(distance[3]);

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
