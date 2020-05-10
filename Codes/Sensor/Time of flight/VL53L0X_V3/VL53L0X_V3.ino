#include "VL53L0X.h"
#include <Wire.h>

#define frontAddress  0x30//default address
#define rightAddress  0x31
#define leftAddress   0x32

#define xshutLeft     PB4
#define xshutRight    PB3
#define xshutFront    PB5

VL53L0X sensor;

VL53L0X *sensorFront = new VL53L0X();
VL53L0X *sensorRight = new VL53L0X();
VL53L0X *sensorLeft = new VL53L0X();

//VL53L0X sensor,sensorFront,sensorLeft,sensorRight;

uint32_t distance[3];

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  disableDebugPorts();
  pinMode(xshutFront, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  pinMode(xshutRight, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  checkDevice();

  //power up the right sensor and change its address
  pinMode(xshutRight, INPUT);
  delay(10);
  sensorRight->setAddress(rightAddress);
  Serial.println("Eta1 ");

  pinMode(xshutLeft, INPUT);
  delay(10);
  sensorLeft->setAddress(leftAddress);
  Serial.println("Eta2 ");

  pinMode(xshutFront, INPUT);
  delay(10);
  sensorFront->setAddress(frontAddress);
  Serial.println("Eta  front");

  Serial.println(sensorRight->getAddress(), HEX);
  Serial.println(sensorFront->getAddress(), HEX);
  Serial.println(sensorLeft->getAddress(), HEX);


  Serial.println(sensorRight->init());
  delay(20);
  sensorLeft->setTimeout(500);

  Serial.println(sensorLeft->init());
  delay(20);
  sensorLeft->setTimeout(500);
  Serial.println("Eta 5");

  Serial.println(sensorFront->init());
  Serial.println("Eta4 ");
  delay(20);
  sensorLeft->setTimeout(500);

  /*if (!sensorLeft.init()) {
    Serial.println("Left sensor error");
    while (1);
    }

    if (!sensorRight.init()) {
    Serial.println("Right sensor error");
    while (1);
    }

    if (!sensorFront.init()) {
    Serial.println("Front sensor error");
    while (1);
    }
  */

  // Serial.println(sensorLeft->init(true));
  //Serial.println("Eta3");


  //
  //  Serial.println("Eta 6");


  //checkDevice();
}

void loop() {
  // Serial.println(sensor.readRangeContinuousMillimeters());
  //  Serial.println("....");
  //  distance[0] = sensorFront->readRangeContinuousMillimeters();
  //  distance[1] = sensorRight->readRangeContinuousMillimeters();
  //  distance[2] = sensorLeft->readRangeContinuousMillimeters();
  //  Serial.print(distance[0]);  Serial.print("\t");
  //  Serial.print(distance[1]);  Serial.print("\t");
  //  Serial.println(distance[2]);  //Serial.print("\t");
}

void setId() {

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

void setID() {

}
