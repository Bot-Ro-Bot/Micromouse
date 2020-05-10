#include "VL53L0X.h"
#include<Wire.h>
#define frameSize 2

VL53L0X sensor;

uint16_t distance;
uint16_t sensorData[frameSize];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
}

void loop() {
  //  distance = sensor.readRangeSingleMillimeters();

//  Serial.print(sensor.readRangeContinuousMillimeters());
//  if (sensor.timeoutOccurred()) {
//    Serial.print(" TIMEOUT");
//  }

  Serial.println();

  //rabin sir ko aafnai filter haha
  //distance = ( 0.96f * distance + 0.96f * distance) / 20 ;
  //Serial.println(distance); Serial.print("\t \t");
  //
  //  for (int8_t i = 0; i < frameSize; i++) {
  //    sensorData[i + 1] = sensorData[i];
  //  }
  //
  //  sensorData[0] = distance;
  //
  //  for (int8_t i = 0; i < frameSize; i++) {
  //    distance += sensorData[i];
  //  }
  //
  //   distance = distance / frameSize;
  //
  //  Serial.println(distance);
}
