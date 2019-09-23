#include "VL53L0X.h"
#include<Wire.h>
VL53L0X sensor,sensorLeft,sensorRight;

uint8_t distance;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  //high accuracy mode
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);


}

void loop() {
 distance=sensor.readRangeSingleMillimeters();
 
}
