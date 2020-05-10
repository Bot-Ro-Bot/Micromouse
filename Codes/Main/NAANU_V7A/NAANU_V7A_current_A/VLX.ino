#pragma once

#include "VL53L0X.h"

VL53L0X sensor[4];


//0 front right
//1 front left
//2 right
//3 left

bool initializeVlx() {
  for (uint8_t i = 0; i < 4; i++) {
    distance[i] = 0;
    distanceOld[i] = 0;
  }
  pinMode(xshutFrontRight, OUTPUT);
  pinMode(xshutFrontLeft, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutLeft, OUTPUT);
  setID();

  //  Serial.println(sensor[0].getAddress(), HEX);
  //  Serial.println(sensor[1].getAddress(), HEX);
  //  Serial.println(sensor[2].getAddress(), HEX);
  //  Serial.println(sensor[3].getAddress(), HEX);

  sensor[0].init();
  sensor[0].setTimeout(200);
  sensor[0].startContinuous(0);
  //  Serial.println("0");

  sensor[2].init();
  sensor[2].setTimeout(200);
  sensor[2].startContinuous(0);
  //  Serial.println("0");

  sensor[3].init();
  sensor[3].setTimeout(200);
  sensor[3].startContinuous(0);

  //  Serial.println("0");

  sensor[1].init();
  sensor[1].setTimeout(200);
  sensor[1].startContinuous(0);
  //  Serial.println("0");


}


void setID() {
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

  // keeping front right sensor ON while turning OFF other sensors
  digitalWrite(xshutFrontRight, HIGH);
  digitalWrite(xshutFrontLeft, LOW);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  delay(10);
  sensor[0].setAddress(frontRightAddress);
  delay(10);
  //  Serial.println("front");

  // keeping front left sensor and front right sensor ON while turning OFF other sensors
  digitalWrite(xshutFrontLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutLeft, LOW);
  delay(10);
  sensor[1].setAddress(frontLeftAddress);
  delay(10);

  // keeping front sensors and the right sensor ON while turning OFF sensor on the left
  digitalWrite(xshutLeft, HIGH);
  digitalWrite(xshutRight, LOW);
  delay(10);
  sensor[3].setAddress(leftAddress);
  delay(10);

  digitalWrite(xshutRight, HIGH);
  delay(10);
  sensor[2].setAddress(rightAddress);
  delay(10);
}

void readDistance() {
  //takes less than 1ms (700us) to retreive all 3 data
  distance[0] = sensor[0].readReg16Bit(sensor[0].RESULT_RANGE_STATUS + 10);
  sensor[0].writeReg(sensor[0].SYSTEM_INTERRUPT_CLEAR, 0x01);
  distance[1] = sensor[1].readReg16Bit(sensor[1].RESULT_RANGE_STATUS + 10);
  sensor[1].writeReg(sensor[1].SYSTEM_INTERRUPT_CLEAR, 0x01);
  distance[2] = sensor[2].readReg16Bit(sensor[2].RESULT_RANGE_STATUS + 10);
  sensor[2].writeReg(sensor[2].SYSTEM_INTERRUPT_CLEAR, 0x01);
  distance[3] = sensor[3].readReg16Bit(sensor[3].RESULT_RANGE_STATUS + 10);
  sensor[3].writeReg(sensor[3].SYSTEM_INTERRUPT_CLEAR, 0x01);

  //  distance[0] = sensor[FRONT_R].readRangeContinuousMillimeters();
  //  distance[1] = sensor[FRONT_L].readRangeContinuousMillimeters();
  //  distance[2] = sensor[LEFT].readRangeContinuousMillimeters();
  //  distance[3] = sensor[RIGHT].readRangeContinuousMillimeters();

  //exponential time average filter
  distance[0] = (0.93 * distance[0] + 0.07 * distanceOld[0]) - 55; //61;
  distance[1] = (0.93 * distance[1] + 0.07 * distanceOld[1]) - 20;
  distance[2] = ( 0.93 * distance[2] + 0.07 * distanceOld[2]) - 40;
  distance[3] = ( 0.93 * distance[3] + 0.07 * distanceOld[3]);

  distance[0] = distance[0] == 0  ? 8910 : distance[0];
  distance[1] = distance[1] == 0 ? 8910 : distance[1];
  distance[2] = distance[2] == 0  ? 8910 : distance[2];
  distance[3] = distance[3] == 0   ? 8910 : distance[3];

  distanceOld[0] = distance[0];
  distanceOld[1] = distance[1];
  distanceOld[2] = distance[2];
  distanceOld[3] = distance[3];
}

void printDistance() {

  //  Serial.println(sensor[0].getAddress(), HEX);
  //  Serial.println(sensor[1].getAddress(), HEX);
  //  Serial.println(sensor[2].getAddress(), HEX);
  //  Serial.println(sensor[3].getAddress(), HEX);
  Serial.print("FRONT_R: "); Serial.print(distance[0]);  Serial.print("\t");
  Serial.print("FRONT_l: "); Serial.print(distance[1]);  Serial.print("\t");
  Serial.print("RIGHT: "); Serial.print(distance[2]);  Serial.print("\t");
  Serial.print("LEFT: "); Serial.println(distance[3]);
}

void calibrate() {

}
