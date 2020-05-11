#include"Battery.h"

float checkVoltage() {
  batteryVoltage = analogRead(batteryPin) / 112.81;

  return batteryVoltage;
}
