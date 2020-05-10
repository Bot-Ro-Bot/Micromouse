#pragma once

#include"config.h"

float readBatteryVoltage() {
  return analogRead(BATTERY_PIN) / 112.81;
}
