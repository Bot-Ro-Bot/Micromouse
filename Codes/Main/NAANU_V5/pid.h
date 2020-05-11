#pragma once

class PID {
  public:
    PID(float kp = 0, float ki = 0, float kd = 0);
    int16_t calculatePID( int16_t input, int16_t setPoint = 0);
    void resetValue();

  private:
    float integral, derivative, kp, ki, kd;
    int16_t previousError, error, output;
    long currentTime, previousTime;
};

PID::PID(float kp, float ki, float kd) {
  integral = derivative = 0;
  previousError = error = output = 0;
  currentTime = previousTime = 0;
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

}

int16_t PID :: calculatePID( int16_t input, int16_t setPoint) {
  currentTime = millis();
  error = setPoint - input;
  if (error > 180) {
    error = error - 360;
  }
  integral =  integral + (error * (currentTime - previousTime) / 1000) ;
  derivative = (error - previousError) / ((currentTime - previousTime) / 1000);
  output = kp * error + ki * integral + kd * derivative;
  output = (output > 40) ? 40 : (output < -40) ? -40 : output ;
  previousError = error;
  previousTime = currentTime;
  return output;
}

void PID::resetValue() {
  integral = derivative = 0;
  previousError = error = output = 0;
  currentTime = previousTime = 0;
}
