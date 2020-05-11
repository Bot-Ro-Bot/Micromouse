#pragma once
#include"math.h"
#define wallDistance_F 90
#define wallDistance 85

//supposed linearity
/*200=255=12.6v
  v=rw ,r=42mm,w=200/255=62.4
  v=274.36mm/s
  w=dtheta /dt
  90 degree turn implied
  v=rw,r=6cm,v=274.36mm/s
  w=4.573rad/s
  therefore, dt=0.34516s=345ms*/

bool wallLeft() {
  readDistance();
  if (distance[LEFT] < wallDistance) {
    return 1;
  } else return 0;
}

bool wallFront() {
  readDistance();
  if (distance[FRONT_L] < wallDistance_F && distance[FRONT_R] < wallDistance_F) {
    return 1;
  } else return 0;
}
bool wallRight() {
  readDistance();
  if (distance[RIGHT] < wallDistance) {
    return 1;
  } else return 0;
}

int calculateAngle() {
  float angle;
  for (int i = 0; i < 10; i++) {
    readDistance();
    angle += asin((distance[FRONT_L] - distance[FRONT_R]) / 56.0) * 57.320;
  }
  return angle / 10;
}
//void turnRight() {
//  resetMPU();
//  countLeft = 0;
//  countRight = 0;
//  int16_t angle = abs(calculations());
//  uint8_t pwm = 85;
//  motor.Motion(rotateRight, pwm, pwm);
//  long count = millis();
//  //pid ko yehi initialize garne
//  while (1) {
//    if (( abs(abs(calculations()) - angle)  > 33) || ((millis() - count) > 500) || countLeft > 110) {
//      motor.Motion(Stop, 0, 0);
//      motor.Motion(rotateLeft, 255, 255);
//      delay(5);
//      motor.Motion(Stop, 0, 0);
//      return;
//      //update heading
//      //      X += (Direction)RelativeX(myDirection);
//      //      Y += (Direction)RelativeY(myDirection);
//      //      moveForward();
//    }
//    pwm -= 10;
//    if (pwm < 30) {
//      pwm = 90;
//    }
//    delay(5);
//  }
//}


/*
    if (( abs(abs(calculations()) - angle)  > 80) || ((millis() - count) > 350 * (12.2 / readBatteryVoltage())) ) {
      motor.Motion(rotateRight, 255, 255);
      motor.Motion(Stop);
  //      update heading
      Node::updateHeading(rHeading, 'l');
    }
*/
//void turnLeft() {
//  resetMPU();
//  countRight = 0;
//  countLeft = 0;
//  int16_t angle = abs(calculations());
//  uint8_t pwm = 90;
//
//  long count = millis();
//  while (1) {
//    if (( abs(abs(calculations()) - angle)  > 55) || ((millis() - count) > 500)) {
//      motor.Motion(Stop, 0, 0);
//      motor.Motion(rotateRight, 255, 255);
//      delay(5);
//      motor.Motion(Stop, 0, 0);
//      //update heading
//      //      X += (Direction)RegyroYaw = 0;

//      //      Y += (Direction)RelativeY(myDirection);
//      //      moveForward();
//      return;
//    }
//    motor.Motion(rotateLeft, pwm, pwm);
//    pwm -= 10;
//    if (pwm < 30) {
//      pwm = 90;
//    }
//    delay(5);
//  }
//}

void turnRight() {
  //  align();
  countRight = 0;
  countLeft = 0;
  //  int angle = calculateAngle();
  //  if (angle > 25) {
  //    angle -= 5;
  //  }

  resetMPU();  pidVlx = new PID(&Input, &Output, 0, kp_vlx, ki_vlx, kd_vlx, REVERSE);
  pidVlx->SetMode(AUTOMATIC);
  pidVlx->SetSampleTime(5);
  pidVlx->SetOutputLimits(-50, 50);

  //  || countLeft < 250
  while (abs(calculations()) < 90 && abs(countRight) < 350 && abs(countLeft) < 350) {
    Input = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    motor.Motion(rotateRight, 32 - Output, 32 + Output + 1);
    delay(4);
    //    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
  }

  motor.Motion(rotateLeft, 255, 255);
  delay(2);
  motor.Motion(Brake);
  countRight = 0;
  countLeft = 0;
  //update heading
  Serial.println("in turnRight");
  return;
}

void turnLeft() {
  //  align();
  //  int angle = calculateAngle();
  //  int angle = 0;
  //  if (angle < 15 && angle > 7) {
  //    angle -= 10;
  //  }
  //  int angle = 0;
  countRight = 0;
  countLeft = 0;
  resetMPU();
  //
  while ( abs(calculations()) < 90 && abs(countRight) < 350 && abs(countLeft) < 350 )  {
    Input = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    motor.Motion(rotateLeft, 32 - Output, 32 + Output + 1);
    //    delay(4);
    //    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);

    //    printData();
    delay(4);

  }
  motor.Motion(rotateRight, 255, 255);
  delay(2);
  motor.Motion(Brake);
  countRight = 0;
  countLeft = 0;
  Serial.println("in turnLeft");
  return;
}

void pid_turnLeft() {
  resetMPU();
  int angle = 0;
  int count = 0;
  countLeft = 0;
  countRight = 0;
  double in = 0, out = 0;
  pidEncoder = new PID(&in, &out, 0, kp_Encoder, ki_Encoder, kd_Encoder, REVERSE);
  pidEncoder->SetMode(AUTOMATIC);
  pidEncoder->SetSampleTime(5);
  pidEncoder->SetOutputLimits(-50, 50);

  while (1) {
    if (count > 10) break;
    angle = calculations();
    Input = 90 - angle;
    pidMpu->Compute();
    in = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    if (Input > 2) {
      motor.Motion(rotateLeft, 35 + Output, 35 + Output);
    } else if (Input < -2) {
      motor.Motion(rotateRight, abs(Output) + 35,  abs(Output) + 35);
    } else  {
      count++;
    }
    delay(4);
    //  Serial.println(Output);
    printData();
  }
  motor.Motion(rotateLeft, 255, 255);
  motor.Motion(Brake);
  Serial.println("in turnLeft");
}

void align() {
  readDistance();
  while (1) {
    readDistance();
    if (distance[FRONT_L] < 65 && distance[FRONT_L] > 55 && distance[FRONT_R] < 65 && distance[FRONT_R] > 55) {
      break;
    }
    int adjust = distance[FRONT_L] - distance[FRONT_R];
    if (distance[FRONT_L] < 60 && distance[FRONT_R] < 60) {
      if (adjust > 0) {
        motor.Motion(Back, 35, 45);
      }
      else {
        motor.Motion(Back, 40, 35);
      }
    } else if (distance[FRONT_L] > 60 && distance[FRONT_R] > 60) {
      if (adjust > 0) {
        motor.Motion(Forward, 40, 35);
      }
      else {
        motor.Motion(Forward, 35, 40);
      }
    }
  }
  motor.Motion(Brake);

}

//5cm offset ko laagi 341
//18 ko laagi 477 nai ho
//void moveForward() {
//  readDistance();
//  countLeft = 0;
//  countRight = 0;
//  //  double InputRight = 0, OutputRight = 0, InputLeft = 0, OutputLeft = 0;
//  //  pidRight = new PID(&InputRight, &OutputRight, 0, kp_Encoder_Rotation, ki_Encoder_Rotation, kd_Encoder_Rotation, REVERSE);
//  //  pidRight->SetMode(AUTOMATIC);
//  //  pidRight->SetSampleTime(5);
//  //  pidRight->SetOutputLimits(-50, 50);
//  //
//  //  pidLeft = new PID(&InputLeft, &OutputLeft, 0, kp_Encoder_Rotation, ki_Encoder_Rotation, kd_Encoder_Rotation, REVERSE);
//  //  pidLeft->SetMode(AUTOMATIC);
//  //  pidLeft->SetSampleTime(5);
//  //  pidLeft->SetOutputLimits(-50, 50);
//
//  //  motor.Motion(Forward);
//
//  while (countLeft < 950 || countRight < 950) {
//
//    if ( (distance[FRONT_L] < 95 && distance[FRONT_R] < 95) ) {
//      //      X += (Direction)RelativeX(myDirection);
//      //      Y += (Direction)RelativeY(myDirection);
//      break;
//    }
//    if (wallRight() && wallLeft()) {
//      Input = distance[RIGHT] - distance[LEFT];
//      pidVlx->Compute();
//    }
//    else if (!wallLeft() && !wallRight()) {
//      Input = countLeft - countRight;
//      pidEncoder->Compute();
//      motor.Motion(Forward, 30 - Output, 30 + Output + 1);
//    }
//    else if (wallRight() && !wallLeft()) {
//      int tempCount = countRight;
//      while ((countRight - tempCount) < 40) {
//        Input = countLeft - countRight;
//        pidEncoder->Compute();
//        motor.Motion(Forward, 30 - Output, 30 + Output + 1);
//        break;
//      }
//      break;
//    }
//    //    if (countRight > 150) {
//    //      Input = ( distance[RIGHT] - 47);
//    //      pidVlx->Compute();
//    //    } else Output = 0;
//    else if (wallLeft() && !wallRight()) {
//      int tempCount = countRight;
//      while ((countRight - tempCount) < 40) {
//        Input = countLeft - countRight;
//        pidEncoder->Compute();
//        motor.Motion(Forward, 30 - Output, 30 + Output + 1);
//        break;
//      }
//      break;
//    }
//    //    if (countRight > 150) {
//    //      Input = (45 - distance[LEFT]);
//    //      pidVlx->Compute();
//    //    } else Output = 0;
//
//    motor.Motion(Forward, baseSpeed + Output, baseSpeed - Output);
//    readDistance();
//  }
//  //    Serial.print( "Left:    "); Serial.print(baseSpeed + Output); Serial.print( "Right:    "); Serial.println(baseSpeed - Output);
//  motor.Motion(Back, 255, 255);
//  motor.Motion(Brake);
//
//  //  align();
//  //  BUZZER_ON;
//  //  delay(40);
//  //  BUZZER_OFF;
//  //  delay(1000);
//}


void _right() {
  countRight = 0;
  countLeft = 0;
  while ((countRight + countLeft) < 1516) {
    Input = (distance[RIGHT] - 47);
    pidVlx->Compute();
    motor.Motion(Forward, 30 - Output, 30 + Output + 1);
  }
}


void moveForward() {
  //tune this value maze ko anusar 622 chai black binding ko miu ko value
  countRight = 0;
  countLeft = 0;
  readDistance();
  while (countRight < 925 || countLeft < 925 ) {
    if ( (distance[FRONT_L] < 75 && distance[FRONT_R] < 75) ) {
      //      X += (Direction)RelativeX(myDirection);
      //      Y += (Direction)RelativeY(myDirection);
      break;
    }
    if (wallRight() && wallLeft()) {
      Input = distance[RIGHT] - distance[LEFT];
      pidVlx->Compute();
    }
    else if (wallRight()) {
      Input = (distance[RIGHT] - 44);
      //      if (Input > 19) {
      //        countLeft--;
      //      } else if (Input < -19) {
      //        countRight--;
      //      }
      pidVlx->Compute();
      //      digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
    }
    else if (wallLeft() ) {
      Input = (44 - distance[LEFT]);
      pidVlx->Compute();
      //      digitalWrite(LED_PIN, 1 ^ digitalRead(LED_PIN));
      //      digitalWrite(BUZZER_PIN, 1 ^ digitalRead(BUZZER_PIN));
    }
    else {
      Input = countLeft - countRight;
      pidEncoder->Compute();
      motor.Motion(Forward, 45 - Output, 45 + Output + 1);
      Output = 0;
      //      digitalWrite(BUZZER_PIN, 1 ^ digitalRead(BUZZER_PIN));
      //      continue;
    }
    motor.Motion(Forward, baseSpeed + Output, baseSpeed - Output);
    readDistance();
    Serial.println("in moveFOrward");
  }
  motor.Motion(Back, 255, 255);
  motor.Motion(Brake);
  return;
}
