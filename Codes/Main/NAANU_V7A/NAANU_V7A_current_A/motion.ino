#pragma once
#include"math.h"
#define wallDistance_F 80//100//90
#define wallDistance 82//95//85
#define cellDistance 935 //in counts of the encoder

bool wallLeft() {
  readDistance();
  if (distance[LEFT] < wallDistance) {
    return 1;
  }
  else {
    //    distance[LEFT] = 95;
    return 0;
  }

}

bool wallFront() {
  //  temp = East;
  readDistance();
  if (distance[FRONT_L] < wallDistance_F && distance[FRONT_R] < wallDistance_F) {
    return 1;
  } else return 0;
}

bool wallRight() {
  readDistance();
  if (distance[RIGHT] < wallDistance) {
    return 1;
  }
  else {
    //    distance[RIGHT] = 95;
    return 0;
  }
}

int calculateAngle() {
  readDistance();
  if (distance[FRONT_L] < 90 && distance[FRONT_R] < 90) {
    float angle;
    for (int i = 0; i < 10; i++) {
      readDistance();
      angle += asin((distance[FRONT_L] - distance[FRONT_R]) / 56.0) * 57.320;
    }
    return angle / 10;
  }
  else  return 0;
}

//void turnRight() {
//  //  align();
//  countRight = 0;
//  countLeft = 0;
//  //  int angle = calculateAngle();
//  //  if (angle > 25) {
//  //    angle -= 5;
//  //  }
//
//  resetMPU();  pidVlx = new PID(&Input, &Output, 0, kp_vlx, ki_vlx, kd_vlx, REVERSE);
//  pidVlx->SetMode(AUTOMATIC);
//  pidVlx->SetSampleTime(5);
//  pidVlx->SetOutputLimits(-50, 50);
//
//  //  || countLeft < 250
//  while (abs(calculations()) < 90 && abs(countRight) < 350 && abs(countLeft) < 350) {
//    Input = abs(countLeft) - abs(countRight);
//    pidEncoder->Compute();
//    motor.Motion(rotateRight, 32 - Output, 32 + Output + 1);
//    delay(4);
//    //    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
//  }
//
//  motor.Motion(rotateLeft, 255, 255);
//  delay(2);
//  motor.Motion(Brake);
//  countRight = 0;
//  countLeft = 0;
//  //update heading
//  Serial.println("in turnRight");
//  return;
//}

//void turnLeft() {
//  //  align();
//  //  int angle = calculateAngle();
//  //  int angle = 0;
//  //  if (angle < 15 && angle > 7) {
//  //    angle -= 10;
//  //  }
//  //  int angle = 0;
//  countRight = 0;
//  countLeft = 0;
//  resetMPU();
//  //
//  while ( abs(calculations()) < 90 && abs(countRight) < 350 && abs(countLeft) < 350 )  {
//    Input = abs(countLeft) - abs(countRight);
//    pidEncoder->Compute();
//    motor.Motion(rotateLeft, 32 - Output, 32 + Output + 1);
//    //    delay(4);
//    //    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);
//
//    //    printData();
//    delay(4);
//
//  }
//  motor.Motion(rotateRight, 255, 255);
//  delay(2);
//  motor.Motion(Brake);
//  countRight = 0;
//  countLeft = 0;
//  Serial.println("in turnLeft");
//  return;
//}


//pid bata right jaane code
void turnLeft() {
  turnAngle(-calculateAngle());
  resetMPU();
  int angle = 0;
  int count = 0;
  countLeft = 0;
  countRight = 0;
  double in = 0, out = 0;
  //     Input = (distance[FRONT_L] - distance[FRONT_R] + 10) * 0.5;
  //      pidVlx->Compute();
  while (1) {
    if (count > 10) break;
    angle = calculations();
    angle = angle % 180;
    Input = 90 - angle;
    pidMpu->Compute();
    in = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    if (Input > 2) {
      motor.Motion(rotateLeft, 35 + Output, 35 + Output);
      //motor.Motion(rotateLeft, 30, 35 + Output);
    } else if (Input < -2) {
      motor.Motion(rotateRight, abs(Output) + 35,  abs(Output) + 35);
    } else  {
      count++;
    }
    delay(4);
    //  Serial.println(Output);
    //      printData();
  }
  //  motor.Motion(rotateLeft, 255, 255);
  motor.Motion(Brake);
  Serial.println("in turnLeft");
}

//pid bata right jaane code
void turnRight() {
  turnAngle(-calculateAngle());
  resetMPU();
  int angle = 0;
  int count = 0;
  countLeft = 0;
  countRight = 0;
  double in = 0, out = 0;
  while (1) {
    if (count > 10) break;
    angle = abs(calculations());
    angle = angle % 180;
    Input = 90 - angle;
    pidMpu->Compute();
    in = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    if (Input > 2) {
      motor.Motion(rotateRight, 35 + Output, 35 + Output);
      //    motor.Motion(rotateRight,35+Output,31);
    } else if (Input < -2) {
      motor.Motion(rotateLeft, abs(Output) + 35,  abs(Output) + 35);
    } else  {
      count++;
    }
    delay(4);
    //  Serial.println(Output);
    printData();
  }
  //  motor.Motion(rotateLeft, 255, 255);
  motor.Motion(Brake);
  Serial.println("in turnRight");
  //  while (1);
}


void turnAngle(int setPoint) {
  resetMPU();
  int angle = 0;
  int count = 0;
  countLeft = 0;
  countRight = 0;

  motion decision1, decision2;
  if (setPoint < 0) {
    decision1 = rotateRight;
    decision2 = rotateLeft;
  }
  else {
    decision1 = rotateLeft;
    decision2 = rotateRight;
  }

  setPoint = abs(setPoint);
  double in = 0, out = 0;
  while (1) {
    if (count > 10) break;
    angle = abs(calculations());
    Input = setPoint - angle;
    pidMpu->Compute();
    in = abs(countLeft) - abs(countRight);
    pidEncoder->Compute();
    if (Input > 2) {
      motor.Motion(decision1, 35 + Output, 35 + Output);
    } else if (Input < -2) {
      motor.Motion(decision2, abs(Output) + 35,  abs(Output) + 35);
    } else  {
      count++;
    }
    delay(4);
    //  Serial.println(Output);
    printData();
  }
  //  motor.Motion(rotateLeft, 255, 255);
  motor.Motion(Brake);
}

void _right() {
  countRight = 0;
  countLeft = 0;
  while ((countRight + countLeft) < 1516) {
    Input = (distance[RIGHT] - 47);
    pidVlx->Compute();
    motor.Motion(Forward, 30 - Output, 30 + Output + 1);
  }
}

//  ( ((distance[FRONT_L] - distance[FRONT_R]) < 3) && ((distance[FRONT_L] - distance[FRONT_R]) > -3)

//void moveForward() {
//  //tune this value maze ko anusar 622 chai black binding ko miu ko value
//  countRight = 0;
//  countLeft = 0;
//  uint8_t bufferFull = 0;
//  //  readDistance();
//  while (countRight < 925 || countLeft < 925 ) {
//    if ( wallFront() ) {
//      break;
//    }
//    if (wallRight() && wallLeft()) {
//      Input = distance[RIGHT] - distance[LEFT];
//      pidVlx->Compute();
//      Serial.print("Both wall error:  "); Serial.println(Input);
//    }
//    else if (wallRight()) {
//      if (countRight < 220 && countLeft < 220) {
//        Input = (distance[RIGHT] - 44);
//        //      if (Input > 35 && bufferFull < 15) {
//        //        bufferFull++;
//        //        Input = Input * 0.7;
//        //      }
//        pidVlx->Compute();
//        Serial.print("Right wall error:  "); Serial.println(Input);
//      }
//
//    }
//    else if (wallLeft() ) {
//      if (countRight < 220 && countLeft < 220) {
//        Input = (44 - distance[LEFT]);
//        pidVlx->Compute();
//        Serial.print("Left wall error:  "); Serial.println(Input);
//      }
//    }
//    else if (!wallLeft() && !wallRight()) {
//      turnAngle(-calculateAngle());
//      Input = countRight - countLeft;
//      pidEncoder->Compute();
//      Serial.print("No wall error:  "); Serial.println(Input);
//    }
//    motor.Motion(Forward, baseSpeed + Output, baseSpeed - Output);
//  }
//  motor.Motion(Back, 255, 255);
//  motor.Motion(Brake);
//  return;
//}

//pid ko moveforward ho bhanera
void moveForward() {
  int count = 0;
  countLeft = 0;
  countRight = 0;
  readDistance();
  double in = 0, out = 0;
  while (1) {
    //    readDistance();
    in = 2 * cellDistance - countRight - countLeft;
    pidEncoder->Compute();
    //    (distance[FRONT_L] < 50 || distance[FRONT_R] < 50) ||
    if (wallFront())
    {
      if (distance[FRONT_L] < 78 || distance[FRONT_R] < 78)
      {
        motor.Motion(Brake);
        delay(2);
        motor.Motion(Back, 180 - distance[FRONT_L], 180 - distance[FRONT_R]);
        delay(6);
        motor.Motion(Brake);
        delay(2);
        break;
      }
    }


    if ( count > 8 ) break;

    if (wallRight() && wallLeft()) {
      Input = distance[RIGHT] - distance[LEFT];
      pidVlx->Compute();
    }
    else if (wallRight()) {
      Input = (distance[RIGHT] - 50);
      pidVlx->Compute();
    }
    else if (wallLeft()) {
      Input = (44 - distance[LEFT]);
      pidVlx->Compute();
    }
    else {
      Input = ( countRight - countLeft);
      pidEncoder->Compute();
      out = 0;
    }
    //    in = (cellDistance - countRight + cellDistance - countLeft) / 2.0;

    if (in > 3) {
      motor.Motion(Forward, 43 + out + Output, 43 + out - Output);
    }
    else if (in < -3) {
      motor.Motion(Back, 45 + abs(out), 45 + abs(out));
    }
    else count++;
    //    Serial.print("Left :  "  ); Serial.print(countLeft); Serial.print(" Right :  "  ); Serial.println(countRight);

  }
  //  motor.Motion(Back);
  //  delay(2);
  motor.Motion(Brake);
  delay(1);
  return;
}



void speedRun() {
  for (int i = 0; i <path; i++) {
    if (shortPath[i] == 'Z') {
      BUZZER_ON;
      delay(500);
      BUZZER_OFF;
      delay(500);
      BUZZER_ON;
      delay(400);
      BUZZER_OFF;
      delay(400);
      BUZZER_ON;
      delay(200);
      BUZZER_OFF;
      delay(200);
      while (1);
    }
    else if (shortPath[i] == 'F') {
      moveForward();
    } else if (shortPath[i] == 'R') {
      turnRight();
      moveForward();
    } else if (shortPath[i] == 'L') {
      turnLeft();
      moveForward();
    }
  }
}
