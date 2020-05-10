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
      //      motor.Motion(rotateRight, 35 + Output, 31);
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

void moveForward() {
  int count = 0;
  countLeft = 0;
  countRight = 0;
  readDistance();
  double in = 0, out = 0;
  while (1)
  {
    in = 2 * cellDistance - countRight - countLeft;
    pidEncoder->Compute();
    if (wallFront())
    {
      if (distance[FRONT_L] < 78 || distance[FRONT_R] < 78)
      {
        motor.Motion(Brake);
        delay(2);
        motor.Motion(Back, 255 - distance[FRONT_L], 255 - distance[FRONT_R]);
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
    if (in > 3) {
      motor.Motion(Forward, 45 + out + Output, 45 + out - Output);
    }
    else if (in < -3) {
      motor.Motion(Back, 45 + abs(out), 45 + abs(out));
    }
    else
    {
      motor.Motion(Back, 255 - distance[FRONT_L], 255 - distance[FRONT_R]);
      delay(1);
      motor.Motion(Brake);
      delay(1);
      count++;
    }
  }
}


void moveCells(int cells) {
  int count = 0;
  countLeft = 0;
  countRight = 0;
  readDistance();
  double in = 0, out = 0;
  while (1) {
    //    readDistance();
    in = 2 * cellDistance * cells - countRight - countLeft;
    pidEncoder->Compute();
    if (wallFront())
    {
      if (distance[FRONT_L] < 78 || distance[FRONT_R] < 78)
      {
        motor.Motion(Brake);
        delay(2);
        motor.Motion(Back, 255 - distance[FRONT_L], 255 - distance[FRONT_R]);
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
    if (in > 3) {
      motor.Motion(Forward, 43 + out + Output, 43 + out - Output);
    }
    else if (in < -3) {
      motor.Motion(Back, 45 + abs(out), 45 + abs(out));
    }
    else count++;
  }
  //  motor.Motion(Back);
  //  delay(2);
  motor.Motion(Brake);
  delay(1);
  return;
}

void moveForwardSpeedRun() {
  int count = 0;
  countLeft = 0;
  countRight = 0;
  readDistance();
  double in = 0, out = 0;
  while (1)
  {
    in = 2 * cellDistance - countRight - countLeft;
    pidEncoder->Compute();
    //    (distance[FRONT_L] < 50 || distance[FRONT_R] < 50) ||
    if (wallFront())
    {
      if (distance[FRONT_L] < 78 || distance[FRONT_R] < 78)
      {
        motor.Motion(Brake);
        delay(2);
        motor.Motion(Back, 255 - distance[FRONT_L], 255 - distance[FRONT_R]);
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
    if (in > 3) {
      motor.Motion(Forward, SPEED + out + Output, SPEED + out - Output);
    }
    else {
      motor.Motion(Back, 255 - distance[FRONT_L], 255 - distance[FRONT_R]);
      delay(1);
      motor.Motion(Brake);
      delay(1);
      count++;
    }
  }
}



void speedRun() {
  for (int i = 0; i < path; i++) {
    if (shortPath[i] == _END) {
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
      Serial.println("Destination Arrived");
      while (1);
    }
    else if (shortPath[i] == _FORWARD) {
      moveForwardSpeedRun();
      //      Serial.println("Moving Forward");
    } else if (shortPath[i] == _RIGHT) {
      turnRight();
      moveForward();
      //      Serial.println("Moving Right");
    } else if (shortPath[i] == _LEFT) {
      turnLeft();
      moveForward();
      //      Serial.println("Moving Left");
    }
  }
}
