#pragma once
#include "Node.h"
Node node[16][16];
int X = 0, Y = 0;

Node testNode;

void printNode() {
  Serial.println(testNode.nodeDetails());
}

void printPotential() {
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      Serial.print("Nodes : ");  Serial.print(" X ");  Serial.print(i);  Serial.print(" ,Y : ");  Serial.print(j);  Serial.print(" :: "); Serial.println(node[i][j].value);
    }
  }
}
void updateDirection() {
  myDirection = OppositeOf(myDirection);
}

void printDirection() {
  Serial.println(myDirection);
}

void updatePosition() {
  myDirection = (Direction)LeftOf(myDirection);

  X += (Direction)RelativeX(myDirection);
  Y += (Direction)RelativeY(myDirection);
}

void printPosition() {
  Serial.println(X);
  Serial.println(Y);
}
void potential() {
  uint8_t i, j;
  for (i = 0; i < sizeX; i++) {
    for (j = 0; j < sizeY; j++) {
      node[i][j].value = 0;
    }
  }
  for (j = 0; j < sizeY / 2; j++) {
    for (i = 0; i < sizeX / 2; i++) {
      node[i][j].value = 14 - i - j; //1st quadrat
    }
  }
  for (j = 0; j < sizeY / 2; j++) {
    for (i = sizeX / 2; i < sizeX; i++) {
      node[i][j].value = i - j - 1; //2nd quadrant
    }
  }
  for (j = sizeY / 2; j < sizeY; j++) {
    for (i = 0; i < sizeX / 2; i++) {
      node[i][j].value = j - i - 1; //4th quadrant
    }
  }
  for (j = sizeY / 2; j < sizeY; j++) {
    for (i = sizeX / 2; i < sizeX; i++) {
      node[i][j].value = i + j - 16; //3rd quadrant
    }
  }
}


void TremauxAlgorithm() {

  if((X== 7 && Y == 7)||(X== 8 && Y == 8))
  {
    BUZZER_ON;
    delay(200);
    BUZZER_OFF;
    delay(200);
    BUZZER_ON;
    delay(200);
    BUZZER_OFF;
    delay(200);
    BUZZER_ON;
    delay(200);
    BUZZER_OFF;
  }
  
  Direction tmp = (Direction)myDirection;
  Direction tmpLeft = (Direction)LeftOf(myDirection);
  Direction tmpRight = (Direction)RightOf(myDirection);
  tmpRight = (Direction)RightOf(myDirection);
  myDirection = tmp;

  if (node[X][Y].nodeDetails() == 10) {
    Serial.println("Dead End found");
    node[X][Y].map = 2;
    myDirection = OppositeOf(myDirection);
    turnRight();
    turnRight();
    X += (Direction)RelativeX(myDirection);
    Y += (Direction)RelativeY(myDirection);
    moveForward();
  }

  else if (node[X][Y].nodeDetails() == 1)
  {
    Serial.println("PATH 1");
    if (!wallLeft() && node[X + (Direction)RelativeX(tmpLeft)][Y + (Direction)RelativeY(tmpLeft)].map < 2) {
      node[X][Y].map += 1;
      turnLeft();
      myDirection = tmpLeft;
      moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
      return;
    }
    else if (!wallRight() && node[X + (Direction)RelativeX(tmpRight)][Y + (Direction)RelativeY(tmpRight)].map < 2) {
      node[X][Y].map += 1;
      turnRight();
      myDirection = tmpRight;
      moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
      return;
    }
    else if (!wallFront() && node[X + (Direction)RelativeX(myDirection)][Y + (Direction)RelativeX(myDirection)].map < 2) {
      Serial.println("moving Forward");
      node[X][Y].map += 1 ;
      moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
      return;
    }
    //    else {
    //      node[X][Y].map += 1 ;
    //      moveForward();
    //      X += (Direction)RelativeX(myDirection);
    //      Y += (Direction)RelativeY(myDirection);
    //      return;
    //    }
  }


  else if (node[X][Y].nodeDetails() == 2) {
    Serial.println("PATH 2");
    if (!wallLeft()  && !wallRight())
    {
      if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].value <= node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].value)
      {
        if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map < 2 )
        {
          node[X][Y].map += 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map < 2 )
        {
          node[X][Y].map += 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
    }
    if (!wallFront() && !wallLeft())
    {
      if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].value <= node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].value)
      {
        if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map < 2 )
        {
          node[X][Y].map += 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map < 2 )
        {
          node[X][Y].map += 1;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
    }
    if (!wallFront() && !wallRight())
    {
      if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].value <= node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].value)
      {
        if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map < 2 )
        {
          node[X][Y].map += 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map < 2 )
        {
          node[X][Y].map += 1;
          moveForward();
          X += (Direction)RelativeX(myDirection);
          Y += (Direction)RelativeY(myDirection);
          return;
        }
      }
    }
  }

  
  else if (node[X][Y].nodeDetails() == 3) {
    Direction tmp = (Direction)myDirection;
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    tmpRight = (Direction)RightOf(myDirection);
    myDirection = tmp;
    Serial.println("PATH 3");
    if ((2 * node[X + (Direction)RelativeX(tmpLeft)][Y + (Direction)RelativeY(tmpLeft)].value - (node[X + (Direction)RelativeX(myDirection)][Y + (Direction)RelativeY(myDirection)].value + node[X + (Direction)RelativeX(tmpRight)][Y + (Direction)RelativeY(tmpRight)].value)) < 0) {
      Serial.println("turned Left");
      node[X][Y].map = 1 ;
      myDirection = tmpLeft;
      turnLeft();
      if (!wallFront()) moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
    }
    else if ((2 * node[X + (Direction)RelativeX(myDirection)][Y + (Direction)RelativeY(myDirection)].value - (node[X + (Direction)RelativeX(tmpRight)][Y + (Direction)RelativeY(tmpRight)].value + node[X + (Direction)RelativeX(tmpLeft)][Y + (Direction)RelativeY(tmpLeft)].value)) < 0) {
      node[X][Y].map = 1 ;
      if (!wallFront()) moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
    }
    else if ((2 * node[X + (Direction)RelativeX(tmpRight)][Y + (Direction)RelativeY(tmpRight)].value - (node[X + (Direction)RelativeX(myDirection)][Y + (Direction)RelativeY(myDirection)].value + node[X + (Direction)RelativeX(tmpLeft)][Y + (Direction)RelativeY(tmpLeft)].value)) < 0) {
      Serial.println("turned Right");
      node[X][Y].map = 1 ;
      myDirection = tmpRight;
      turnRight();
      if (!wallFront()) moveForward();
      X += (Direction)RelativeX(myDirection);
      Y += (Direction)RelativeY(myDirection);
    }
  }
}
