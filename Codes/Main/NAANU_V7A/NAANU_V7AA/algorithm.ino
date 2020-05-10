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

  X += RelativeX(myDirection);
  Y += RelativeY(myDirection);
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

void wetRun()
{
  Direction tmpLeft = (Direction)LeftOf(myDirection);
  Direction tmpRight = (Direction)RightOf(myDirection);
  if (!motion.wallLeft() && (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map == 11))
  {
    motion.turnLeft();
    myDirection = tmpLeft;
    Serial.println("Turn Left");
    motion.moveForward();
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
  else if (!motion.wallFront() && (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map == 44))
  {
    motion.moveForward();
    Serial.println("Moving Forward");
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
  else if (!motion.wallRight() && (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map == 22))
  {
    motion.turnRight();
    Serial.println("Turn Right");
    myDirection = tmpRight;
    motion.moveForward();
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
}

void returnNow()
{
  Direction tmpLeft = (Direction)LeftOf(myDirection);
  Direction tmpRight = (Direction)RightOf(myDirection);
  if (!motion.wallLeft() && (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map == 1))
  {
    node[X][Y].map = 11;
    motion.turnLeft();
    myDirection = tmpLeft;
    Serial.println("Turn Left");
    motion.moveForward();
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
  else if (!motion.wallFront() && (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map == 1))
  {
    node[X][Y].map = 44;
    motion.moveForward();
    Serial.println("Moving Forward");
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
  else if (!motion.wallRight() && (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map == 1))
  {
    node[X][Y].map = 22;
    motion.turnRight();
    Serial.println("Turn Right");
    myDirection = tmpRight;
    motion.moveForward();
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
  }
}


void TremauxAlgorithm() {
  //X= abs(X);
  //Y=abs(Y);

  if (Y == 7 && X == 1)
  {
    BUZZER_ON;
    delay(500);
    BUZZER_OFF;
  }
  //
  //  if ((X == 7 && Y == 7) || (X == 8 && Y == 8))
  //  {
  //    BUZZER_ON;
  //    delay(200);
  //    BUZZER_OFF;
  //    delay(200);
  //    BUZZER_ON;
  //    delay(200);
  //    BUZZER_OFF;
  //    delay(200);
  //    BUZZER_ON;
  //    delay(200);
  //    BUZZER_OFF;
  //  }


  if (node[X][Y].nodeDetails() == 10) {

    Direction tmp = (Direction)myDirection;
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    tmpRight = (Direction)RightOf(myDirection);
    myDirection = tmp;

    Serial.println("Dead End found");
    node[X][Y].map = 2;
    //    myDirection = OppositeOf(myDirection);
    myDirection = (Direction)RightOf(myDirection);
    turnRight();
    myDirection = (Direction)RightOf(myDirection);
    turnRight();
    X += RelativeX(myDirection);
    Y += RelativeY(myDirection);
    moveForward();
  }
  void wetRun()
  {
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    if (!motion.wallLeft() && (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map == 11))
    {
      motion.turnLeft();
      myDirection = tmpLeft;
      Serial.println("Turn Left");
      motion.moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
    else if (!motion.wallFront() && (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map == 44))
    {
      motion.moveForward();
      Serial.println("Moving Forward");
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
    else if (!motion.wallRight() && (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map == 22))
    {
      motion.turnRight();
      Serial.println("Turn Right");
      myDirection = tmpRight;
      motion.moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
  }

  void returnNow()
  {
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    if (!motion.wallLeft() && (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map == 1))
    {
      node[X][Y].map = 11;
      motion.turnLeft();
      myDirection = tmpLeft;
      Serial.println("Turn Left");
      motion.moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
    else if (!motion.wallFront() && (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map == 1))
    {
      node[X][Y].map = 44;
      motion.moveForward();
      Serial.println("Moving Forward");
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
    else if (!motion.wallRight() && (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map == 1))
    {
      node[X][Y].map = 22;
      motion.turnRight();
      Serial.println("Turn Right");
      myDirection = tmpRight;
      motion.moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
    }
  }

  else if (node[X][Y].nodeDetails() == 2) {
    Serial.println("PATH 2");

    Direction tmp = (Direction)myDirection;
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    tmpRight = (Direction)RightOf(myDirection);
    myDirection = tmp;

    if (!wallLeft()  && !wallRight())
    {
      //checking the weight of left and right
      if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].value <= node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].value)
      {
        if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map < 1 )
        {
          node[X][Y].map = 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map < 1 )
        {
          node[X][Y].map = 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
    }
    //for front and left
    if (!wallFront() && !wallLeft())
    {
      if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].value <= node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].value)
      {
        if (node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map < 1 )
        {
          node[X][Y].map = 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map < 1 )
        {
          node[X][Y].map = 1;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          turnLeft();
          myDirection = tmpLeft;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
    }
    //for front and right
    if (!wallFront() && !wallRight())
    {
      if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].value <= node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].value)
      {
        if (node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map < 1 )
        {
          node[X][Y].map = 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
      else
      {
        if (node[X + RelativeX(myDirection)][Y + RelativeY(myDirection)].map < 1 )
        {
          node[X][Y].map = 1;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
        else
        {
          node[X][Y].map = 1;
          turnRight();
          myDirection = tmpRight;
          moveForward();
          X += RelativeX(myDirection);
          Y += RelativeY(myDirection);
          return;
        }
      }
    }

  }//end of node.details == 2

  else //(node[X][Y].nodeDetails() == 1)
  {

    Direction tmp = (Direction)myDirection;
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    tmpRight = (Direction)RightOf(myDirection);
    myDirection = tmp;
    Serial.println("PATH 1");
    if (!wallLeft() && node[X + RelativeX(tmpLeft)][Y + RelativeY(tmpLeft)].map < 2) {
      node[X][Y].map += 1;
      turnLeft();
      myDirection = tmpLeft;
      moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
      return;
    }
    else if (!wallRight() && node[X + RelativeX(tmpRight)][Y + RelativeY(tmpRight)].map < 2) {
      node[X][Y].map += 1;
      turnRight();
      myDirection = tmpRight;
      moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
      return;
    }
    else if (!wallFront() && node[X + RelativeX(myDirection)][Y + RelativeX(myDirection)].map < 2) {
      Serial.println("moving Forward");
      node[X][Y].map += 1 ;
      moveForward();
      X += RelativeX(myDirection);
      Y += RelativeY(myDirection);
      return;
    }
  }
}
