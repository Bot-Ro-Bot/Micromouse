#pragma once

#include<wirish.h>

#define sizeX 16
#define sizeY 16


#define Direction int

const int North = 0, West = 1, South = 2, East = 3 ;

#define LeftOf(Dir)((Dir+1)%4)
#define RightOf(Dir)(Dir==North?East:(Dir-1))

#define OppositeOf(Dir) (Dir==North?South:Dir==South?North:Dir==East?West:East)
#define Decrement(Var) Var--;Var=(Var<1)? 1:Var;

#define RelativeX(Dir)(Dir==West?-1:(Dir==East?1:0))            //relative position for X-axis
#define RelativeY(Dir)(Dir==South?-1:(Dir==North?1:0))          //relative position for Y-axis

//enum Direction {
//  North = 0, West, South, East
//};

Direction myDirection = North;

class Node {
  public:
    uint8_t map; //visited values
    uint8_t value; //potential values

    Node();
    static uint8_t nodeDetails();
};

Node::Node() {
  map = 0;
  value = 0;
}

uint8_t Node::nodeDetails() {
  if (!wallLeft() && !wallFront() && !wallRight() ) {
    return 3;        //3-way
  }
  if ((!wallLeft() && !wallFront() ) || (!wallFront() && !wallRight()) || ( !wallRight() && !wallLeft() ) ) {
    return 2;//2way
  }
  if ((!wallLeft()) || !wallFront() || !wallRight()) {
    return 1;//1way
  }
  if (wallFront() && wallRight() && wallLeft()) {
    return 10;
  }
}
