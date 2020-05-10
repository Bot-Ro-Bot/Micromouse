#include "Node.h"
#include "API.h"
#define SizeX 16
#define SizeY 16

enum Direction {
  North, West, South, East
} ;


#define LeftOf(Dir)((Dir+1)%4)
#define RightOf(Dir)(Dir==North?East:(Dir-1))
#define OppositeOf(Dir) (Dir==North?South:Dir==South?North:Dir==East?West:East)
#define Decrement(Var) Var--;Var=(Var<1)? 1:Var;

#define RelativeX(Dir)(Dir==West?-1:(Dir==East?1:0))            //relative position for X-axis
#define RelativeY(Dir)(Dir==South?-1:(Dir==North?1:0))          //relative position for Y-axis

int Map[SizeX][SizeY];
uint8_t row, col;
int X = 0, Y = 0;
Node node[SizeX][SizeY];
API motion;


void TremauxAlgorithm();
void initMap();
void potentialWall();

enum Direction myDirection = North;
//Direction tmpLef/t = (Direction)LeftOf(myDirection);
//Direction tmp/Right = (Direction)RightOf(myDirection);




void setup() {
  initMap();
  potentialWall();

}

void loop() {
  TremauxAlgorithm();
  if(Map[7][7] && Map[7][8] && Map[8][7] && Map [8][8] > 0) 
  {
    while(1);
  }

}

void initMap()
{
  for (row = 0; row < SizeX; row++)
  {
    for (col=0; col < SizeY; col++) 
    {
      Map[row][col] = 0;
    }
  }
}


void potentialWall()
{
  uint8_t i,j;
  for (i = 0; i < SizeX; i++) {for (j = 0; j < SizeY; j++) {node[i][j].value = 0;}}

  for(j=0;j<SizeY/2;j++){for(i=0;i<SizeX/2;i++){node[i][j].value = 14-i-j;//1st quadrat
  for(j=0;j<SizeY/2;j++){for(i=SizeX/2;i<SizeX;i++){node[i][j].value = i-j-1;//2nd quadrant
  for(j=SizeY/2;j<SizeY;j++){for(i=0;i<SizeX/2;i++){node[i][j].value = j-i-1;//4th quadrant
  for(j=SizeY/2;j<SizeY;j++){for(i=SizeX/2;i<SizeX;i++){node[i][j].value = i+j-16;//3rd quadrant 
}

void TremauxAlgorithm()
{
  if (node[X][Y].nodeDetails() == -1)
  {
    Serial.println("Dead End found"); 
    Map[X][Y]=2;
    myDirection=OppositeOf(myDirection);
    motion.turnRight();
    motion.turnRight();
    X +=(Direction)RelativeX(myDirection);
    Y +=(Direction)RelativeY(myDirection);
    if(!motion.wallFront()) motion.moveForward();  
  }

  else if (node[X][Y].nodeDetails() == 1)
  {
    Serial.println("PATH 1");

    if (!motion.wallLeft() && Map[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)]<2)
    {
      Map[X][Y]+=1;
      motion.turnLeft();   
      myDirection=tmpLeft;
      if (!motion.wallFront()) motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection);
      // return;
    }
    else if (!motion.wallRight() && Map[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)]<2)
    {
      Map[X][Y]+=1;
      motion.turnRight();
      myDirection=tmpRight;
      if (!motion.wallFront()) motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection); 
      
      // return;  
    }

    else if (!motion.wallFront() && Map[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeX(myDirection)]<2)
    {
      Serial.println("moving Forward");
      Map[X][Y]+=1 ;
      motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection); 
      // return;  
    }
    else
    {
      Map[X][Y]+=1 ;
      motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection); 
    }
  }
  else if (node[X][Y].nodeDetails()==2)
  {
    Serial.println("PATH 2");

    if (!motion.wallLeft() && ! motion.wallFront() && motion.wallRight())
    {
     Serial.println("front and left path");
     Direction tmpLeft = (Direction)LeftOf(myDirection);
      if (node[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)].value <= node[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)].value)
      {
        if (Map[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)]<1)
        {
          Serial.println("turned Left");
          Map[X][Y]=1 ;
          motion.turnLeft();
          myDirection=tmpLeft;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          if (!motion.wallFront()) motion.moveForward();
           
          return;
        }
        else
        {
          Map[X][Y]+=1 ;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          motion.moveForward();
          
          //return;
        }
      }
      else
      {
       if (Map[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)]<1)
        {
          Map[X][Y]+=1 ;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          motion.moveForward();
          
          
        }
        else
        {
          Serial.println("turned Left");
          Map[X][Y]=1 ;
          motion.turnLeft();
          myDirection=tmpLeft;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          if (!motion.wallFront()) motion.moveForward();
          
          return;
        } 
      }
    }
    

    if (!motion.wallRight() && ! motion.wallFront() && motion.wallLeft())
    {
      Serial.println("Forward and Right path "); 
      Direction tmpRight = (Direction)RightOf(myDirection);
      if (node[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)].value < node[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)].value)
      {
        if (Map[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)]<1)
        {
          Serial.println("turned Right");
          Map[X][Y]=1 ;
          motion.turnRight();
          myDirection=tmpRight;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          if (!motion.wallFront()) motion.moveForward();
           
        }
        else
        {
          Map[X][Y]+=1 ;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          motion.moveForward();
           
        }
      }
      else
      {
        if (Map[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)]<1)
        {
          
          Map[X][Y]+=1 ;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          motion.moveForward();
          
        }
        else
        {
          Serial.println("turned Right");
          Map[X][Y]=1 ;
          motion.turnRight();
          myDirection=tmpRight;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          if (!motion.wallFront()) motion.moveForward();
        }
      }
      
    }
     if (!motion.wallRight() && ! motion.wallLeft() && motion.wallFront())
    {
      Direction tmpLeft = (Direction)LeftOf(myDirection);
      Direction tmpRight = (Direction)RightOf(myDirection);
      Serial.println("left and right path");

      if (node[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)].value <= node[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)].value)
      {
        if (Map[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)]<1)
        {
          Map[X][Y]=1 ;
          motion.turnRight();
          Serial.println("turned Right");
          myDirection=tmpRight;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection); 
          if (!motion.wallFront()) motion.moveForward();
          
        }
        else
        {
          Map[X][Y]=1 ;
          Serial.println("turned Left");
          motion.turnLeft();
          myDirection=tmpLeft;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          if (!motion.wallFront()) motion.moveForward();
         
        }
      }
      else
      {
        if (Map[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)]<1)
        {
          Map[X][Y]=1 ;
          motion.turnLeft();
          Serial.println("turned Left");
          myDirection=tmpLeft;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          if (!motion.wallFront()) motion.moveForward();

        }
        else
        {
          Map[X][Y]=1 ;
          motion.turnRight();
          Serial.println("turned Right");
          myDirection=tmpRight;
          X+=(Direction)RelativeX(myDirection);
          Y+=(Direction)RelativeY(myDirection);
          if (!motion.wallFront()) motion.moveForward();
          
        }
      }
    }
  }
  else if (node[X][Y].nodeDetails()==3)
  {
    Direction tmpLeft = (Direction)LeftOf(myDirection);
    Direction tmpRight = (Direction)RightOf(myDirection);
    Serial.println("PATH 3"); 
    if((2*node[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)].value-(node[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)].value+node[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)].value))<0)
    {
      Serial.println("turned Left");
      Map[X][Y]=1 ;
      myDirection=tmpLeft;
      motion.turnLeft();
      if (!motion.wallFront()) motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection);
    }
    else if ((2*node[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)].value-(node[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)].value+node[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)].value))<0)
    {
      Map[X][Y]=1 ;
      if (!motion.wallFront()) motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection); 
    }

    else if ((2*node[X+(Direction)RelativeX(tmpRight)][Y+(Direction)RelativeY(tmpRight)].value-(node[X+(Direction)RelativeX(myDirection)][Y+(Direction)RelativeY(myDirection)].value+node[X+(Direction)RelativeX(tmpLeft)][Y+(Direction)RelativeY(tmpLeft)].value))<0)
    {
      Serial.println("turned Right");
      Map[X][Y]=1 ;
      myDirection=tmpRight;
      motion.turnRight();
      if (!motion.wallFront()) motion.moveForward();
      X+=(Direction)RelativeX(myDirection);
      Y+=(Direction)RelativeY(myDirection);
    }
   
  }
}
