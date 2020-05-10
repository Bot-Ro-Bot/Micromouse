#include"API.h"
#define wallDistance 200
//front1 0
//front2 1
//right 3
//left 2
int distance[4]; 
bool API::wallFront()
{
  if (distance[0]>wallDistance && distance[1]>wallDistance) 
  return true;  
}

bool API::wallLeft()
{
  if (distance[2]>wallDistance) 
  return true;  
}

bool API::wallRight()
{
  if (distance[3]>wallDistance) 
  return true;  
}
