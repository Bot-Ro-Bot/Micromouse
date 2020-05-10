#include"Node.h"
#include"API.h"

API sensor;

int Node::nodeDetails()
{
    
  // infoByte=0;
 //infoByte |=0x01;
  // infoByte |=0x02;
  //infoByte |=0x03;
  
  if(!sensor.wallLeft() && !sensor.wallFront() && !sensor.wallRight() )
  {
      return 3;//infoByte |=0xF0;           //3-way 
      
  }
  if((!sensor.wallLeft() && !sensor.wallFront() ) || (!sensor.wallFront() && !sensor.wallRight()) || ( !sensor.wallRight() && !sensor.wallLeft() ) ) 
  {
      return 2;//infoByte |=0xA0;          //2way
      
  }
  if((!sensor.wallLeft()) || !sensor.wallFront() || !sensor.wallRight()) 
  {
      return 1;//infoByte |=0x50;         //1way
    
  }
  if (sensor.wallFront() && sensor.wallRight() && sensor.wallLeft())
  {
    return -1;//infoByte |=0x00;
    
  }
}
