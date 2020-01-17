#include "Node.h"

Node::Node(){
   for(int i =0; i < 4; i++){
    child_node[i] = -1;
  }
  };
//Node::Node(int token){
//  id = token;
//  for(int i =0; i < 4; i++){
//    child_node[i] = -1;
//  }
//}

//int Node::getX(){
//  return id % 10;
//}
//
//int Node::getY(){
//  return id / 10;
//}

void Node::setChildNode(int direction, int coordinate){
  child_node[direction] = coordinate;
}
  
