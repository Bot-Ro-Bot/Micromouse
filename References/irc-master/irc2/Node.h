#ifndef NODE_H
#define NODE_H
class Node{
  public:
//    int id;
    bool visited;
    int child_node[4];    //0 left 1 right 2 up 3 down
    Node();
//    Node(int token);
//
//    int getX();
    void setChildNode(int, int );
//    int getY();
};

#endif
