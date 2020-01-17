#include<NewPing.h>
#include<PID_v1.h>
#include<Servo.h>
#include "Motor.h"
#include "Node.h"
#include "EEPROMAnything.h"

#define decreaseMaze(var) { var -= 1; var = ((var) < 1) ? 1 : (var); }

typedef enum {
    NORTH,
    WEST,
    SOUTH,
    EAST
    } Direction;


#define decreaseMaze(var) { var -= 1; var = ((var) < 1) ? 1 : (var); }

#define LeftOf(Dir)(Direction)(((int)Dir+1) % 4)      
#define RightOf(Dir)(Dir == NORTH ? EAST : (Direction)((int)Dir-1))   

#define RelPosX(Dir) (Dir == WEST ? -1 : (Dir == EAST ? 1 : 0))
#define RelPosY(Dir) (Dir == NORTH ? 1 : (Dir == SOUTH ? -1 : 0))

#define OppositeDir(Dir) (Dir == EAST ? WEST : Dir == WEST ? EAST : Dir == NORTH ? SOUTH : NORTH)

#define TRIG_PIN 42
#define ECHO_PIN 45
#define MAX_DIST 200

#define Kp 0.35//0.7
#define Kd 0.45//0.5//4
#define Ki 0//0.4//0.8

#define MAX_OUTPUT_LIMITS 30

#define X_SIZE 9
#define Y_SIZE 9

#define NEAR_WALL 2000

#define FORWARD_STEPS 43
#define ROTATION_LEFT_STEPS 19
#define ROTATION_RIGHT_STEPS 18
#define COMPLETE_ROTATE 35

#define INT_MAX 999
#define WEIGHT 1
#define END_NODE 8

#define CAL_SPEED 80


#define TONE_PIN 11
#define BOX_PIN 36
#define QR_BOX_PIN 37
#define QR_BOX_READ 38

#define QR_BOX_DIST 1800


struct configuration{
  int checkpoints = 0;
  int box_coordinates[5] = {-1};
  bool dry_run = true;
  Node mNodes[89];
} config;



Direction bot_dir = NORTH;

PID *pid;

NewPing *sonar[3];

Motor *motor;
Servo servo;

int back_sensor;
double sensor_values[3];
int maze[X_SIZE][Y_SIZE];
int sensor_value_l, sensor_value_r;
int wall_count,next_move;
double Input,Output;

int bot_path[81];
int priority; 

int bot_x_pos = 0;
int bot_y_pos = 0;

int present_node, next_node;

int bot_path_counter = 0;
int loop_count = 0;
int incremental_count = 0;

int shortest_path[89];

bool qr_present = false;
int next_check_point = 0;
int rel_x, rel_y;


Node *nodes[89];

int cal_data[5], cal_count;

bool wall[3];
bool first_maze_completion = false;
bool is_first_maze_completed = false;
bool box_present = false;
bool box_present_left = false;
bool box_present_right = false;
bool is_maze_completed = false;
bool save_check_point = false;

void setupSensor();
void setupPID();

void updateSensorData();

void control();
void initialize_map();
void initialize_nodes();

void sound();

int findPreviousMove(int,int);
void checkDistance();
bool isFirstMazeCompleted();
bool isMazeCompleted();

void solveMaze(int);
void configureMazePath(int*);
void updateChildNodes(int,int,int);
int minDistance(int,int);
int *findShortestPath(int,int);
int checkpoint = 0;
int total_sensor_value = 0;

int getX(int);
int getY(int);




void setup() {

  Serial.begin(9600);


  servo.attach(8);

  servo.write(90);

//  EEPROM_writeAnything(0,config);

  EEPROM_readAnything(0,config);

  
  pinMode(5, INPUT_PULLUP);
  pinMode(BOX_PIN, INPUT);
  pinMode(QR_BOX_PIN, INPUT);
  pinMode(QR_BOX_READ, OUTPUT);
  pinMode(TONE_PIN, OUTPUT);
  setupPID();
  initialize_map();

  if(config.dry_run)
    initialize_nodes();


  if(!config.dry_run){
    for(int i = 0; i < 89; i++){
      nodes[i] = &config.mNodes[i];
    }

    //define maze if not dry run;
    solveMaze(config.checkpoints);
  }

  

  motor = new Motor();

  for(uint8_t i = 0; i< 3; i++){
    sonar[i] = new NewPing(TRIG_PIN + i, ECHO_PIN + i, 100);
  }

  
  updateSensorData();  
  updateSensorData();  

  sound();

 
  
  while(digitalRead(5));
}

void seeData(){
   updateSensorData();
   Serial.println(sensor_values[0]);
   Serial.println(sensor_values[1]);
   Serial.println(sensor_values[2]);
   
   delay(200);
}

void loop1(){
  seeData();
}

void loop2(){
  motor->updatePWM(0);
  motor->moveForward(0);
}
void loop(){
  
 updateSensorData();

 if(motor->getState() == MotorState::FORWARD){ 

     if(next_node != 43 || present_node != 43)
        control();
     else
        motor->moveForward();
  }


 if(motor->getState() == MotorState::FORCE_STOP){


  

  //    if(!wall[0] || !wall[2] || wall[1]){
        motor->stopMovingForward();
        motor->setState(NULL_STATE);
        delay(200);
        checkDistance();
        
        motor->setState(NULL_STATE);
        delay(200);    
  //    }else{
        motor->setState(NULL_STATE);
//    }

 }



 if(motor->getState() != MotorState::NULL_STATE) return;

   if(wall[1]){
     
       box_present = digitalRead(BOX_PIN);
       qr_present = digitalRead(QR_BOX_PIN);
    
     }

 
 updateSensorData(); 

 wall_count = 3 - (int)wall[0] - (int)wall[1] - (int)wall[2];
 
 next_move = 4;
 
 maze[bot_x_pos][bot_y_pos] = wall_count;

 present_node = bot_x_pos + bot_y_pos * 10;


  if(config.dry_run){
    if(is_first_maze_completed){
      int *path = findShortestPath(present_node, 43); 
      configureMazePath(path);
      first_maze_completion = true;
      is_first_maze_completed = false;
      return;
    } 
  }

  if(!config.dry_run){
    if(present_node == next_check_point){
      config.checkpoints = config.checkpoints + 1;
      EEPROM_writeAnything(0,config);
      solveMaze(config.checkpoints);
    }
  }



  if(!first_maze_completion){


  if (!wall[0] || box_present_left || !config.dry_run) {
    rel_x = bot_x_pos+RelPosX(LeftOf(bot_dir));
    rel_y = bot_y_pos+RelPosY(LeftOf(bot_dir));

    
    updateChildNodes(present_node, rel_x + rel_y * 10, LeftOf(bot_dir));

    if (maze[rel_x][rel_y]==0)
    {  
      next_move = 0;
    } else if (maze[rel_x][rel_y]!=1 ){
      next_move = 0;
      decreaseMaze(maze[bot_x_pos][bot_y_pos]); 
    }else{
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);

    }
  }


  if (!wall[1] || !config.dry_run || box_present ) {  
    rel_x = bot_x_pos+RelPosX(bot_dir);
    rel_y = bot_y_pos+RelPosY(bot_dir);

    if(config.dry_run)
          
      if(wall[1] && box_present && present_node != 43){
  
          sound();
          if(qr_present){
            while(qr_present){
              digitalWrite(QR_BOX_READ, HIGH);
              changeDistanceForQr();
              qr_present = digitalRead(QR_BOX_PIN);
              digitalWrite(QR_BOX_READ, LOW);
              motor->setState(NULL_STATE);
            }
          }
        
          while(box_present){
            updateSensorData();
            box_present = wall[1];
          }
  
         if(config.dry_run){
            save_check_point = false;
            config.box_coordinates[checkpoint++] = rel_x + rel_y * 10;
            EEPROM_writeAnything(0,config);
          }
          
      }
    
    updateChildNodes(present_node, rel_x + rel_y * 10, bot_dir);

    if (maze[rel_x][rel_y]==0) {
      next_move = 1;
    } else if(maze[rel_x][rel_y]!=1){
      next_move = 1;
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);
     }else{
        decreaseMaze(maze[bot_x_pos][bot_y_pos]);
     }
    
  }
    
  if (!wall[2] || box_present_right || !config.dry_run) {    
    rel_x = bot_x_pos+RelPosX(RightOf(bot_dir));
    rel_y = bot_y_pos+RelPosY(RightOf(bot_dir));
    updateChildNodes(present_node, rel_x + rel_y * 10, RightOf(bot_dir));

    if(maze[rel_x][rel_y]==0)
    {
      next_move = 2;
    } else if(maze[rel_x][rel_y]!=1){
      next_move = 2;
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);
    }else{
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);
    }
  }

  } else {

   if (!wall[2] || box_present_right || !config.dry_run) {    
      rel_x = bot_x_pos+RelPosX(RightOf(bot_dir));
      rel_y = bot_y_pos+RelPosY(RightOf(bot_dir));
      updateChildNodes(present_node, rel_x + rel_y * 10, RightOf(bot_dir));
  
      if(maze[rel_x][rel_y]==0)
      {
        next_move = 2;
      } else if(maze[rel_x][rel_y]!=1){
        next_move = 2;
        decreaseMaze(maze[bot_x_pos][bot_y_pos]);
      }else{
        decreaseMaze(maze[bot_x_pos][bot_y_pos]);
      }
  }
   
  if (!wall[1] || !config.dry_run || box_present ) {  
    rel_x = bot_x_pos+RelPosX(bot_dir);
    rel_y = bot_y_pos+RelPosY(bot_dir);

      if(config.dry_run)
        if(wall[1] && box_present &&present_node != 43){
    
            sound();
            if(qr_present){
              while(qr_present){
                digitalWrite(QR_BOX_READ, HIGH);
                changeDistanceForQr();
                qr_present = digitalRead(QR_BOX_PIN);
                digitalWrite(QR_BOX_READ, LOW);
                motor->setState(NULL_STATE);
              }
            }
          
            while(box_present){
              updateSensorData();
              box_present = wall[1];
            }
    
            
           if(config.dry_run){
              save_check_point = false;
              config.box_coordinates[checkpoint++] = rel_x + rel_y * 10;
              EEPROM_writeAnything(0,config);
            }
            
        }
    
    
    updateChildNodes(present_node, rel_x + rel_y * 10, bot_dir);

    if (maze[rel_x][rel_y]==0) {
      next_move = 1;
    } else if(maze[rel_x][rel_y]!=1){
      next_move = 1;
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);
     }else{
        decreaseMaze(maze[bot_x_pos][bot_y_pos]);
     }
    
  }


 if (!wall[0] || box_present_left || !config.dry_run) {
    rel_x = bot_x_pos+RelPosX(LeftOf(bot_dir));
    rel_y = bot_y_pos+RelPosY(LeftOf(bot_dir));
    
    updateChildNodes(present_node, rel_x + rel_y * 10, LeftOf(bot_dir));

    if (maze[rel_x][rel_y]==0)
    {  
      next_move = 0;
    } else if (maze[rel_x][rel_y]!=1 ){
      next_move = 0;
      decreaseMaze(maze[bot_x_pos][bot_y_pos]); 
    }else{
      decreaseMaze(maze[bot_x_pos][bot_y_pos]);

    }
  }
  }


//   if(box_present){
//        sound();
//        if(qr_present){
//          while(qr_present){
//            digitalWrite(QR_BOX_READ, HIGH);
//            changeDistanceForQr();
//            qr_present = digitalRead(QR_BOX_PIN);
//            digitalWrite(QR_BOX_READ, LOW);
//            motor->setState(NULL_STATE);
//          }
//        }
//      
//        while(box_present){
//          updateSensorData();
//          box_present = wall[1];
//        }
//
//        if(config.dry_run){
//          checkpoint += 1;
//          config.box_coordinates[checkpoint] = next_node;
//          EEPROM_writeAnything(0,config);
//        }
//         //write checkpoint if necessary checkpoint
//        delay(200);
//        return;
//      }

   
   if(box_present){
        sound();
        if(qr_present){
          while(qr_present){
            digitalWrite(QR_BOX_READ, HIGH);
            changeDistanceForQr();
            qr_present = digitalRead(QR_BOX_PIN);
            digitalWrite(QR_BOX_READ, LOW);
            motor->setState(NULL_STATE);
          }
        }
      
        while(box_present){
          updateSensorData();
          box_present = wall[1];
        }
        save_check_point = true;
   }




    if(first_maze_completion && present_node == 43){
    box_present = true;
    qr_present = true;

    if(bot_x_pos+RelPosX(LeftOf(bot_dir)) == 4 && bot_y_pos+RelPosY(LeftOf(bot_dir)) == 4){
      next_move = 0;
    }else if(bot_x_pos+RelPosX(bot_dir) == 4 && bot_y_pos+RelPosY(bot_dir) == 4){
      next_move = 1;
    }else if(bot_x_pos+RelPosX(RightOf(bot_dir)) == 4 && bot_y_pos+RelPosY(RightOf(bot_dir)) == 4){
      next_move = 2;
      }
  }

  
    switch (next_move) {
      case 0: // go left
        delay(200);       
        if(box_present_left){
          box_present_left = false;
          box_present = true;
        }
        motor->rotateLeft(ROTATION_LEFT_STEPS);
        delay(200);
//        checkBackSensor();
        bot_dir = LeftOf(bot_dir); // change to new direction
        break ;
        
      case 2: // go right
        delay(200);

        if(box_present_right){
          box_present_right = false;
          box_present = true;
        }
        motor->rotateRight(ROTATION_RIGHT_STEPS);
        delay(200);
//        checkBackSensor();
        bot_dir =RightOf(bot_dir) ; // change to new direction
        break;
        
    }

   if(box_present){
        sound();
        if(qr_present){
          while(qr_present){
            digitalWrite(QR_BOX_READ, HIGH);
            changeDistanceForQr();
            qr_present = digitalRead(QR_BOX_PIN);
            digitalWrite(QR_BOX_READ, LOW);
            motor->setState(NULL_STATE);
          }
        }
      
        while(box_present){
          updateSensorData();
          box_present = wall[1];
        }
        save_check_point = true;
   }

    
    if (next_move != 4) { 
      
      bot_path[bot_path_counter++] = bot_dir;
      
      bot_x_pos += RelPosX(bot_dir); 
      bot_y_pos += RelPosY(bot_dir); 

      next_node = bot_x_pos + bot_y_pos *10;

      nodes[present_node]->setChildNode(bot_dir, next_node);
      nodes[next_node]->setChildNode(OppositeDir(bot_dir), present_node);

     if(!config.dry_run){
        if(next_node == next_check_point){
          box_present = true;
        }
     }

   if(box_present){
        sound();
        if(qr_present){
          while(qr_present){
            digitalWrite(QR_BOX_READ, HIGH);
            changeDistanceForQr();
            qr_present = digitalRead(QR_BOX_PIN);
            digitalWrite(QR_BOX_READ, LOW);
            motor->setState(NULL_STATE);
          }
        }
      
        while(box_present){
          updateSensorData();
          box_present = wall[1];
        }
        save_check_point = true;
   }



//     if(box_present){
//      sound();
//        if(qr_present){
//          while(qr_present){
//            digitalWrite(QR_BOX_READ, HIGH);
//            changeDistanceForQr();
//            qr_present = digitalRead(QR_BOX_PIN);
//            digitalWrite(QR_BOX_READ, LOW);
//            motor->setState(NULL_STATE);
//          }
//        }
//      
//        while(box_present){
//          updateSensorData();
//          box_present = wall[1];
//        }
//
//        if(config.dry_run){
//          checkpoint += 1;
//          config.box_coordinates[checkpoint] = next_node;
//          EEPROM_writeAnything(0,config);
//        }
//         //write checkpoint if necessary checkpoint
//        delay(200);
//      }


      if(save_check_point){
        if(config.dry_run){
          save_check_point = false;
          config.box_coordinates[checkpoint++] = next_node;
          EEPROM_writeAnything(0,config);
        }
        delay(100);
      }
      
      motor->updatePWM(0);
      motor->moveForward(FORWARD_STEPS);      
      return;
      
    }

    if(wall[0] && wall[1] && wall[2]){
      
        if(sensor_values[1] < QR_BOX_DIST){
          while(sensor_values[1] <= QR_BOX_DIST){
             motor->setPWM(CAL_SPEED);
             motor->moveBackward(500);
             updateSensorData();
          }
        }
        motor->stopMoving();

      servo.write(170);
      delay(1000);
      box_present_right = digitalRead(BOX_PIN);
      if(box_present_right){

        sound();  
        servo.write(90);
        
        while(sensor_values[1] >= 1090){
          motor->setPWM(CAL_SPEED); 
          motor->moveForward(500);
          updateSensorData();
        }
        motor->stopMoving();
        motor->setState(NULL_STATE);
        return;
      }

      servo.write(10);
      delay(1000);
      box_present_left = digitalRead(BOX_PIN);
      
      if(box_present_left){

        servo.write(90);
        sound();

        while(sensor_values[1] >= 1090){
            motor->setPWM(CAL_SPEED); 
            motor->moveForward(500);
            updateSensorData();
        }
        motor->stopMoving();
        motor->setState(NULL_STATE);

        return;
      }

     while(sensor_values[1] >= 1090){
          motor->setPWM(CAL_SPEED); 
          motor->moveForward(500);
          updateSensorData();
      }
      motor->stopMoving();
      
      motor->setState(NULL_STATE);
      servo.write(90); 
    }
    
    maze[bot_x_pos][bot_y_pos] = 1;   //visited the dead end
    
    if(!first_maze_completion){
      is_first_maze_completed = isFirstMazeCompleted();
      if(is_first_maze_completed) {
        first_maze_completion = true;
        motor->stopMoving();
        motor->setState(NULL_STATE);
        writeData();
        return;
      }
    }

    if(first_maze_completion){
      
      is_maze_completed = isMazeCompleted();
      
      if(is_maze_completed){
        motor->stopMoving();
        sound();
        delay(50);
        sound();
        sound();
        sound();
        config.dry_run = false;
        writeData();
        return;
      }
      
    }

    delay(200);

    motor->completeRotate(COMPLETE_ROTATE);

    bot_dir = OppositeDir(bot_dir);
    
    bot_path[bot_path_counter] = bot_dir; 

    
    delay(200);

    motor->setState(NULL_STATE);

    while ((maze[bot_x_pos][bot_y_pos] == 1) && (bot_path_counter >= 0)) {
      
      updateSensorData();
      
      digitalWrite(13, HIGH);

      if(motor->getState() == MotorState::FORWARD){
        control();
      }

       if(motor->getState() == MotorState::FORCE_STOP){
          motor->stopMovingForward();
          checkDistance();
          motor->setState(NULL_STATE);  
       }

       if(motor->getState() != MotorState::NULL_STATE) continue;


      int bot_move = findPreviousMove(bot_dir, bot_path[--bot_path_counter]);     

      switch (bot_move) {
        
        case 0: // go back right
          delay(200);
          motor->rotateRight(ROTATION_RIGHT_STEPS);
          delay(200);
          bot_dir = RightOf(bot_dir); // change to new direction
          break;

          
        case 2: // go back left
          delay(200);
          motor->rotateLeft(ROTATION_LEFT_STEPS);
          delay(200);
          bot_dir = LeftOf(bot_dir); // change to new direction
          break;
      }
      
//      move_forward();

      motor->updatePWM(0);
      motor->moveForward(FORWARD_STEPS);
      
      bot_x_pos += RelPosX(bot_dir);
      bot_y_pos += RelPosY(bot_dir); // change position
    }

    digitalWrite(13,LOW);

      
}

void writeData(){
    for(int i = 0; i < 89; i++){
       config.mNodes[i] = *nodes[i] ;
      }
    EEPROM_writeAnything(0, config);
}

void updateChildNodes(int pre, int nxt, int dir){
   nodes[pre]->setChildNode(dir, nxt);
//   nodes[next_node]->setChildNode(OppositeDir(bot_dir), pre);
}


void changeDistanceForQr(){
  while(digitalRead(QR_BOX_PIN)){
      if(sensor_values[1] < 2000){
//        while(sensor_values[1] <= 2000){
           motor->setPWM(CAL_SPEED);
           motor->moveBackward(500);
           updateSensorData();
//        }
      }
      motor->stopMoving();
  } 

  while(sensor_values[1] >= 1090){
      motor->setPWM(CAL_SPEED); 
      motor->moveForward(500);
      updateSensorData();
  }
  motor->stopMoving();
    
}


  
void updateSensorData(){

  for(uint8_t i = 0; i < 3; i++){
    sensor_values[i] = (sonar[i]->ping()  / 57.0) * 100;
  }
  
  for(uint8_t i = 0; i < 3; i ++){
    sensor_values[i] = sensor_values[i]  == 0 ? 60000 : sensor_values[i];
    wall[i] = sensor_values[i] < NEAR_WALL ? true : false;
  }
}


void setupPID(){
  pid = new PID(&Input, &Output, 0, Kp, Ki, Kd, REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetSampleTime(5);
  pid->SetOutputLimits(0-MAX_OUTPUT_LIMITS, MAX_OUTPUT_LIMITS);
}

void initialize_map(){
  for(uint8_t i = 0; i < X_SIZE; i++)
    for(uint8_t j = 0; j < Y_SIZE; j++){
      if( i == 4 && j != 4){ maze[i][j] = 1;}
      maze[i][j]=0;       //1 visited, 2,3 more position
    }
}
  

void initialize_nodes(){
  for(int i=0; i < 89; i++){
    nodes[i]  = new Node();
  }
}

void control(){
  
  sensor_value_l = sensor_values[0];
  sensor_value_r = sensor_values[2];

  if(sensor_value_l == 6000){
     for(uint8_t i = 0 ; i < 5; i++){
        updateSensorData();
        sensor_value_l = sensor_values[0];
        if(sensor_value_l != 6000) break;  
     }
  }
  
  if(sensor_value_r == 6000){
     for(uint8_t i = 0 ; i < 5; i++){
        updateSensorData();
        sensor_value_r = sensor_values[2];
        if(sensor_value_r != 6000) break;  
     }
  }

  if(wall[0] && wall[2]){

    Input = (sensor_value_l - sensor_value_r);
    pid->Compute();
    motor->updatePWM(Output);   

  }else if(!wall[0] && wall[2]){

    Input = 1090 - sensor_value_r;
    pid->Compute();
    motor->updatePWM(Output);
    
  }else if(wall[0] && !wall[2]){
  
    Input = sensor_value_l - 1090;
    pid->Compute();
    motor->updatePWM(Output);
  
  }else{
    motor->updatePWM(0);
  }
  
}


int findPreviousMove(int bot_pos, int prev_pos){
  if(RightOf(bot_pos) == prev_pos) return 2;
  if(LeftOf(bot_pos) == prev_pos) return 0;

  return 4;
}

bool isFirstMazeCompleted(){
  for(uint8_t col = 0; col < 4; col++){
      for(uint8_t row = 0; row  < 9; row++){
       if(maze[col][row] == 0) return false;
    }
  }
  return true;
}

bool isMazeCompleted(){
  for(uint8_t col = 0; col < 9; col++){
      for(uint8_t row = 0; row  < 9; row++){
       if(maze[col][row] == 0) return false;
    }
  }
  return true;  
}

void checkDistance(){
  
  for(uint8_t i = 0; i < 5; i++){
    updateSensorData();
  }
  
//  delay(200);
  if(sensor_values[1] < 2000){
      if(sensor_values[1] < 1090){
        while(sensor_values[1] <= 1090){
           motor->setPWM(CAL_SPEED);
           motor->moveBackward(500);
           updateSensorData();
        }
//        motor->stopMovingBackSlow();
      }else if(sensor_values[1] > 1090){
        while(sensor_values[1] >= 1090){
            motor->setPWM(CAL_SPEED); 
            motor->moveForward(500);
            updateSensorData();
        }
//        motor->stopMoving();
      }
      motor->stopMoving();

    }

}



void solveMaze(int checkpoints){
  int *path;

    switch(checkpoints){
      case 0:
        path = findShortestPath(0,config.box_coordinates[checkpoints]);
        configureMazePath(path);
        break;
      case 1:
        path = findShortestPath(present_node ,config.box_coordinates[checkpoints]);
        configureMazePath(path);
        break;
      case 2:
        path = findShortestPath(present_node,config.box_coordinates[checkpoints]);
        configureMazePath(path);
        break;
      case 3:
        path = findShortestPath(present_node,config.box_coordinates[checkpoints]);
        configureMazePath(path);
        break;
      case 4:
        path = findShortestPath(present_node,config.box_coordinates[checkpoints]);
        configureMazePath(path);
        break;
      case 5:
        path = findShortestPath(present_node, END_NODE);
        configureMazePath(path);
        break;
    }
}


void sound(){
    digitalWrite(TONE_PIN, HIGH);
    delay(50);
    digitalWrite(TONE_PIN, LOW);
}


void configureMazePath(int *solved_path){
  int count = 0;

  if(!is_first_maze_completed){
    for(int i = 0; i < 9; i++){
      for(int j = 0;  j < 9; j++){
        maze[i][j] = 1;
      }
    }
  }
  else if(is_first_maze_completed){
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 9; j++){
        maze[i][j] = 1;
      }
    }
  }

  next_check_point = solved_path[count];
  
  while(solved_path[count] != -1){
//    Node *node = nodes[solved_path[count]];
//    maze[node->getX()][node->getY()] = 0;
    maze[getX(solved_path[count])][getY(solved_path[count])] = 0;
    count++;
  }  

  
}


int getX(int c){
    return c % 10;
}

int getY(int c){
    return c / 10;
}


int minDistance(int dist[], bool sptSet[]) 
{ 
   // Initialize min value 
   int min = INT_MAX, min_index; 
   
   for (int v = 0; v < 81; v++) 
     if (sptSet[v] == false && dist[v] <= min) 
         min = dist[v], min_index = v; 
   
   return min_index; 
}

int *findShortestPath(int from, int to){

  
  int dist[89];
  bool visited[89];
  int via[89];
  
   for (int i = 0; i < 89; i++){ 
      dist[i] = INT_MAX, visited[i] = false;
      shortest_path[i] = -1;
      via[i] = -1;
   } 
  
   dist[from] = 0;
   via[from] = from;
  
   for(int count  = 0; count < 89; count++){
      int minIndex = minDistance(dist, visited);
      visited[minIndex] = true;
  
      if(to == minIndex) break;
  
      for(int ind = 0; ind < 4; ind++){
          int child_index = nodes[minIndex]->child_node[ind];
      
          if(child_index < 0) continue;
          
          if(visited[child_index]) continue;
                                     
          if((dist[minIndex] + WEIGHT) < dist[child_index]){
              dist[child_index] = dist[minIndex] + WEIGHT;
              via[child_index] = minIndex;
          }
      }
  }
  int last_index = to;
  int cnt = 0;
  for(cnt; cnt < 89; cnt ++){
      shortest_path[cnt] = last_index;
      int last = via[last_index];
      if(last ==from)break;
      last_index = last;
  }
  
  shortest_path[cnt + 1] = from;
  
  return shortest_path;
}
