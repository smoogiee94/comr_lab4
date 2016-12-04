#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#include "graph.cpp"
#include <vector>
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#define TIMEFORGRID 3000 //originally 3000
#define TURNOFFSET 620
using namespace std;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int SFSensor = A0;  // Analog input pin that the potentiometer is attached to
const int SLSensor = A1;  // Analog input pin that the potentiometer is attached to
const int SRSensor = A2;  // Analog input pin that the potentiometer is attached to
const int LFSensor = A3;  // Analog input pin that the potentiometer is attached to

char prevTurn = ' ';
vector<int> pathPlanning(Graph* inputGraph, char dir, int startPt, int endPt);
vector<int> currPos;
int startLoc = 0;
int endLoc = 0;
char dir = 'E';
int menuItem = 0;

int previousGrid = 0;
float SFValue = 0;    
float SLValue = 0;    
float SRValue = 0;    
float LFValue = 0;   

int visited = 0;
int turns = 0;
int currTime = 0;
int x = 0;
int y = 0;

//      Graph* unknownMaze = new Graph();
Graph* maze3 = new Graph();      
char maze[4][4] = 
  {
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'}
  };
Servo LServo;
Servo RServo;
int previousTime = 0;
int setColorTime = 0;
int previousColorTime = 0;

//list for shortest path
list<int> shortestPath;

void setup() {
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  // initialize serial communications at 9600 bps:
  LServo.attach(2);
  RServo.attach(3);
  LServo.write(90);
  RServo.write(90);
  Serial.begin(9600);
  visited = 1;
  //setupMaze1();
  //setupMaze2();
  setupMaze3();
    for (int i = 0; i < 4; ++i){
    for (int j = 0; j < 4; ++j){
      int gridNumberReturned = cartesianToGrid(i, j);
      Serial.println(gridNumberReturned);
    }
  }

}




void loop() {

  lcd.setCursor(0, 0);
  uint8_t buttons = lcd.readButtons();
  
  if (buttons){
    delay(200);
    lcd.clear();


    //switch statements for updating information
    if (buttons & BUTTON_UP){
      if (menuItem == 0){
        ++startLoc;
      }
      if (menuItem == 1){
        switch(dir){
          case 'E':
            dir = 'W';
            break;
          case 'W':
            dir = 'N';
            break;
          case 'N':
            dir = 'S';
            break;
          case 'S':
            dir = 'E';
            break;
        }
      }
      if (menuItem == 2){
        ++endLoc;
      }
    }

    if (buttons & BUTTON_DOWN){
      if (menuItem == 0){
        --startLoc;
      }
      if (menuItem == 1){
        switch(dir){
          case 'E':
            dir = 'W';
            break;
          case 'W':
            dir = 'N';
            break;
          case 'N':
            dir = 'S';
            break;
          case 'S':
            dir = 'E';
            break;
        }
      }
      if (menuItem == 2){
        --endLoc;
      }
    }

    if (buttons & BUTTON_LEFT){
      --menuItem;
    }

    if (buttons & BUTTON_RIGHT){
      ++menuItem;
    }

    if (buttons & BUTTON_SELECT){
      if (menuItem == 3){
        previousGrid = startLoc;
        //int currLoc = startLoc;

        convertToCartesian(startLoc, &y, &x);
        maze[x][y] = 'O';

                    lcd.setCursor(0, 0);
  lcd.write(maze[3][0]);

  lcd.setCursor(1, 0);
  lcd.write(maze[3][1]);

  lcd.setCursor(2, 0);
  lcd.write(maze[3][2]);
  
  lcd.setCursor(3, 0);
  lcd.write(maze[3][3]);


  lcd.setCursor(0, 1);
  lcd.write(maze[2][0]);

  lcd.setCursor(1, 1);
  lcd.write(maze[2][1]);

  lcd.setCursor(2, 1);
  lcd.write(maze[2][2]);
  
  lcd.setCursor(3, 1);
  lcd.write(maze[2][3]);


  lcd.setCursor(5, 0);
  lcd.write(maze[1][0]);

  lcd.setCursor(6, 0);
  lcd.write(maze[1][1]);

  lcd.setCursor(7, 0);
  lcd.write(maze[1][2]);
  
  lcd.setCursor(8, 0);
  lcd.write(maze[1][3]);


  lcd.setCursor(5, 1);
  lcd.write(maze[0][0]);

  lcd.setCursor(6, 1);
  lcd.write(maze[0][1]);

  lcd.setCursor(7, 1);
  lcd.write(maze[0][2]);
  
  lcd.setCursor(8, 1);
  lcd.write(maze[0][3]);

  visited = 1;

  while(checkVisited() != 16){
  
  lcd.setCursor(10, 0);
  lcd.write(dir);
  lcd.setCursor(10, 1);
  lcd.print("  ");
  lcd.setCursor(10, 1);
  lcd.print(cartesianToGrid(y, x));

  setTimer();

  if (sensorRead("right") > 16){
    if (prevTurn != 'R'){
      updateDir('R');
      RServo.write(100);
      LServo.write(100);
      delay(TURNOFFSET);
      RServo.write(90);
      LServo.write(90);
      delay(200);
      prevTurn = 'R';
    }
    else{
          prevTurn = 'N';
    RServo.write(80);
    LServo.write(100);
    setTimer();
    previousTime = millis();
    while (getElapsedTime() < TIMEFORGRID){
      bfsWallFollowing(9, 5);
      setTimer();
    }
    switch(dir){
        case('N') : 
          if (x + 1 <= 3)
          modifyAndPrint(++x, y);
          break;
        case('S') : 
          if (x - 1 >= 0)
          modifyAndPrint(--x, y);
          break;
        case('W') :
          if (y - 1 >= 0)
          modifyAndPrint(x, --y);
          break;
        case('E') :
          if (y + 1 <= 3)
          modifyAndPrint(x, ++y);
          break;
    }
    }


  }

  else if (sensorRead("front") > 20){
    prevTurn = 'N';
    RServo.write(80);
    LServo.write(100);
    setTimer();
    previousTime = millis();
    while (getElapsedTime() < TIMEFORGRID){
      bfsWallFollowing(9, 5);
      setTimer();
    }
    switch(dir){
        case('N') : 
          if (x + 1 <= 3)
          modifyAndPrint(++x, y);
          break;
        case('S') : 
          if (x - 1 >= 0)
          modifyAndPrint(--x, y);
          break;
        case('W') :
          if (y - 1 >= 0)
          modifyAndPrint(x, --y);
          break;
        case('E') :
          if (y + 1 <= 3)
          modifyAndPrint(x, ++y);
          break;
    }
  }

  else if (sensorRead("left") > 16){
    updateDir('L');
    prevTurn = 'L';
    RServo.write(80);
    LServo.write(80);
    delay(TURNOFFSET);
    RServo.write(90);
    LServo.write(90);
    delay(200);
  }

  else {
    updateDir('U');
    prevTurn = 'U';
    RServo.write(80);
    LServo.write(80);
    delay(TURNOFFSET);
    delay(TURNOFFSET);
    RServo.write(90);
    LServo.write(90);
    delay(200);
  }
  RServo.write(90);
  LServo.write(90);
  delay(1000);
  }






  
  //-----------------
  // Mapping complete
  //-----------------
  int currLoc = cartesianToGrid(y, x);
  while(currLoc != startLoc){
    currLoc = cartesianToGrid(y, x);
    lcd.setCursor(10, 0);
  lcd.write(dir);
  lcd.setCursor(10, 1);
  lcd.print("  ");
  lcd.setCursor(10, 1);
  lcd.print(cartesianToGrid(y, x));

  setTimer();

  if (sensorRead("right") > 16){
    if (prevTurn != 'R'){
      updateDir('R');
      RServo.write(100);
      LServo.write(100);
      delay(TURNOFFSET);
      RServo.write(90);
      LServo.write(90);
      delay(200);
      prevTurn = 'R';
    }
    else{
          prevTurn = 'N';
    RServo.write(80);
    LServo.write(100);
    setTimer();
    previousTime = millis();
    while (getElapsedTime() < TIMEFORGRID){
      bfsWallFollowing(9, 5);
      setTimer();
    }
    switch(dir){
        case('N') : 
          if (x + 1 <= 3)
          modifyAndPrint(++x, y);
          break;
        case('S') : 
          if (x - 1 >= 0)
          modifyAndPrint(--x, y);
          break;
        case('W') :
          if (y - 1 >= 0)
          modifyAndPrint(x, --y);
          break;
        case('E') :
          if (y + 1 <= 3)
          modifyAndPrint(x, ++y);
          break;
    }
    }


  }

  else if (sensorRead("front") > 20){
    prevTurn = 'N';
    RServo.write(80);
    LServo.write(100);
    setTimer();
    previousTime = millis();
    while (getElapsedTime() < TIMEFORGRID){
      bfsWallFollowing(9, 5);
      setTimer();
    }
    switch(dir){
        case('N') : 
          if (x + 1 <= 3)
          modifyAndPrint(++x, y);
          break;
        case('S') : 
          if (x - 1 >= 0)
          modifyAndPrint(--x, y);
          break;
        case('W') :
          if (y - 1 >= 0)
          modifyAndPrint(x, --y);
          break;
        case('E') :
          if (y + 1 <= 3)
          modifyAndPrint(x, ++y);
          break;
    }
  }

  else if (sensorRead("left") > 16){
    updateDir('L');
    prevTurn = 'L';
    RServo.write(80);
    LServo.write(80);
    delay(TURNOFFSET);
    RServo.write(90);
    LServo.write(90);
    delay(200);
  }

  else {
    updateDir('U');
    prevTurn = 'U';
    RServo.write(80);
    LServo.write(80);
    delay(TURNOFFSET);
    delay(TURNOFFSET);
    RServo.write(90);
    LServo.write(90);
    delay(200);
  }
  RServo.write(90);
  LServo.write(90);
  delay(1000);
  }

  //At start location, traverse to goal
  vector<int> movesToGoal = pathPlanning(maze3, dir, startLoc, endLoc);
        


        int itr = 0;
        int moveToDo = movesToGoal.at(itr);
        
        maze[startLoc][endLoc] = 'O';
        currLoc = startLoc;
        setColorTime = millis();
        previousColorTime = millis();
        previousTime = millis();

        if (moveToDo == 1){
              LServo.write(100);
              RServo.write(80);
            }
            else if (moveToDo == 2){
              LServo.write(80);
              RServo.write(80);
              updateDir('L');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 3){
              LServo.write(100);
              RServo.write(100);
              updateDir('R');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 4){
              LServo.write(80);
              RServo.write(80);
              updateDir('U');
              delay(1100);
              subtractTimer(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }

            
        while (currLoc != endLoc){
          Serial.println(moveToDo);
          bfsWallFollowing(9, 5);
            
            setColorTime = millis();
            if (getElapsedTime() >= 500){
              lcd.setBacklight(WHITE);
              if (moveToDo == 1){
              LServo.write(100);
              RServo.write(80);
              bfsWallFollowing(9, 5);
            }
            else if (moveToDo == 2){
              LServo.write(80);
              RServo.write(80);
              updateDir('L');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
              bfsWallFollowing(9, 5);
            }
            else if (moveToDo == 3){
              LServo.write(100);
              RServo.write(100);
              updateDir('R');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
              bfsWallFollowing(9, 5);
            }
            else if (moveToDo == 4){
              LServo.write(80);
              RServo.write(80);
              updateDir('U');
              delay(1100);
              subtractTimer(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
              bfsWallFollowing(9, 5);
            }
            }
            setTimer();
  
            lcd.setCursor(10, 0);
            lcd.write(dir);
  
            lcd.setCursor(10, 1);
            lcd.print(y);
            lcd.setCursor(11, 1);
            lcd.write(',');
            lcd.setCursor(12, 1);
            lcd.print(x);

            

            if (getElapsedTime() >= TIMEFORGRID) //more than 3 seconds have passed, we are in new grid. decide what to do
            {
              
                setPreviousTimer();
                moveToDo = movesToGoal.at(++itr);
                switch(dir){
                  case('N') : 
                  lcd.setBacklight(BLUE);
                  previousColorTime = setColorTime;
                  if (x + 1 <= 3)
                    modifyAndPrint(++x, y);
                  break;
                  case('S') : 
                    lcd.setBacklight(YELLOW);
                    previousColorTime = setColorTime;
                    if (x - 1 >= 0)
                    modifyAndPrint(--x, y);
                    break;
                  case('W') :
                    lcd.setBacklight(GREEN);
                    previousColorTime = setColorTime;
                    if (y - 1 >= 0)
                      modifyAndPrint(x, --y);
                    break;
                  case('E') :
                    lcd.setBacklight(RED);
                    previousColorTime = setColorTime;
                    if (y + 1 <= 3)
                      modifyAndPrint(x, ++y);
                    break;
              }
            }

            //in a new grid 
            //closeLoopCtrlPart2(7, 5);
        }
        LServo.write(90);
        RServo.write(90);
      }
  
  
      }
    }
  


    //switch statements for displaying information


    switch(menuItem){
      case 0: 
        lcd.print("Starting Locat.");
        lcd.setCursor(0, 1);
        lcd.print(startLoc + 1);
        break;
      case 1:
        lcd.print("Orientation");
        lcd.setCursor(0, 1);
        switch(dir){
          case 'E':
            lcd.print("East");
            break;
          case 'W':
            lcd.print("West");
            break;
          case 'N':
            lcd.print("North");
            break;
          case 'S':
            lcd.print("South");
            break;
          default:
            lcd.print("WTF. How did I even get here");
            break;
        }
        break;
      case 2:
        lcd.print("Ending Locat.");
        lcd.setCursor(0, 1);
        lcd.print(endLoc + 1);
        break;
      case 3:
        lcd.print("Exit");
        break;
    }}











void closeLoopCtrlPart2(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (leftServoValue <= 93 && leftServoValue >= 87){
    if (rightServoValue <= 93 && rightServoValue >= 87){

      //180 degree
      if (sensorRead("left") < 11.5){
        switch(dir){
          case('N') : dir = 'S';
            break;
          case('S') : dir = 'N';
            break;
          case('E') : dir = 'W';
            break;
          case('W') : dir = 'E';
            break;
        }

        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(1100);
        subtractTimer(TURNOFFSET);
        subtractTimer(200);
        lcd.setBacklight(WHITE);
      }

      //Left turn
      else{
        switch(dir){
          case('N') : dir = 'W';
            break;
          case('S') : dir = 'E';
            break;
          case('E') : dir = 'N';
            break;
          case('W') : dir = 'S';
            break;
        }
        lcd.setBacklight(WHITE);
        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(550);
        subtractTimer(TURNOFFSET);
      }
    }
  }
  else if (ytr >= 10.0){


    switch(dir){
      case('N') : 
        dir = 'E';
        //++x;
        break;
      case('S') : 
        dir = 'W';
        //--x;
        break;
      case('E') : 
        dir = 'S';
        //++y;
        break;
      case('W') : 
        dir = 'N';
        //--y;
        break;
    }
    lcd.setBacklight(WHITE);
    //right turn
    LServo.write(100);
    RServo.write(80);
    delay(1250);
  lcd.setCursor(10, 0);
  lcd.write(dir);
  lcd.setCursor(10, 1);
  lcd.print("  ");
  lcd.setCursor(10, 1);
  lcd.print(cartesianToGrid(y, x));
    
    RServo.write(100);
    LServo.write(100);
    delay(550);
    RServo.write(80);
    LServo.write(100);
    delay(500);
    delay(500);
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}






int saturationFunctionRightLeft(double val){
  int returnVal = 0;
  if (returnVal - val <= -5){
    return -5;
  }

  else if (returnVal - val >= 5){
    return 5;
  }

  else
    return returnVal - val;
}








int saturationFunctionFront(double val){
  int returnVal = 90;
  if (returnVal - val <= 80){
    return 80;
  }
  else if (returnVal - val >= 100){
    return 100;
  }
  else
    return returnVal - val;
}


void bfsWallFollowing(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  Serial.println(ytr);
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (ytr >= 12){
    RServo.write(80);
    LServo.write(100);
    return;
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}







float sensorRead(String sensorDirection){
  float sensorValue = 0;
  if (sensorDirection.equals("front")){
    sensorValue = 12.509 * pow(analogRead(SFSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 59.635 * pow(analogRead(LFSensor)*0.0048875855327468, -1.034) / 2.54;
      if (sensorValue > 59){
        sensorValue = 59;
      }
    }
  }

  else if (sensorDirection.equals("left")){
    sensorValue = 12.509 * pow(analogRead(SLSensor)*0.0048875855327468, -1.059) / 2.54;
  }

   else if (sensorDirection.equals("right")){
    sensorValue = 12.509 * pow(analogRead(SRSensor)*0.0048875855327468, -1.059) / 2.54;
  }
  else{
    sensorValue = -1;
  }
  return sensorValue;
}


void modifyAndPrint(int x, int y){
/*  if (maze[x][y] == 'O'){
    return;
  }
  maze[x][y] = 'O';
  ++visited;
  return;
  */
   if (maze[x][y] == 'O'){
    return;
  }
  maze[x][y] = 'O';
  ++visited;
  if (x == 0 && y == 0){
    lcd.setCursor(5, 1);
  }
  if (x == 0 && y == 1){
    lcd.setCursor(6, 1);
  }
  if (x == 0 && y == 2){
    lcd.setCursor(7, 1);
  }
  if (x == 0 && y == 3){
    lcd.setCursor(8, 1);
  }


  if (x == 1 && y == 0){
    lcd.setCursor(5, 0);
  }
  if (x == 1 && y == 1){
    lcd.setCursor(6, 0);
  }
  if (x == 1 && y == 2){
    lcd.setCursor(7, 0);
  }
  if (x == 1 && y == 3){
    lcd.setCursor(8, 0);
  }




  if (x == 2 && y == 0){
    lcd.setCursor(0, 1);
  }
  if (x == 2 && y == 1){
    lcd.setCursor(1, 1);
  }
  if (x == 2 && y == 2){
    lcd.setCursor(2, 1);
  }
  if (x == 2 && y == 3){
    lcd.setCursor(3, 1);
  }


  if (x == 3 && y == 0){
    lcd.setCursor(0, 0);
  }
  if (x == 3 && y == 1){
    lcd.setCursor(1, 0);
  }
  if (x == 3 && y == 2){
    lcd.setCursor(2, 0);
  }
  if (x == 3 && y == 3){
    lcd.setCursor(3, 0);
  }


  lcd.print(maze[x][y]);
  return;
}

void setTimer(){
  currTime = millis();
}

void setPreviousTimer(){
  previousTime = currTime;
}
void subtractTimer(int x){
  previousTime += x;
}
int getElapsedTime(){
  return (currTime - previousTime);
}

int checkVisited(){
  int visitedNodes = 0;
  for (int i = 0; i < 4; ++i){
    for (int j = 0; j < 4; ++j){
      if (maze[i][j] == 'X') return 0;

      ++visitedNodes;
    }
  }
  return visitedNodes;
}

void getGrids(int gridNum, char dir, int* front, int* left, int* right, int* back){
      switch(dir){
        case 'N':
          *front = gridNum + 4;
          *back = gridNum - 4;
          *left = gridNum - 1;
          *right = gridNum + 1;
          break;
        case 'S':
          *front = gridNum - 4;
          *back = gridNum + 4;
          *left = gridNum + 1;
          *right = gridNum - 1;
          break;
        case 'E':
          *front = gridNum + 1;
          *back = gridNum - 1;
          *left = gridNum + 4;
          *right = gridNum - 4;
          break;
        case 'W':
          *front = gridNum - 1;
          *back = gridNum + 1;
          *left = gridNum - 4;
          *right = gridNum + 4;
          break;
      }
}

void convertToCartesian(int gridNum, int* x, int* y){
  switch(gridNum){
    case 0:
      *x = 0;
      *y = 0;
      break;
    case 1:
      *x = 1;
      *y = 0;
      break;
    case 2:
      *x = 2;
      *y = 0;
      break;
    case 3:
      *x = 3;
      *y = 0;
       break;

       
    case 4:
      *x = 0;
      *y = 1;
      break;
    case 5:
      *x = 1;
      *y = 1;
      break;
    case 6:
      *x = 2;
      *y = 1;
      break;
    case 7:
      *x = 3;
      *y = 1;
      break;


    case 8:
      *x = 0;
      *y = 2;
      break;
    case 9:
      *x = 1;
      *y = 2;
      break;
    case 10:
      *x = 2;
      *y = 2;
      break;
    case 11:
      *x = 3;
      *y = 2;
      break;


    case 12:
      *x = 0;
      *y = 3;
      break;
    case 13:
      *x = 1;
      *y = 3;
      break;
    case 14:
      *x = 2;
      *y = 3;
      break;
    case 15:
      *x = 3;
      *y = 3;
      break;
  }
}

vector<int> pathPlanning(Graph* inputGraph, char dir, int startPt, int endPt){
  Serial.println("In path planning");
  //---------------------------------
  //moves take place as follows
  // 1: forward
  // 2: turn left
  // 3: turn right
  // 4: turn 180
  shortestPath = inputGraph->BFS(startPt, endPt);
  vector<int> moves;
  list<int>::reverse_iterator i;
  list<int>::reverse_iterator target = shortestPath.rend();
  --target;
  //int x;
  //int y;

  int front;
  int back;
  int left;
  int right;
  i = shortestPath.rbegin();
  currPos.push_back(*i);
  for (i = shortestPath.rbegin(); i != target; ++i){
    getGrids(*i, dir, &front, &left, &right, &back);
    
    if (front == *(i + 1)){
      currPos.push_back(*(i + 1));
      moves.push_back(1);
    }
    if (left == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(2);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'N';
      }
      else if (dir == 'W'){
        dir = 'S';
      }
      else if (dir == 'N'){
        dir = 'W';
      }
      else
        dir = 'E';
    }
    if (right == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(3);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'S';
      }
      else if (dir == 'W'){
        dir = 'N';
      }
      else if (dir == 'N'){
        dir = 'E';
      }
      else
        dir = 'W';
    }
    if (back == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(4);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'W';
      }
      else if (dir == 'W'){
        dir = 'E';
      }
      else if (dir == 'N'){
        dir = 'S';
      }
      else
        dir = 'N';
    }
    //convertToCartesian(*i, y, x);
    //Serial.println(*x);
    //Serial.println(*y);
    //Serial.println();
    //Serial.println(*i);
    //convertToCartesian(*i, &y, &x);
    //Serial.print(x);
    //Serial.print(",");
    //Serial.print(y);
    //Serial.println();
    //Serial.println();
    
  }
  Serial.println("Done with path planning");
  return moves;
}

void updateDir(char turn){
  switch (dir){
    case 'N':
      if (turn == 'L'){
        dir = 'W';
      }
      if (turn == 'R'){
        dir = 'E';
      }
      if (turn == 'U'){
        dir = 'S';
      }
      break;
    case 'S':
      if (turn == 'L'){
        dir = 'E';
      }
      if (turn == 'R'){
        dir = 'W';
      }
      if (turn == 'U'){
        dir = 'N';
      }
      break;
    case 'E':
      if (turn == 'L'){
        dir = 'N';
      }
      if (turn == 'R'){
        dir = 'S';
      }
      if (turn == 'U'){
        dir = 'W';
      }
      break;
    case 'W':
      if (turn == 'L'){
        dir = 'S';
      }
      if (turn == 'R'){
        dir = 'N';
      }
      if (turn == 'U'){
        dir = 'E';
      }
      break;
  }
}



int cartesianToGrid(int x, int y){
  if (x == 0 && y == 0){
    return 0;
  }
  if (x == 1 && y == 0){
    return 1;
  }
  if (x == 2 && y == 0){
    return 2;
  }
  if (x == 3 && y == 0){
    return 3;
  }

  if (x == 0 && y == 1){
    return 4;
  }
  if (x == 1 && y == 1){
    return 5;
  }
  if (x == 2 && y == 1){
    return 6;
  }
  if (x == 3 && y == 1){
    return 7;
  }

  if (x == 0 && y == 2){
    return 8;
  }
  if (x == 1 && y == 2){
    return 9;
  }
  if (x == 2 && y == 2){
    return 10;
  }
  if (x == 3 && y == 2){
    return 11;
  }

  if (x == 0 && y == 3){
    return 12;
  }
  if (x == 1 && y == 3){
    return 13;
  }
  if (x == 2 && y == 3){
    return 14;
  }
  if (x == 3 && y == 3){
    return 15;
  }
}



int getPrevGrid(int gridNum){
  switch(dir){
    case 'N':
      return(gridNum - 4);
      break;
    case 'S':
      return(gridNum + 4);
      break;
    case 'W':
      return(gridNum + 1);
      break;
    case 'E':
      return(gridNum - 1);
      break;
  }
}





void setupMaze3(){
  //-----------------
  // First row verified
  //------------------
  maze3->addEdge(0, 1);
  maze3->addEdge(0, 4);

  maze3->addEdge(1, 0);
  maze3->addEdge(1, 2);

  maze3->addEdge(2, 1);

  maze3->addEdge(3, 7);

  //--------------------
  // Second row verified
  //---------------------
  maze3->addEdge(4, 0);
  maze3->addEdge(4, 8);

  maze3->addEdge(5, 6);

  maze3->addEdge(6, 5);
  maze3->addEdge(6, 10);
  maze3->addEdge(6, 7);

  maze3->addEdge(7, 6);
  maze3->addEdge(7, 3);
  maze3->addEdge(7, 11);

  //---------------------
  // Third row verified
  //---------------------
  maze3->addEdge(8, 4);
  maze3->addEdge(8, 9);
  maze3->addEdge(8, 12);

  maze3->addEdge(9, 8);
  maze3->addEdge(9, 10);

  maze3->addEdge(10, 9);
  maze3->addEdge(10, 6);

  maze3->addEdge(11, 15);
  maze3->addEdge(11, 7);

  //----------------------
  // fourth row verified
  //----------------------
  maze3->addEdge(12, 8);
  maze3->addEdge(12, 13);

  maze3->addEdge(13, 12);
  maze3->addEdge(13, 14);

  maze3->addEdge(14, 13);
  maze3->addEdge(14, 15);

  maze3->addEdge(15, 14);
  maze3->addEdge(15, 11);
  
}


//--------------------------------------
// Portion of code below this was for
// lab 4 part 1
//--------------------------------------
/*#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#include "graph.cpp"
#include <vector>
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#define TIMEFORGRID 3000
#define TURNOFFSET 600
using namespace std;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int SFSensor = A0;  // Analog input pin that the potentiometer is attached to
const int SLSensor = A1;  // Analog input pin that the potentiometer is attached to
const int SRSensor = A2;  // Analog input pin that the potentiometer is attached to
const int LFSensor = A3;  // Analog input pin that the potentiometer is attached to

vector<int> pathPlanning(Graph* inputGraph, char dir, int startPt, int endPt);
vector<int> currPos;
int startLoc = 0;
int endLoc = 0;
char dir = 'E';
int menuItem = 0;

float SFValue = 0;    
float SLValue = 0;    
float SRValue = 0;    
float LFValue = 0;   

int visited = 0;
int turns = 0;
int currTime = 0;
int x = 0;
int y = 0;
char maze[4][4] = 
  {
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'}
  };
Servo LServo;
Servo RServo;

int setColorTime = 0;
int previousColorTime = 0;
int previousTime = 0;

//list for shortest path
list<int> shortestPath;
Graph* maze1 = new Graph();
//Graph* maze2 = new Graph();
//Graph* maze3 = new Graph();
void setup() {
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  // initialize serial communications at 9600 bps:
  LServo.attach(2);
  RServo.attach(3);
  LServo.write(90);
  RServo.write(90);
  Serial.begin(9600);
  setupMaze1();
  visited = 1;
  //setupMaze2();
  //setupMaze3();
}




void loop() { 

  lcd.setCursor(0, 0);
  uint8_t buttons = lcd.readButtons();
  
  if (buttons){
    delay(200);
    lcd.clear();


    //switch statements for updating information
    if (buttons & BUTTON_UP){
      if (menuItem == 0){
        ++startLoc;
      }
      if (menuItem == 1){
        switch(dir){
          case 'E':
            dir = 'W';
            break;
          case 'W':
            dir = 'N';
            break;
          case 'N':
            dir = 'S';
            break;
          case 'S':
            dir = 'E';
            break;
        }
      }
      if (menuItem == 2){
        ++endLoc;
      }
    }

    if (buttons & BUTTON_DOWN){
      if (menuItem == 0){
        --startLoc;
      }
      if (menuItem == 1){
        switch(dir){
          case 'E':
            dir = 'W';
            break;
          case 'W':
            dir = 'N';
            break;
          case 'N':
            dir = 'S';
            break;
          case 'S':
            dir = 'E';
            break;
        }
      }
      if (menuItem == 2){
        --endLoc;
      }
    }

    if (buttons & BUTTON_LEFT){
      --menuItem;
    }

    if (buttons & BUTTON_RIGHT){
      ++menuItem;
    }

    if (buttons & BUTTON_SELECT){
      if (menuItem == 3){
        vector<int> movesToGoal = pathPlanning(maze1, dir, startLoc, endLoc);
        


        int itr = 0;
        int moveToDo = movesToGoal.at(itr);
        
        maze[startLoc][endLoc] = 'O';
        int currLoc = startLoc;
        setColorTime = millis();
        previousColorTime = millis();
        previousTime = millis();

        if (moveToDo == 1){
              LServo.write(100);
              RServo.write(80);
            }
            else if (moveToDo == 2){
              LServo.write(80);
              RServo.write(80);
              updateDir('L');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 3){
              LServo.write(100);
              RServo.write(100);
              updateDir('R');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 4){
              LServo.write(80);
              RServo.write(80);
              updateDir('U');
              delay(1100);
              subtractTimer(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }

            
        while (currLoc != endLoc){
          Serial.println(moveToDo);
          bfsWallFollowing(7, 5);
            
            setColorTime = millis();
            if (getElapsedTime() >= 500){
              lcd.setBacklight(WHITE);
              if (moveToDo == 1){
              LServo.write(100);
              RServo.write(80);
            }
            else if (moveToDo == 2){
              LServo.write(80);
              RServo.write(80);
              updateDir('L');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 3){
              LServo.write(100);
              RServo.write(100);
              updateDir('R');
              delay(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            else if (moveToDo == 4){
              LServo.write(80);
              RServo.write(80);
              updateDir('U');
              delay(1100);
              subtractTimer(TURNOFFSET);
              subtractTimer(TURNOFFSET);
              moveToDo = movesToGoal.at(++itr);
            }
            }
            setTimer();
  
            lcd.setCursor(10, 0);
            lcd.write(dir);
  
            lcd.setCursor(10, 1);
            lcd.print(y);
            lcd.setCursor(11, 1);
            lcd.write(',');
            lcd.setCursor(12, 1);
            lcd.print(x);

            

            if (getElapsedTime() >= TIMEFORGRID) //more than 3 seconds have passed, we are in new grid. decide what to do
            {
              
                setPreviousTimer();
                moveToDo = movesToGoal.at(++itr);
                switch(dir){
                  case('N') : 
                  lcd.setBacklight(BLUE);
                  previousColorTime = setColorTime;
                  if (x + 1 <= 3)
                    modifyAndPrint(++x, y);
                  break;
                  case('S') : 
                    lcd.setBacklight(YELLOW);
                    previousColorTime = setColorTime;
                    if (x - 1 >= 0)
                    modifyAndPrint(--x, y);
                    break;
                  case('W') :
                    lcd.setBacklight(GREEN);
                    previousColorTime = setColorTime;
                    if (y - 1 >= 0)
                      modifyAndPrint(x, --y);
                    break;
                  case('E') :
                    lcd.setBacklight(RED);
                    previousColorTime = setColorTime;
                    if (y + 1 <= 3)
                      modifyAndPrint(x, ++y);
                    break;
              }
            }

            //in a new grid 
            //closeLoopCtrlPart2(7, 5);
        }
        LServo.write(90);
        RServo.write(90);
      }
    }

    //switch statements for displaying information
    switch(menuItem){
      case 0: 
        lcd.print("Starting Locat.");
        lcd.setCursor(0, 1);
        lcd.print(startLoc + 1);
        break;
      case 1:
        lcd.print("Orientation");
        lcd.setCursor(0, 1);
        switch(dir){
          case 'E':
            lcd.print("East");
            break;
          case 'W':
            lcd.print("West");
            break;
          case 'N':
            lcd.print("North");
            break;
          case 'S':
            lcd.print("South");
            break;
          default:
            lcd.print("WTF. How did I even get here");
            break;
        }
        break;
      case 2:
        lcd.print("Ending Locat.");
        lcd.setCursor(0, 1);
        lcd.print(endLoc + 1);
        break;
      case 3:
        lcd.print("Exit");
        break;
    }
  }
}










void closeLoopCtrlPart2(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (leftServoValue <= 93 && leftServoValue >= 87){
    if (rightServoValue <= 93 && rightServoValue >= 87){

      //180 degree
      if (sensorRead("left") < 11.5){
        switch(dir){
          case('N') : dir = 'S';
            break;
          case('S') : dir = 'N';
            break;
          case('E') : dir = 'W';
            break;
          case('W') : dir = 'E';
            break;
        }

        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(1100);
        subtractTimer(TURNOFFSET);
        subtractTimer(TURNOFFSET);
        lcd.setBacklight(WHITE);
      }

      //Left turn
      else{
        switch(dir){
          case('N') : dir = 'W';
            break;
          case('S') : dir = 'E';
            break;
          case('E') : dir = 'N';
            break;
          case('W') : dir = 'S';
            break;
        }
        lcd.setBacklight(WHITE);
        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(TURNOFFSET);
        subtractTimer(TURNOFFSET);
      }
    }
  }
  else if (ytr >= 10.0){


    switch(dir){
      case('N') : 
        dir = 'E';
        //++x;
        break;
      case('S') : 
        dir = 'W';
        //--x;
        break;
      case('E') : 
        dir = 'S';
        //++y;
        break;
      case('W') : 
        dir = 'N';
        //--y;
        break;
    }
    lcd.setBacklight(WHITE);
    //right turn
    LServo.write(100);
    RServo.write(80);
    delay(1250);
    lcd.setCursor(10, 1);
    lcd.print(y);
    lcd.setCursor(11, 1);
    lcd.write(',');
    lcd.setCursor(12, 1);
    lcd.print(x);
    
    RServo.write(100);
    LServo.write(100);
    delay(TURNOFFSET);
    RServo.write(80);
    LServo.write(100);
    delay(500);
    delay(500);
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}






int saturationFunctionRightLeft(double val){
  int returnVal = 0;
  if (returnVal - val <= -5){
    return -5;
  }

  else if (returnVal - val >= 5){
    return 5;
  }

  else
    return returnVal - val;
}








int saturationFunctionFront(double val){
  int returnVal = 90;
  if (returnVal - val <= 80){
    return 80;
  }
  else if (returnVal - val >= 100){
    return 100;
  }
  else
    return returnVal - val;
}


void bfsWallFollowing(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (ytr >= 10.0){
    return;
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}







float sensorRead(String sensorDirection){
  float sensorValue = 0;
  if (sensorDirection.equals("front")){
    sensorValue = 12.509 * pow(analogRead(SFSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 59.635 * pow(analogRead(LFSensor)*0.0048875855327468, -1.034) / 2.54;
      if (sensorValue > 59){
        sensorValue = 59;
      }
    }
  }

  else if (sensorDirection.equals("left")){
    sensorValue = 12.509 * pow(analogRead(SLSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }

   else if (sensorDirection.equals("right")){
    sensorValue = 12.509 * pow(analogRead(SRSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }
  else{
    sensorValue = -1;
  }
  return sensorValue;
}


void modifyAndPrint(int x, int y){
  if (maze[x][y] == 'O'){
    return;
  }
  maze[x][y] = 'O';
  ++visited;
  return;
}

void setTimer(){
  currTime = millis();
}

void setPreviousTimer(){
  previousTime = currTime;
}
void subtractTimer(int x){
  previousTime += x;
}
int getElapsedTime(){
  return (currTime - previousTime);
}

int checkVisited(){
  int visitedNodes = 0;
  for (int i = 0; i < 4; ++i){
    for (int j = 0; j < 4; ++j){
      if (maze[i][j] == 'X') return 0;

      ++visitedNodes;
    }
  }
  return visitedNodes;
}

void getGrids(int gridNum, char dir, int* front, int* left, int* right, int* back){
      switch(dir){
        case 'N':
          *front = gridNum + 4;
          *back = gridNum - 4;
          *left = gridNum - 1;
          *right = gridNum + 1;
          break;
        case 'S':
          *front = gridNum - 4;
          *back = gridNum + 4;
          *left = gridNum + 1;
          *right = gridNum - 1;
          break;
        case 'E':
          *front = gridNum + 1;
          *back = gridNum - 1;
          *left = gridNum + 4;
          *right = gridNum - 4;
          break;
        case 'W':
          *front = gridNum - 1;
          *back = gridNum + 1;
          *left = gridNum - 4;
          *right = gridNum + 4;
          break;
      }
}

void convertToCartesian(int gridNum, int* x, int* y){
  switch(gridNum){
    case 0:
      *x = 0;
      *y = 0;
      break;
    case 1:
      *x = 1;
      *y = 0;
      break;
    case 2:
      *x = 2;
      *y = 0;
      break;
    case 3:
      *x = 3;
      *y = 0;
       break;

       
    case 4:
      *x = 0;
      *y = 1;
      break;
    case 5:
      *x = 1;
      *y = 1;
      break;
    case 6:
      *x = 2;
      *y = 1;
      break;
    case 7:
      *x = 3;
      *y = 1;
      break;


    case 8:
      *x = 0;
      *y = 2;
      break;
    case 9:
      *x = 1;
      *y = 2;
      break;
    case 10:
      *x = 2;
      *y = 2;
      break;
    case 11:
      *x = 3;
      *y = 2;
      break;


    case 12:
      *x = 0;
      *y = 3;
      break;
    case 13:
      *x = 1;
      *y = 3;
      break;
    case 14:
      *x = 2;
      *y = 3;
      break;
    case 15:
      *x = 3;
      *y = 3;
      break;
  }
}

vector<int> pathPlanning(Graph* inputGraph, char dir, int startPt, int endPt){

  //---------------------------------
  //moves take place as follows
  // 1: forward
  // 2: turn left
  // 3: turn right
  // 4: turn 180
  shortestPath = inputGraph->BFS(startPt, endPt);
  vector<int> moves;
  list<int>::reverse_iterator i;
  list<int>::reverse_iterator target = shortestPath.rend();
  --target;
  //int x;
  //int y;

  int front;
  int back;
  int left;
  int right;
  i = shortestPath.rbegin();
  currPos.push_back(*i);
  for (i = shortestPath.rbegin(); i != target; ++i){
    getGrids(*i, dir, &front, &left, &right, &back);
    
    if (front == *(i + 1)){
      currPos.push_back(*(i + 1));
      moves.push_back(1);
    }
    if (left == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(2);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'N';
      }
      else if (dir == 'W'){
        dir = 'S';
      }
      else if (dir == 'N'){
        dir = 'W';
      }
      else
        dir = 'E';
    }
    if (right == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(3);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'S';
      }
      else if (dir == 'W'){
        dir = 'N';
      }
      else if (dir == 'N'){
        dir = 'E';
      }
      else
        dir = 'W';
    }
    if (back == *(i + 1)){
      currPos.push_back(*(i));
      currPos.push_back(*(i + 1));
      moves.push_back(4);
      moves.push_back(1);
      if (dir == 'E'){
        dir = 'W';
      }
      else if (dir == 'W'){
        dir = 'E';
      }
      else if (dir == 'N'){
        dir = 'S';
      }
      else
        dir = 'N';
    }
    //convertToCartesian(*i, y, x);
    //Serial.println(*x);
    //Serial.println(*y);
    //Serial.println();
    //Serial.println(*i);
    //convertToCartesian(*i, &y, &x);
    //Serial.print(x);
    //Serial.print(",");
    //Serial.print(y);
    //Serial.println();
    //Serial.println();
    
  }
  return moves;
}

void updateDir(char turn){
  switch (dir){
    case 'N':
      if (turn == 'L'){
        dir = 'W';
      }
      if (turn == 'R'){
        dir = 'E';
      }
      if (turn == 'U'){
        dir = 'S';
      }
      break;
    case 'S':
      if (turn == 'L'){
        dir = 'E';
      }
      if (turn == 'R'){
        dir = 'W';
      }
      if (turn == 'U'){
        dir = 'N';
      }
      break;
    case 'E':
      if (turn == 'L'){
        dir = 'N';
      }
      if (turn == 'R'){
        dir = 'S';
      }
      if (turn == 'U'){
        dir = 'W';
      }
      break;
    case 'W':
      if (turn == 'L'){
        dir = 'S';
      }
      if (turn == 'R'){
        dir = 'N';
      }
      if (turn == 'U'){
        dir = 'E';
      }
      break;
  }
}


void setupMaze1(){
    //------------
  //first row verified
  //------------
  maze1->addEdge(0, 4);
  maze1->addEdge(0, 1);

  maze1->addEdge(1, 2);
  maze1->addEdge(1, 0);
  
  maze1->addEdge(2, 3);
  maze1->addEdge(2, 6);
  maze1->addEdge(2, 1);
  
  maze1->addEdge(3, 7);
  maze1->addEdge(3, 2);
  
  //---------------
  //second row verified
  //---------------
  maze1->addEdge(4, 0);
  maze1->addEdge(4, 8);
  
  maze1->addEdge(5, 9);
  maze1->addEdge(5, 6);
  
  maze1->addEdge(6, 2);
  maze1->addEdge(6, 10);
  
  maze1->addEdge(7, 3);
  maze1->addEdge(7, 11);
  
  //---------------
  //third row verified
  //---------------
  maze1->addEdge(8, 9);
  maze1->addEdge(8, 4);
  
  maze1->addEdge(9, 5);
  maze1->addEdge(9, 8);
  
  maze1->addEdge(10, 14);
  maze1->addEdge(10, 6);
  
  maze1->addEdge(11, 7);
  
  //------------
  //Fourth row  verified
  //------------
  maze1->addEdge(12, 13);
  
  maze1->addEdge(13, 12);
  maze1->addEdge(13, 14);
  
  maze1->addEdge(14, 13);
  maze1->addEdge(14, 10);
  maze1->addEdge(14, 15);
  
  maze1->addEdge(15, 14);
}


/*
void setupMaze2()
{

  //--------------------------
  // First row verified
  //--------------------------
  maze2->addEdge(0, 1);

  maze2->addEdge(1, 0);
  maze2->addEdge(1, 5);
  maze2->addEdge(1, 2);
  
  maze2->addEdge(2, 3);
  maze2->addEdge(2, 1);

  maze2->addEdge(3, 2);
  maze2->addEdge(3, 7);


  //-----------------------
  // Second row verified
  //-----------------------
  maze2->addEdge(4, 8);

  maze2->addEdge(5, 1);
  maze2->addEdge(5, 9);
  maze2->addEdge(5, 6);

  maze2->addEdge(6, 5);
  maze2->addEdge(6, 10);

  maze2->addEdge(7, 3);
  maze2->addEdge(7, 11);

  //--------------------------
  // Third row verified
  //--------------------------
  maze2->addEdge(8, 4);
  maze2->addEdge(8, 12);

  maze2->addEdge(9, 5);

  maze2->addEdge(10, 6);
  maze2->addEdge(10, 14);

  maze2->addEdge(11, 7);
  maze2->addEdge(11, 15);

  //-------------------------
  // fourth row verified
  //-------------------------

  maze2->addEdge(12, 8);
  maze2->addEdge(12, 13);

  maze2->addEdge(13, 12);
  maze2->addEdge(13, 14);

  maze2->addEdge(14, 13);
  maze2->addEdge(14, 10);
  maze2->addEdge(14, 15);

  maze2->addEdge(15, 14);
  maze2->addEdge(15, 11);
}

*/

/*

void setupMaze3(){
  //-----------------
  // First row verified
  //------------------
  maze3->addEdge(0, 1);
  maze3->addEdge(0, 4);

  maze3->addEdge(1, 0);
  maze3->addEdge(1, 2);

  maze3->addEdge(2, 1);

  maze3->addEdge(3, 7);

  //--------------------
  // Second row verified
  //---------------------
  maze3->addEdge(4, 0);
  maze3->addEdge(4, 8);

  maze3->addEdge(5, 6);

  maze3->addEdge(6, 5);
  maze3->addEdge(6, 10);
  maze3->addEdge(6, 7);

  maze3->addEdge(7, 6);
  maze3->addEdge(7, 3);
  maze3->addEdge(7, 11);

  //---------------------
  // Third row verified
  //---------------------
  maze3->addEdge(8, 4);
  maze3->addEdge(8, 9);
  maze3->addEdge(8, 12);

  maze3->addEdge(9, 8);
  maze3->addEdge(9, 10);

  maze3->addEdge(10, 9);
  maze3->addEdge(10, 6);

  maze3->addEdge(11, 15);
  maze3->addEdge(11, 7);

  //----------------------
  // fourth row verified
  //----------------------
  maze3->addEdge(12, 8);
  maze3->addEdge(12, 13);

  maze3->addEdge(13, 12);
  maze3->addEdge(13, 14);

  maze3->addEdge(14, 13);
  maze3->addEdge(14, 15);

  maze3->addEdge(15, 14);
  maze3->addEdge(15, 11);
  
}*/

