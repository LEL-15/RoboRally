#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define CYCLE_TIME .100
#define SPEED .02899
#define AXEL .084


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 500;
int lineLeft = 1000;
int lineCenter = 1000;
int lineRight = 1000;
//-1 if right, 1 if left, 0 if straight
int movement;
bool rst = false;

float pose_x = 0., pose_y = 0., pose_theta = 0;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  Serial.begin(9600);
}

void readSensors() {
  lineLeft = sparki.lineLeft();
  lineRight = sparki.lineRight();
  lineCenter = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed() {
  // TODO
  unsigned long t = millis();
  while(!(lineLeft < threshold && lineCenter < threshold && lineRight < threshold)){
    sparki.moveForward();
    readSensors();
  }
  //sparki.beep(); // beep!
  unsigned long t2 = millis();
  unsigned long t_diff = t2-t;
  sparki.moveStop();
  sparki.clearLCD();
  sparki.print("Time: ");
  sparki.println(t);
  sparki.updateLCD();
  Serial.println(t_diff);
}


void updateOdometry() {
  //10357
  // TODO
  if(rst)
  {
    pose_theta = 0;
    pose_x = 0;
    pose_y = 0;
    rst = false;
  }
  else
  {
    if(movement != 0)
    {
      pose_theta += (2*movement*SPEED*CYCLE_TIME)/AXEL;
    }
    else
    {
      pose_x += cos(pose_theta)*SPEED*CYCLE_TIME;
      pose_y += sin(pose_theta)*SPEED*CYCLE_TIME;
    }
  }
}

void displayOdometry() {
  // TODO
  Serial.print("X displacement: "); 
  Serial.println(pose_x);
  Serial.print("Y displacement: ");
  Serial.println(pose_y);
  Serial.print("Z orientation "); 
  Serial.println(pose_theta);
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  //Initialize variables for timing
  unsigned long start;
  unsigned long end_loop;
  unsigned long time_loop;
  //If doing real odometry or testing odometry
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      //Start time in loop
      readSensors();
      movement = 0;
      if ( (lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold) )
      {
        start = millis();
        sparki.moveForward();
        movement = 0;
        rst = true;
      }
      else if ( lineLeft < threshold ) // if line is below left line sensor
      {  
        start = millis();
        sparki.moveLeft(); // turn left
        movement = 1;
      }
    
      else if ( lineRight < threshold ) // if line is below right line sensor
      {  
        start = millis();
        sparki.moveRight(); // turn right
        movement = -1;
      }
    
      // if the center line sensor is the only one reading a line
      else if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
      {
        start = millis();
        sparki.moveForward(); // move forward
        movement = 0;
      }
      
      else
      {
        start = millis();
        sparki.moveForward();
        movement = 0;
      }
      end_loop = millis();
      time_loop = end_loop - start;
      delay (100-time_loop);
      sparki.moveStop();
      updateOdometry();
      displayOdometry();
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
}
