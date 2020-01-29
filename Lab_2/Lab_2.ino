#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 500;
int lineLeft = 1000;
int lineCenter = 1000;
int lineRight = 1000;
float speed_motors = 2.899; //cm per sec
int right_move = 0;
int left_move = 0;

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
  
}

void displayOdometry() {
  // TODO
  sparki.print("X displacement: "); 
  sparki.println(pose_x);
  sparki.print("Y displacement: ");
  sparki.println(pose_y);
  sparki.print("Z orientation "); 
  sparki.println(pose_theta);
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
      start = millis();
      readSensors();
      right_move = 0;
      left_move = 0;
      if ( lineLeft < threshold ) // if line is below left line sensor
      {  
        sparki.moveLeft(); // turn left
        right_move = -1;
        left_move = 1;
      }
    
      else if ( lineRight < threshold ) // if line is below right line sensor
      {  
        sparki.moveRight(); // turn right
        left_move = -1;
        right_move = 1;
      }
    
      // if the center line sensor is the only one reading a line
      else if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
      {
        sparki.moveForward(); // move forward
        right_move = 1;
        left_move = 1;
      }
      updateOdometry();
      displayOdometry();
      end_loop = millis();
      time_loop = end_loop - start;
      delay (100-time_loop);
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }


  delay(1000*CYCLE_TIME);
}
