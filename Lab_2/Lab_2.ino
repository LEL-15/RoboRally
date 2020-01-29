#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed() {
  // TODO
}


void updateOdometry() {
  // TODO
}

void displayOdometry() {
  // TODO
  sparki.print("X displacement: " 
  sparki.println(pose_x);)
  sparki.print("Y displacement: " 
  sparki.println(pose_y);)
  sparki.print("Z orientation " 
  sparki.println(pose_theta);)
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // TODO
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }


  delay(1000*CYCLE_TIME);
}
