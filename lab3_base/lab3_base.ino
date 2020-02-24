#include <Sparki.h>
#include <math.h>
#include <stdlib.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .100 // Default 50ms cycle time
#define AXEL 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define SEC_PER_ROT 9.12
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1

float P1 = .1;
float P2 = .1;
float P3 = .1;
float phiLeft = 0;
float phiRight = 0;
float phiLeftRatio = 0;
float phiRightRatio = 0;
float RAD_PER_SEC;

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

//Values we use
bool rst = false;
int movement = 0;
float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  RAD_PER_SEC = pow(SEC_PER_ROT, -1) * 2 * M_PI;
  Serial.begin(9600);

  // Set test cases here!
  set_pose_destination(.25, 0, M_PI/2);
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry(int movement) {
  // TODO: Update pose_x, pose_y, pose_theta
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
      pose_theta += (2*movement*ROBOT_SPEED*CYCLE_TIME)/AXEL;
    }
    else
    {
      pose_x += cos(pose_theta)*ROBOT_SPEED*CYCLE_TIME;
      pose_y += sin(pose_theta)*ROBOT_SPEED*CYCLE_TIME;
    }
  }

  //Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

void updateOdometry3()
{
  float ratio = WHEEL_RADIUS/AXEL;
  //Old version
  //pose_theta += (phiRight*ratio - phiLeft*ratio);
  //pose_x += cos(pose_theta)*WHEEL_RADIUS*.5*(phiLeft+phiRight);
  //pose_y += sin(pose_theta)*WHEEL_RADIUS*.5*(phiLeft+phiRight);


  //Elly Effort
  //Radians wheel turn is equal to (max radians per second)*(percent max)*(seconds)
  pose_theta += ratio*RAD_PER_SEC*CYCLE_TIME*(phiRightRatio - phiLeftRatio);
  pose_x += cos(pose_theta)*WHEEL_RADIUS*.5*RAD_PER_SEC*CYCLE_TIME*(phiRightRatio + phiLeftRatio);
  pose_y += sin(pose_theta)*WHEEL_RADIUS*.5*RAD_PER_SEC*CYCLE_TIME*(phiRightRatio + phiLeftRatio);
  
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

float updateDistance(){
  return sqrt(pow(pose_x - dest_pose_x, 2)+ pow(pose_y - dest_pose_y, 2));
}

float updateBearing()
{
  return atan2((pose_y - dest_pose_y),(pose_x - dest_pose_x)) - dest_pose_theta;
}

float updateHeading()
{
  return (pose_theta - dest_pose_theta);
}

float feedbackDist()
{
  return P1*updateDistance();
}

float feedbackRot()
{
  return P2*updateBearing() + P3*updateHeading();
}

void updatePhi() 
{
  float changeX = feedbackDist();
  float changeTheta = feedbackRot();
  phiLeft = (2*changeX - changeTheta*AXEL)/(2*WHEEL_RADIUS);
  phiRight = (2*changeX + changeTheta*AXEL)/(2*WHEEL_RADIUS);
}
void displayOdometry() {
  
  Serial.print("X displacement: "); 
  Serial.println(pose_x);
  Serial.print("Y displacement: ");
  Serial.println(pose_y);
  Serial.print("Z orientation "); 
  Serial.println(pose_theta);
  Serial.print("Left Phi"); 
  Serial.println(phiLeft);
  Serial.print("Right Phi"); 
  Serial.println(phiRight);
  
  
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));
}

void loop() {
  unsigned long begin_time = 0;
  unsigned long delay_time;

  float distance = updateDistance();
  unsigned long start_time;
  unsigned long end_time;
 
  float goal_angle;

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      distance = updateDistance();
      while (distance > .05){
        goal_angle = atan2((dest_pose_y - pose_y) , (dest_pose_x - pose_x));
        while(abs(goal_angle - pose_theta) > M_PI/36){
          if(goal_angle < pose_theta){
            sparki.moveRight();
            start_time = millis();
            movement = -1;
          }
          else{
            sparki.moveLeft();
            movement = 1;
            start_time = millis();
          }
          end_time = millis();
          delay(100 - (end_time - start_time));
          sparki.moveStop();
          updateOdometry(movement);
          displayOdometry();
        }
        
        sparki.moveForward();
        delay(100);
        sparki.moveStop();
        updateOdometry(0);
        displayOdometry();
        distance = updateDistance();
      }
      while (abs(dest_pose_theta - pose_theta) > M_PI/36){
        if(dest_pose_theta < pose_theta){
            sparki.moveRight();
            start_time = millis();
            movement = -1;
         }
         else{
            sparki.moveLeft();
            movement = 1;
            start_time = millis();
          }
          end_time = millis();
          delay(100 - (end_time - start_time));
          sparki.moveStop();
          updateOdometry(movement);
          displayOdometry();
      }


      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));
      updatePhi();
      if(distance > .1)
      {
        P1 = 1;
      }
      else if (distance > .03)
      {
        P1 = .5;
      }
      else if(abs(pose_theta - dest_pose_theta) < M_PI/12){
        current_state = 4;
      }
      if (current_state == 3){
        updateOdometry3();
        displayOdometry();
        if(phiLeft >= 0 and phiRight > 0){
          if(phiLeft > phiRight){
            phiLeftRatio = 1*P1; 
            phiRightRatio = phiRight/phiLeft*P1;
          }
          else{
            phiRightRatio = 1*P1;
            phiLeftRatio = phiLeft/phiRight*P1;
          }
        }
        else{
          phiLeftRatio = 1;
          phiRightRatio = 1;
          if(phiLeftRatio < 0){
            phiLeftRatio = -1;
          }
          if(phiLeft < 0){
            phiRightRatio = -1;
          }
        }
        
      }
      start_time = millis();
      sparki.motorRotate(MOTOR_LEFT, DIR_CCW, phiLeftRatio);
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW, phiRightRatio);
      delay_time = end_time - begin_time;
      if(delay_time < 1000*CYCLE_TIME){
        delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
      }
      else{
        delay(10);
      }
      break;
    //Case when robot is at destination after Part3
    case 4:
      Serial.println("Sparki stopped."); 
      sparki.moveStop();
      sparki.motorStop(MOTOR_LEFT);
      sparki.motorStop(MOTOR_RIGHT);
      delay(10000);
      break;
  }
}
