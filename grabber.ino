#include <Sparki.h> // include the sparki library
int state = 0;

void setup()
{
    sparki.servo(SERVO_CENTER); // Center the Servo
    sparki.RGB(RGB_GREEN); // turn the light green
    delay(1000); 
}

void loop()
{
  int cm = sparki.ping(); // measures the distance with Sparki's eyes
  int threshold = 500;

  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor
  switch(state) {
   case 0  :
      sparki.moveRight(30); // rotate right 30 degrees
      if(cm != -1) // make sure its not too close or too far
      { 
          if(cm < 30) // if the distance measured is less than 20 centimeters
          {
              sparki.RGB(RGB_RED); // turn the light red
              sparki.beep(); // beep!
              state = 1;
              sparki.gripperOpen(10);
              delay(8000);
          }
      }
      break; /* optional */
  
   case 1  :
      sparki.moveForward(); // rotate right 30 degrees
      if(cm != -1) // make sure its not too close or too far
      { 
          if(cm < 7) // if the distance measured is less than 20 centimeters
          {
              sparki.RGB(RGB_BLUE); // turn the light red
              sparki.beep(); // beep!
              state = 2;
              sparki.moveStop();
              sparki.gripperClose(10);
              delay(8000);
              sparki.moveRight(180);
          }
      }
      break;
   case 2  :
      sparki.moveForward(); // rotate right 30 degrees
       // if the center line sensor is the only one reading a line
      if ( (lineCenter < threshold))
      {
          sparki.RGB(RGB_YELLOW); // turn the light red
          sparki.beep(); // beep!
          state = 3;
          sparki.moveStop();
      } 
      break;
    
    case 3  :
      if ( lineLeft < threshold ) // if line is below left line sensor
      {  
        sparki.moveLeft(); // turn left
      }
    
      if ( lineRight < threshold ) // if line is below right line sensor
      {  
        sparki.moveRight(); // turn right
      }
    
      // if the center line sensor is the only one reading a line
      if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
      {
        sparki.moveForward(); // move forward
      }  
    
      sparki.clearLCD(); // wipe the screen
    
      sparki.print("Line Left: "); // show left line sensor on screen
      sparki.println(lineLeft);
    
      sparki.print("Line Center: "); // show center line sensor on screen
      sparki.println(lineCenter);
    
      sparki.print("Line Right: "); // show right line sensor on screen
      sparki.println(lineRight);
    
      sparki.updateLCD(); // display all of the information written to the screen
      if(lineLeft < threshold && lineRight < threshold && lineCenter < threshold){
          sparki.RGB(RGB_GREEN); // turn the light red
          sparki.beep(); // beep!
          state = 4;
          sparki.moveStop();
          sparki.gripperOpen(10);
      }
      break;
  }
  delay(100); // wait 0.1 seconds (100 milliseconds)
}
