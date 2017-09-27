/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h> 

// Define servo objects for the snake segments
Servo s0;
Servo s1; 
Servo s2;
Servo s3;
Servo s4; 
Servo s5;
Servo s6;
Servo s7;
Servo s8;
Servo s9; 
Servo s10;
Servo s11;
Servo servo[12] = {s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11};


  
// Define variables
int forwardPin = 14;  // Remote control movement pins
int reversePin = 15;
int rightPin = 16;
int leftPin = 17;

int forwardVal = 0;  // Remote control variables
int reverseVal = 0;
int rightVal = 0;
int leftVal = 0;

int counter = 0; // Loop counter variable
float lag = .5236; // Phase lag between segments
int frequency = 1; // Oscillation frequency of segments.
int amplitude = 40; // Amplitude of the serpentine motion of the snake
int rightOffset = 5; // Right turn offset
int leftOffset = -5; // Left turn offset
int offset = 6; // Variable to correct servos that are not exactly centered
int delayTime = 5; // Delay between limb movements
int startPause = 5000;  // Delay time to position robot
int test = -3; // Test varialble takes values from -6 to +5

double segmentAngle[12];
  
void setup() 
{ 
// Set movement pins as inputs
  pinMode(forwardPin, INPUT);
  pinMode(reversePin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(leftPin, INPUT);
  
// Set movement pins to low
  digitalWrite(forwardPin, LOW);
  digitalWrite(reversePin, LOW);
  digitalWrite(rightPin, LOW);
  digitalWrite(leftPin, LOW);
  
// Attach segments to pins  
   for(int i=0; i<12; i++){
    servo[i].attach(i+2);
   }
 
// Put snake in starting position
    for(int i=0; i<12; i++){
      segmentAngle[i] = 93+amplitude*cos((5-i)*lag);
      servo[i].write(segmentAngle[i]);
    }
     
  delay(startPause);  // Pause to position robot
} 
  
  
void loop() 
{
//  Read movement pins
  forwardVal = digitalRead(forwardPin);
  reverseVal = digitalRead(reversePin);
  rightVal = digitalRead(rightPin);
  leftVal = digitalRead(leftPin);

  // Right turn
  if (rightVal == HIGH){
      delay(delayTime);
      for(int i=0; i<12; i++){
         segmentAngle[i] = 93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+(5-i)*lag);
         servo[i].write(segmentAngle[i]);
      }
      if(forwardVal == HIGH){
        counter++;
      }
      else if(reverseVal == HIGH){
        counter--;
      }      
  } 
  
  // Left turn
  else if (leftVal == HIGH){ 
      delay(delayTime);
      for(int i=0; i<12; i++){
         segmentAngle[i] = 93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+(5-i)*lag);
         servo[i].write(segmentAngle[i]);
      }
      if(forwardVal == HIGH){
        counter++;
      }
      else if(reverseVal == HIGH){
        counter--;
      }     
  } 
  else{
    // Forward motion
    if (forwardVal == HIGH){
        delay(delayTime);
        for(int i=0; i<12; i++){
           segmentAngle[i] = 93+amplitude*cos(frequency*counter*3.14159/180+(5-i)*lag);
           servo[i].write(segmentAngle[i]);
        }
        counter++;
    }
  
  // Reverse motion
    if (reverseVal == HIGH){
        delay(delayTime);
        for(int i=0; i<12; i++){
           segmentAngle[i] = 93+amplitude*cos(frequency*counter*3.14159/180+(5-i)*lag);
           servo[i].write(segmentAngle[i]);
        }
        counter--;    
    }
    
  }

  
  
}
