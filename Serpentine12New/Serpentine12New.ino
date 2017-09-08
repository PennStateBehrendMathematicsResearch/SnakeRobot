/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h> 

// Define servo objects for the snake segments
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
Servo s12;
  
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
int amplitude = 45; // Amplitude of the serpentine motion of the snake
int rightOffset = 5; // Right turn offset
int leftOffset = -5; // Left turn offset
int offset = 6; // Variable to correct servos that are not exactly centered
int delayTime = 5; // Delay between limb movements
int startPause = 5000;  // Delay time to position robot
int test = -3; // Test varialble takes values from -6 to +5
  
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
   s1.attach(2);
   s2.attach(3);
   s3.attach(4);
   s4.attach(5);
   s5.attach(6);
   s6.attach(7);
   s7.attach(8);
   s8.attach(9);
   s9.attach(10);
   s10.attach(11);
   s11.attach(12);
   s12.attach(13);
 
// Put snake in starting position
   s1.write(93+amplitude*cos(5*lag));
   s2.write(93+amplitude*cos(4*lag)); 
   s3.write(93+amplitude*cos(3*lag));
   s4.write(93+amplitude*cos(2*lag));
   s5.write(93+amplitude*cos(1*lag));
   s6.write(93+amplitude*cos(0*lag));
   s7.write(93+amplitude*cos(-1*lag));
   s8.write(93+amplitude*cos(-2*lag));
   s9.write(93+amplitude*cos(-3*lag));
   s10.write(93+amplitude*cos(-4*lag));
   s11.write(93+amplitude*cos(-5*lag));
   s12.write(93+amplitude*cos(-6*lag));
 
   
  delay(startPause);  // Pause to position robot
} 
  
  
void loop() 
{
//  Read movement pins
  forwardVal = digitalRead(forwardPin);
  reverseVal = digitalRead(reversePin);
  rightVal = digitalRead(rightPin);
  leftVal = digitalRead(leftPin);
  
// Forward motion
  if (forwardVal == HIGH){
    for(counter = 0; counter < 360; counter += 1)  {
      delay(delayTime);
      s1.write(93+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }
  }

// Reverse motion
  if (reverseVal == HIGH){
    for(counter = 360; counter > 0; counter -= 1)  {
      delay(delayTime);
      s1.write(93+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }    
  }
  
// Right turn
  if (rightVal == HIGH){
// Ramp up turn offset
    for(counter = 0; counter < 10; counter += 1)  {
      delay(delayTime);
      s1.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+.1*counter*rightOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }  
// Continue right turn
    for(counter = 11; counter < 350; counter += 1)  {
      delay(delayTime);
      s1.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+rightOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }    
// Ramp down turn offset
    for(counter = 350; counter < 360; counter += 1)  {
      delay(delayTime);
      s1.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+.1*(360-counter)*rightOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    } 
  } 
  
// Left turn
  if (leftVal == HIGH){
// Ramp up turn offset
    for(counter = 0; counter < 10; counter += 1)  {
      delay(delayTime);
      s1.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+.1*counter*leftOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }  
// Continue left turn
    for(counter = 11; counter < 350; counter += 1)  {
      delay(delayTime);
      s1.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+leftOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    }    
// Ramp down turn offset
    for(counter = 350; counter < 360; counter += 1)  {
      delay(delayTime);
      s1.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+5*lag));
      s2.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+4*lag));
      s3.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+3*lag));
      s4.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+2*lag));
      s5.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+1*lag));
      s6.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180+0*lag));    
      s7.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-1*lag));
      s8.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-2*lag));    
      s9.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-3*lag));
      s10.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-4*lag));
      s11.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-5*lag));    
      s12.write(93+.1*(360-counter)*leftOffset+amplitude*cos(frequency*counter*3.14159/180-6*lag));     
    } 
  } 
}
