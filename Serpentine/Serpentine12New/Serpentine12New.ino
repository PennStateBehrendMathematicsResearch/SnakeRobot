/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h>

#include <SnakeRobotShared.h>

#define TWO_BUTTON_REVERSE_TURN_LAYOUT

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];

// Servo forward angles (use to correct inaccuracies in robot assembly)
float forwardAngles[NUM_SERVOS] = {92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 91.0};
  
// Define variables
int forwardPin = 14;  // Remote control movement pins
int reversePin = 15;
int rightPin = 17;
int leftPin = 16;

int forwardVal = 0;  // Remote control variables
int reverseVal = 0;
int rightVal = 0;
int leftVal = 0;

float lag = .5236; // Phase lag between segments
float frequency = 0.35; // Oscillation frequency of segments.
int amplitude = 35; // Amplitude of the serpentine motion of the snake
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
int startPause = 5000;  // Delay time to position robot

float turnSetpoint = 0.0; // Center angle set point
float currentTurnAngle = 0.0; // Current value of the center angle
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
float waveValue = 0.0; // Current base value for the wave generator (sine) function
bool runningWave = false,
     runningWavePrevious = runningWave;
float turnRampRate = 3.0; // Maximum rate at which turn offsets are ramped (degrees per second)
unsigned long currentTimeStamp,
              lastTimeStamp;

void setup() 
{ 
  // Serial.begin(115200);
  
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
  
  // Attach segment servos to pins and initialize them to their
  // starting positions

  int portNumbers[NUM_SERVOS];
  float initialValues[NUM_SERVOS];
  
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    portNumbers[i] = 13 - i;
    initialValues[i] = forwardAngles[i] + currentTurnAngle + amplitude * sin(i * lag);
    // Serial.print("Started servo connected to " + String(13 - i) + " with " + String(centerAngle+amplitude*cos(i * lag)) + ".\n"); 
  }

  // Pause to position robot
  delay(startPause);
  
  robotServoSetup(robotServos, portNumbers, initialValues, forwardAngles, NUM_SERVOS, 330, 45.0);
} 
  
  
void loop() 
{
  //  Read movement pins
    forwardVal = digitalRead(forwardPin);
    reverseVal = digitalRead(reversePin);
    rightVal = digitalRead(rightPin);
    leftVal = digitalRead(leftPin);

#ifdef TWO_BUTTON_REVERSE_TURN_LAYOUT
    if (reverseVal == HIGH)
    {
      runningWave = true;
      reverseDirection = true;
    }
    else
    {
      reverseDirection = false;
    }

    // Right turn
    if (rightVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = rightOffset;
    }
    // Left turn
    else if (leftVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = leftOffset;
    }
    // Straight movement
    else if (forwardVal == HIGH || reverseVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = 0.0;
    }
    else
    {
      runningWave = false;
    }
#else
    // Right turn
    if (rightVal == HIGH){
      runningWave = true;
      turnSetpoint = rightOffset;
      
      // waveOffsetMultiplier = 1.0;
      // Serial.println("Turning right...");
    }
    // Left turn
    else if (leftVal == HIGH){
      runningWave = true;
      turnSetpoint = leftOffset;
      
      // Serial.println("Turning left...");
      // waveOffsetMultiplier = 1.0;
    }
    // Straight movement
    else
    {
      turnSetpoint = 0.0;
    }
    
    // Forward motion
    if (forwardVal == HIGH){
      runningWave = true;
      reverseDirection = false;
    }
    // Reverse motion
    else if (reverseVal == HIGH){
      runningWave = true;
      reverseDirection = true;
    }
    // Stop if a turn is not being made
    else if(leftVal == LOW && rightVal == LOW)
    {
      runningWave = false;
    }
#endif

    // If the robot has just started running
    if(runningWave && !runningWavePrevious)
    {
      // Reset the last time stamp to the current millisecond timer value
      lastTimeStamp = millis();
    }

    runningWavePrevious = runningWave;

    // If the robot is running
    if(runningWave)
    {
      // Calculate the elapsed time 
      currentTimeStamp = millis();
      float elapsedTime = (currentTimeStamp - lastTimeStamp) / 1000.0;
      lastTimeStamp = currentTimeStamp;

      // Calculate the new base wave generator value
      waveValue += ((-2.0 * M_PI * frequency)* (elapsedTime * (reverseDirection ? -1.0 : 1.0)));

      // Calculate the maximum change in the current center angle
      float maxAngleChange = turnRampRate * elapsedTime;

      // Calculate the new center angle
      currentTurnAngle = coerceToRange(currentTurnAngle - maxAngleChange, currentTurnAngle + maxAngleChange, turnSetpoint);
      // Serial.println("Current center angle: " + String(currentCenterAngle));

      // Loop to update robot servos
      for(int i = 0; i < NUM_SERVOS; i++)
      {
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        robotServos[i].write(forwardAngles[i] + currentTurnAngle + amplitude*sin(waveValue + (i * lag)));
      }
    }
}
