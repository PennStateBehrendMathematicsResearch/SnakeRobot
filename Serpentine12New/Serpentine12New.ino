/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h>

#include "StatisticalFunctions.h"

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];
float servoAngles[NUM_SERVOS],
      calculatedXPositions[NUM_SERVOS + 1],
      calculatedYPositions[NUM_SERVOS + 1];

const float HEAD_LENGTH = 13.75,
            SEGMENT_LENGTH = 7.0,
            TAIL_LENGTH = 11.5;
  
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
float frequency = 0.1; // Oscillation frequency of segments.
int amplitude = 25; // Amplitude of the serpentine motion of the snake
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
int startPause = 5000;  // Delay time to position robot

float forwardAngle = 93.0; // Center servo angle for forward or reverse motion (without turning)
float centerAngle = forwardAngle; // Center angle set point
float currentCenterAngle = forwardAngle; // Current value of the center angle
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
float waveValue = 0.0; // Current base value for the wave generator (sine) function
bool runningWave = false,
     runningWavePrevious = runningWave;
float turnRampRate = 3; // Maximum rate at which turn offsets are ramped (degrees per second)
unsigned long currentTimeStamp,
              lastTimeStamp;

// This method returns the result of constraining value to the range
// between lowerBound and upperBound by returning lowerBound if value
// is less than lowerBound, upperBound if value is greater than
// upperBound, and value otherwise.
float coerceToRange(float lowerBound, float upperBound, float value)
{
  if(value > upperBound)
  {
    value = upperBound;
  }
  else if(value < lowerBound)
  {
    value = lowerBound;
  }

  return value;
}

void setup() 
{ 
  Serial.begin(115200);
  
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

  centerAngle = forwardAngle;
  currentCenterAngle = forwardAngle;
  
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    robotServos[i].attach(13 - i);
    robotServos[i].write(centerAngle+amplitude*sin(i * lag));
    // Serial.print("Started servo connected to " + String(13 - i) + " with " + String(centerAngle+amplitude*cos(i * lag)) + ".\n"); 
  }

  // Pause to position robot
  delay(startPause);
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
      runningWave = true;
      centerAngle = forwardAngle + rightOffset;
      // waveOffsetMultiplier = 1.0;
      // Serial.println("Turning right...");
    }
    // Left turn
    else if (leftVal == HIGH){
      runningWave = true;
      centerAngle = forwardAngle + leftOffset;
      // Serial.println("Turning left...");
      // waveOffsetMultiplier = 1.0;
    }
    // Straight movement
    else
    {
      centerAngle = forwardAngle;
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
      currentCenterAngle = coerceToRange(currentCenterAngle - maxAngleChange, currentCenterAngle + maxAngleChange, centerAngle);
      // Serial.println("Current center angle: " + String(currentCenterAngle));

      // Loop to update robot servos
      for(int i = 0; i < NUM_SERVOS; i++)
      {
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        servoAngles[i] = currentCenterAngle + amplitude*sin(waveValue + (i * lag));
        robotServos[i].write(servoAngles[i]);
      }

      float angleFromStart = 0.0;

      calculatedXPositions[0] = HEAD_LENGTH;
      calculatedYPositions[0] = 0.0;

      for(int i = 1; i < (NUM_SERVOS + 1); i++)
      {
        float currentLength = (i < NUM_SERVOS) ? SEGMENT_LENGTH : TAIL_LENGTH;
        
        angleFromStart += (servoAngles[i - 1] - forwardAngle);
        calculatedXPositions[i] = calculatedXPositions[i - 1] + currentLength * cos(M_PI * angleFromStart / 180.0);
        calculatedYPositions[i] = calculatedYPositions[i - 1] + currentLength * sin(M_PI * angleFromStart / 180.0);
      }

      float xRegression = StatisticalFunctions::simpleLeastSquaresLinearRegressionSlope(calculatedXPositions, NUM_SERVOS + 1);
      float yRegression = StatisticalFunctions::simpleLeastSquaresLinearRegressionSlope(calculatedYPositions, NUM_SERVOS + 1);

      float headAngle = 180.0 * atan2(yRegression, xRegression) / M_PI;

      Serial.print("Head angle: ");
      Serial.println(headAngle);
    }
}
