/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h>

#include <SnakeRobotShared.h>

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];

// Servo forward angles (use to correct inaccuracies in robot assembly)
const float forwardAngles[NUM_SERVOS] = {92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 91.0};
  
// Define variables
int forwardPin = 14;  // Remote control movement pins
int reversePin = 15;
int rightPin = 17;
int leftPin = 16;

int forwardVal = 0;  // Remote control variables
int reverseVal = 0;
int rightVal = 0;
int leftVal = 0;

float lag = (M_PI / 3.0); // Phase lag between segments
float frequency = 0.25; // Oscillation frequency of segments.
float maxAmplitude = 20.0; // Amplitude of the serpentine motion of the snake
float currentAmplitude = maxAmplitude;
float amplitudeSetpoint = maxAmplitude;
float amplitudeRampRate = 10.0;
int startPause = 5000;  // Delay time to position robot

// const float forwardAngle = 93.0; // Center servo angle for forward or reverse motion (without turning)
// float centerAngle = forwardAngle; // Center angle set point
// float currentCenterAngle = forwardAngle; // Current value of the center angle
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
float waveValue = 0.0; // Current base value for the wave generator (sine) function
bool runningWave = false,
     runningWavePrevious = runningWave;

bool runningTurn = false;
enum RobotTurnDirection {LEFT_TURN, RIGHT_TURN};
RobotTurnDirection currentTurnDirection = RobotTurnDirection::LEFT_TURN,
                   previousTurnDirection = RobotTurnDirection::LEFT_TURN;

const float VERTICAL_TURN_UP_ANGLE = -30.0,
            VERTICAL_TURN_DOWN_ANGLE = 30.0,
            HORIZONTAL_TURN_LEFT_ANGLE = -60.0,
            HORIZONTAL_TURN_RIGHT_ANGLE = 60.0;

// 260.0 max
float horizontalTurnRampRate = 20.0,
      verticalTurnRampRate = 20.0,
      horizontalResetRampRate = 100.0,
      verticalResetRampRate = 100.0; // Maximum rate at which turn offsets are ramped (degrees per second)
float horizontalTurnSetPoint = 0.0,
      horizontalTurnCurrentValue = horizontalTurnSetPoint,
      verticalTurnSetPoint = VERTICAL_TURN_UP_ANGLE,
      verticalTurnCurrentValue = verticalTurnSetPoint;

unsigned long currentTimeStamp,
              lastTimeStamp;

void advanceTurnSetPoints(RobotTurnDirection turnDirection, float& horizontalSetPoint, float& verticalSetPoint)
{
  if(turnDirection == RobotTurnDirection::LEFT_TURN)
  {
    if(verticalSetPoint == VERTICAL_TURN_DOWN_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
    {
      verticalSetPoint = VERTICAL_TURN_UP_ANGLE;
    }
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE || horizontalSetPoint == 0.0))
    {
      horizontalSetPoint = HORIZONTAL_TURN_RIGHT_ANGLE;
    }
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_RIGHT_ANGLE)
    {
      verticalSetPoint = VERTICAL_TURN_DOWN_ANGLE;
    }
    else
    {
      horizontalSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
    }
  }
  else
  {
    if(verticalSetPoint == VERTICAL_TURN_DOWN_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
    {
      horizontalSetPoint = HORIZONTAL_TURN_RIGHT_ANGLE;
    }
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
    {
      verticalSetPoint = VERTICAL_TURN_DOWN_ANGLE;
    }
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_RIGHT_ANGLE || horizontalSetPoint == 0.0))
    {
      horizontalSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
    }
    else
    {
      verticalSetPoint = VERTICAL_TURN_UP_ANGLE;
    }
  }
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

  float initialValues[NUM_SERVOS];
  int portNumbers[NUM_SERVOS];

  /* centerAngle = forwardAngle;
  currentCenterAngle = forwardAngle; */
  
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    portNumbers[i] = 13 - i;
  }

  initialValues[0] = forwardAngles[0] + horizontalTurnCurrentValue;
  initialValues[1] = forwardAngles[1] + verticalTurnCurrentValue;
  
  for(int i = 2; i < NUM_SERVOS; i++)
  {
    initialValues[i] = forwardAngles[i] + currentAmplitude * sin((i - 2) * lag);
  }
    // Serial.print("Started servo connected to " + String(13 - i) + " with " + String(centerAngle+amplitude*cos(i * lag)) + ".\n");

  // Pause before movement begins
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

    // Serial.println("Pin states: Fwd " + String(forwardVal) + " | Rev " + String(reverseVal) + " | Right " + String(rightVal) + " | Left " + String(leftVal));
    
    // Forward motion
    if (forwardVal == HIGH){
      runningWave = true;
      runningTurn = false;
      reverseDirection = false;
    }
    // Reverse motion
    else if (reverseVal == HIGH){
      runningWave = true;
      runningTurn = false;
      reverseDirection = true;
    }
    // Check if a turn is being made
    else
    {
      runningWave = false;

      // Right turn
      if (rightVal == HIGH){
        runningTurn = true;
        currentTurnDirection = RobotTurnDirection::RIGHT_TURN;
        // waveOffsetMultiplier = 1.0;
        // Serial.println("Turning right...");
      }
      // Left turn
      else if (leftVal == HIGH){
        runningTurn = true;
        
        currentTurnDirection = RobotTurnDirection::LEFT_TURN;
        // Serial.println("Turning left...");
        // waveOffsetMultiplier = 1.0;
      }
      // Straight movement
      else
      {
        runningTurn = false;
      }
    }

    // If the robot has just started running
    /* if(runningWave && !runningWavePrevious)
    {
      // Reset the last time stamp to the current millisecond timer value
      lastTimeStamp = millis();
    }

    runningWavePrevious = runningWave; */

    currentTimeStamp = millis();
    float elapsedTime = (currentTimeStamp - lastTimeStamp) / 1000.0;
    lastTimeStamp = currentTimeStamp;

    if(runningTurn || runningWave)
    {
      if(runningTurn)
      {
        amplitudeSetpoint = 0.0;
      }
      else
      {
        // Calculate the new base wave generator value
        waveValue += ((-2.0 * M_PI * frequency)* (elapsedTime * (reverseDirection ? -1.0 : 1.0)));

        amplitudeSetpoint = maxAmplitude;
      }
      
      float maxAmplitudeChange = amplitudeRampRate * elapsedTime;
      
      currentAmplitude = coerceToRange(currentAmplitude - maxAmplitudeChange, currentAmplitude + maxAmplitudeChange, amplitudeSetpoint);

      // Loop to update robot servos
      for(int i = 2; i < NUM_SERVOS; i++)
      {
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        robotServos[i].write(forwardAngles[i] + currentAmplitude * sin(waveValue + ((i - 2) * lag)));
      }
      
      float activeHorizontalRampRate = horizontalTurnRampRate,
            activeVerticalRampRate = verticalTurnRampRate;
      
      if(runningTurn)
      {
        if(currentAmplitude == 0.0)
        {
          if(((horizontalTurnCurrentValue == horizontalTurnSetPoint) && (verticalTurnCurrentValue == verticalTurnSetPoint))
            || (previousTurnDirection != currentTurnDirection))
          {
            advanceTurnSetPoints(currentTurnDirection, horizontalTurnSetPoint, verticalTurnSetPoint);
          }
        }
        else
        {
          if(currentTurnDirection == RobotTurnDirection::LEFT_TURN)
          {
            horizontalTurnSetPoint = HORIZONTAL_TURN_RIGHT_ANGLE;
          }
          else
          {
            horizontalTurnSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
          }
          
          verticalTurnSetPoint = VERTICAL_TURN_DOWN_ANGLE;
        }
  
        previousTurnDirection = currentTurnDirection;
      }
      else
      {
        horizontalTurnSetPoint = 0.0;
        verticalTurnSetPoint = VERTICAL_TURN_UP_ANGLE;

        activeHorizontalRampRate = horizontalResetRampRate;
        activeVerticalRampRate = verticalResetRampRate;
      }

      float maxHorizontalAngleChange = activeHorizontalRampRate * elapsedTime,
            maxVerticalAngleChange = activeVerticalRampRate * elapsedTime;

      horizontalTurnCurrentValue = coerceToRange(horizontalTurnCurrentValue - maxHorizontalAngleChange,
                                                 horizontalTurnCurrentValue + maxHorizontalAngleChange, horizontalTurnSetPoint);

      verticalTurnCurrentValue = coerceToRange(verticalTurnCurrentValue - maxVerticalAngleChange,
                                               verticalTurnCurrentValue + maxVerticalAngleChange, verticalTurnSetPoint);

      robotServos[0].write(forwardAngles[0] + horizontalTurnCurrentValue);
      robotServos[1].write(forwardAngles[1] + verticalTurnCurrentValue);

      // Serial.println("Wrote " + String(horizontalTurnCurrentValue) + " to horizontal servo.");
      // Serial.println("Wrote " + String(verticalTurnCurrentValue) + " to vertical servo.");
    }
}
