/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

#include <Servo.h>

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];
  
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
int amplitude = 20; // Amplitude of the serpentine motion of the snake
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
int startPause = 5000;  // Delay time to position robot

const float forwardAngle = 93.0; // Center servo angle for forward or reverse motion (without turning)
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

const float VERTICAL_TURN_UP_ANGLE = forwardAngle - 30.0,
            VERTICAL_TURN_DOWN_ANGLE = forwardAngle + 30.0,
            HORIZONTAL_TURN_LEFT_ANGLE = forwardAngle - 60.0,
            HORIZONTAL_TURN_RIGHT_ANGLE = forwardAngle + 60.0;

// 260.0 max
float horizontalTurnRampRate = 20.0,
      verticalTurnRampRate = 20.0,
      horizontalResetRampRate = 100.0,
      verticalResetRampRate = 100.0; // Maximum rate at which turn offsets are ramped (degrees per second)
float horizontalTurnSetPoint = forwardAngle,
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
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE || horizontalSetPoint == forwardAngle))
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
    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_RIGHT_ANGLE || horizontalSetPoint == forwardAngle))
    {
      horizontalSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
    }
    else
    {
      verticalSetPoint = VERTICAL_TURN_UP_ANGLE;
    }
  }
}

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

  /* centerAngle = forwardAngle;
  currentCenterAngle = forwardAngle; */
  
  for(int i = 0; i < (NUM_SERVOS); i++)
  {
    robotServos[i].attach(13 - i);

    if(i < (NUM_SERVOS - 2))
    {
      robotServos[i].write(forwardAngle + amplitude*sin(i * lag));
    }
    else if(i == NUM_SERVOS - 2)
    {
      robotServos[i].write(horizontalTurnCurrentValue);
    }
    else if(i == NUM_SERVOS - 1)
    {
      robotServos[i].write(verticalTurnCurrentValue);
    }
    // Serial.print("Started servo connected to " + String(13 - i) + " with " + String(centerAngle+amplitude*cos(i * lag)) + ".\n"); 
  }

  // Pause to position robot
  // delay(startPause);
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

    // If the robot is running
    if(runningWave)
    {
      
      // Calculate the elapsed time 
      // currentTimeStamp = millis();
      // float elapsedTime = (currentTimeStamp - lastTimeStamp) / 1000.0;
      // lastTimeStamp = currentTimeStamp;

      // Calculate the new base wave generator value
      waveValue += ((-2.0 * M_PI * frequency)* (elapsedTime * (reverseDirection ? -1.0 : 1.0)));

      // Calculate the maximum change in the current center angle
      // float maxAngleChange = turnRampRate * elapsedTime;

      // Calculate the new center angle
      // currentCenterAngle = coerceToRange(currentCenterAngle - maxAngleChange, currentCenterAngle + maxAngleChange, centerAngle);
      // Serial.println("Current center angle: " + String(currentCenterAngle));

      // Loop to update robot servos
      for(int i = 0; i < (NUM_SERVOS - 2); i++)
      {
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        robotServos[i].write(forwardAngle + amplitude*sin(waveValue + (i * lag)));
      }
    }

    if(runningTurn || runningWave)
    {
      float activeHorizontalRampRate = horizontalTurnRampRate,
            activeVerticalRampRate = verticalTurnRampRate;
      
      if(runningTurn)
      {
        if(((horizontalTurnCurrentValue == horizontalTurnSetPoint) && (verticalTurnCurrentValue == verticalTurnSetPoint))
            || (previousTurnDirection != currentTurnDirection))
        {
          advanceTurnSetPoints(currentTurnDirection, horizontalTurnSetPoint, verticalTurnSetPoint);
        }
  
        previousTurnDirection = currentTurnDirection;
      }
      else
      {
        horizontalTurnSetPoint = forwardAngle;
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

      robotServos[NUM_SERVOS - 2].write(horizontalTurnCurrentValue);
      robotServos[NUM_SERVOS - 1].write(verticalTurnCurrentValue);

      // Serial.println("Wrote " + String(horizontalTurnCurrentValue) + " to horizontal servo.");
      // Serial.println("Wrote " + String(verticalTurnCurrentValue) + " to vertical servo.");
    }
}
