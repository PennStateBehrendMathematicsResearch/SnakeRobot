/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/
#include <Servo.h>

#include <SnakeRobotShared.h>

#include <ConcertinaMotion.h>

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];

// Servo forward angles (use to correct inaccuracies in robot assembly)
const float forwardAngles[NUM_SERVOS] = {92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 91.0};
  
// Define variables
// Remote control movement pins
int forwardPin = 17; // DIO pin corresponding to 'A'
int reversePin = 16; // DIO pin corresponding to 'B'
int leftPin = 15;    // DIO pin corresponding to 'C'
int rightPin = 14;   // DIO pin corresponding to 'D'

int forwardVal = 0;  // Remote control variables
int reverseVal = 0;
int rightVal = 0;
int leftVal = 0;

float lag = (M_PI / 6.0); // Phase lag between segments
float totalPeriod = 8.0; // Oscillation frequency of segments.
float minAmplitude = 20; // Amplitude of the serpentine motion of the snake
float maxAmplitude = 60;

float minSinePower = 1.0;
float maxSinePower = 2.0;

float turnRampRate = 3.0;
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
bool runningWave = false,
     runningWavePrevious = runningWave;

ConcertinaMotionGenerator motionGenerator(NUM_SERVOS, forwardAngles, 0.0);

//bool runningTurn = false;
//enum RobotTurnDirection {LEFT_TURN, RIGHT_TURN};
//RobotTurnDirection currentTurnDirection = RobotTurnDirection::LEFT_TURN,
//                   previousTurnDirection = RobotTurnDirection::LEFT_TURN;

float compressExpandMovementPortion = 0.4,
      anchorReleaseMovementPortion = 0.1;

// float compressionValue = 0.0;

const float FRONT_ANCHOR_TURN_DOWN_ANGLE = 20.0,
            FRONT_ANCHOR_TURN_UP_ANGLE = -25.0,
            REAR_ANCHOR_TURN_UP_ANGLE = 30.0,
            REAR_ANCHOR_TURN_DOWN_ANGLE = -25.0;

const float HEAD_LENGTH = 13.5,
            LINK_LENGTH = 7.0,
            TAIL_LENGTH = 11.5;

float servoValueBuffer[NUM_SERVOS];

// float currentAnchorAngle = ANCHOR_TURN_UP_ANGLE;
// float anchorSetPointAngle = ANCHOR_TURN_UP_ANGLE;
// float anchorTurnRampRate = 20.0;

//const float VERTICAL_TURN_UP_ANGLE = forwardAngle - 30.0,
//            VERTICAL_TURN_DOWN_ANGLE = forwardAngle + 30.0,
//            HORIZONTAL_TURN_LEFT_ANGLE = forwardAngle - 60.0,
//            HORIZONTAL_TURN_RIGHT_ANGLE = forwardAngle + 60.0;
//
//// 260.0 max
//float horizontalTurnRampRate = 20.0,
//      verticalTurnRampRate = 20.0,
//      horizontalResetRampRate = 100.0,
//      verticalResetRampRate = 100.0; // Maximum rate at which turn offsets are ramped (degrees per second)
//float horizontalTurnSetPoint = forwardAngle,
//      horizontalTurnCurrentValue = horizontalTurnSetPoint,
//      verticalTurnSetPoint = VERTICAL_TURN_UP_ANGLE,
//      verticalTurnCurrentValue = verticalTurnSetPoint;

unsigned long currentTimeStamp,
              lastTimeStamp;

//void advanceTurnSetPoints(RobotTurnDirection turnDirection, float& horizontalSetPoint, float& verticalSetPoint)
//{
//  if(turnDirection == RobotTurnDirection::LEFT_TURN)
//  {
//    if(verticalSetPoint == VERTICAL_TURN_DOWN_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
//    {
//      verticalSetPoint = VERTICAL_TURN_UP_ANGLE;
//    }
//    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE || horizontalSetPoint == forwardAngle))
//    {
//      horizontalSetPoint = HORIZONTAL_TURN_RIGHT_ANGLE;
//    }
//    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_RIGHT_ANGLE)
//    {
//      verticalSetPoint = VERTICAL_TURN_DOWN_ANGLE;
//    }
//    else
//    {
//      horizontalSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
//    }
//  }
//  else
//  {
//    if(verticalSetPoint == VERTICAL_TURN_DOWN_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
//    {
//      horizontalSetPoint = HORIZONTAL_TURN_RIGHT_ANGLE;
//    }
//    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && horizontalSetPoint == HORIZONTAL_TURN_LEFT_ANGLE)
//    {
//      verticalSetPoint = VERTICAL_TURN_DOWN_ANGLE;
//    }
//    else if(verticalSetPoint == VERTICAL_TURN_UP_ANGLE && (horizontalSetPoint == HORIZONTAL_TURN_RIGHT_ANGLE || horizontalSetPoint == forwardAngle))
//    {
//      horizontalSetPoint = HORIZONTAL_TURN_LEFT_ANGLE;
//    }
//    else
//    {
//      verticalSetPoint = VERTICAL_TURN_UP_ANGLE;
//    }
//  }
//}

/* struct OffsetCalibrationParameters
{
  static float compressionValue,
               centerAngle;
}; */

//void robotServoSetup(Servo* servoArray, int* portNumbers, float* initialValues, int arraySize, float setupStartAngle, unsigned long setupPauseLength, float rampRate)
//{
//  for(int i = 0; i < arraySize; i++)
//  {
//    /*
//    Serial.print("Started setup for ");
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.println(setupStartAngle);
//    */
//
//    servoArray[i].attach(portNumbers[i]);
//    servoArray[i].write(setupStartAngle);
//    delay(setupPauseLength);
//
//    unsigned long currentSetupTimeStamp = millis();
//    unsigned long lastSetupTimeStamp = currentSetupTimeStamp;
//    float currentSetupAngle = setupStartAngle;
//
//    while(currentSetupAngle != initialValues[i])
//    {
//      currentSetupTimeStamp = millis();
//      float elapsedTime = (currentSetupTimeStamp - lastSetupTimeStamp) / 1000.0;
//      lastSetupTimeStamp = currentSetupTimeStamp;
//
//      float maxAngleChange = elapsedTime * rampRate;
//      currentSetupAngle = coerceToRange(currentSetupAngle - maxAngleChange, currentSetupAngle + maxAngleChange, initialValues[i]);
//      servoArray[i].write(currentSetupAngle);
//
//      /*
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.println(currentSetupAngle);
//      */
//
//      delay(20);
//    }
//  }
//}

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
  
  motionGenerator.setMovementPeriod(totalPeriod);
  motionGenerator.setMovementPortions(compressExpandMovementPortion, anchorReleaseMovementPortion);
  motionGenerator.setBodyWaveParameters(minAmplitude, maxAmplitude, minSinePower, maxSinePower);
  motionGenerator.setHeadAngleLimits(FRONT_ANCHOR_TURN_DOWN_ANGLE, FRONT_ANCHOR_TURN_UP_ANGLE);
  motionGenerator.setTailAngleLimits(REAR_ANCHOR_TURN_DOWN_ANGLE, REAR_ANCHOR_TURN_UP_ANGLE);
  motionGenerator.setPhaseLag(lag);
  motionGenerator.setRampRate(turnRampRate);

  motionGenerator.calibrateOffset(0.5, HEAD_LENGTH, LINK_LENGTH, TAIL_LENGTH);
  /* centerAngle = forwardAngle;
  currentCenterAngle = forwardAngle;
  offset = getCalibratedOffset(0.5, forwardAngle);
  Serial.print("Offset: ");
  Serial.println(offset);
  OffsetEvaluator evaluator(0.5, forwardAngle);
  evaluator.evaluate(M_PI / 2.0); */

  // ConcertinaMovementInfo movementInfo = getConcertinaSetPoints(waveValue, totalPeriod, compressExpandMovementPortion, anchorReleaseMovementPortion);
  
  int portNumbers[NUM_SERVOS];

  
  for(int i = 0; i < (NUM_SERVOS); i++)
  { 
    portNumbers[i] = 13 - i;
  }

  /*
    if(i == 0)
    {
      initialValues[i] = movementInfo.frontAnchorValue;
    }
    else if(i < (NUM_SERVOS - 1))
    {
      // robotServos[i].write(forwardAngle + minAmplitude*sin(i * lag));
      float sineValue = sin(offset + (i - 1) * lag),
              currentAmplitude = linearInterpolate(minAmplitude, maxAmplitude, movementInfo.compressionValue),
              currentSinePower = linearInterpolate(minSinePower, maxSinePower, movementInfo.compressionValue);
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        initialValues[i] = currentCenterAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower));
    }
    else if(i == NUM_SERVOS - 1)
    {
      initialValues[i] = movementInfo.rearAnchorValue;
    }
//    else if(i == NUM_SERVOS - 2)
//    {
//      robotServos[i].write(horizontalTurnCurrentValue);
//    }
//    else if(i == NUM_SERVOS - 1)
//    {
//      robotServos[i].write(verticalTurnCurrentValue);
//    }
    // Serial.print("Started servo connected to " + String(13 - i) + " with " + String(centerAngle+amplitude*cos(i * lag)) + ".\n"); 
  }
*/
  motionGenerator.getServos(servoValueBuffer, 0.0, false);

  robotServoSetup(robotServos, portNumbers, servoValueBuffer, forwardAngles, NUM_SERVOS, 330, 45.0);

  // Pause to position robot
  // delay(startPause);
} 
  
  
void loop() 
{
  float turnOffset = 0.0;
  
  //  Read movement pins
    forwardVal = digitalRead(forwardPin);
    reverseVal = digitalRead(reversePin);
    rightVal = digitalRead(rightPin);
    leftVal = digitalRead(leftPin);

    // Right turn
    if (rightVal == HIGH){
      runningWave = true;
      turnOffset = rightOffset;
      // waveOffsetMultiplier = 1.0;
      // Serial.println("Turning right...");
    }
    // Left turn
    else if (leftVal == HIGH){
      runningWave = true;
      turnOffset = leftOffset;
      // Serial.println("Turning left...");
      // waveOffsetMultiplier = 1.0;
    }
    // Straight movement
    else
    {
      turnOffset = 0.0;
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

      motionGenerator.setRampedTurnOffset(turnOffset);
      motionGenerator.getServos(servoValueBuffer, elapsedTime, reverseDirection);

      for(unsigned short i = 0; i < NUM_SERVOS; i++)
      {
        robotServos[i].write(servoValueBuffer[i]);
      }

      // Calculate the maximum change in the current center angle
      // float maxAngleChange = turnRampRate * elapsedTime;

//      if(currentMovementStage == ConcertinaMovementStages::COMPRESS_MOVEMENT)
//      {
//        float maxCompressionChange = elapsedTime * (frequency / compressMovementPortion);
//
//        compressionValue = coerceToRange(compressionValue - maxCompressionChange, compressionValue + maxCompressionChange, 1.0);
//
//        if(compressionValue == 1.0)
//        {
//          currentMovementStage = ConcertinaMovementStages::EXPAND_MOVEMENT;
//        }
//      }
//      else if(currentMovementStage == ConcertinaMovementStages::EXPAND_MOVEMENT)
//      {
//        float maxCompressionChange = elapsedTime * (frequency / expandMovementPortion);
//
//        compressionValue = coerceToRange(compressionValue - maxCompressionChange, compressionValue + maxCompressionChange, 0.0);
//
//        if(compressionValue == 0.0)
//        {
//          currentMovementStage = ConcertinaMovementStages::COMPRESS_MOVEMENT;
//        }
//      }
//      else if(currentMovementStage == ConcertinaMovementStages::ANCHOR_MOVEMENT)
//      {
//        anchorSetPointAngle = ANCHOR_TURN_DOWN_ANGLE;
//      }
//
//      if(currentMovementStage == ConcertinaMovementStages::COMPRESS_MOVEMENT || currentMovementStage == ConcertinaMovementStages::EXPAND_MOVEMENT)
//      {
//        // Calculate the maximum change in the current center angle
//        float maxAngleChange = turnRampRate * elapsedTime;
//  
//        // Calculate the new center angle
//        currentCenterAngle = coerceToRange(currentCenterAngle - maxAngleChange, currentCenterAngle + maxAngleChange, centerAngle);
//      }

      // Calculate the new center angle
      // currentCenterAngle = coerceToRange(currentCenterAngle - maxAngleChange, currentCenterAngle + maxAngleChange, centerAngle);
      // Serial.println("Current center angle: " + String(currentCenterAngle));

//    if(runningTurn || runningWave)
//    {
//      float activeHorizontalRampRate = horizontalTurnRampRate,
//            activeVerticalRampRate = verticalTurnRampRate;
//      
//      if(runningTurn)
//      {
//        if(((horizontalTurnCurrentValue == horizontalTurnSetPoint) && (verticalTurnCurrentValue == verticalTurnSetPoint))
//            || (previousTurnDirection != currentTurnDirection))
//        {
//          advanceTurnSetPoints(currentTurnDirection, horizontalTurnSetPoint, verticalTurnSetPoint);
//        }
//  
//        previousTurnDirection = currentTurnDirection;
//      }
//      else
//      {
//        horizontalTurnSetPoint = forwardAngle;
//        verticalTurnSetPoint = VERTICAL_TURN_UP_ANGLE;
//
//        activeHorizontalRampRate = horizontalResetRampRate;
//        activeVerticalRampRate = verticalResetRampRate;
//      }
//
//      float maxHorizontalAngleChange = activeHorizontalRampRate * elapsedTime,
//            maxVerticalAngleChange = activeVerticalRampRate * elapsedTime;
//
//      horizontalTurnCurrentValue = coerceToRange(horizontalTurnCurrentValue - maxHorizontalAngleChange,
//                                                 horizontalTurnCurrentValue + maxHorizontalAngleChange, horizontalTurnSetPoint);
//
//      verticalTurnCurrentValue = coerceToRange(verticalTurnCurrentValue - maxVerticalAngleChange,
//                                               verticalTurnCurrentValue + maxVerticalAngleChange, verticalTurnSetPoint);
//
//      robotServos[NUM_SERVOS - 2].write(horizontalTurnCurrentValue);
//      robotServos[NUM_SERVOS - 1].write(verticalTurnCurrentValue);
//
//      // Serial.println("Wrote " + String(horizontalTurnCurrentValue) + " to horizontal servo.");
//      // Serial.println("Wrote " + String(verticalTurnCurrentValue) + " to vertical servo.");
//    }
    }
}
