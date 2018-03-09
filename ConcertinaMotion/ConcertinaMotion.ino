/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/
#include <Servo.h>

#include <SnakeRobotShared.h>

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

float lag = (M_PI / 4.0); // Phase lag between segments
float totalPeriod = 8.0; // Oscillation frequency of segments.
float minAmplitude = 20; // Amplitude of the serpentine motion of the snake
float maxAmplitude = 60;

float minSinePower = 1.0;
float maxSinePower = 2.0;

const float forwardAngle = 93.0; // Center servo angle for forward or reverse motion (without turning)
float centerAngle = forwardAngle; // Center angle set point
float currentCenterAngle = forwardAngle; // Current value of the center angle
float turnRampRate = 3.0;
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
float waveValue = 0; // Current base value for the wave generator (sine) function
float offset = -2.2779505;
bool runningWave = false,
     runningWavePrevious = runningWave;

//bool runningTurn = false;
//enum RobotTurnDirection {LEFT_TURN, RIGHT_TURN};
//RobotTurnDirection currentTurnDirection = RobotTurnDirection::LEFT_TURN,
//                   previousTurnDirection = RobotTurnDirection::LEFT_TURN;

enum ConcertinaMovementStage {COMPRESS_EXPAND_MOVEMENT, ANCHOR_RELEASE_MOVEMENT};
ConcertinaMovementStage currentMovementStage = ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT;

float compressExpandMovementPortion = 0.4,
      anchorReleaseMovementPortion = 0.1;

// float compressionValue = 0.0;

const float FRONT_ANCHOR_TURN_DOWN_ANGLE = forwardAngle + 10.0,
            FRONT_ANCHOR_TURN_UP_ANGLE = forwardAngle - 25.0,
            REAR_ANCHOR_TURN_UP_ANGLE = forwardAngle + 30.0,
            REAR_ANCHOR_TURN_DOWN_ANGLE = forwardAngle - 20.0;

const float HEAD_LENGTH = 15.0,
            LINK_LENGTH = 7.0,
            TAIL_LENGTH = 11.5;

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

struct ConcertinaMovementInfo
{
  ConcertinaMovementStage movementStage;
  float compressionValue;
  float frontAnchorValue;
  float rearAnchorValue;
};

/* struct OffsetCalibrationParameters
{
  static float compressionValue,
               centerAngle;
}; */

ConcertinaMovementInfo getConcertinaSetPoints(float cycleTime, float movementPeriod, float compressExpandPortion, float anchorReleasePortion)
{
  ConcertinaMovementInfo resultInfo;
  float stageTime;

  float firstAnchorThreshold = (compressExpandPortion * movementPeriod),
        firstCompressExpandThreshold = firstAnchorThreshold + (anchorReleasePortion * movementPeriod),
        secondAnchorThreshold = firstCompressExpandThreshold + (compressExpandPortion * movementPeriod);

  if(cycleTime < firstAnchorThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT;
    stageTime = cycleTime;

    resultInfo.compressionValue = linearInterpolate(0.0, 1.0, (stageTime / (compressExpandPortion * movementPeriod)));
    resultInfo.frontAnchorValue = FRONT_ANCHOR_TURN_DOWN_ANGLE;
    resultInfo.rearAnchorValue = REAR_ANCHOR_TURN_UP_ANGLE;
  }
  else if(cycleTime < firstCompressExpandThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::ANCHOR_RELEASE_MOVEMENT;
    stageTime = cycleTime - firstAnchorThreshold;

    resultInfo.compressionValue = 1.0;
    resultInfo.frontAnchorValue = linearInterpolate(FRONT_ANCHOR_TURN_DOWN_ANGLE, FRONT_ANCHOR_TURN_UP_ANGLE, (stageTime / (anchorReleasePortion * movementPeriod)));
    resultInfo.rearAnchorValue = linearInterpolate(REAR_ANCHOR_TURN_UP_ANGLE, REAR_ANCHOR_TURN_DOWN_ANGLE, (stageTime / (anchorReleasePortion * movementPeriod)));// ANCHOR_TURN_DOWN_ANGLE + ((ANCHOR_TURN_UP_ANGLE - ANCHOR_TURN_DOWN_ANGLE) * (stageTime / (anchorReleasePortion * movementPeriod)));
  }
  else if(cycleTime < secondAnchorThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT;
    stageTime = cycleTime - firstCompressExpandThreshold;

    resultInfo.compressionValue = linearInterpolate(1.0, 0.0, (stageTime / (compressExpandPortion * movementPeriod))); // 1.0 - (stageTime / (compressExpandPortion * movementPeriod));
    resultInfo.frontAnchorValue = FRONT_ANCHOR_TURN_UP_ANGLE;
    resultInfo.rearAnchorValue = REAR_ANCHOR_TURN_DOWN_ANGLE;
  }
  else
  {
    resultInfo.movementStage = ConcertinaMovementStage::ANCHOR_RELEASE_MOVEMENT;
    stageTime = cycleTime - secondAnchorThreshold;

    resultInfo.compressionValue = 0.0;
    resultInfo.frontAnchorValue = linearInterpolate(FRONT_ANCHOR_TURN_UP_ANGLE, FRONT_ANCHOR_TURN_DOWN_ANGLE, (stageTime / (anchorReleasePortion * movementPeriod)));
    resultInfo.rearAnchorValue = linearInterpolate(REAR_ANCHOR_TURN_DOWN_ANGLE, REAR_ANCHOR_TURN_UP_ANGLE, (stageTime / (anchorReleasePortion * movementPeriod))); // ANCHOR_TURN_UP_ANGLE + ((ANCHOR_TURN_DOWN_ANGLE - ANCHOR_TURN_UP_ANGLE) * (stageTime / (anchorReleasePortion * movementPeriod)));
  }

  return resultInfo;
}

class OffsetEvaluator : public SingleVariableFunction
{
private:
  float compressionValue,
        centerAngle;
public:

  OffsetEvaluator(float compressionValue, float centerAngle):
    compressionValue(compressionValue),
    centerAngle(centerAngle)
    {
      
    }

  float evaluate(float value) const
  {
    float currentXPosition = HEAD_LENGTH + LINK_LENGTH,
          sumXPositions = HEAD_LENGTH + (HEAD_LENGTH + LINK_LENGTH),
          sumYPositions = 0.0,
          currentYPosition = 0.0,
          angleFromStart = 0.0,
          thisAngle;

    /* Serial.print("Start X: ");
    Serial.println(currentXPosition);

    Serial.print("Start Y: ");
    Serial.println(currentYPosition); */
    
    for(unsigned short i = 0; i < (NUM_SERVOS - 2); i++)
    {
      float sineValue = sin(value + i * lag),
                currentAmplitude = linearInterpolate(minAmplitude, maxAmplitude, this->compressionValue),
                currentSinePower = linearInterpolate(minSinePower, maxSinePower, this->compressionValue);
          // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
      
      thisAngle = M_PI * (this->centerAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower))) / 180.0;
      angleFromStart += (thisAngle - (M_PI * forwardAngle / 180.0));

      /* Serial.print("Sine value: ");
      Serial.println(sineValue);
      
      Serial.print("Current servo angle: ");
      Serial.println(thisAngle);

      Serial.print("Angle from start: ");
      Serial.println(angleFromStart); */
  
      currentXPosition += LINK_LENGTH * cos(angleFromStart);
      currentYPosition += LINK_LENGTH * sin(angleFromStart);

      /* Serial.print("X");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(currentXPosition);
  
      Serial.print("Y");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(currentYPosition); */
  
      sumXPositions += currentXPosition;
      sumYPositions += currentYPosition;
    }
  
    currentXPosition += TAIL_LENGTH * cos(angleFromStart);
    currentYPosition += TAIL_LENGTH * sin(angleFromStart);

    /* Serial.print("End X: ");
    Serial.println(currentXPosition);

    Serial.print("End Y: ");
    Serial.println(currentYPosition); */
  
    sumXPositions += currentXPosition;
    sumYPositions += currentYPosition;

    /* Serial.print("Sum X: ");
    Serial.println(sumXPositions);

    Serial.print("Sum Y: ");
    Serial.println(sumYPositions); */
  
    float endSlope = currentYPosition / currentXPosition,
          averageX = sumXPositions / (NUM_SERVOS + 2),
          averageY = sumYPositions / (NUM_SERVOS + 2);

    /* Serial.print("Average X: ");
    Serial.println(averageX);

    Serial.print("Average Y: ");
    Serial.println(averageY); */
  
    float errorValue = averageY - (endSlope * averageX);

    /* Serial.print("Error value: ");
    Serial.println(errorValue); */
  
    return errorValue;
  }
};

float getCalibratedOffset(float calCompressionValue, float calCenterAngle)
{
  float calibrationResult = 0.0;

  OffsetEvaluator evaluator(calCompressionValue, calCenterAngle);

  BisectionResult result = findBisectionSolution(&evaluator, 0.001, 0.0, M_PI);

  if(result.foundSolution)
  {
    calibrationResult = result.value;
  }

  return calibrationResult;
}

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

float sign(float value)
{
  float returnValue = 0.0;

  if(value < 0)
  {
    returnValue = -1.0;
  }
  else if(value > 0)
  {
    returnValue = 1.0;
  }

  return returnValue;
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
  offset = getCalibratedOffset(0.5, forwardAngle);
  Serial.print("Offset: ");
  Serial.println(offset);
  OffsetEvaluator evaluator(0.5, forwardAngle);
  evaluator.evaluate(M_PI / 2.0);

  ConcertinaMovementInfo movementInfo = getConcertinaSetPoints(waveValue, totalPeriod, compressExpandMovementPortion, anchorReleaseMovementPortion);

  float initialValues[NUM_SERVOS];
  int portNumbers[NUM_SERVOS];
  
  for(int i = 0; i < (NUM_SERVOS); i++)
  { 
    portNumbers[i] = 13 - i;

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

  robotServoSetup(robotServos, portNumbers, initialValues, NUM_SERVOS, forwardAngle, 330, 45.0);

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
      waveValue += (elapsedTime * (reverseDirection ? -1.0 : 1.0));
      waveValue = fmod(waveValue, totalPeriod);

      if(waveValue < 0.0)
      {
        waveValue += totalPeriod;
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
      ConcertinaMovementInfo movementInfo = getConcertinaSetPoints(waveValue, totalPeriod, compressExpandMovementPortion, anchorReleaseMovementPortion);

      if(movementInfo.movementStage == ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT)
      {
        // Calculate the maximum change in the current center angle
        float maxAngleChange = turnRampRate * elapsedTime;
  
        // Calculate the new center angle
        currentCenterAngle = coerceToRange(currentCenterAngle - maxAngleChange, currentCenterAngle + maxAngleChange, centerAngle);
      }

      Serial.print(waveValue);
      Serial.print(" | ");
      Serial.print(movementInfo.compressionValue);
      Serial.print(" | ");
      Serial.print(currentCenterAngle);
      Serial.print(" | ");
      // Loop to update robot servos
      for(int i = 1; i < (NUM_SERVOS - 1); i++)
      {
        float sineValue = sin(offset + (i - 1) * lag),
              currentAmplitude = linearInterpolate(minAmplitude, maxAmplitude, movementInfo.compressionValue), // minAmplitude + (movementInfo.compressionValue * (maxAmplitude - minAmplitude)),
              currentSinePower = linearInterpolate(minSinePower, maxSinePower, movementInfo.compressionValue); // + (movementInfo.compressionValue * (maxSinePower - minSinePower));
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        robotServos[i].write(currentCenterAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower)));

        Serial.print(abs(sineValue));
        Serial.print(" ^ ");
        Serial.print(currentSinePower);
        Serial.print(" : ");
        Serial.print(currentCenterAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower)));
        Serial.print(' ');
      }

      Serial.println();

      // float maxAnchorAngleChange = anchorTurnRampRate * elapsedTime;
      // currentAnchorAngle = coerceToRange(anchorSetPointAngle - maxAnchorAngleChange, anchorSetPointAngle + maxAnchorAngleChange, currentAnchorAngle);

      // robotServos[NUM_SERVOS - 1].write(currentAnchorAngle);

      Serial.print("Front anchor value: ");
      Serial.println(movementInfo.frontAnchorValue);
      robotServos[0].write(movementInfo.frontAnchorValue);
      
      Serial.print("Rear anchor value: ");
      Serial.println(movementInfo.rearAnchorValue);
      robotServos[NUM_SERVOS - 1].write(movementInfo.rearAnchorValue);
    }

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
