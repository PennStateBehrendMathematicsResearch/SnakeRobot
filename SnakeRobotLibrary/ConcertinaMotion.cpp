#include "ConcertinaMotion.h"

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

OffsetEvaluator::OffsetEvaluator(unsigned short numServos, float headLength, float linkLength, float tailLength, float forwardAngle, float centerAngle, float phaseLag, float sinePower,
                  float amplitude):
  numServos(numServos),
  headLength(headLength),
  linkLength(linkLength),
  tailLength(tailLength),
  forwardAngle(forwardAngle),
  centerAngle(centerAngle),
  phaseLag(phaseLag),
  sinePower(sinePower),
  amplitude(amplitude)
{
  
}

float OffsetEvaluator::evaluate(float value) const
{
  float currentXPosition = this->headLength + this->linkLength,
          sumXPositions = this->headLength + (this->headLength + this->linkLength),
          sumYPositions = 0.0,
          currentYPosition = 0.0,
          angleFromStart = 0.0,
          thisAngle;

    /* Serial.print("Start X: ");
    Serial.println(currentXPosition);

    Serial.print("Start Y: ");
    Serial.println(currentYPosition); */
    
    for(unsigned short i = 0; i < (this->numServos - 2); i++)
    {
      float sineValue = sin(value + i * this->phaseLag);
          // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
      
      thisAngle = M_PI * (this->centerAngle + (this->amplitude * sign(sineValue) * pow(abs(sineValue), this->sinePower))) / 180.0;
      angleFromStart += (thisAngle - (M_PI * this->forwardAngle / 180.0));

      /* Serial.print("Sine value: ");
      Serial.println(sineValue);
      
      Serial.print("Current servo angle: ");
      Serial.println(thisAngle);

      Serial.print("Angle from start: ");
      Serial.println(angleFromStart); */
  
      currentXPosition += this->linkLength * cos(angleFromStart);
      currentYPosition += this->linkLength * sin(angleFromStart);

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
  
    currentXPosition += this->tailLength * cos(angleFromStart);
    currentYPosition += this->tailLength * sin(angleFromStart);

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
          averageX = sumXPositions / (this->numServos + 2),
          averageY = sumYPositions / (this->numServos + 2);

    /* Serial.print("Average X: ");
    Serial.println(averageX);

    Serial.print("Average Y: ");
    Serial.println(averageY); */
  
    float errorValue = averageY - (endSlope * averageX);

    /* Serial.print("Error value: ");
    Serial.println(errorValue); */
  
    return errorValue;
}

ConcertinaMovementInfo ConcertinaMotionGenerator::getConcertinaSetPoints()
{
  ConcertinaMovementInfo resultInfo;
  float stageTime;

  float firstAnchorThreshold = (this->compressExpandPortion * this->movementPeriod),
        firstCompressExpandThreshold = firstAnchorThreshold + (this->anchorReleasePortion * this->movementPeriod),
        secondAnchorThreshold = firstCompressExpandThreshold + (this->compressExpandPortion * this->movementPeriod);

  if(this->cycleTime < firstAnchorThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT;
    stageTime = this->cycleTime;

    resultInfo.compressionValue = linearInterpolate(0.0, 1.0, (stageTime / (this->compressExpandPortion * this->movementPeriod)));
    resultInfo.frontAnchorValue = this->headLowerAngle;
    resultInfo.rearAnchorValue = this->tailUpperAngle;
  }
  else if(this->cycleTime < firstCompressExpandThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::ANCHOR_RELEASE_MOVEMENT;
    stageTime = this->cycleTime - firstAnchorThreshold;

    resultInfo.compressionValue = 1.0;
    resultInfo.frontAnchorValue = linearInterpolate(this->headLowerAngle, this->headUpperAngle, (stageTime / (this->anchorReleasePortion * this->movementPeriod)));
    resultInfo.rearAnchorValue = linearInterpolate(this->tailUpperAngle, this->tailLowerAngle, (stageTime / (this->anchorReleasePortion * this->movementPeriod)));// ANCHOR_TURN_DOWN_ANGLE + ((ANCHOR_TURN_UP_ANGLE - ANCHOR_TURN_DOWN_ANGLE) * (stageTime / (anchorReleasePortion * movementPeriod)));
  }
  else if(this->cycleTime < secondAnchorThreshold)
  {
    resultInfo.movementStage = ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT;
    stageTime = this->cycleTime - firstCompressExpandThreshold;

    resultInfo.compressionValue = linearInterpolate(1.0, 0.0, (stageTime / (this->compressExpandPortion * this->movementPeriod))); // 1.0 - (stageTime / (compressExpandPortion * movementPeriod));
    resultInfo.frontAnchorValue = this->headUpperAngle;
    resultInfo.rearAnchorValue = this->tailLowerAngle;
  }
  else
  {
    resultInfo.movementStage = ConcertinaMovementStage::ANCHOR_RELEASE_MOVEMENT;
    stageTime = this->cycleTime - secondAnchorThreshold;

    resultInfo.compressionValue = 0.0;
    resultInfo.frontAnchorValue = linearInterpolate(this->headUpperAngle, this->headLowerAngle, (stageTime / (this->anchorReleasePortion * this->movementPeriod)));
    resultInfo.rearAnchorValue = linearInterpolate(this->tailLowerAngle, this->tailUpperAngle, (stageTime / (this->anchorReleasePortion * this->movementPeriod))); // ANCHOR_TURN_UP_ANGLE + ((ANCHOR_TURN_DOWN_ANGLE - ANCHOR_TURN_UP_ANGLE) * (stageTime / (anchorReleasePortion * movementPeriod)));
  }

  return resultInfo;
}

ConcertinaMotionGenerator::ConcertinaMotionGenerator(float forwardAngle, float startTurnAngle):
  forwardAngle(forwardAngle),
  turnSetpoint(startTurnAngle),
  currentTurnAngle(startTurnAngle)
{
  
}

/*
void ConcertinaMotionGenerator::setForwardAngle(float newForwardAngle)
{
  this->turnSetpoint += (newForwardAngle - this->forwardAngle);
  this->forwardAngle = newForwardAngle;
}
*/

void ConcertinaMotionGenerator::setMovementPeriod(float movementPeriod)
{
  this->movementPeriod = movementPeriod;
}

void ConcertinaMotionGenerator::setMovementPortions(float compressExpandPortion, float anchorReleasePortion)
{
  this->compressExpandPortion = compressExpandPortion;
  this->anchorReleasePortion = anchorReleasePortion;
}

void ConcertinaMotionGenerator::setBodyWaveParameters(float minAmplitude, float maxAmplitude,
                                                      float minPower, float maxPower)
{
  this->minAmplitude = minAmplitude;
  this->maxAmplitude = maxAmplitude;
  this->minPower = minPower;
  this->maxPower = maxPower;
}

void ConcertinaMotionGenerator::setHeadAngleLimits(float bottomAngle, float topAngle)
{
  this->headLowerAngle = bottomAngle;
  this->headUpperAngle = topAngle;
}

void ConcertinaMotionGenerator::setTailAngleLimits(float bottomAngle, float topAngle)
{
  this->tailLowerAngle = bottomAngle;
  this->tailUpperAngle = topAngle;
}

void ConcertinaMotionGenerator::setPhaseLag(float phaseLag)
{
  this->phaseLag = phaseLag;
}

void ConcertinaMotionGenerator::setRampRate(float rampRate)
{
  this->rampRate = rampRate;
}

void ConcertinaMotionGenerator::setRampedTurnOffset(float turnOffset)
{
  this->turnSetpoint = (this->forwardAngle + turnOffset);
}

void ConcertinaMotionGenerator::calibrateOffset(unsigned short numServos, float compressionValue, float headLength, float linkLength, float tailLength)
{
  float amplitude = linearInterpolate(this->minAmplitude, this->maxAmplitude, compressionValue),
        sinePower = linearInterpolate(this->minPower, this->maxPower, compressionValue);

  OffsetEvaluator evaluator(numServos, headLength, linkLength, tailLength, this->forwardAngle, this->forwardAngle, this->phaseLag, sinePower, amplitude);

  BisectionResult result = findBisectionSolution(&evaluator, 0.001, 0.0, M_PI);

  if(result.foundSolution)
  {
    this->offset = result.value;
    Serial.print(F("Calibrated offset: "));
    Serial.println(result.value);
  }
  else
  {
    Serial.println(F("Error: Offset could not be calculated."));
  }
}

void ConcertinaMotionGenerator::getServos(float* servoValuesOut, unsigned short numServos, float elapsedTime, bool reverseDirection)
{
  this->cycleTime += (elapsedTime * (reverseDirection ? -1.0 : 1.0));
  this->cycleTime = fmod(this->cycleTime, this->movementPeriod);

  if(this->cycleTime < 0.0)
  {
    this->cycleTime += this->movementPeriod;
  }
  
  ConcertinaMovementInfo movementInfo = this->getConcertinaSetPoints();

  if(movementInfo.movementStage == ConcertinaMovementStage::COMPRESS_EXPAND_MOVEMENT)
  {
    // Calculate the maximum change in the current center angle
    float maxAngleChange = this->rampRate * elapsedTime;

    // Calculate the new center angle
    this->currentTurnAngle = coerceToRange(this->currentTurnAngle - maxAngleChange, this->currentTurnAngle + maxAngleChange, this->turnSetpoint);
  }

  Serial.print(this->cycleTime);
  Serial.print(F(" | "));
  Serial.print(movementInfo.compressionValue);
  Serial.print(F(" | "));
  Serial.print(this->currentTurnAngle);
  Serial.print(F(" | "));
  // Loop to update robot servos
  for(int i = 1; i < (numServos - 1); i++)
  {
    float sineValue = sin(this->offset + (i - 1) * this->phaseLag),
          currentAmplitude = linearInterpolate(this->minAmplitude, this->maxAmplitude, movementInfo.compressionValue), // minAmplitude + (movementInfo.compressionValue * (maxAmplitude - minAmplitude)),
          currentSinePower = linearInterpolate(this->minPower, this->maxPower, movementInfo.compressionValue); // + (movementInfo.compressionValue * (maxSinePower - minSinePower));
    // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
    servoValuesOut[i] = this->currentTurnAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower));

    Serial.print(abs(sineValue));
    Serial.print(F(" ^ "));
    Serial.print(currentSinePower);
    Serial.print(F(" : "));
    Serial.print(this->currentTurnAngle + (currentAmplitude * sign(sineValue) * pow(abs(sineValue), currentSinePower)));
    Serial.print(' ');
  }

  Serial.println();

  // float maxAnchorAngleChange = anchorTurnRampRate * elapsedTime;
  // currentAnchorAngle = coerceToRange(anchorSetPointAngle - maxAnchorAngleChange, anchorSetPointAngle + maxAnchorAngleChange, currentAnchorAngle);

  // robotServos[NUM_SERVOS - 1].write(currentAnchorAngle);

  Serial.print(F("Front anchor value: "));
  Serial.println(movementInfo.frontAnchorValue);
  servoValuesOut[0] = movementInfo.frontAnchorValue;
  
  Serial.print(F("Rear anchor value: "));
  Serial.println(movementInfo.rearAnchorValue);
  servoValuesOut[numServos - 1] = movementInfo.rearAnchorValue;
}

