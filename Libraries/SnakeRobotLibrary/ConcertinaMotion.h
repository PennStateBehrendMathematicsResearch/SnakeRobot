#ifndef CONCERTINA_MOTION_H
#define CONCERTINA_MOTION_H

#include <SnakeRobotShared.h>

float sign(float value);

enum ConcertinaMovementStage {COMPRESS_EXPAND_MOVEMENT, ANCHOR_RELEASE_MOVEMENT};

struct ConcertinaMovementInfo
{
  ConcertinaMovementStage movementStage;
  float compressionValue;
  float frontAnchorValue;
  float rearAnchorValue;
};

class OffsetEvaluator : public SingleVariableFunction
{
private:
  unsigned short numServos;

  const float* forwardAngles;
  
  float turnOffset,
        forwardAngle,
        headLength,
        linkLength,
        tailLength,
        phaseLag,
        sinePower,
        amplitude;
public:
  OffsetEvaluator(unsigned short numServos, float headLength, float linkLength, float tailLength, const float* forwardAngles, float turnOffset, float phaseLag, float sinePower,
                  float amplitude);

  float evaluate(float value) const;
};

class ConcertinaMotionGenerator
{
private:
  const float* forwardAngles = NULL;
  unsigned short numServos = 0;
  
  float cycleTime = 0.0,
        movementPeriod = 8.0,
        compressExpandPortion = 0.4,
        anchorReleasePortion = 0.1,
        minAmplitude = 30.0,
        maxAmplitude = 60.0,
        minPower = 1.0,
        maxPower = 2.0,
        offset = 0.0,
        phaseLag = (M_PI / 6.0),
        headLowerAngle = -20.0,
        headUpperAngle = 10.0,
        tailLowerAngle = -10.0,
        tailUpperAngle = 30.0,
        currentTurnAngle = 0.0,
        turnSetpoint = 0.0,
        rampRate = 3.0;
  
  ConcertinaMovementInfo getConcertinaSetPoints();
public:
  ConcertinaMotionGenerator(unsigned short numServos, const float* forwardAngles, float startTurnAngle);

  // void setForwardAngle(float newForwardAngle);
  void setMovementPeriod(float movementPeriod);
  void setMovementPortions(float compressExpandPortion, float anchorReleasePortion);
  void setBodyWaveParameters(float minAmplitude, float maxAmplitude, float minPower, float maxPower);
  void setHeadAngleLimits(float bottomAngle, float topAngle);
  void setTailAngleLimits(float bottomAngle, float topAngle);
  void setPhaseLag(float phaseLag);
  void setRampRate(float rampRate);
  void setRampedTurnOffset(float turnOffset);
  void calibrateOffset(float compressionValue, float headLength, float linkLength, float tailLength);
  void getServos(float* servoValuesOut, float elapsedTime, bool reverseDirection);
};

#endif
