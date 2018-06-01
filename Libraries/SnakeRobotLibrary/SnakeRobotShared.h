#ifndef _SNAKEROBOTSHARED_H
#define _SNAKEROBOTSHARED_H

#include <Arduino.h>
#include <Servo.h>

class SingleVariableFunction
{
public:
  virtual float evaluate(float value) const = 0;
};

struct BisectionResult
{
  bool foundSolution;
  float value;
};

float coerceToRange(float lowerBound, float upperBound, float value);
float linearInterpolate(float startValue, float endValue, float interpolateValue);
void robotServoSetup(Servo* servoArray, const int* portNumbers, const float* initialValues, const float* setupStartAngles, int arraySize, unsigned long setupPauseLength, float rampRate);
BisectionResult findBisectionSolution(SingleVariableFunction* bisectionFunction, float tolerance, float lowerBound, float upperBound);
#endif
