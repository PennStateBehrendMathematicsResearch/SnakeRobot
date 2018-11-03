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

template<typename OutputType>
inline void printWithLineEnd(Stream& outStream, const OutputType& data)
{
  outStream.print(data);
  outStream.print('\n');
}

float coerceToRange(float lowerBound, float upperBound, float value);
float linearInterpolate(float startValue, float endValue, float interpolateValue);
float getRampedValue(float currentValue, float setpoint, float rampRate, float elapsedTime);
void robotServoSetup(Servo* servoArray, const int* portNumbers, const float* initialValues, const float* setupStartAngles, int arraySize, unsigned long setupPauseLength, float rampRate);
BisectionResult findBisectionSolution(SingleVariableFunction* bisectionFunction, float tolerance, float lowerBound, float upperBound);
#endif
