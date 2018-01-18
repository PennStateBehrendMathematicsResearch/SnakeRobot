#ifndef _SNAKEROBOTSHARED_H
#define _SNAKEROBOTSHARED_H
float coerceToRange(float lowerBound, float upperBound, float value);
float linearInterpolate(float startValue, float endValue, float interpolateValue);
void robotServoSetup(Servo* servoArray, int* portNumbers, float* initialValues, int arraySize, float setupStartAngle, unsigned long setupPauseLength, float rampRate);
#endif
