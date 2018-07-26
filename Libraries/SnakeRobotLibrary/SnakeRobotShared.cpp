#include "SnakeRobotShared.h"

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

float linearInterpolate(float startValue, float endValue, float interpolateValue)
{
  return (startValue + (endValue - startValue) * interpolateValue);
}

float getRampedValue(float currentValue, float setpoint, float rampRate, float elapsedTime)
{
  float maxChange = elapsedTime * rampRate;
  return coerceToRange(currentValue - maxChange, currentValue + maxChange, setpoint);
}

void robotServoSetup(Servo* servoArray, const int* portNumbers, const float* initialValues, const float* setupStartAngles, int arraySize, unsigned long setupPauseLength, float rampRate)
{
  for(int i = 0; i < arraySize; i++)
  {

    /*
    Serial.print("Started setup for ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(setupStartAngles[i]);
    */

    servoArray[i].attach(portNumbers[i]);
    servoArray[i].write(setupStartAngles[i]);
    delay(setupPauseLength);

    unsigned long currentSetupTimeStamp = millis();
    unsigned long lastSetupTimeStamp = currentSetupTimeStamp;
    float currentSetupAngle = setupStartAngles[i];

    while(currentSetupAngle != initialValues[i])
    {
      currentSetupTimeStamp = millis();
      float elapsedTime = (currentSetupTimeStamp - lastSetupTimeStamp) / 1000.0;
      lastSetupTimeStamp = currentSetupTimeStamp;

      float maxAngleChange = elapsedTime * rampRate;
      currentSetupAngle = coerceToRange(currentSetupAngle - maxAngleChange, currentSetupAngle + maxAngleChange, initialValues[i]);
      servoArray[i].write(currentSetupAngle);

      /*
      Serial.print(i);
      Serial.print(": ");
      Serial.println(currentSetupAngle);
      */

      delay(20);
    }
  }
}

BisectionResult findBisectionSolution(SingleVariableFunction* bisectionFunction, float tolerance, float lowerBound, float upperBound)
{
  float midPoint = (lowerBound + upperBound) / 2;
  float midPointValue = bisectionFunction->evaluate(midPoint);

  BisectionResult result;
  result.foundSolution = true;

  while(abs(midPointValue) > tolerance && result.foundSolution)
  {
    float upperBoundValue = bisectionFunction->evaluate(upperBound);

    if(upperBoundValue * midPointValue < 0)
    {
      lowerBound = midPoint;
    }
    else
    {
      float lowerBoundValue = bisectionFunction->evaluate(lowerBound);

      if(lowerBoundValue * midPointValue < 0)
      {
        upperBound = midPoint;
      }
      else
      {
        result.foundSolution = false;
      }
    }

    midPoint = (lowerBound + upperBound) / 2;
    midPointValue = bisectionFunction->evaluate(midPoint);
  }

  if(result.foundSolution)
  {
    result.value = midPoint;
  }

  return result;
}

