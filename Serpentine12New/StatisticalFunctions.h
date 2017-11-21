#ifndef _STATISTICAL_FUNCTIONS_H
#define _STATISTICAL_FUNCTIONS_H

namespace StatisticalFunctions
{
  float arraySum(const float* sumArray, int sumArrayLength);
  float arrayAverage(const float* averageArray, int averageArrayLength);

  float simpleLeastSquaresLinearRegressionSlope(const float* xValues, const float* yValues, int arraySize);
  float simpleLeastSquaresLinearRegressionSlope(const float* yValues, const int arraySize);
}

#endif
