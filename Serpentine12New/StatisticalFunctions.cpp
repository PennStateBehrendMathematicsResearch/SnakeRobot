namespace StatisticalFunctions
{
  float arraySum(const float* sumArray, int sumArrayLength)
  {
    float sumValue = 0.0;

    for(int i = 0; i < sumArrayLength; i++)
    {
      sumValue += sumArray[i];
    }

    return sumValue;
  }

  float arrayAverage(const float* averageArray, int averageArrayLength)
  {
    return (arraySum(averageArray, averageArrayLength) / averageArrayLength);
  }

  float simpleLeastSquaresLinearRegressionSlope(const float* xValues, const float* yValues, const int arraySize)
  {
    float averageXValue = arrayAverage(xValues, arraySize);
    float averageYValue = arrayAverage(yValues, arraySize);

    float numeratorTerms[arraySize];
    float denominatorTerms[arraySize];
    float numeratorSum, denominatorSum;

    for(int i = 0; i < arraySize; i++)
    {
      numeratorTerms[i] = ((xValues[i] - averageXValue) * (yValues[i] - averageYValue));
    }

    numeratorSum = arraySum(numeratorTerms, arraySize);

    for(int i = 0; i < arraySize; i++)
    {
      denominatorTerms[i] = ((xValues[i] - averageXValue) * (xValues[i] - averageXValue));
    }

    denominatorSum = arraySum(denominatorTerms, arraySize);

    return (numeratorSum / denominatorSum);
  }

  float simpleLeastSquaresLinearRegressionSlope(const float* yValues, const int arraySize)
  {
    float averageXValue = (float)arraySize / 2.0;
    float averageYValue = arrayAverage(yValues, arraySize);

    float numeratorTerms[arraySize];
    float denominatorTerms[arraySize];
    float numeratorSum, denominatorSum;

    for(int i = 0; i < arraySize; i++)
    {
      numeratorTerms[i] = (((float)i - averageXValue) * (yValues[i] - averageYValue));
    }

    numeratorSum = arraySum(numeratorTerms, arraySize);

    for(int i = 0; i < arraySize; i++)
    {
      denominatorTerms[i] = (((float)i - averageXValue) * ((float)i - averageXValue));
    }

    denominatorSum = arraySum(denominatorTerms, arraySize);

    return (numeratorSum / denominatorSum);
  }
}

