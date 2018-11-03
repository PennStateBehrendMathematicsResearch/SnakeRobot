/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/

// Compiler flag to set a button layout which uses two buttons for reverse turns; if not set, a single-button layout that persists movement direction for turns will be used (see README.md in
// the Serpentine folder for more information)
// #define TWO_BUTTON_REVERSE_TURN_LAYOUT

// #define USE_HEAD_SETTING_KNOBS

#define USE_SERIAL_COMMANDS

#define USE_IMMEDIATE_COMMAND

#include <Servo.h>

#include <SnakeRobotShared.h>
// #include <SerialCommands.h>

#ifdef USE_SERIAL_COMMANDS
#include <CommandParser.h>
#endif

// Total number of robot servos
const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];

// Servo forward angles (use to correct inaccuracies in robot assembly)
float forwardAngles[NUM_SERVOS] = {92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 91.0};

// This function returns the hardware port number for a given logical servo number (logical servo numbers begin from 0 at the head of the snake and increase down the snake body)
int getServoPortNumber(int servoNumber)
{
  return (13 - servoNumber);
}
  
// Define variables:

// Remote control movement pins
int forwardPin = 17; // DIO pin corresponding to 'A'
int reversePin = 16; // DIO pin corresponding to 'B'
int leftPin = 15;    // DIO pin corresponding to 'C'
int rightPin = 14;   // DIO pin corresponding to 'D'

float frequency = 0.35; // Oscillation frequency of segments.
float amplitude = 35.0; // Amplitude of the serpentine motion of the snake
float phaseLag = (M_PI / 6.0); // Phase lag between segments
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
int startPause = 5000;  // Delay time to position robot

const float AMPLITUDE_RAMP_RATE = 15.0,
            PHASE_LAG_RAMP_RATE = 0.2;

float turnSetpoint = 0.0; // Center angle set point
float currentTurnAngle = 0.0; // Current value of the center angle
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
float waveValue = 0.0; // Current base value for the wave generator (sine) function
bool runningWave = false,
     runningWavePrevious = runningWave;
float turnRampRate = 3.0; // Maximum rate at which turn offsets are ramped (degrees per second)
unsigned long currentTimeStamp,
              lastTimeStamp;

#ifdef USE_HEAD_SETTING_KNOBS
const int AMPLITUDE_SETTING_PIN = 5,
          PHASE_LAG_SETTING_PIN = 4;

const int AMPLITUDE_SETTING_MIN_INPUT = 1023,
          AMPLITUDE_SETTING_MAX_INPUT = 0,
          PHASE_LAG_SETTING_MIN_INPUT = 1023,
          PHASE_LAG_SETTING_MAX_INPUT = 0;

const float AMPLITUDE_SETTING_MIN_VALUE = 20.0,
            AMPLITUDE_SETTING_MAX_VALUE = 45.0,
            PHASE_LAG_SETTING_MIN_VALUE = (M_PI / 12.0),
            PHASE_LAG_SETTING_MAX_VALUE = (M_PI / 3.0);

float getKnobAmplitude()
{
  float amplitudeRatio = static_cast<float>((analogRead(AMPLITUDE_SETTING_PIN) - AMPLITUDE_SETTING_MIN_INPUT))
                         / (AMPLITUDE_SETTING_MAX_INPUT - AMPLITUDE_SETTING_MIN_INPUT);

  return linearInterpolate(AMPLITUDE_SETTING_MIN_VALUE, AMPLITUDE_SETTING_MAX_VALUE, amplitudeRatio);
}

float getKnobPhaseLag()
{
  float phaseLagRatio = static_cast<float>((analogRead(PHASE_LAG_SETTING_PIN) - PHASE_LAG_SETTING_MIN_INPUT))
                         / (PHASE_LAG_SETTING_MAX_INPUT - PHASE_LAG_SETTING_MIN_INPUT);

  return linearInterpolate(PHASE_LAG_SETTING_MIN_VALUE, PHASE_LAG_SETTING_MAX_VALUE, phaseLagRatio);
}
#endif

#ifdef USE_SERIAL_COMMANDS
void frequencyCommandHandler(float* paramArray, int numParams)
{
  if(numParams == 1)
  {
    frequency = paramArray[0];
    Serial.print(F("Frequency set to "));
    printWithLineEnd(Serial, frequency);
  }
}

  #ifdef USE_IMMEDIATE_COMMAND
bool immediateTimestampSet = false;
float immediateCommandTimeout = 0.5;
unsigned long lastImmediateCommandTimestamp;
void immediateCommandHandler(float* paramArray, int numParams)
{
  if(numParams == 2)
  {
    frequency = abs(paramArray[0]);
    reverseDirection = (paramArray[0] < 0.0);
    turnSetpoint = paramArray[1];

    lastImmediateCommandTimestamp = millis();
    immediateTimestampSet = true;

    Serial.print(F("runimm - Freq.: "));
    Serial.print(frequency);
    if(reverseDirection)
    {
      Serial.print(F(" (rev.)"));
    }
    Serial.print(F("; Turn: "));
    printWithLineEnd(Serial, turnSetpoint);
  }
}
  #endif

  #ifndef USE_HEAD_SETTING_KNOBS
float amplitudeSetpoint = amplitude;
float phaseLagSetpoint = phaseLag;

void amplitudeCommandHandler(float* paramArray, int numParams)
{
  if(numParams == 1)
  {
    amplitudeSetpoint = paramArray[0];
    Serial.print(F("Amplitude setpoint set to "));
    printWithLineEnd(Serial, amplitudeSetpoint);
  }
}

void phaseLagCommandHandler(float* paramArray, int numParams)
{
  bool phaseLagSet = false;
  
  if(numParams == 1)
  {
    phaseLagSetpoint = paramArray[0];

    Serial.print(F("Phase lag setpoint set to "));
    printWithLineEnd(Serial, phaseLagSetpoint);
  }
  else if(numParams == 2)
  {
    if(paramArray[1] != 0.0)
    {
      phaseLagSetpoint = (paramArray[0] / paramArray[1]) * M_PI;

      Serial.print(F("Phase lag setpoint set to "));
      printWithLineEnd(Serial, phaseLagSetpoint);
    }
    else
    {
      printWithLineEnd(Serial, F("Error: The denominator value given is zero."));
    }
  }
}
  #endif
  
ParsedCommandHandler commandArray[] = {{"setfreq", 1, frequencyCommandHandler}
  #ifndef USE_HEAD_SETTING_KNOBS
    , {"setamp", 1, amplitudeCommandHandler}, {"setpl", 2, phaseLagCommandHandler}
  #endif
  #ifdef USE_IMMEDIATE_COMMAND
    , {"runimm", 2, immediateCommandHandler}
  #endif
  };

void invalidCommandHandler(char* commandName)
{
  Serial.print(F("Error: The name of the command entered (\""));
  Serial.print(commandName);
  printWithLineEnd(Serial, F("\") is invalid."));
}

CommandParser serialParser(&Serial, 16, commandArray, sizeof(commandArray) / sizeof(ParsedCommandHandler*), invalidCommandHandler);
#endif

void setup() 
{ 
  Serial.begin(115200);
  printWithLineEnd(Serial, F("Program execution has started."));
  
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

  // Pause to position robot
  delay(startPause);
  
  // Determine port numbers and initial values for servos
  int portNumbers[NUM_SERVOS];
  float initialValues[NUM_SERVOS];

#ifdef USE_HEAD_SETTING_KNOBS
  amplitude = getKnobAmplitude();
  phaseLag = getKnobPhaseLag();
#endif
  
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    portNumbers[i] = getServoPortNumber(i);
    initialValues[i] = forwardAngles[i] + currentTurnAngle + amplitude * sin(i * phaseLag);
    // Serial.print("Started servo connected to " + String(portNumbers[i]) + " with " + String(initialValues[i]) + ".\n"); 
  }

  // Attach and setup robot servos
  robotServoSetup(robotServos, portNumbers, initialValues, forwardAngles, NUM_SERVOS, 330, 45.0);
} 
  
  
void loop() 
{
#ifdef USE_SERIAL_COMMANDS
    if(Serial.available())
    {
      serialParser.parseCommand();
    }
#endif

#if (defined USE_SERIAL_COMMANDS) && (defined USE_IMMEDIATE_COMMAND)
  if(immediateTimestampSet && (millis() - lastImmediateCommandTimestamp) / 1000.0 <= immediateCommandTimeout)
  {
    runningWave = true;
  }
  else
  {
    if(runningWavePrevious)
    {
      printWithLineEnd(Serial, F("The last immediate command has timed out."));
    }
    
    runningWave = false;
  }
#else
  //  Read movement pins
  int forwardVal = digitalRead(forwardPin),
      reverseVal = digitalRead(reversePin),
      rightVal = digitalRead(rightPin),
      leftVal = digitalRead(leftPin);

  #ifdef TWO_BUTTON_REVERSE_TURN_LAYOUT
// #pragma message("Two-button reverse turn layout is ENABLED.")
/*
    Serial.print("Forward: ");
    Serial.print(forwardVal);
    Serial.print("Reverse: ");
    Serial.print(reverseVal);
    Serial.print("Left: ");
    Serial.print(leftVal);
    Serial.print("Right: ");
    printWithLineEnd(Serial, rightVal);
*/

    // Determine forward/reverse state
    if (reverseVal == HIGH)
    {
      runningWave = true;
      reverseDirection = true;
    }
    else
    {
      reverseDirection = false;
    }

    // Right turn
    if (rightVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = rightOffset;
    }
    // Left turn
    else if (leftVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = leftOffset;
    }
    // Straight movement
    else if (forwardVal == HIGH || reverseVal == HIGH)
    {
      runningWave = true;
      turnSetpoint = 0.0;
    }
    else
    {
      runningWave = false;
    }
  #else
    // Right turn
    if (rightVal == HIGH){
      runningWave = true;
      turnSetpoint = rightOffset;
    }
    // Left turn
    else if (leftVal == HIGH){
      runningWave = true;
      turnSetpoint = leftOffset;
    }
    // Straight movement
    else
    {
      turnSetpoint = 0.0;
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
  #endif
#endif

    // If the robot has just started running
    if(runningWave && !runningWavePrevious)
    {
      // Reset the last time stamp to the current millisecond timer value
      lastTimeStamp = millis();
    }

    // If the robot is running
    if(runningWave)
    { 
      // Calculate the elapsed time 
      currentTimeStamp = millis();
      float elapsedTime = (currentTimeStamp - lastTimeStamp) / 1000.0;
      lastTimeStamp = currentTimeStamp;

#ifdef USE_HEAD_SETTING_KNOBS
      amplitude = getRampedValue(amplitude, getKnobAmplitude(), AMPLITUDE_RAMP_RATE, elapsedTime);
      phaseLag = getRampedValue(phaseLag, getKnobPhaseLag(), PHASE_LAG_RAMP_RATE, elapsedTime);
#else
  #ifdef USE_SERIAL_COMMANDS
      amplitude = getRampedValue(amplitude, amplitudeSetpoint, AMPLITUDE_RAMP_RATE, elapsedTime);
      phaseLag = getRampedValue(phaseLag, phaseLagSetpoint, PHASE_LAG_RAMP_RATE, elapsedTime);
  #endif
#endif

      /*
      Serial.print("Current amplitude: ");
      Serial.print(amplitude);
      Serial.print(" Current phase lag: ");
      printWithLineEnd(Serial, phaseLag);
      */

      // Calculate the new base wave generator value
      waveValue += ((-2.0 * M_PI * frequency)* (elapsedTime * (reverseDirection ? -1.0 : 1.0)));

      // Calculate the maximum change in the current center angle
      float maxAngleChange = turnRampRate * elapsedTime;

      // Calculate the new center angle
      currentTurnAngle = coerceToRange(currentTurnAngle - maxAngleChange, currentTurnAngle + maxAngleChange, turnSetpoint);
      // printWithLineEnd(Serial, "Current center angle: " + String(currentCenterAngle));

      // Loop to update robot servos
      for(int i = 0; i < NUM_SERVOS; i++)
      {
        // s1.write(currentAngle + amplitude*cos(frequency*counter*3.14159/180+5*lag));
        robotServos[i].write(forwardAngles[i] + currentTurnAngle + amplitude * sin(waveValue + (i * phaseLag)));
        // Serial.print(forwardAngles[i] + currentTurnAngle + amplitude*sin(waveValue + (i * lag)));
        // Serial.print(' ');
      }

      // printWithLineEnd(Serial, );
    }

    runningWavePrevious = runningWave;
}
