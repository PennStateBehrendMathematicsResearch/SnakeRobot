/*
Remote control file for serpentine motion
of a snake robot with 12 servos
*/
#include <Servo.h>

#include <SnakeRobotShared.h>

#include <ConcertinaMotion.h>

const unsigned short NUM_SERVOS = 12;
Servo robotServos[NUM_SERVOS];

// Servo forward angles (use to correct inaccuracies in robot assembly)
const float forwardAngles[NUM_SERVOS] = {92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 92.0, 91.0};

int getServoPortNumber(int servoNumber)
{
  return (13 - servoNumber);
}

int abortPin = 17;

int abortVal = 0;  // Remote control variables
bool demoAborted = false;

float lag = (M_PI / 6.0); // Phase lag between segments
float totalPeriod = 6.0; // Oscillation frequency of segments.
float minAmplitude = 20; // Amplitude of the serpentine motion of the snake
float maxAmplitude = 60;

float minSinePower = 1.0;
float maxSinePower = 2.0;

const float forwardAngle = 93.0; // Center servo angle for forward or reverse motion (without turning)
float turnRampRate = 3.0;
int rightOffset = 7; // Right turn offset
int leftOffset = -7; // Left turn offset
bool reverseDirection = false; // Boolean flag to store whether or not the robot is moving in a reverse direction
bool runningWave = false,
     runningWavePrevious = runningWave;

ConcertinaMotionGenerator motionGenerator(NUM_SERVOS, forwardAngles, 0.0);

float compressExpandMovementPortion = 0.4,
      anchorReleaseMovementPortion = 0.1;

const float FRONT_ANCHOR_TURN_DOWN_ANGLE = 20.0,
            FRONT_ANCHOR_TURN_UP_ANGLE = -25.0,
            REAR_ANCHOR_TURN_UP_ANGLE = 30.0,
            REAR_ANCHOR_TURN_DOWN_ANGLE = -25.0;

const float HEAD_LENGTH = 13.5,
            LINK_LENGTH = 7.0,
            TAIL_LENGTH = 11.5;

float servoValueBuffer[NUM_SERVOS];

unsigned long currentTimeStamp,
              lastTimeStamp,
              demoBeginTimeStamp;

void setup() 
{ 
  Serial.begin(115200);

  pinMode(abortPin, INPUT);
  digitalWrite(abortPin, LOW);
  
  // Attach segment servos to pins and initialize them to their
  // starting positions
  
  motionGenerator.setMovementPeriod(totalPeriod);
  motionGenerator.setMovementPortions(compressExpandMovementPortion, anchorReleaseMovementPortion);
  motionGenerator.setBodyWaveParameters(minAmplitude, maxAmplitude, minSinePower, maxSinePower);
  motionGenerator.setHeadAngleLimits(FRONT_ANCHOR_TURN_DOWN_ANGLE, FRONT_ANCHOR_TURN_UP_ANGLE);
  motionGenerator.setTailAngleLimits(REAR_ANCHOR_TURN_DOWN_ANGLE, REAR_ANCHOR_TURN_UP_ANGLE);
  motionGenerator.setPhaseLag(lag);
  motionGenerator.setRampRate(turnRampRate);

  motionGenerator.calibrateOffset(0.5, HEAD_LENGTH, LINK_LENGTH, TAIL_LENGTH);
  
  int portNumbers[NUM_SERVOS];

  
  for(int i = 0; i < (NUM_SERVOS); i++)
  { 
    portNumbers[i] = getServoPortNumber(i);
  }
  
  motionGenerator.getServos(servoValueBuffer, 0.0, false);

  robotServoSetup(robotServos, portNumbers, servoValueBuffer, forwardAngles, NUM_SERVOS, 330, 45.0);

  demoBeginTimeStamp = millis();
} 
  
  
void loop() 
{
  float turnOffset = 0.0;
  
  unsigned long currentDemoTimeStamp = millis();
  float demoTime = (currentDemoTimeStamp - demoBeginTimeStamp) / 1000.0;

  abortVal = digitalRead(abortPin);

  if(abortVal)
  {
    demoAborted = true;
  }

  if(demoTime <= 45.0 && !demoAborted)
  {
    runningWave = true;

    if(demoTime <= 15.0)
    {
      reverseDirection = false;
      turnOffset = 0.0;
    }
    else if(demoTime <= 30.0)
    {
      reverseDirection = true;
      turnOffset = 0.0;
    }
    else
    {
      reverseDirection = false;
      turnOffset = rightOffset;
    }
  }
  else
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

      motionGenerator.setRampedTurnOffset(turnOffset);
      motionGenerator.getServos(servoValueBuffer, elapsedTime, reverseDirection);

      for(unsigned short i = 0; i < NUM_SERVOS; i++)
      {
        robotServos[i].write(servoValueBuffer[i]);
      }
    }
}
