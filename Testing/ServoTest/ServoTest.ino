#include <Servo.h>

const short NUM_SERVOS = 12;

Servo servoArray[NUM_SERVOS];

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    servoArray[i].attach(13 - i);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  String thisCommand = Serial.readStringUntil('\n');

  if(thisCommand.length() > 0)
  {
    int servoNum, degreeValue;
    
    Serial.print("Received: ");
    Serial.println(thisCommand);

    if(sscanf(thisCommand.c_str(), "%d %d", &servoNum, &degreeValue) == 2)
    {
      if(servoNum >= 0 && servoNum < NUM_SERVOS)
      {
        servoArray[servoNum].write(degreeValue);
        Serial.print("Set servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.print(degreeValue);
        Serial.println(" degrees.");
      }
      else
      {
        Serial.println("Invalid servo number.");
      }
    }
    else
    {
      Serial.println("Could not parse command.");
    }
  }

  
}
