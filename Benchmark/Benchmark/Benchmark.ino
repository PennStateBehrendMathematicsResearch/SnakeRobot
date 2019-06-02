const int START_GATE_LASER_PIN = 22,
          START_GATE_PHOTORESISTOR_PIN = A0,
          START_GATE_THRESHOLD = 900;

const int END_GATE_LASER_PIN = 23,
          END_GATE_PHOTORESISTOR_PIN = A1,
          END_GATE_THRESHOLD = 900;

const double BENCHMARK_TRACK_LENGTH = 123.0;
const char* BENCHMARK_TRACK_LENGTH_UNITS = "cm";

const unsigned OUTPUT_PRECISION = 3;

bool timingInProgress = false;
unsigned long startTimestamp;

void setup() {
  pinMode(START_GATE_LASER_PIN, OUTPUT);
  digitalWrite(START_GATE_LASER_PIN, HIGH);

  pinMode(END_GATE_LASER_PIN, OUTPUT);
  digitalWrite(END_GATE_LASER_PIN, HIGH);

  delay(500);

  Serial.begin(38400);
}

void loop() {
  if(timingInProgress)
  {
    if(analogRead(END_GATE_PHOTORESISTOR_PIN) <= END_GATE_THRESHOLD)
    {
      unsigned long currentTimerValue = millis();
      double elapsedSeconds = (currentTimerValue - startTimestamp) / 1000.0;
      double averageSpeed = BENCHMARK_TRACK_LENGTH / elapsedSeconds;
      
      timingInProgress = false;
      
      Serial.println(F("Timing completed:"));
      
      Serial.print(F("Time: "));
      Serial.print(elapsedSeconds, OUTPUT_PRECISION);
      Serial.println(F(" seconds"));

      Serial.print(F("Average speed: "));
      Serial.print(averageSpeed, OUTPUT_PRECISION);
      Serial.print(F(" "));
      Serial.print(BENCHMARK_TRACK_LENGTH_UNITS);
      Serial.println(F("/s"));
    }
  }
  else
  {
    if(analogRead(START_GATE_PHOTORESISTOR_PIN) <= START_GATE_THRESHOLD)
    {
      startTimestamp = millis();

      timingInProgress = true;
      Serial.println(F("Timing started."));
    }
  }
}
