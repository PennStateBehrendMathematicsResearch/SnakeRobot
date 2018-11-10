const int START_GATE_LASER_PIN = 22,
          START_GATE_PHOTORESISTOR_PIN = A0;

const int END_GATE_LASER_PIN = 23,
          END_GATE_PHOTORESISTOR_PIN = A1;

void setup() {
  pinMode(START_GATE_LASER_PIN, OUTPUT);
  digitalWrite(START_GATE_LASER_PIN, HIGH);

  pinMode(END_GATE_LASER_PIN, OUTPUT);
  digitalWrite(END_GATE_LASER_PIN, HIGH);

  Serial.begin(115200);
}

void loop() {
  Serial.print(F("Start gate photoresistor value: "));
  Serial.print(analogRead(START_GATE_PHOTORESISTOR_PIN));

  Serial.print(F("    End gate photoresistor value: "));
  Serial.println(analogRead(END_GATE_PHOTORESISTOR_PIN));
}
