const byte PIR_PIN = 2;
const byte SPEAKER_PIN = 8;
const unsigned int TONE_FREQUENCY_HZ = 10000;

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  noTone(SPEAKER_PIN);
  Serial.begin(9600);
  digitalWrite(PIR_PIN , LOW);
}

void loop() {
  if (digitalRead(PIR_PIN) == HIGH) {
    //Serial.println("PIN HIGH");
    tone(SPEAKER_PIN, TONE_FREQUENCY_HZ);
    delay(600);
    digitalWrite(PIR_PIN , LOW);
    //Serial.println("FINISHED");
  } else {
    noTone(SPEAKER_PIN);
    //Serial.println("PIN LOW");
  }
}
