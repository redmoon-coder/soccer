#include <PeanutKingSoccerV4.h>
const int KICKER_PIN = D1_P;
const unsigned long KICK_TIME = 200;  // max ON time in ms

void setup() {
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);
  Serial.begin(9600);
}

void kick() {
  digitalWrite(KICKER_PIN, HIGH);
  delay(KICK_TIME);
  digitalWrite(KICKER_PIN, LOW);
}

void loop() {
  // Example: kick every 2 seconds
  kick();
  delay(100);
}