#include "Buzzer.h"
/*
You can instantiate multiple buzzers if needed.

It’s non-blocking (no delay()), so it works smoothly with your display and RTC tasks.
*/
Buzzer::Buzzer(int p) {
  pin = p;
}

void Buzzer::begin() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Buzzer::beep(uint8_t times, uint16_t duration, uint16_t gap) {
  if (times == 0) return;
  beepsLeft = times;
  beepDuration = duration;
  beepGap = gap;
  buzzerOn = true;
  lastToggle = millis();
  digitalWrite(pin, HIGH);
}

void Buzzer::update() {
  if (beepsLeft == 0 && !buzzerOn) return;

  uint32_t now = millis();
  uint32_t elapsed = now - lastToggle;

  if (buzzerOn && elapsed >= beepDuration) {
    digitalWrite(pin, LOW);
    buzzerOn = false;
    lastToggle = now;
  }
  else if (!buzzerOn && elapsed >= beepGap && beepsLeft > 0) {
    digitalWrite(pin, HIGH);
    buzzerOn = true;
    lastToggle = now;
    beepsLeft--;
  }
}

void Buzzer::stop(){
  digitalWrite(pin, LOW);
}