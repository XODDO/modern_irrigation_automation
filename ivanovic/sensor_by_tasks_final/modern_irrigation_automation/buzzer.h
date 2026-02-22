#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"

class Buzzer {
  private:
    int pin;
    bool buzzerOn = false;
    uint32_t lastToggle = 0;
    uint8_t beepsLeft = 0;
    uint16_t beepDuration = 100; // default ms
    uint16_t beepGap = 150;      // default ms

  public:
    Buzzer(int pin);
    void begin();
    void beep(uint8_t times, uint16_t duration = 100, uint16_t gap = 150);
    void update();
    void stop();
};

#endif
