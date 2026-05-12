#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

class Battery {
  public:
    using Callback = void (*)();   // function pointer type

    Battery(uint8_t pin, float scaleFactor, uint8_t cells = 1);

    void begin();
    void update();  // take smoothed reading

    float getVoltage() const;
    uint8_t getLevel() const;     // 0=dead, 1=low, 2=good, 3=excellent
    bool isLow() const;           // uses hysteresis

    void onLow(Callback cb);      // register callback
    void onRecovered(Callback cb);

  private:
    uint8_t _pin;
    float _scaleFactor;
    uint8_t _cells;

    float _voltage = 0.0f;
    uint8_t _level = 0;
    bool _lowState = false;       // hysteresis flag

    Callback _lowCallback = nullptr;
    Callback _recoveredCallback = nullptr;

    static const int NUM_SAMPLES = 10;
    static constexpr float LOW_THRESHOLD = (2*3.6); // 11.5f >> for Lead Acid Batt
    static constexpr float HIGH_THRESHOLD = (2*3.8); // 12.2f; >> for Lead Acid Batt
};

#endif
