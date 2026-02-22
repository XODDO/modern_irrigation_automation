#include "Battery.h"
/* Features in this version:
Smoothing (10 samples averaged).

Scaling factor configurable per resistor divider setup.

Cell count multiplier (for packs).

Voltage levels (0–3).

Hysteresis at 11.5 V ↔ 12.2 V.

*/

Battery::Battery(uint8_t pin, float scaleFactor, uint8_t cells)
: _pin(pin), _scaleFactor(scaleFactor), _cells(cells) {}

void Battery::begin() {
  pinMode(_pin, INPUT);
  analogReadResolution(12);              // ESP32: 0–4095
  analogSetAttenuation(ADC_11db);        // up to ~3.6V input
}

void Battery::update() {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(_pin);
  }
  float avg = sum / float(NUM_SAMPLES);

  // Scale to real battery voltage
  _voltage = _cells * (avg * _scaleFactor / 3096.0f);  

    // if _scaleFactor == 14.4 >> 1 Lead Acid Batteries
    // if _scaleFactor = 8.4 >> 2 Li ion Cells

  // Voltage-based levels
  if (_voltage <= 7.2f) { // < 3.5V POWER_CRITICAL // standard lithium polymer (LiPo) cell's lowest allowed voltage is typically 3.0V, per cell to avoid irreversible damage
    _level = 0;
  } else if (_voltage <= 7.6f) { // < 3.8V POWER_LOW 
    _level = 1;
  } else if (_voltage <= 8.0f) { // < 4.0V POWER_MODERATE
    _level = 2;
  } else { // > 4.0 // POWER_EXCELLENT
    _level = 3;
  }

  // Apply hysteresis and trigger callbacks
  if (_lowState) {
    if (_voltage > HIGH_THRESHOLD) {
      _lowState = false;
      if (_recoveredCallback) _recoveredCallback();
    }
  } else {
    if (_voltage < LOW_THRESHOLD) {
      _lowState = true;
      if (_lowCallback) _lowCallback();
    }
  }
}

float Battery::getVoltage() const {
  return _voltage;
}

uint8_t Battery::getLevel() const {
  return _level;
}

bool Battery::isLow() const {
  return _lowState;
}

void Battery::onLow(Callback cb) {
  _lowCallback = cb;
}

void Battery::onRecovered(Callback cb) {
  _recoveredCallback = cb;
}
