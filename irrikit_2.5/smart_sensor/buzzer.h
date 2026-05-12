#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"

// Structure to define a single tone in a sequence
struct ToneStep {
  uint16_t frequency; // Frequency in Hz (0 for silence/pause)
  uint16_t duration;  // Duration in milliseconds
};

// Structure to define a complete tone sequence
struct ToneSequence {
  const ToneStep* steps;  // Pointer to an array of ToneStep
  uint8_t numSteps;       // Number of steps in the sequence
  uint16_t gap;           // Gap after the whole sequence (for repetitions)
};

class Buzzer {
  private:
    int pin;
    bool buzzerOn = false;
    uint32_t lastToggle = 0;
    uint8_t beepsLeft = 0;
    uint16_t beepDuration = 100; // default ms
    uint16_t beepGap = 150;      // default ms
    uint16_t frequency = 0;       // 0 = DC (simple on/off), >0 = tone frequency
    
    // For tone generation
    uint32_t halfPeriod = 0;      // microseconds for half the period
    uint32_t lastMicros = 0;
    
    // --- New members for sequence playback ---
    const ToneSequence* currentSequence = nullptr;
    uint8_t currentStep = 0;
    uint32_t stepStartTime = 0;
    bool isPlayingSequence = false;
    uint8_t sequenceRepetitionsLeft = 0;
    // -----------------------------------------

    // Predefined tones (frequencies in Hz)
    static const uint16_t TONE_BOOT = 262;        // C4
    static const uint16_t TONE_SUCCESS = 523;     // C5
    static const uint16_t TONE_ERROR = 220;       // A3
    static const uint16_t TONE_DATA_SAVED = 392;  // G4
    static const uint16_t TONE_DATA_SENT = 440;   // A4
    static const uint16_t TONE_ALERT = 659;       // E5

  public:
    Buzzer(int pin);
    void begin();
    
    // Original method for compatibility
    void beep(uint8_t times, uint16_t duration = 100, uint16_t gap = 150);
    
    // New method for tone beeps
    void toneBeep(uint8_t times, uint16_t duration, uint16_t gap, uint16_t freq);
    
    // --- New method for playing tone sequences ---
    void playSequence(const ToneSequence* sequence, uint8_t repetitions = 1);
    // ---------------------------------------------
    
    // Convenience methods for different tones
    void bootBeep();
    void successBeep();
    void errorBeep();
    void dataSavedBeep();
    void dataSentBeep();
    void alertBeep();
    // --- New convenience method for a Windows-like error ---
    void windowsErrorBeep();
    // -------------------------------------------------------
    
    void update();
    
    // Stop any ongoing beep
    void stop();
};

#endif