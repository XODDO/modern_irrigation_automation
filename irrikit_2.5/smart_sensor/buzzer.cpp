#include "Buzzer.h"

Buzzer::Buzzer(int p) {
  pin = p;
}

void Buzzer::begin() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Buzzer::beep(uint8_t times, uint16_t duration, uint16_t gap) {
  toneBeep(times, duration, gap, 0); // 0 frequency = DC mode
}

void Buzzer::toneBeep(uint8_t times, uint16_t duration, uint16_t gap, uint16_t freq) {
  if (times == 0) return;
  
  beepsLeft = times;
  beepDuration = duration;
  beepGap = gap;
  frequency = freq;
  
  if (frequency > 0) {
    halfPeriod = 500000 / frequency; // microseconds for half period (500000 = 1,000,000 / 2)
    lastMicros = micros();
  }
  
  buzzerOn = true;
  lastToggle = millis();
  
  if (frequency == 0) {
    // DC mode - simple on/off
    digitalWrite(pin, HIGH);
  }
  // For tone mode, the first toggle happens in update()
}

/*
void Buzzer::update() {
  if (beepsLeft == 0 && !buzzerOn) return;

  uint32_t now = millis();
  uint32_t elapsed = now - lastToggle;

  // Handle tone generation during active beep
  if (buzzerOn && frequency > 0) {
    uint32_t nowMicros = micros();
    if (nowMicros - lastMicros >= halfPeriod) {
      // Toggle the pin for tone generation
      digitalWrite(pin, !digitalRead(pin));
      lastMicros = nowMicros;
    }
  }

  // Handle beep timing (same as before)
  if (buzzerOn && elapsed >= beepDuration) {
    digitalWrite(pin, LOW);
    buzzerOn = false;
    lastToggle = now;
    frequency = 0; // Reset frequency when beep ends
  }
  else if (!buzzerOn && elapsed >= beepGap && beepsLeft > 0) {
    digitalWrite(pin, HIGH);
    buzzerOn = true;
    lastToggle = now;
    beepsLeft--;
    
    // Restore frequency for next beep if there are more beeps left
    if (beepsLeft > 0 && frequency > 0) {
      lastMicros = micros();
    }
  }
}



void Buzzer::stop() {
  beepsLeft = 0;
  buzzerOn = false;
  digitalWrite(pin, LOW);
  frequency = 0;
}
*/
// Convenience methods
void Buzzer::bootBeep() {
  toneBeep(1, 100, 0, TONE_BOOT);  // Single short beep at boot tone
}

void Buzzer::successBeep() {
  toneBeep(1, 150, 0, TONE_SUCCESS);  // Single success beep
}

void Buzzer::errorBeep() {
  toneBeep(3, 100, 100, TONE_ERROR);  // Triple error beep
}

void Buzzer::dataSavedBeep() {
  toneBeep(2, 80, 50, TONE_DATA_SAVED);  // Double beep for data saved
}

void Buzzer::dataSentBeep() {
  toneBeep(2, 80, 50, TONE_DATA_SENT);  // Double beep for data sent
}

void Buzzer::alertBeep() {
  toneBeep(5, 200, 100, TONE_ALERT);  // Alert pattern
}

#include "Buzzer.h"

// ... (constructor and existing methods like begin(), beep(), toneBeep() remain the same) ...

void Buzzer::update() {
  // --- Priority 1: Playing a Tone Sequence ---
  if (isPlayingSequence) {
    uint32_t now = millis();

    // If we are between steps or just starting, this block handles advancing
    if (currentStep < currentSequence->numSteps) {
      const ToneStep& step = currentSequence->steps[currentStep];

      // Check if it's time to move to the next step
      if (now - stepStartTime >= step.duration) {
        // Move to next step
        currentStep++;
        stepStartTime = now;

        if (currentStep < currentSequence->numSteps) {
          // Play the next step
          const ToneStep& nextStep = currentSequence->steps[currentStep];
          if (nextStep.frequency == 0) {
            digitalWrite(pin, LOW); // Ensure pin is low for silence
            frequency = 0;
            buzzerOn = false; // Not actively generating tone
          } else {
            // Set up for the new frequency
            frequency = nextStep.frequency;
            halfPeriod = 500000 / frequency;
            lastMicros = micros();
            // Ensure pin is set correctly. The tone generation logic below will toggle it.
            // We don't set it HIGH here because the first toggle will happen immediately.
            buzzerOn = true;
          }
        } else {
          // Finished all steps in this repetition
          digitalWrite(pin, LOW);
          frequency = 0;
          buzzerOn = false;

          // Check if we need to repeat the whole sequence
          if (sequenceRepetitionsLeft > 1) {
            sequenceRepetitionsLeft--;
            currentStep = 0;
            stepStartTime = now + currentSequence->gap; // Add gap between repetitions
            // We'll let the next update cycle handle starting the first step after the gap
          } else {
            // Sequence is completely done
            isPlayingSequence = false;
            currentSequence = nullptr;
            beepsLeft = 0; // Clear any old beep state
            buzzerOn = false;
          }
        }
      }
    }

    // Handle tone generation for the current active step
    if (isPlayingSequence && currentStep < currentSequence->numSteps) {
      const ToneStep& currentStepData = currentSequence->steps[currentStep];
      if (currentStepData.frequency > 0) {
        uint32_t nowMicros = micros();
        if (nowMicros - lastMicros >= halfPeriod) {
          digitalWrite(pin, !digitalRead(pin));
          lastMicros = nowMicros;
        }
      }
    }
    // Return early? No, we still need to let the simple beep logic run if a sequence isn't playing.
    // But to keep it clean, we can put the sequence logic in its own block and let the rest run only if not playing a sequence.
    // Let's restructure: if playing a sequence, we only do sequence logic.
    return; // If we are playing a sequence, don't process simple beeps below.
  }

  // --- Priority 2: Simple Beeps (Original Logic) ---
  // (Keep your original update() code for simple beeps here)
  // ... (your existing if/else block for handling beepsLeft and buzzerOn) ...
  // Make sure to include the tone generation part from your original code as well.
  if (beepsLeft == 0 && !buzzerOn) return;

  uint32_t now = millis();
  uint32_t elapsed = now - lastToggle;

  // Handle tone generation during active beep
  if (buzzerOn && frequency > 0) {
    uint32_t nowMicros = micros();
    if (nowMicros - lastMicros >= halfPeriod) {
      digitalWrite(pin, !digitalRead(pin));
      lastMicros = nowMicros;
    }
  }

  // Handle beep timing
  if (buzzerOn && elapsed >= beepDuration) {
    digitalWrite(pin, LOW);
    buzzerOn = false;
    lastToggle = now;
    frequency = 0;
  }
  else if (!buzzerOn && elapsed >= beepGap && beepsLeft > 0) {
    digitalWrite(pin, HIGH);
    buzzerOn = true;
    lastToggle = now;
    beepsLeft--;

    if (beepsLeft > 0 && frequency > 0) {
      lastMicros = micros();
    }
  }
}

void Buzzer::playSequence(const ToneSequence* sequence, uint8_t repetitions) {
  if (sequence == nullptr || sequence->numSteps == 0) return;

  // Stop any currently playing sound
  stop();

  currentSequence = sequence;
  sequenceRepetitionsLeft = repetitions;
  currentStep = 0;
  isPlayingSequence = true;
  stepStartTime = millis();

  // Immediately start the first step
  const ToneStep& firstStep = currentSequence->steps[0];
  if (firstStep.frequency == 0) {
    digitalWrite(pin, LOW);
    frequency = 0;
    buzzerOn = false;
  } else {
    frequency = firstStep.frequency;
    halfPeriod = 500000 / frequency;
    lastMicros = micros();
    // The tone generation will start on the next update cycle
    buzzerOn = true;
  }
}

void Buzzer::stop() {
  // Stop both simple beeps and sequences
  beepsLeft = 0;
  isPlayingSequence = false;
  buzzerOn = false;
  digitalWrite(pin, LOW);
  frequency = 0;
  currentSequence = nullptr;
}

// ... (other convenience methods like bootBeep, etc.) ...

void Buzzer::windowsErrorBeep() {
  // Define the steps for a Windows-like error sound
  // This is an example; you'll need to adjust frequencies/durations
  static const ToneStep steps[] = {
    {784, 80},   // G5 note
    {0, 20},     // Brief silence
    {659, 80},   // E5 note
    {0, 20},     // Brief silence
    {523, 150},  // C5 note (slightly longer)
    {0, 50},     // Pause
    {494, 100},  // B4 note
    {0, 30}, 
    {440, 120},  // A4 note
    {0, 30},
    {392, 150},  // G4 note (longer, lower)
  };
  
  static const ToneSequence sequence = {
    steps,
    sizeof(steps) / sizeof(steps[0]),
    200 // Gap between repetitions (if we wanted to repeat)
  };
  
  playSequence(&sequence, 1); // Play once
}