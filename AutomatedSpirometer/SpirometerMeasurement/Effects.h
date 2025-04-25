#ifndef EFFECTS_H
#define EFFECTS_H

#include <Arduino.h>

class Effects {
public:
  static void begin(int vibPin, int backlightPin);

  static void startVibration(int pulses, int onDuration, int offDuration);
  static void updateVibration();

  static void startScreenFlash(int flashes, int onDuration, int offDuration);
  static void updateScreenFlash();

  static void stopAll();

  static bool isScreenFlashing();

private:
  static int vibrationPin;
  static int backlightPin;

  static bool vibrationActive;
  static unsigned long vibrationStartTime;
  static int vibrationPulsesRemaining;
  static bool vibrationOnPhase;
  static int vibrationOnDuration;
  static int vibrationOffDuration;

  static bool screenFlashActive;
  static unsigned long screenFlashStartTime;
  static int screenFlashesRemaining;
  static bool screenOnPhase;
  static int screenOnDuration;
  static int screenOffDuration;
};

#endif