#ifndef EFFECTS_H
#define EFFECTS_H

#include <Arduino.h>

class Effects {
public:
  static void begin(int vibPin, int backlightPin);
  static void beginTone(int buzzerPin);

  static void startVibration(int pulses, int onDuration, int offDuration);
  static void updateVibration();
  static void stopVibration(); // not really needed - will be handled in updateVibration

  static void startScreenFlash(int flashes, int onDuration, int offDuration);
  static void updateScreenFlash();
  static void stopScreenFlash();

  struct ToneStep {
    int frequency;
    int duration;
  };

  static void updateTone();
  static void stopTone();
  static void successTone();
  static void measurementsCompleteTone();
  static void reminderTone();
  static void startToneSequence(std::initializer_list<ToneStep> sequence);

  static bool isScreenFlashing();

private:
  // Vibration
  static int vibrationPin;
  static bool vibrationActive;
  static unsigned long vibrationStartTime;
  static int vibrationPulsesRemaining;
  static bool vibrationOnPhase;
  static int vibrationOnDuration;
  static int vibrationOffDuration;

  // Screen Flash
  static int backlightPin;
  static bool screenFlashActive;
  static unsigned long screenFlashStartTime;
  static int screenFlashesRemaining;
  static bool screenOnPhase;
  static int screenOnDuration;
  static int screenOffDuration;

  // Tone
  static int buzzerPin;
  static const int maxToneSequenceLength = 20;
  
  static ToneStep toneSequence[maxToneSequenceLength];
  static int toneCount;
  static int currentToneIndex;
  static bool tonePlaying;
  static unsigned long toneStartTime;
  static unsigned long interToneDelay;
  static bool inInterDelay;

  static void clearToneQueue();
  static void startToneSequence();
};

#endif