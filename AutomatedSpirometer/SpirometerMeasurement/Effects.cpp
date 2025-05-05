#include "Effects.h"

int Effects::vibrationPin = -1;
int Effects::backlightPin = -1;
int Effects::buzzerPin = -1;

bool Effects::vibrationActive = false;
unsigned long Effects::vibrationStartTime = 0;
int Effects::vibrationPulsesRemaining = 0;
bool Effects::vibrationOnPhase = false;
int Effects::vibrationOnDuration = 0;
int Effects::vibrationOffDuration = 0;

bool Effects::screenFlashActive = false;
unsigned long Effects::screenFlashStartTime = 0;
int Effects::screenFlashesRemaining = 0;
bool Effects::screenOnPhase = false;
int Effects::screenOnDuration = 0;
int Effects::screenOffDuration = 0;

Effects::ToneStep Effects::toneSequence[maxToneSequenceLength];
int Effects::toneCount = 0;
int Effects::currentToneIndex = 0;
bool Effects::tonePlaying = false;
unsigned long Effects::toneStartTime = 0;
unsigned long Effects::interToneDelay = 30;
bool Effects::inInterDelay = false;

void Effects::begin(int vibPin, int backlight) {
  vibrationPin = vibPin;
  backlightPin = backlight;
  pinMode(vibrationPin, OUTPUT);
  digitalWrite(vibrationPin, LOW);
  pinMode(backlightPin, OUTPUT);
  digitalWrite(backlightPin, HIGH);
}

void Effects::beginTone(int buzzer) {
  buzzerPin = buzzer;
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

void Effects::startVibration(int pulses, int onDuration, int offDuration) {
  vibrationActive = true;
  vibrationPulsesRemaining = pulses;
  vibrationOnDuration = onDuration;
  vibrationOffDuration = offDuration;
  vibrationOnPhase = true;
  vibrationStartTime = millis();
  digitalWrite(vibrationPin, HIGH);
}

void Effects::updateVibration() {
  if (!vibrationActive) return;

  unsigned long now = millis();
  if (vibrationPulsesRemaining <= 0) {
    vibrationActive = false;
    digitalWrite(vibrationPin, LOW);
    return;
  }

  if (vibrationOnPhase && now - vibrationStartTime >= (unsigned long)vibrationOnDuration) {
    digitalWrite(vibrationPin, LOW);
    vibrationOnPhase = false;
    vibrationStartTime = now;
  } else if (!vibrationOnPhase && now - vibrationStartTime >= (unsigned long)vibrationOffDuration) {
    vibrationPulsesRemaining--;
    if (vibrationPulsesRemaining > 0) {
      digitalWrite(vibrationPin, HIGH);
      vibrationOnPhase = true;
      vibrationStartTime = now;
    } else {
      vibrationActive = false;
      digitalWrite(vibrationPin, LOW);
    }
  }
}

void Effects::stopVibration() {
  vibrationActive = false;
  vibrationPulsesRemaining = 0;
  vibrationOnPhase = false;
  digitalWrite(vibrationPin, LOW);
}

void Effects::startScreenFlash(int flashes, int onDuration, int offDuration) {
  screenFlashActive = true;
  screenFlashesRemaining = flashes;
  screenOnDuration = onDuration;
  screenOffDuration = offDuration;
  screenOnPhase = false;
  screenFlashStartTime = millis();
  digitalWrite(backlightPin, LOW);
}

void Effects::updateScreenFlash() {
  if (!screenFlashActive) return;

  unsigned long now = millis();
  if (screenOnPhase && now - screenFlashStartTime >= (unsigned long)screenOnDuration) {
    digitalWrite(backlightPin, LOW);
    screenOnPhase = false;
    screenFlashStartTime = now;
  } else if (!screenOnPhase && now - screenFlashStartTime >= (unsigned long)screenOffDuration) {
    screenFlashesRemaining--;
    if (screenFlashesRemaining > 0) {
      digitalWrite(backlightPin, HIGH);
      screenOnPhase = true;
      screenFlashStartTime = now;
    } else {
      screenFlashActive = false;
      digitalWrite(backlightPin, HIGH);  // Make sure backlight is ON

      // Fully wake the display once flashing is done
      extern void turnOnDisplay();
      turnOnDisplay();
    }
  }
}

void Effects::stopScreenFlash() {
  screenFlashActive = false;
  screenFlashesRemaining = 0;
  digitalWrite(backlightPin, HIGH);  // Make sure screen is ON after stopping
}

void Effects::startToneSequence() {
  if (toneCount > 0) {
    tonePlaying = true;
    currentToneIndex = 0;
    toneStartTime = millis();
    tone(buzzerPin, toneSequence[0].frequency);
  }
}

void Effects::updateTone() {
  if (!tonePlaying) return;

  unsigned long now = millis();
  ToneStep &step = toneSequence[currentToneIndex];

  if (!inInterDelay && now - toneStartTime >= (unsigned long)step.duration) {
    noTone(buzzerPin);
    inInterDelay = true;
    toneStartTime = now;
  } else if (inInterDelay && now - toneStartTime >= interToneDelay) {
    currentToneIndex++;
    if (currentToneIndex < toneCount) {
      tone(buzzerPin, toneSequence[currentToneIndex].frequency);
      toneStartTime = now;
      inInterDelay = false;
    } else {
      tonePlaying = false;
      toneCount = 0;
      inInterDelay = false;
      noTone(buzzerPin);
    }
  }
}

void Effects::clearToneQueue() {
  tonePlaying = false;
  toneCount = 0;
  currentToneIndex = 0;
  inInterDelay = false;
  noTone(buzzerPin);
}

void Effects::stopTone() {
  clearToneQueue();
}

void Effects::successTone() {
  clearToneQueue();
  toneSequence[toneCount++] = { 784, 200 };
  toneSequence[toneCount++] = { 880, 200 };
  toneSequence[toneCount++] = { 988, 200 };
  toneSequence[toneCount++] = { 1047, 300 };
  toneSequence[toneCount++] = { 784, 200 };
  toneSequence[toneCount++] = { 1319, 300 };
  toneSequence[toneCount++] = { 1175, 200 };
  toneSequence[toneCount++] = { 1568, 400 };
  startToneSequence();
}

void Effects::measurementsCompleteTone() {
  clearToneQueue();
  successTone();
  toneSequence[toneCount++] = { 0, 200 };
  toneSequence[toneCount++] = { 784, 200 };
  toneSequence[toneCount++] = { 880, 200 };
  toneSequence[toneCount++] = { 988, 200 };
  toneSequence[toneCount++] = { 1047, 300 };
  toneSequence[toneCount++] = { 784, 200 };
  toneSequence[toneCount++] = { 1319, 300 };
  toneSequence[toneCount++] = { 1175, 200 };
  toneSequence[toneCount++] = { 1568, 400 };
  startToneSequence();
}

void Effects::reminderTone() {
  clearToneQueue();
  toneSequence[toneCount++] = { 880, 200 };
  toneSequence[toneCount++] = { 988, 200 };
  toneSequence[toneCount++] = { 1047, 250 };
  toneSequence[toneCount++] = { 0, 200 };
  toneSequence[toneCount++] = { 1047, 100 };
  toneSequence[toneCount++] = { 1319, 150 };
  startToneSequence();
}

bool Effects::isScreenFlashing() {
  return screenFlashActive;
}