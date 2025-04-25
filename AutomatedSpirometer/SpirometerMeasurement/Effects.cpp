#include "Effects.h"

int Effects::vibrationPin = -1;
int Effects::backlightPin = -1;

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

void Effects::begin(int vibPin, int backlight) {
  vibrationPin = vibPin;
  backlightPin = backlight;
  pinMode(vibrationPin, OUTPUT);
  digitalWrite(vibrationPin, LOW);
  pinMode(backlightPin, OUTPUT);
  digitalWrite(backlightPin, HIGH);
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
      digitalWrite(backlightPin, HIGH);
    }
  }
}

void Effects::stopAll() {
  vibrationActive = false;
  screenFlashActive = false;
  digitalWrite(vibrationPin, LOW);
  digitalWrite(backlightPin, HIGH);
}

bool Effects::isScreenFlashing() {
  return screenFlashActive;
}