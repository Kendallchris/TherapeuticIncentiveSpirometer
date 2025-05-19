#include "../include/Effects.h"

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
  ToneStep& step = toneSequence[currentToneIndex];

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

void Effects::startToneSequence(std::initializer_list<ToneStep> sequence) {
  clearToneQueue();  // Make sure nothing lingering
  toneCount = 0;

  for (const auto& step : sequence) {
    if (toneCount < maxToneSequenceLength) {
      toneSequence[toneCount++] = step;
    }
  }

  if (toneCount > 0) {
    tonePlaying = true;
    currentToneIndex = 0;
    toneStartTime = millis();
    tone(buzzerPin, toneSequence[0].frequency);
    inInterDelay = false;
  }
}

bool Effects::isScreenFlashing() {
  return screenFlashActive;
}

void Effects::triggerConfetti(lv_obj_t *parent) {
  // Predefined vibrant colors
  lv_color_t colors[] = {
    lv_color_hex(0xFF3B30), // red
    lv_color_hex(0xFF9500), // orange
    lv_color_hex(0xFFCC00), // yellow
    lv_color_hex(0x34C759), // green
    lv_color_hex(0x007AFF), // blue
    lv_color_hex(0x5856D6), // purple
    lv_color_hex(0xFF2D55)  // pink
  };
  const int numColors = sizeof(colors) / sizeof(colors[0]);

  for (int i = 0; i < 20; ++i) {
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 10, 10);  // Increased size
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dot, colors[rand() % numColors], LV_PART_MAIN); // ← replaced line
    lv_obj_set_style_border_width(dot, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(dot, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

    int start_x = rand() % (LV_HOR_RES - 10);
    int start_y = 0 - (rand() % 20);  // Closer to top of screen
    lv_obj_set_pos(dot, start_x, start_y);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, dot);
    lv_anim_set_exec_cb(&a, [](void *obj, int32_t v) {
      lv_obj_set_y((lv_obj_t *)obj, v);
    });
    lv_anim_set_values(&a, start_y, LV_VER_RES);
    lv_anim_set_time(&a, 2000 + rand() % 1000);  // Slower fall
    lv_anim_set_delay(&a, rand() % 300);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
  }

  // Sanity dot to confirm rendering (always shows at top)
  lv_obj_t *test_dot = lv_obj_create(parent);
  lv_obj_remove_style_all(test_dot);
  lv_obj_set_size(test_dot, 12, 12);
  lv_obj_set_style_radius(test_dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(test_dot, lv_color_hex(0xFF0000), LV_PART_MAIN);
  lv_obj_set_pos(test_dot, 60, 60);  // Middle of screen
}