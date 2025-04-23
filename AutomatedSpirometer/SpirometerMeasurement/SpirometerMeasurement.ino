// TODO:
//       Utilize Vibrate() instead of explicitly passing and digitalwriting to activate vibration motor
//       Adjust reminder logic (currently takes number of flashes and duration but should hardwire instead to not pass so many variables)
//       Change buttons - right now a lot of them are setup as actual touchscreen buttons (make just dummy buttons)

#include <TFT_eSPI.h>
#include <lvgl.h>
#include <Wire.h>  // Include Wire library for I2C communication
// #include <esp_task_wdt.h> // Watch Dog Task (will reset device if times out/freezes up for too long) *couldn't figure out how to get the library to be recognized*
#include "HomeScreen.h"
#include "DataLogger.h"
#include "MeasurementScreen.h"
#include "Accelerometer.h"
#include "ResetConfirmation.h"
#include "ReminderSystem.h"

// Pin definitions
const int ledPin = 13;          // Pin for the onboard LED
const int sensorPin = 16;       // Analog pin connected to the TCRT5000 sensor
const int buttonPin = 15;       // Measurement mode button (green)
const int resetButtonPin = 14;  // For wake-up button (red)
const int screenBacklightPin = 22;
const int vibrationMotorPin = 23;
const int buzzerPin = 9;  // Using PIEZO BUZZER TRANSDUCER

// ADXL345 I2C Address
#define ADXL345_I2C_ADDR 0x53

// Define ILI9341 sleep and wake commands (ILI9341_DRIVER doesn't define these for some reason but ILI9488_DRIVER - used for 3.5" display - does)
#define TFT_SLPIN 0x10   // Sleep In
#define TFT_SLPOUT 0x11  // Sleep Out

// Override for rotation
#define ROTATED_WIDTH 320
#define ROTATED_HEIGHT 240
#undef TFT_WIDTH
#undef TFT_HEIGHT
#define TFT_WIDTH ROTATED_WIDTH
#define TFT_HEIGHT ROTATED_HEIGHT

#define MAX_TONE_SEQUENCE_LENGTH 20

// Angle threshold for tilt detection (in degrees)
const float tiltAngleThreshold = 30.0;

struct ToneStep {
  int frequency;
  int duration;
};

TFT_eSPI tft = TFT_eSPI();
lv_color_t buf[TFT_WIDTH * 20];
lv_disp_draw_buf_t draw_buf;

// Threshold for object detection
const int detectionThreshold = 100;

// State variables
bool previousSensorState = LOW;
bool measurementMode = false;
bool isAsleep = false;
bool awaitingObjectDetection = false;
bool showingSuccess = false;

// Timer variables
unsigned long lastActivityTime = 0;
unsigned long lastResetTime = 0;
const unsigned long sleepDelay = 120000;  // 120 seconds inactivity
const unsigned long detectionDelay = 5000;
unsigned long measurementStartTime = 0;
const unsigned long measurementTimeout = 60000;
unsigned long successStartTime = 0;
const unsigned long successDuration = 8000;

// Create instances
HomeScreen homeScreen(tft);
MeasurementScreen measurementScreen(
  tft,
  awaitingObjectDetection,
  showingSuccess,
  homeScreen,
  vibrationMotorPin);
DataLogger dataLogger;
Accelerometer accelerometer(ADXL345_I2C_ADDR, tiltAngleThreshold);
ResetConfirmation resetScreen(tft, dataLogger);
ReminderSystem reminderSystem(
  vibrationMotorPin,
  screenBacklightPin,
  buttonPin,
  tft,
  isAsleep,
  dataLogger);
ReminderScreen reminderScreen(tft, dataLogger);

// instance and global variables for buzzer
ToneStep toneSequence[MAX_TONE_SEQUENCE_LENGTH];
int toneCount = 0;
int currentToneIndex = 0;
bool tonePlaying = false;
unsigned long toneStartTime = 0;
unsigned long interToneDelay = 30;  // Delay between tones
bool inInterDelay = false;

// Forward declarations
void enterSleepMode();
void wakeUp();
void handleSleepMode();
void handleWakeup();
void handleResetButton();
void handleMeasurementMode();
void exitMeasurementMode();
void detectObject();
void vibrate(int duration);
void successTone();
void measurementsCompleteTone();
void reminderTone();
void stopTone();

// Use a safer "partial reset" that carefully sets only certain flags, if truly needed.
void resetAllScreenFlags() {
  Serial.println("[DEBUG] resetAllScreenFlags() called...");
  measurementMode = false;
  awaitingObjectDetection = false;
  showingSuccess = false;
  ResetConfirmation::isActive = false;
  ReminderScreen::isActive = false;
}

/** LVGL Display flush callback **/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t w = area->x2 - area->x1 + 1;
  uint16_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(screenBacklightPin, OUTPUT);
  digitalWrite(screenBacklightPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);

  pinMode(vibrationMotorPin, OUTPUT);
  digitalWrite(vibrationMotorPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // TFT init
  tft.init();
  tft.setRotation(1);

  // Accelerometer init
  Serial.println("Initializing Accelerometer...");
  accelerometer.initialize();
  accelerometer.saveReferenceOrientation();

  // LVGL init
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, TFT_WIDTH * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.hor_res = TFT_WIDTH;
  disp_drv.ver_res = TFT_HEIGHT;
  lv_disp_drv_register(&disp_drv);

  // Setup watchdog task
  // esp_task_wdt_init(5, true);  // 5 seconds timeout, panic=true (auto reset)
  // esp_task_wdt_add(NULL);      // Add current task (loop) to watchdog

  // Show home screen
  resetAllScreenFlags();
  homeScreen.show();
}

void loop() {
  lv_timer_handler();
  //lv_task_handler();

  updateToneSequence();  // async audio

  if (measurementMode && !awaitingObjectDetection && measurementScreen.isCountdownActive()) {
    measurementScreen.updateCountdown();
  }

  if (isAsleep) {
    handleWakeup();
    return;
  }

  handleSleepMode();
  handleResetButton();
  handleMeasurementMode();

  reminderSystem.checkReminder();

  // 🧠 Optional watchdog support
  // esp_task_wdt_reset();

  delay(200);
}


void handleWakeup() {
  if (digitalRead(resetButtonPin) == LOW || digitalRead(buttonPin) == LOW || accelerometer.detectTilt()) {
    wakeUp();
  }
  // Even while asleep, we check if a reminder is due
  reminderSystem.checkReminder();
}

void handleSleepMode() {
  if (millis() - lastActivityTime >= sleepDelay) {
    enterSleepMode();
  }
}

void handleResetButton() {
  if (ResetConfirmation::isActive) {
    if (digitalRead(resetButtonPin) == LOW) {
      resetScreen.confirmReset();
      delay(300);
    } else if (digitalRead(buttonPin) == LOW) {
      resetScreen.cancelReset();
      delay(300);
    }
  } else {
    // Only if not measuring
    if (digitalRead(resetButtonPin) == LOW && !measurementMode) {
      resetAllScreenFlags();
      ResetConfirmation::isActive = true;
      resetScreen.show();
      delay(300);
    }
  }
}

void handleMeasurementMode() {
  // If Reminder Screen is active, allow dismissing
  if (ReminderScreen::isActive) {
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("Dismissing reminder via hardware button...");
      reminderSystem.dismissReminder();
      delay(300);
    }
    return;
  }

  // If reset screen is up, don't overlap
  if (ResetConfirmation::isActive) return;

  // If success screen is active, check if we auto-exit or manual-exit
  if (showingSuccess) {
    Serial.println("[DEBUG] Success screen active, checking timeout...");

    // Auto exit
    if (millis() - successStartTime >= successDuration) {
      Serial.println("[DEBUG] Success screen timed out, returning home...");
      showingSuccess = false;
      measurementScreen.clearSuccessState();
      return;
    }
    // Manual exit
    if (digitalRead(buttonPin) == LOW || digitalRead(resetButtonPin) == LOW) {
      Serial.println("[DEBUG] Button pressed, returning home from success...");
      showingSuccess = false;
      measurementScreen.clearSuccessState();
      delay(300);
    }
    return;
  }

  // Now handle toggling measurement mode with the green button
  if (digitalRead(buttonPin) == LOW) {
    lastActivityTime = millis();

    if (!measurementMode) {
      lastActivityTime = millis();
      reminderSystem.resetTimer();

      Serial.println("Entering measurement mode...");
      // Instead of resetAllScreenFlags(), do partial:
      ResetConfirmation::isActive = false;
      ReminderScreen::isActive = false;
      showingSuccess = false;

      measurementMode = true;
      awaitingObjectDetection = false;
      measurementStartTime = millis();
      previousSensorState = false;  // After a successfull measurement this will be true - want to set back to false before assessing next measurement

      measurementScreen.showWaitingWithCountdown();
      lv_refr_now(NULL);
    } else {
      // This means we were in measurementMode; so user pressed again
      Serial.println("Measurement canceled via button.");
      measurementMode = false;
      awaitingObjectDetection = false;
      exitMeasurementMode();
    }
  }

  // If measurement mode is on but we haven't started object detection yet, keep countdown updated
  if (measurementMode && !awaitingObjectDetection) {
    static unsigned long lastTick = 0;
    if (measurementMode && !awaitingObjectDetection) {
      if (millis() - lastTick >= 1000) {  // 1 s cadence
        measurementScreen.updateCountdown();
        lastTick = millis();
      }
    }
    // Once countdown is finished
    if (!measurementScreen.isCountdownActive()) {
      Serial.println("Countdown complete. Now awaiting object detection...");
      awaitingObjectDetection = true;
    }
  }

  // If we are awaiting detection, check the sensor
  if (measurementMode && awaitingObjectDetection) {
    detectObject();

    // Timeout handling if object not detected after measurementTimeout
    if (millis() - measurementStartTime > measurementTimeout) {
      Serial.println("[TIMEOUT] Measurement timeout. No object detected. Exiting measurement mode.");
      measurementScreen.showNoObject();
      exitMeasurementMode();
      return;
    }
  }
}

void exitMeasurementMode() {
  Serial.println("Exiting measurement mode...");
  measurementMode = false;
  awaitingObjectDetection = false;
  showingSuccess = false;

  stopTone();  // precaution in case success screen is dismissed indirectly
  homeScreen.show();
  lv_refr_now(NULL);
}

void detectObject() {
  // Safety check
  if (measurementScreen.isCountdownActive() || !awaitingObjectDetection) {
    return;
  }

  int sensorValue = analogRead(sensorPin);
  bool objectDetected = (sensorValue < detectionThreshold);

  // We only react if the sensor state changes
  if (objectDetected != previousSensorState) {
    if (objectDetected) {
      Serial.println("Object detected!");

      int currentCount = dataLogger.getCurrentHourMeasurements(true);

      if (currentCount < 10) {
        // We can still record this measurement
        dataLogger.incrementMeasurement();  // physically capped at 10
        currentCount++;                     // because we just incremented

        int percentageComplete = (currentCount * 100) / 10;
        measurementScreen.showSuccess(currentCount, percentageComplete);

      } else {
        // They are already at 10, so let's show "unrecorded" success
        measurementScreen.showUnrecordedSuccess();
      }

      showingSuccess = true;
      successStartTime = millis();
    } else {
      Serial.println("No object detected. Returning home...");
      measurementScreen.showNoObject();
      exitMeasurementMode();
    }

    previousSensorState = objectDetected;
  }
}

void initADXL345() {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
}

void enterSleepMode() {
  Serial.println("Entering sleep mode...");
  accelerometer.saveReferenceOrientation();
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
  digitalWrite(ledPin, LOW);
  digitalWrite(screenBacklightPin, LOW);
  isAsleep = true;
}

void wakeUp() {
  Serial.println("Waking up from sleep mode...");

  turnOnDisplay();

  // partial reset
  isAsleep = false;
  measurementMode = false;
  awaitingObjectDetection = false;
  showingSuccess = false;
  ResetConfirmation::isActive = false;
  ReminderScreen::isActive = false;

  Serial.println("Reloading Home Screen after wake...");
  homeScreen.show();

  lv_refr_now(NULL);

  lastActivityTime = millis();
}

void turnOnDisplay() {
  digitalWrite(screenBacklightPin, HIGH);
  delay(50);
  tft.writecommand(TFT_SLPOUT);
  delay(150);
  tft.writecommand(TFT_DISPON);
  delay(50);
}

void vibrate(int duration) {
  digitalWrite(vibrationMotorPin, HIGH);
  delay(duration);
  digitalWrite(vibrationMotorPin, LOW);
}

void queueTone(int frequency, int duration) {
  if (toneCount < MAX_TONE_SEQUENCE_LENGTH) {
    toneSequence[toneCount++] = { frequency, duration };
  }
}

void clearToneQueue() {
  tonePlaying = false;
  toneCount = 0;
  currentToneIndex = 0;
  inInterDelay = false;
  noTone(buzzerPin);
}

void startToneSequence() {
  if (toneCount > 0) {
    tonePlaying = true;
    currentToneIndex = 0;
    toneStartTime = millis();
    tone(buzzerPin, toneSequence[0].frequency);
  }
}

void updateToneSequence() {
  if (!tonePlaying || !showingSuccess) {
    stopTone();  // ensures tone stops instantly if screen was closed
    return;
  }

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

// Public API functions
void successTone() {
  clearToneQueue();
  queueTone(784, 200);
  queueTone(880, 200);
  queueTone(988, 200);
  queueTone(1047, 300);
  queueTone(784, 200);
  queueTone(1319, 300);
  queueTone(1175, 200);
  queueTone(1568, 400);
  startToneSequence();
}

void measurementsCompleteTone() {
  clearToneQueue();
  successTone();
  queueTone(0, 200);  // Rest
  queueTone(784, 200);
  queueTone(880, 200);
  queueTone(988, 200);
  queueTone(1047, 300);
  queueTone(784, 200);
  queueTone(1319, 300);
  queueTone(1175, 200);
  queueTone(1568, 400);
  startToneSequence();
}

void reminderTone() {
  clearToneQueue();

  // Rising melody (gentle but noticeable)
  queueTone(880, 200);   // A5
  queueTone(988, 200);   // B5
  queueTone(1047, 250);  // C6
  queueTone(0, 200);     // Pause

  // Double pulse ending
  queueTone(1047, 100);  // C6
  queueTone(1319, 150);  // E6

  startToneSequence();
}

void stopTone() {
  clearToneQueue();
}
