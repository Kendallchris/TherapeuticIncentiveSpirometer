// TODO:

#include <TFT_eSPI.h>
#include <lvgl.h>
#include <Wire.h>
#include <Snooze.h>
#include "HomeScreen.h"
#include "DataLogger.h"
#include "MeasurementScreen.h"
#include "Accelerometer.h"
#include "ResetConfirmation.h"
#include "ReminderSystem.h"
#include "ReminderScreen.h"
#include "Effects.h"

// Pin definitions
const int ledPin = 13;          // Pin for the onboard LED
const int sensorPin = 16;       // Analog pin connected to the TCRT5000 sensor
const int buttonPin = 15;       // Measurement mode button (green)
const int resetButtonPin = 14;  // For wake-up button (red)
const int screenBacklightPin = 22;
const int vibrationMotorPin = 23;
const int buzzerPin = 9;        // Using PIEZO BUZZER TRANSDUCER
const int accInteruptPin = 17;  // INT1 hardware interrupt pin for accelerometer to wake from sleep

SnoozeDigital snoozeButton;       // Wake by green button
SnoozeDigital snoozeResetButton;  // Wake by reset button
SnoozeTimer snoozeTimer;          // Wake by reminder timer
SnoozeDigital snoozeAccel;        // Accelerometer INT1
SnoozeBlock config(snoozeButton, snoozeResetButton, snoozeTimer, snoozeAccel);

// ADXL345 I2C Address
#define ADXL345_I2C_ADDR 0x53

// Define ILI9341 sleep and wake commands (ILI9341_DRIVER doesn't define these for some reason but ILI9488_DRIVER - used for 3.5" display - does)
#define TFT_SLPIN 0x10   // Sleep In
#define TFT_SLPOUT 0x11  // Sleep Out

// Override for rotation
#define ROTATED_WIDTH 240
#define ROTATED_HEIGHT 320
#undef TFT_WIDTH
#undef TFT_HEIGHT
#define TFT_WIDTH ROTATED_WIDTH
#define TFT_HEIGHT ROTATED_HEIGHT

#define MAX_TONE_SEQUENCE_LENGTH 20
#define REMINDER_SOON_THRESHOLD_MS 30000  // 30 seconds threshold to trigger reminder immediately before sleep

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
volatile bool accelerometerTriggered = false;
volatile bool buttonPressed = false;
volatile bool resetButtonPressed = false;

// Timer variables
unsigned long lastActivityTime = 0;
unsigned long lastResetTime = 0;
const unsigned long sleepDelay = 60000;  // 60 seconds inactivity
const unsigned long detectionDelay = 5000;
unsigned long measurementStartTime = 0;
const unsigned long measurementTimeout = 50000;  // > 50 seconds on measurement = fail ... return home
unsigned long successStartTime = 0;
const unsigned long successDuration = 8000;

// Create instances
HomeScreen homeScreen(tft);
MeasurementScreen measurementScreen(
  tft,
  awaitingObjectDetection,
  showingSuccess,
  homeScreen);
DataLogger dataLogger;
Accelerometer accelerometer(ADXL345_I2C_ADDR);
ResetConfirmation resetScreen(tft, dataLogger);
ReminderSystem reminderSystem(
  buttonPin,
  tft,
  isAsleep,
  dataLogger);
ReminderScreen reminderScreen(tft, dataLogger);

// Forward declarations
void enterSleepMode();
void wakeUp();
void handleSleepMode();
void handleResetButton();
void handleMeasurementMode();
void exitMeasurementMode();
void detectObject();
void onAccelerometerInterrupt();

void onAccelerometerInterrupt() {
  accelerometerTriggered = true;
  accelerometer.clearInterrupt();
}

void onButtonInterrupt() {
  buttonPressed = true;
}

void onResetButtonInterrupt() {
  resetButtonPressed = true;
}

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

  attachInterrupt(digitalPinToInterrupt(accInteruptPin), onAccelerometerInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(resetButtonPin), onResetButtonInterrupt, FALLING);

  Effects::beginTone(buzzerPin);  // could probably combine this with 'begin' for simplicity
  Effects::begin(vibrationMotorPin, screenBacklightPin);

  // TFT init
  tft.init();
  tft.setRotation(0);

  // Accelerometer init
  Serial.println("Initializing Accelerometer...");
  accelerometer.initialize();
  accelerometer.setupMotionInterrupt();

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

  // Configure Snooze wakeup pins
  snoozeButton.pinMode(buttonPin, INPUT_PULLUP, FALLING);            // Green button press
  snoozeResetButton.pinMode(resetButtonPin, INPUT_PULLUP, FALLING);  // Red button press
  snoozeAccel.pinMode(accInteruptPin, INPUT_PULLUP, RISING);

  // Show home screen
  resetAllScreenFlags();
  homeScreen.show();
}

void loop() {
  // Sleep mode will only see if it need to turn on/activate reminder
  if (isAsleep) {
    delay(50);
    return;
  }

  lv_timer_handler();

  // async feedback functions
  Effects::updateTone();
  Effects::updateVibration();
  Effects::updateScreenFlash();

  handleSleepMode();
  handleResetButton();
  handleMeasurementMode();
  reminderSystem.checkReminder();

  delay(100);
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

    // Auto exit after timeout
    if (millis() - successStartTime >= successDuration) {
      Serial.println("[DEBUG] Success screen timed out, returning home...");
      showingSuccess = false;
      measurementScreen.clearSuccessState();
      return;
    }
    // Manual exit with button
    if (digitalRead(buttonPin) == LOW || digitalRead(resetButtonPin) == LOW) {
      Serial.println("[DEBUG] Button pressed, returning home from success...");
      showingSuccess = false;
      measurementScreen.clearSuccessState();
      delay(300);
    }
    return;
  }

  // --- Normal measurement handling ---

  // Now handle toggling measurement mode with the green button
  if (digitalRead(buttonPin) == LOW) {
    lastActivityTime = millis();

    if (!measurementMode) {
      reminderSystem.resetTimer();

      Serial.println("Entering measurement mode...");
      // Instead of resetAllScreenFlags(), do partial:
      ResetConfirmation::isActive = false;
      ReminderScreen::isActive = false;
      showingSuccess = false;

      measurementMode = true;
      awaitingObjectDetection = false;
      measurementStartTime = millis();
      previousSensorState = false;  // After a successful measurement this will be true - want to reset it before next

      measurementScreen.showWaitingWithCountdown();
      lv_refr_now(NULL);
    } else {
      // Already measuring, so cancel measurement
      Serial.println("Measurement canceled via button.");
      measurementMode = false;
      awaitingObjectDetection = false;
      exitMeasurementMode();
    }
    delay(300);  // debounce
    return;      // Important: stop here after button press
  }

  // --- Countdown ticking phase ---
  static unsigned long lastCountdownTick = 0;
  unsigned long now = millis();
  if (measurementMode && !awaitingObjectDetection) {
    if ((now - lastCountdownTick) >= 1000) {  // Every 1000ms
      measurementScreen.updateCountdown();
      lastCountdownTick = now;
    }

    // After countdown finishes, switch to waiting for object
    if (!measurementScreen.isCountdownActive()) {
      Serial.println("Countdown complete. Now awaiting object detection...");
      awaitingObjectDetection = true;
    }
    return;
  }

  // --- Object detection phase ---
  if (measurementMode && awaitingObjectDetection) {
    detectObject();

    // Timeout handling if object not detected after measurementTimeout
    if ((now - measurementStartTime) > measurementTimeout) {
      Serial.println("[TIMEOUT] Measurement timeout. No object detected. Exiting measurement mode.");
      measurementScreen.showNoObject();
      exitMeasurementMode();
    }
  }
}

void exitMeasurementMode() {
  Serial.println("[DEBUG] Exiting measurement mode...");

  // Stop any ongoing effects
  Effects::stopTone();  // precaution: ensure no tone is hanging
  Effects::stopVibration();

  // Reset measurement-related states
  measurementMode = false;
  awaitingObjectDetection = false;
  showingSuccess = false;

  // Reset screens if needed
  ResetConfirmation::isActive = false;
  ReminderScreen::isActive = false;

  // Reload the home screen cleanly
  homeScreen.show();
  lv_scr_load(lv_scr_act());  // Safer: explicitly reload the active screen
  lv_refr_now(NULL);          // Force an immediate refresh (avoid stale screen)

  // Update lastActivityTime to prevent immediate sleep after exit
  lastActivityTime = millis();
}

void detectObject() {
  // Safety check
  if (measurementScreen.isCountdownActive() || !awaitingObjectDetection) {
    return;
  }

  static int lastSensorValue = 0;
  int sensorValue = analogRead(sensorPin);

  // Simple smoothing: average current and last reading
  int averagedValue = (sensorValue + lastSensorValue) / 2;
  lastSensorValue = sensorValue;

  bool objectDetected = (averagedValue < detectionThreshold);

  // Only react if the sensor state has changed
  if (objectDetected != previousSensorState) {
    if (objectDetected) {
      Serial.println("Object detected!");

      int currentCount = dataLogger.getCurrentMeasurements();

      if (currentCount < 10) {
        dataLogger.incrementMeasurement();
        currentCount++;

        int percentageComplete = (currentCount * 100) / 10;
        measurementScreen.showSuccess(currentCount, percentageComplete);

      } else {
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

// Check why we woke and if we should continue to fully wake or return to sleep
void lightWakeCheck() {
  Serial.println("[DEBUG] Light wake triggered…");

  bool fullWakeRequired = false;
  bool callReminderOnWake = false;

  /* ---------- what caused the wake? ---------- */
  if (buttonPressed || resetButtonPressed) {  // buttons
    Serial.println("[DEBUG] Button wake detected!");
    fullWakeRequired = true;
    buttonPressed = false;
    resetButtonPressed = false;

  } else if (accelerometerTriggered) {  // INT1 (ADXL345)
    accelerometerTriggered = false;
    Serial.println("[DEBUG] Accelerometer triggered – full wake!");
    fullWakeRequired = true;  // ← always wake now
                              //    (tilt test removed)

  } else {  // timer wake
    Serial.println("[DEBUG] Timer wake (light wake only).");

    reminderSystem.handleTimerWake();  // re‑align timer

    if (millis() - reminderSystem.getLastReminderTime() >= reminderSystem.getReminderInterval()) {
      Serial.println("[DEBUG] Reminder due after timer wake!");
      fullWakeRequired = true;
      callReminderOnWake = true;  // defer until awake
    }
  }

  /* ---------- decide what to do next ---------- */
  if (fullWakeRequired) {
    performFullWake();  // power display, etc.
    if (callReminderOnWake) reminderSystem.triggerReminder();
  } else {
    Serial.println("[DEBUG] No full wake needed. Re‑arming INT1 and sleeping…");
    rearmAccelerometerAfterWake();
    Serial.flush();
    Snooze.sleep(config);  // go straight back out
  }
}

void performFullWake() {
  turnOnDisplay();        // FIRST - power up the display properly
  resetAllScreenFlags();  // Clear screen states

  isAsleep = false;  // Update sleep state

  Serial.println("[DEBUG] Reloading Home Screen after full wake...");
  homeScreen.show();
  lv_refr_now(NULL);  // Immediate screen redraw

  rearmAccelerometerAfterWake();  // 🛠️ now use helper instead of duplicating

  reminderSystem.resetTimer();

  lastActivityTime = millis();  // Reset idle timer
}

void enterSleepMode() {
  Serial.println("[DEBUG] Preparing to enter light sleep mode...");

  // Calculate time until next reminder
  unsigned long timeSinceLastReminder = millis() - reminderSystem.getLastReminderTime();
  unsigned long timeUntilNextReminder = reminderSystem.getReminderInterval() - timeSinceLastReminder;

  Serial.print("[DEBUG] Time until next reminder (ms): ");
  Serial.println(timeUntilNextReminder);

  if (timeUntilNextReminder <= REMINDER_SOON_THRESHOLD_MS) {
    Serial.println("[DEBUG] Reminder due soon, triggering immediately before sleep.");
    reminderSystem.triggerReminder();
    reminderSystem.resetTimer();
    lastActivityTime = millis();
    return;
  }

  unsigned long sleepIntervalMs = timeUntilNextReminder;
  if (sleepIntervalMs == 0) {
    sleepIntervalMs = 100;  // Prevent 0 sleep interval crash
  }

  reminderSystem.prepareForSleep(sleepIntervalMs);

  uint32_t sleepIntervalSec = (sleepIntervalMs + 999) / 1000;  // round up

  if (sleepIntervalSec == 0) {
    sleepIntervalSec = 1;  // Snooze requires minimum 1 second
  }

  Serial.print("[DEBUG] Sleep interval set to (seconds): ");
  Serial.println(sleepIntervalSec);

  snoozeTimer.setTimer(sleepIntervalSec);

  // Put display to sleep
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
  digitalWrite(ledPin, LOW);
  digitalWrite(screenBacklightPin, LOW);

  isAsleep = true;

  Serial.flush();
  Snooze.sleep(config);

  // --- 🛠️ AFTER WAKEUP ---
  isAsleep = false;

  rearmAccelerometerAfterWake();  // 🛠️ after waking up, reconfigure accelerometer

  lightWakeCheck();  // Then check if full wake needed
}

void wakeUp() {
  Serial.println("[DEBUG] Calling performFullWake...");
  performFullWake();
}

void turnOnDisplay() {
  digitalWrite(screenBacklightPin, HIGH);
  delay(50);
  tft.writecommand(TFT_SLPOUT);
  delay(150);
  tft.writecommand(TFT_DISPON);
  delay(50);
}

void rearmAccelerometerAfterWake() {
  Serial.println("[DEBUG] Re‑arming accelerometer…");

  /* 1️ Re‑start I²C HW in case LPI2C clock stopped */
  Wire.begin();
  delay(1);

  /* 2️ Fresh ADXL345 configuration */
  accelerometer.initialize();            // put in measure mode
  accelerometer.setupMotionInterrupt();  // map ACTIVITY → INT1 etc.

  /* 3️ Clear any latched interrupt line **before** we re‑attach */
  accelerometer.clearInterrupt();

  /* 4️ (Re)‑attach INT1 as CHANGE edge so it will fire even if the
           line was left HIGH after a previous event. */
  detachInterrupt(digitalPinToInterrupt(accInteruptPin));
  attachInterrupt(digitalPinToInterrupt(accInteruptPin), onAccelerometerInterrupt, RISING);

  /* 5️ Reset flag so the next edge is seen as new */
  accelerometerTriggered = false;

  Serial.println("[DEBUG] Accelerometer fully re‑armed.");
}
