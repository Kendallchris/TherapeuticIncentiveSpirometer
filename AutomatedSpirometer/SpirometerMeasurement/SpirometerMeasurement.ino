// TODO:
//       Adjust buzzing logic for reminder to allow for motor to work better

#include <TFT_eSPI.h>  // Include the TFT_eSPI library
#include <lvgl.h>
#include <Wire.h>         // Include Wire library for I2C communication
#include "HomeScreen.h"   // Include HomeScreen for managing the UI
#include "DataLogger.h"   // Include DataLogger for managing measurements
#include "MeasurementScreen.h"
#include "Accelerometer.h"
#include "ResetConfirmation.h"
#include "ReminderSystem.h"

// Pin definitions
const int ledPin = 13;         // Pin for the onboard LED
const int sensorPin = 16;      // Analog pin connected to the TCRT5000 sensor
const int buttonPin = 15;       // GPIO2 for measurement mode button (green)
const int resetButtonPin = 14;  // GPIO3 for wake-up button (red)
const int screenBacklightPin = 22;
const int vibrationMotorPin = 23;

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

// Angle threshold for tilt detection (in degrees)
const float tiltAngleThreshold = 30.0;

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
const unsigned long sleepDelay = 60000;  // 60 seconds inactivity
const unsigned long hourDuration = 3600000;
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

  // TFT init
  tft.init();
  tft.setRotation(1);

  // Accelerometer init
  Serial.println("Initializing Accelerometer...");
  accelerometer.initialize();
  accelerometer.saveReferenceOrientation();

  // Debug dimensions
  Serial.print("Runtime TFT_WIDTH: ");
  Serial.println(TFT_WIDTH);
  Serial.print("Runtime TFT_HEIGHT: ");
  Serial.println(TFT_HEIGHT);

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

  // Show home screen
  resetAllScreenFlags();
  homeScreen.show();
}

void loop() {
  lv_timer_handler();
  lv_task_handler();
  delay(5);

  if (isAsleep) {
    handleWakeup();
    return;
  }

  handleSleepMode();
  handleResetButton();
  handleMeasurementMode();

  reminderSystem.checkReminder();

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
      Serial.println("Entering measurement mode...");
      // Instead of resetAllScreenFlags(), do partial:
      ResetConfirmation::isActive = false;
      ReminderScreen::isActive = false;
      showingSuccess = false;

      measurementMode = true;
      awaitingObjectDetection = false;
      measurementStartTime = millis();
      previousSensorState = false; // After a successfull measurement this will be true - want to set back to false before assessing next measurement

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
    measurementScreen.updateCountdown();
    // Once countdown is finished
    if (!measurementScreen.isCountdownActive()) {
      Serial.println("Countdown complete. Now awaiting object detection...");
      awaitingObjectDetection = true;
    }
  }

  // If we are awaiting detection, check the sensor
  if (measurementMode && awaitingObjectDetection) {
    detectObject();
  }
}

void exitMeasurementMode() {
  Serial.println("Exiting measurement mode...");
  measurementMode = false;
  awaitingObjectDetection = false;
  showingSuccess = false;

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
    lastActivityTime = millis();
    reminderSystem.resetTimer();

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
