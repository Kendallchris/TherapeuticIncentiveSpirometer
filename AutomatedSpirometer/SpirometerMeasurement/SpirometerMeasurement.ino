// TODO:
//       Add motivational message after a few successful measurements (i.e. “Good job! Keep it up!”)
//       Whenever a new screen is loaded, make sure all other screen flags are set to false
//       Adjust buzzing logic for reminder to allow for motor to work better
//       Adjust success logic to make buzzing logic in the success function in measurement screen (have buzzing while displaying success message)

#include <TFT_eSPI.h>  // Include the TFT_eSPI library
#include <lvgl.h>
#include <Wire.h>         // Include Wire library for I2C communication
#include "HomeScreen.h"   // Include HomeScreen for managing the UI
#include "DataLogger.h"   // Include DataLogger for managing measurements
#include "DataStorage.h"  // Include DataStorage for persistent storage
#include "MeasurementScreen.h"
#include "Accelerometer.h"
#include "ResetConfirmation.h"
#include "ReminderSystem.h"


// Pin definitions
const int ledPin = 13;         // Pin for the onboard LED
const int sensorPin = 14;      // Analog pin A0 (Pin 14) connected to the TCRT5000 sensor's collector
const int buttonPin = 2;       // GPIO2 for measurement mode button GREEN
const int resetButtonPin = 3;  // GPIO3 for wake-up button RED
const int screenBacklightPin = 4;
const int vibrationMotorPin = 20;

// ADXL345 I2C Address
#define ADXL345_I2C_ADDR 0x53

// Define ILI9341 sleep and wake commands (ILI9341_DRIVER doesn't define these for some reason but ILI9488_DRIVER - used for 3.5" display - does)
#define TFT_SLPIN 0x10   // Sleep In
#define TFT_SLPOUT 0x11  // Sleep Out

// Handle rotating the screen to landscape for TFT
// Override TFT_WIDTH and TFT_HEIGHT for rotation
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

// Threshold for object detection; adjust this based on testing
const int detectionThreshold = 100;  // Experiment to find the optimal value

// State variables
bool previousSensorState = LOW;        // Tracks previous sensor state; assume no object detected initially
bool measurementMode = false;          // Tracks if we are in measurement mode
bool isAsleep = false;                 // Tracks if the system is in "sleep" mode
bool awaitingObjectDetection = false;  // Tracks if waiting for object detection to succeed
bool showingSuccess = false;           // Tracks if the success screen is currently displayed

// Timer variables
unsigned long lastActivityTime = 0;              // Tracks the last time there was activity
unsigned long lastResetTime = 0;                 // Tracks the last hourly reset time
const unsigned long sleepDelay = 60000;          // 60 seconds of inactivity before sleep
const unsigned long hourDuration = 3600000;      // 1 hour in milliseconds
const unsigned long detectionDelay = 5000;       // 5-second buffer time before checking for "No Object Detected"
unsigned long measurementStartTime = 0;          // Track when measurement mode starts
const unsigned long measurementTimeout = 60000;  // 60 seconds measurement timeout (if nothing is detected and used doesn't click to cancel, returns to home screen)
unsigned long successStartTime = 0;              // Tracks when the success screen was shown
const unsigned long successDuration = 2000;      // Success screen duration in milliseconds (2 seconds)

// Create instances
HomeScreen homeScreen(tft);
MeasurementScreen measurementScreen(tft, awaitingObjectDetection);
DataLogger dataLogger;
Accelerometer accelerometer(ADXL345_I2C_ADDR, tiltAngleThreshold);
ResetConfirmation resetScreen(tft, dataLogger);
ReminderSystem reminderSystem(vibrationMotorPin, screenBacklightPin, buttonPin, tft, isAsleep, dataLogger);
ReminderScreen reminderScreen(tft, dataLogger);

// Reference orientation
int16_t refX = 0, refY = 0, refZ = 0;

// Add this display flush callback for LVGL
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  // Serial.println("Display flush called!");  // Debug output
  uint16_t w = area->x2 - area->x1 + 1;
  uint16_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);  // Inform LVGL that flushing is done
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Pin setup
  pinMode(screenBacklightPin, OUTPUT);
  digitalWrite(screenBacklightPin, HIGH);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);
  // Vibration Setup
  pinMode(vibrationMotorPin, OUTPUT);
  digitalWrite(vibrationMotorPin, LOW);

  // TFT initialization
  tft.init();
  tft.setRotation(1);  // Landscape mode

  // Accelerometer initialization
  Serial.println("Initializing Accelerometer...");
  accelerometer.initialize();  // Ensure this call exists
  accelerometer.saveReferenceOrientation();

  // Verify runtime values
  Serial.print("Runtime TFT_WIDTH: ");
  Serial.println(TFT_WIDTH);
  Serial.print("Runtime TFT_HEIGHT: ");
  Serial.println(TFT_HEIGHT);

  // LVGL initialization
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, TFT_WIDTH * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.hor_res = TFT_WIDTH;   // Rotated width
  disp_drv.ver_res = TFT_HEIGHT;  // Rotated height
  lv_disp_drv_register(&disp_drv);

  // Debug LVGL resolution
  Serial.print("LVGL hor_res: ");
  Serial.println(disp_drv.hor_res);
  Serial.print("LVGL ver_res: ");
  Serial.println(disp_drv.ver_res);

  // Initial screen
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true));
}

void loop() {
  lv_timer_handler();  // Handle LVGL tasks
  lv_task_handler();
  delay(5);            // Allow time for LVGL to process

  // Handle success screen delay
  if (showingSuccess && (millis() - successStartTime >= successDuration)) {
    showingSuccess = false;  // Reset success screen state
    exitMeasurementMode();   // Transition back to the home screen
  }

  if (isAsleep) {
    handleWakeup();
    return;
  }

  handleSleepMode();
  handleResetButton();
  handleMeasurementMode();

  // Check if it's time for a reminder
  reminderSystem.checkReminder();

  delay(200);  // Stable sensor reading interval
}

void handleWakeup() {
  if (digitalRead(resetButtonPin) == LOW || digitalRead(buttonPin) == LOW || accelerometer.detectTilt()) {
    wakeUp();
  }
  // Check if it's time for a reminder
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
      delay(300);  // Debounce
    } else if (digitalRead(buttonPin) == LOW) {
      resetScreen.cancelReset();
      delay(300);  // Debounce
    }
  } else {
    if (digitalRead(resetButtonPin) == LOW && !measurementMode) {
      resetScreen.show();  // Show reset confirmation screen
      delay(300);          // Debounce delay
    }
  }
}

void handleMeasurementMode() {
    if (ReminderScreen::isActive) {
        if (digitalRead(buttonPin) == LOW) {  
            Serial.println("Dismissing reminder via hardware button...");
            reminderSystem.dismissReminder();
            delay(300);  // Debounce
        }
        return;  // Prevent measurement mode from activating
    }

    if (ResetConfirmation::isActive) return;  // ResetConfirmation takes priority

    if (digitalRead(buttonPin) == LOW) {
        lastActivityTime = millis();
        if (!measurementMode) {
            Serial.println("Entering measurement mode...");
            measurementMode = true;
            awaitingObjectDetection = false;
            measurementStartTime = millis();
            measurementScreen.showWaitingWithCountdown();
            lv_refr_now(NULL);
        } else {
            Serial.println("Measurement canceled.");
            measurementMode = false;
            awaitingObjectDetection = false;
            exitMeasurementMode();
        }
    }

    if (measurementMode && !awaitingObjectDetection) {
        measurementScreen.updateCountdown();

        if (!measurementScreen.isCountdownActive()) {
            Serial.println("Countdown complete. Transitioning to measurement phase.");
            awaitingObjectDetection = true;
        }
    }

    if (measurementMode && awaitingObjectDetection) {
        detectObject();
    }
}

void exitMeasurementMode() {
  Serial.println("Exiting measurement mode...");
  measurementMode = false;
  awaitingObjectDetection = false;
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true));
  lv_refr_now(NULL);  // Force refresh
}

void detectObject() {
  if (measurementScreen.isCountdownActive() || !awaitingObjectDetection) {
    return;
  }

  int sensorValue = analogRead(sensorPin);
  bool objectDetected = (sensorValue < detectionThreshold);

  if (objectDetected != previousSensorState) {
    lastActivityTime = millis();
    reminderSystem.resetTimer();  // Reset reminder timer after every measurement attempt

    if (objectDetected) {
      Serial.println("Object detected!");
      dataLogger.incrementMeasurement();

      // Trigger vibration and flash the screen
      vibrate(2000);
      //flashScreen(3);

      // Show success screen and start success timer
      measurementScreen.showSuccess();
      showingSuccess = true;        // Set success screen state
      successStartTime = millis();  // Record the start time
    } else {
      Serial.println("No object detected.");
      measurementScreen.showNoObject();
      exitMeasurementMode();  // Transition back to the home screen for "No Object"
    }
    previousSensorState = objectDetected;
  }
}

void initADXL345() {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(0x2D);  // Power control register
  Wire.write(0x08);  // Set to measurement mode
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

  // Restore display power
  turnOnDisplay();

  // Reload the LVGL screen
  Serial.println("Reloading Home Screen...");
  lv_scr_load(lv_scr_act());  // Ensure the current LVGL screen is loaded
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true));

  // Force a full refresh after waking up
  lv_refr_now(NULL);

  isAsleep = false;
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

// Probably want to move this to the Measurement screen success fucntion instead so can do this concurrently with showing success screen
void vibrate(int duration) {
  digitalWrite(vibrationMotorPin, HIGH);
  delay(duration);
  digitalWrite(vibrationMotorPin, LOW);
}

// void flashScreen(int flashes) {
//   for (int i = 0; i < flashes; i++) {
//     digitalWrite(screenBacklightPin, LOW);
//     delay(200);
//     digitalWrite(screenBacklightPin, HIGH);
//     delay(200);
//   }
//   turnOnDisplay();
//   //lv_scr_load(lv_scr_act());  // Ensure the current LVGL screen is loaded
//   homeScreen.show(dataLogger.getCurrentHourMeasurements(true));
//   //lv_refr_now(NULL);
// }
