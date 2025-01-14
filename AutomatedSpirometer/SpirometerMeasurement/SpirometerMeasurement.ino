// TODO: when taking measurement and "Waiting for object, press green button to cancel" screen is up, the sensor is not sensing for an object (this is potentially okay)

// Currently: Success screen is not a thing - is not shown - just goes straight to home screen upon successful reading

#include <TFT_eSPI.h>  // Include the TFT_eSPI library
#include <lvgl.h>
#include <Wire.h>         // Include Wire library for I2C communication
#include "HomeScreen.h"   // Include HomeScreen for managing the UI
#include "DataLogger.h"   // Include DataLogger for managing measurements
#include "DataStorage.h"  // Include DataStorage for persistent storage
#include "MeasurementScreen.h"
#include "Accelerometer.h"

// Pin definitions
const int ledPin = 13;          // Pin for the onboard LED
const int sensorPin = 14;       // Analog pin A0 (Pin 14) connected to the TCRT5000 sensor's collector
const int buttonPin = 2;        // GPIO2 for measurement mode button GREEN
const int wakeUpButtonPin = 3;  // GPIO3 for wake-up button RED
const int screenBacklightPin = 4;

// ADXL345 I2C Address
#define ADXL345_I2C_ADDR 0x53

// Handle rotating the screen to landscape for TFT
// Override TFT_WIDTH and TFT_HEIGHT for rotation
#define ROTATED_WIDTH 480
#define ROTATED_HEIGHT 320
#undef TFT_WIDTH
#undef TFT_HEIGHT
#define TFT_WIDTH ROTATED_WIDTH
#define TFT_HEIGHT ROTATED_HEIGHT

// Angle threshold for tilt detection (in degrees)
const float tiltAngleThreshold = 30.0;

TFT_eSPI tft = TFT_eSPI();
lv_color_t buf[TFT_WIDTH * 20];
lv_disp_draw_buf_t draw_buf;

// Create instances
HomeScreen homeScreen(tft);
MeasurementScreen measurementScreen(tft);
DataLogger dataLogger;
Accelerometer accelerometer(ADXL345_I2C_ADDR, tiltAngleThreshold);

// Threshold for object detection; adjust this based on testing
const int detectionThreshold = 100;  // Experiment to find the optimal value

// State variables
bool previousSensorState = HIGH;       // Tracks previous sensor state; assume no object detected initially
bool measurementMode = false;          // Tracks if we are in measurement mode
bool isAsleep = false;                 // Tracks if the system is in "sleep" mode
bool awaitingObjectDetection = false;  // Tracks if waiting for object detection to succeed

// Timer variables
unsigned long lastActivityTime = 0;              // Tracks the last time there was activity
unsigned long lastResetTime = 0;                 // Tracks the last hourly reset time
const unsigned long sleepDelay = 60000;          // 60 seconds of inactivity before sleep
const unsigned long hourDuration = 3600000;      // 1 hour in milliseconds
const unsigned long detectionDelay = 5000;       // 5-second buffer time before checking for "No Object Detected"
unsigned long measurementStartTime = 0;          // Track when measurement mode starts
const unsigned long measurementTimeout = 60000;  // 60 seconds measurement timeout (if nothing is detected and used doesn't click to cancel, returns to home screen)

// Reference orientation
int16_t refX = 0, refY = 0, refZ = 0;

// Add this display flush callback for LVGL
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  Serial.println("Display flush called!");  // Debug output
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
  pinMode(wakeUpButtonPin, INPUT_PULLUP);

  // TFT initialization
  tft.init();
  tft.setRotation(1);  // Landscape mode

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
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
}

void loop() {
  lv_timer_handler();  // Handle LVGL tasks
  delay(5);            // Allow time for LVGL to process

  if (isAsleep) {
    handleWakeup();
    return;
  }

  handleSleepMode();
  handleHourlyReset();
  handleMeasurementMode();
  delay(200);  // Stable sensor reading interval
}

void handleWakeup() {
  if (digitalRead(wakeUpButtonPin) == LOW || accelerometer.detectTilt()) {
    wakeUp();
  }
}

void handleSleepMode() {
  if (millis() - lastActivityTime >= sleepDelay) {
    enterSleepMode();
  }
}

void handleHourlyReset() {
  if (millis() - lastResetTime >= hourDuration) {
    dataLogger.resetHourlyData();
    lastResetTime = millis();
    homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
  }
}

void handleMeasurementMode() {
  if (digitalRead(buttonPin) == LOW) {
    lastActivityTime = millis();
    if (!measurementMode) {
      Serial.println("Entering measurement mode...");
      measurementMode = true;
      awaitingObjectDetection = true;  // Set state before loading screen
      measurementStartTime = millis();
      measurementScreen.showWaitingWithCountdown();
      lv_refr_now(NULL);  // Force refresh
    } else if (awaitingObjectDetection) {
      Serial.println("Exiting measurement mode...");
      exitMeasurementMode();
    }
  }

  if (measurementMode && awaitingObjectDetection) {
    measurementScreen.updateCountdown();
    if (millis() - measurementStartTime >= measurementTimeout) {
      Serial.println("Measurement timed out. Returning to home screen.");
      exitMeasurementMode();
    }

    detectObject();
  }
}

void exitMeasurementMode() {
  measurementMode = false;
  awaitingObjectDetection = false;
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
  lv_refr_now(NULL);  // Force refresh
}

void detectObject() {
  if (measurementScreen.isCountdownActive()) {
    return;  // Skip object detection during countdown
  }

  int sensorValue = analogRead(sensorPin);
  bool objectDetected = (sensorValue < detectionThreshold);

  if (objectDetected != previousSensorState) {
    lastActivityTime = millis();
    if (objectDetected) {
      Serial.println("Object detected!");
      dataLogger.incrementMeasurement();
      measurementScreen.showSuccess();
      exitMeasurementMode();
    } else {
      Serial.println("No object detected.");
      measurementScreen.showNoObject();
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
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());

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
