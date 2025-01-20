// TODO: when taking measurement and "Waiting for object, press green button to cancel" screen is up, the sensor is not sensing for an object (this is potentially okay)

#include <TFT_eSPI.h>     // Include the TFT_eSPI library
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

// Define ILI9341 sleep and wake commands (ILI9341_DRIVER doesn't define these for some reason but ILI9488_DRIVER - used for 3.5" display - does)
#define TFT_SLPIN  0x10  // Sleep In
#define TFT_SLPOUT 0x11  // Sleep Out

// Angle threshold for tilt detection (in degrees)
const float tiltAngleThreshold = 30.0;

// Create instances
TFT_eSPI tft = TFT_eSPI();
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
const unsigned long sleepDelay = 10000;          // 10 seconds of inactivity before sleep
const unsigned long hourDuration = 3600000;      // 1 hour in milliseconds
const unsigned long detectionDelay = 5000;       // 5-second buffer time before checking for "No Object Detected"
unsigned long measurementStartTime = 0;          // Track when measurement mode starts
const unsigned long measurementTimeout = 60000;  // 60 seconds measurement timeout (if nothing is detected and used doesn't click to cancel, returns to home screen)

// Reference orientation
int16_t refX = 0, refY = 0, refZ = 0;

void setup() {
  // Set up serial communication
  Serial.begin(9600);  // Start serial communication at 9600 baud rate

  // Set up pins
  pinMode(ledPin, OUTPUT);              // Set LED pin as output for user control
  pinMode(buttonPin, INPUT);            // Set measurement button pin as input
  pinMode(wakeUpButtonPin, INPUT);      // Set wake-up button pin as input
  pinMode(screenBacklightPin, OUTPUT);  // Set backlight control pin as output

  // Turn on the backlight at startup
  digitalWrite(screenBacklightPin, HIGH);

  // Initialize the display
  tft.init();                                                                                              // Initialize the display
  tft.setRotation(1);                                                                                      // Set display orientation (adjust if needed)
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());  // Display the initial home screen with measurements

  // Initialize accelerometer
  Wire.begin();
  accelerometer.initialize();

  // Initialize timers
  lastActivityTime = millis();
  lastResetTime = millis();
}

void loop() {
  // Check for wake-up button press or accelerometer tilt if the system is in sleep mode
  if (isAsleep) {
    if (digitalRead(wakeUpButtonPin) == LOW || accelerometer.detectTilt()) {
      wakeUp();
    }
    return;
  }

  // Check if inactivity threshold has been reached to enter sleep mode
  if (millis() - lastActivityTime >= sleepDelay) {
    enterSleepMode();
  }

  // Check if an hour has passed since the last reset
  if (millis() - lastResetTime >= hourDuration) {
    dataLogger.resetHourlyData();  // Move current hour data to previous and reset current
    lastResetTime = millis();      // Reset the hourly timer

    // Update home screen with new data
    homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
  }

  // Check if measurement button is pressed to enter or cancel measurement mode
  if (digitalRead(buttonPin) == LOW) {
    lastActivityTime = millis();  // Reset the activity timer

    if (!measurementMode) {
      // Start measurement mode
      measurementMode = true;
      awaitingObjectDetection = true;
      measurementStartTime = millis();  // Record the start time of measurement mode

      // Use the MeasurementScreen class to show the waiting screen
      measurementScreen.showWaitingWithCountdown();
    } else if (awaitingObjectDetection) {
      // Cancel measurement mode if waiting for object detection
      measurementMode = false;
      awaitingObjectDetection = false;

      // Show the home screen with updated data
      homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
      Serial.println("Measurement mode canceled");
    }
  }

  if (measurementMode && awaitingObjectDetection) {
    if (millis() - measurementStartTime >= measurementTimeout) {
      Serial.println("Measurement timed out. Returning to home screen.");
      measurementMode = false;
      awaitingObjectDetection = false;
      homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
      return;
    }

    int sensorValue = analogRead(sensorPin);
    bool objectDetected = (sensorValue < detectionThreshold);

    if (objectDetected != previousSensorState) {
      lastActivityTime = millis();

      if (objectDetected) {
        digitalWrite(ledPin, HIGH);
        Serial.println("Object detected!");
        dataLogger.incrementMeasurement();

        // Show success screen
        measurementScreen.showSuccess();

        // Reset measurement mode and show home screen
        measurementMode = false;
        awaitingObjectDetection = false;
        homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
      } else {
        digitalWrite(ledPin, LOW);
        Serial.println("No object detected.");
        measurementScreen.showNoObject();
      }

      previousSensorState = objectDetected;
    }
  }

  delay(200);  // Short delay for stable reading
}

void initADXL345() {
  Wire.beginTransmission(ADXL345_I2C_ADDR);
  Wire.write(0x2D);  // Power control register
  Wire.write(0x08);  // Set to measurement mode
  Wire.endTransmission();
}

void enterSleepMode() {
  Serial.println("Entering sleep mode...");

  // Save reference orientation
  accelerometer.saveReferenceOrientation();

  // Put the display and system into sleep mode
  tft.writecommand(TFT_DISPOFF);          // Turn off display
  tft.writecommand(TFT_SLPIN);            // Put display into sleep mode
  digitalWrite(ledPin, LOW);              // Turn off onboard LED
  digitalWrite(screenBacklightPin, LOW);  // Turn off screen backlighting
  isAsleep = true;
}

void wakeUp() {
  Serial.println("Waking up from sleep mode...");
  turnOnDisplay();
  tft.fillScreen(TFT_BLACK);
  delay(10);
  isAsleep = false;
  lastActivityTime = millis();
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
}

void turnOnDisplay() {
  digitalWrite(screenBacklightPin, HIGH);
  delay(50);
  tft.writecommand(TFT_SLPOUT);
  delay(150);
  tft.writecommand(TFT_DISPON);
  delay(50);
}
