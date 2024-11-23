// TODO: when taking measurement and "Waiting for object, press green button to cancel" screen is up, the sensor is not sensing for an object (this is potentially okay)

#include <TFT_eSPI.h>           // Include the TFT_eSPI library
#include <Wire.h>               // Include Wire library for I2C communication
#include "HomeScreen.h"         // Include HomeScreen for managing the UI
#include "DataLogger.h"         // Include DataLogger for managing measurements
#include "DataStorage.h"        // Include DataStorage for persistent storage

// Pin definitions
const int ledPin = 13;           // Pin for the onboard LED
const int sensorPin = 14;        // Analog pin A0 (Pin 14) connected to the TCRT5000 sensor's collector
const int buttonPin = 2;         // GPIO2 for measurement mode button GREEN
const int wakeUpButtonPin = 3;   // GPIO3 for wake-up button RED

// ADXL345 I2C Address
#define ADXL345_I2C_ADDR 0x53

// Angle threshold for tilt detection (in degrees)
const float tiltAngleThreshold = 45.0;

// Create instances
TFT_eSPI tft = TFT_eSPI();
HomeScreen homeScreen(tft);
DataLogger dataLogger;

// Threshold for object detection; adjust this based on testing
const int detectionThreshold = 100; // Experiment to find the optimal value

// State variables
bool previousSensorState = HIGH;     // Tracks previous sensor state; assume no object detected initially
bool measurementMode = false;        // Tracks if we are in measurement mode
bool isAsleep = false;               // Tracks if the system is in "sleep" mode
bool awaitingObjectDetection = false; // Tracks if waiting for object detection to succeed

// Timer variables
unsigned long lastActivityTime = 0;         // Tracks the last time there was activity
unsigned long lastResetTime = 0;            // Tracks the last hourly reset time
const unsigned long sleepDelay = 60000;     // 10 seconds of inactivity before sleep
const unsigned long hourDuration = 3600000; // 1 hour in milliseconds
const unsigned long detectionDelay = 5000;  // 5-second buffer time before checking for "No Object Detected"
unsigned long measurementStartTime = 0;     // Track when measurement mode starts

// Reference orientation
int16_t refX = 0, refY = 0, refZ = 0;

void setup() {
    // Set up serial communication
    Serial.begin(9600);                 // Start serial communication at 9600 baud rate

    // Set up pins
    pinMode(ledPin, OUTPUT);            // Set LED pin as output for user control
    pinMode(buttonPin, INPUT);          // Set measurement button pin as input
    pinMode(wakeUpButtonPin, INPUT);    // Set wake-up button pin as input

    // Initialize the display
    tft.init();                         // Initialize the display
    tft.setRotation(1);                 // Set display orientation (adjust if needed)
    homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());  // Display the initial home screen with measurements

    // Initialize ADXL345 accelerometer
    Wire.begin();
    initADXL345();

    // Initialize timers
    lastActivityTime = millis();
    lastResetTime = millis();
}

void loop() {
    // Check for wake-up button press or accelerometer tilt if the system is in sleep mode
    if (isAsleep) {
        if (digitalRead(wakeUpButtonPin) == LOW || detectTilt()) { 
            wakeUp();
        }
        return; // Exit the loop if asleep
    }

    // Check if inactivity threshold has been reached to enter sleep mode
    if (millis() - lastActivityTime >= sleepDelay) {
        enterSleepMode();
    }

    // Check if an hour has passed since the last reset
    if (millis() - lastResetTime >= hourDuration) {
        dataLogger.resetHourlyData(); // Move current hour data to previous and reset current
        lastResetTime = millis();     // Reset the hourly timer

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
            showMeasurementScreen();
        } else if (awaitingObjectDetection) {
            // Cancel measurement mode if waiting for object detection
            measurementMode = false;
            awaitingObjectDetection = false;
            homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());
            Serial.println("Measurement mode canceled");
        }
    }

    if (measurementMode && awaitingObjectDetection) {
        // Check if enough time has passed to start looking for object detection
        if (millis() - measurementStartTime < detectionDelay) {
            return;  // Wait until buffer time has passed
        }

        // Read the sensor as an analog value
        int sensorValue = analogRead(sensorPin);

        // Determine if the sensor value crosses the detection threshold
        bool objectDetected = (sensorValue < detectionThreshold); // Detect object if sensor reading is below threshold

        // Only update if the detection state has changed
        if (objectDetected != previousSensorState) {
            lastActivityTime = millis();  // Reset activity timer on detection

            if (objectDetected) {  // Object detected
                digitalWrite(ledPin, HIGH);  // Turn on LED for visual feedback
                Serial.println("Object detected!");  // Debugging
                
                // Increment the measurement count and update display, counting beyond 10 if needed
                dataLogger.incrementMeasurement();
                homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());

                // Display "SUCCESS!" for 2 seconds, then return to the home screen
                tft.fillScreen(TFT_BLACK);
                tft.setCursor(10, 10);
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.println("SUCCESS!");
                delay(2000);  // Wait 2 seconds

                // Return to home screen after success
                measurementMode = false;
                awaitingObjectDetection = false;
                homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());

            } else {  // No object detected after buffer delay
                digitalWrite(ledPin, LOW);   // Turn off LED
                Serial.println("No object detected.");  // Debugging
                
                // Update display for no object detected
                tft.fillScreen(TFT_BLACK);
                tft.setCursor(10, 10);
                tft.setTextColor(TFT_RED, TFT_BLACK);
                tft.println("No object detected.");
            }
            // Update the previous sensor state to the current state
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

    // Store the reference orientation
    Wire.beginTransmission(ADXL345_I2C_ADDR);
    Wire.write(0x32);  // Start with DATAX0 register
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345_I2C_ADDR, 6, true);

    refX = Wire.read() | (Wire.read() << 8);
    refY = Wire.read() | (Wire.read() << 8);
    refZ = Wire.read() | (Wire.read() << 8);

    Serial.print("Reference Orientation - X: "); Serial.print(refX);
    Serial.print(", Y: "); Serial.print(refY);
    Serial.print(", Z: "); Serial.println(refZ);

    // Put the display and system into sleep mode
    tft.writecommand(TFT_DISPOFF);      // Turn off display
    tft.writecommand(TFT_SLPIN);        // Put display into sleep mode
    digitalWrite(ledPin, LOW);          // Turn off onboard LED
    isAsleep = true;                    // Set sleep mode flag
}

bool detectTilt() {
    Wire.beginTransmission(ADXL345_I2C_ADDR);
    Wire.write(0x32);  // Start with DATAX0 register
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345_I2C_ADDR, 6, true);

    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);

    // Calculate dot product and magnitudes
    float dotProduct = (x * refX) + (y * refY) + (z * refZ);
    float magnitudeCurrent = sqrt((float)(x * x + y * y + z * z));
    float magnitudeRef = sqrt((float)(refX * refX + refY * refY + refZ * refZ));

    // Avoid division by zero
    if (magnitudeCurrent == 0 || magnitudeRef == 0) {
        Serial.println("Error: Magnitude is zero!");
        return false;
    }

    // Calculate the angle in degrees
    float cosTheta = dotProduct / (magnitudeCurrent * magnitudeRef);
    cosTheta = constrain(cosTheta, -1.0, 1.0);  // Clamp value to avoid numerical errors
    float angle = acos(cosTheta) * 180.0 / PI;

    Serial.print("Delta Angle: "); Serial.println(angle);

    // Check if the calculated angle exceeds the threshold
    if (angle > tiltAngleThreshold) {
        Serial.println("Tilt detected!");
        return true;  // Tilt detected
    }

    return false;  // No tilt
}

void wakeUp() {
    Serial.println("Waking up from sleep mode...");
    tft.writecommand(TFT_DISPON);       // Turn on display
    tft.writecommand(TFT_SLPOUT);       // Wake display from sleep
    isAsleep = false;                   // Clear sleep mode flag
    lastActivityTime = millis();        // Reset activity timer
    homeScreen.show(dataLogger.getCurrentHourMeasurements(true), dataLogger.getPreviousHourMeasurements());  // Return to home screen with current and previous measurements
}

void showMeasurementScreen() {
    tft.fillScreen(TFT_BLACK);          // Clear the display
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Waiting for object...");
    tft.setCursor(10, 40);
    tft.println("Press green button to cancel");
    Serial.println("Measurement mode activated");
}
