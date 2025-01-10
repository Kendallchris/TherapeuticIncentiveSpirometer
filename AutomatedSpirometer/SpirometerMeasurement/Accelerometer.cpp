#include "Accelerometer.h"
#include <Arduino.h>  // For Serial and math functions

Accelerometer::Accelerometer(uint8_t address, float tiltThreshold)
    : i2cAddress(address), tiltAngleThreshold(tiltThreshold), refX(0), refY(0), refZ(0) {}

void Accelerometer::initialize() {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x2D);  // Power control register
    Wire.write(0x08);  // Set to measurement mode
    Wire.endTransmission();
}

void Accelerometer::saveReferenceOrientation() {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x32);  // Start with DATAX0 register
    Wire.endTransmission(false);
    Wire.requestFrom(i2cAddress, 6, true);

    refX = Wire.read() | (Wire.read() << 8);
    refY = Wire.read() | (Wire.read() << 8);
    refZ = Wire.read() | (Wire.read() << 8);

    Serial.print("Reference Orientation - X: ");
    Serial.print(refX);
    Serial.print(", Y: ");
    Serial.print(refY);
    Serial.print(", Z: ");
    Serial.println(refZ);
}

bool Accelerometer::detectTilt() {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x32);  // Start with DATAX0 register
    Wire.endTransmission(false);
    Wire.requestFrom(i2cAddress, 6, true);

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

    Serial.print("Delta Angle: ");
    Serial.println(angle);

    // Check if the calculated angle exceeds the threshold
    if (angle > tiltAngleThreshold) {
        Serial.println("Tilt detected!");
        return true;  // Tilt detected
    }

    return false;  // No tilt
}
