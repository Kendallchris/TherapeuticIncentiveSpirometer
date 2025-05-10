# Automated Spirometer (Teensy 4.0)

This project is designed for the Teensy 4.0 and uses LVGL and TFT_eSPI for the GUI.

## Getting Started

### Hardware
- Teensy 4.0
- ILI9341-compatible 2.4" TFT
- ADXL345 Accelerometer
- TSC2007 touch controller
- Buzzer, Vibration Motor, etc.

### Setup in Arduino IDE

1. Install Teensyduino and select Teensy 4.0.
2. Install libraries listed in `lib/README.md` (with specified versions).
3. Replace configuration files in:
    - `Arduino/libraries/lv_conf.h`
    - `Arduino/libraries/TFT_eSPI/User_Setup.h`
4. Open `SpirometerMeasurement.ino` and upload.

### Directory Structure

- `include/` – all header files
- `src/` – all source files
- `config/` – custom setup files for TFT_eSPI and LVGL
