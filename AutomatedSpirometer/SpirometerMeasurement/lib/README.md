# Dependencies for AutomatedSpirometer (Teensy 4.0)

To run this project in Arduino IDE, install the following libraries (with matching versions):

# Required Arduino Libraries

| Library                   | Version   | Purpose                                   |
|---------------------------|-----------|-------------------------------------------|
| TFT_eSPI                  | 2.5.43    | Display driver for ILI9341                |
| LVGL                      | 8.3.11    | UI framework                              |
| Adafruit ADXL345          | 1.3.4     | Accelerometer driver                      |
| Adafruit Unified Sensor   | 1.1.14    | Dependency of ADXL345                     |
| Adafruit BusIO            | 1.16.2    | Dependency of ADXL345                     |

## Custom Configuration

Be sure to:

1. Replace the `lv_conf.h` above the root of your installed LVGL library with `config/lvgl_custom/lv_conf.h`
2. Replace your `TFT_eSPI/User_Setup.h` with `config/TFT_eSPI/User_Setup.h`
