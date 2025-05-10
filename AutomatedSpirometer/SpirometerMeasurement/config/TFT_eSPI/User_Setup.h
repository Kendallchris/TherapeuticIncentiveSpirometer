// User defined information reported by "Read_User_Setup" test & diagnostics example
#define USER_SETUP_INFO "ILI9488 Setup for Teensy 4.0"

// ##################################################################################
// Section 1. Call up the right driver file and any options for it
// ##################################################################################

// Only define one driver, the others must be commented out
//#define ILI9488_DRIVER     // Enable support for ILI9488 (compatible with TFT_eSPI) - for 3.5"
#define ILI9341_DRIVER		// Enable support for ILI9341 (compatible with TFT_eSPI) - for 2.4"

// ##################################################################################
// Section 2. Define the pins that are used to interface with the display here
// ##################################################################################

// Define the pin numbers for the Teensy 4.0
//#define TFT_CS    10        // Chip select control pin
//#define TFT_DC    8         // Data Command control pin
//#define TFT_RST   9         // Reset pin (if connected)

// Changed pins for PCB
#define TFT_CS    10        // Chip select control pin
#define TFT_DC    5         // Data Command control pin
#define TFT_RST   4         // Reset pin (if connected)

// Define the SPI connections for the Teensy 4.0
#define TFT_SCLK  13        // SPI Clock pin
#define TFT_MOSI  11        // SPI Data (MOSI) pin

// Dummy Pin for Touch
#define TOUCH_CS  -1
#define TFT_MISO  -1  // Not connected

// ##################################################################################
// Section 3. Define the fonts that are to be used here
// ##################################################################################

// Load fonts (optional)
#define LOAD_GLCD    // Font 1. Original Adafruit 8 pixel font
#define LOAD_FONT2   // Font 2. Small 16 pixel high font
#define LOAD_FONT4   // Font 4. Medium 26 pixel high font
#define LOAD_FONT6   // Font 6. Large 48 pixel font
#define LOAD_FONT7   // Font 7. 7 segment 48 pixel font
#define LOAD_FONT8   // Font 8. Large 75 pixel font
#define LOAD_GFXFF   // FreeFonts. Include access to the 48 Adafruit_GFX free fonts

// Enable smooth fonts
#define SMOOTH_FONT

// ##################################################################################
// Section 4. Set the SPI Frequency
// ##################################################################################

// Define the SPI clock frequency for the ILI9488 display
//#define SPI_FREQUENCY  27000000        // Set to 27 MHz, works well with ILI9488 - for 3.5"
#define SPI_FREQUENCY  40000000        // Set to 40 MHz, works well with ILI9341 - for 2.4"
#define SPI_READ_FREQUENCY  20000000   // Set the SPI read frequency (20 MHz)
#define SPI_TOUCH_FREQUENCY  2500000   // SPI clock rate for touch (if used)

// Enable SPI Transactions if needed for compatibility with other SPI devices
#define SUPPORT_TRANSACTIONS
