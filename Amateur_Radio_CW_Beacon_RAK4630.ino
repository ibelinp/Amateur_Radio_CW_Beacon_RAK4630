/*
 * Project: Amateur_Radio_CW_Beacon_RAK4630
 * Description: Transmits CW beacon at 1296.2 MHz and 903.3 MHz using SX1262, reads GPS data from RAK12500 (ZOE-M8Q), displays satellite count, UTC time, and Maidenhead grid square on an OLED display.
 * Author: Pieter Ibelings, N4IP
 * Date: 2025-05-17
 * Version: 1.0
 * Copyright (c) 2025 Pieter Ibelings
 * TODO: Add more frequencies to FREQUENCIES array for additional amateur radio bands. Works 144-1300 MHz, but will require SX1262 matching component changes for full power.
 * Change TXPOWER and WPM (Words Per Minute) as needed.
 * Add as many frequencies to FREQUENCIES[] list.
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>

#define ENABLE_GPS 1 // Set to 1 to enable GPS, 0 to disable
#define ENABLE_LCD 1 // Set to 1 to enable OLED, 0 to disable
#define VBAT_PIN A0  // VBAT pin (P0.04 on RAK4630)
#define VBAT_THRESHOLD 3.3 // Low battery threshold in volts

// SPI pins
#define SPI_MOSI 44  // MOSI P1.12
#define SPI_MISO 45  // MISO P1.13
#define SPI_SCK  43  // SCK P1.11

// I2C pins (I2C1 port)
#define I2C_SDA 13  // SDA P0.13 (PIN_WIRE_SDA)
#define I2C_SCL 14  // SCL P0.14 (PIN_WIRE_SCL)

// UART1 pins (for RAK12500 GPS)
#define UART_RX 15  // RXD1 P0.15 (PIN_SERIAL1_RX)
#define UART_TX 16  // TXD1 P0.16 (PIN_SERIAL1_TX)

// SX1262 control pins
#define NSS    42  // SPI Chip Select P1.10
#define DIO1   47  // Interrupt pin (DIO1) P1.15
#define RESET  38  // Reset pin P1.06
#define BUSY   46  // Busy pin P1.14

// SX1262 MISC RAK4631 pins
#define DIO2  39  // DIO2 pin P1.07 (input, SX1262 drives TX switch)
#define ANT_SW   37  // ANT_SW pin P1.05 *** Powers PE4259 Active High

// GPS pins
#define GPS_1PPS 21  // IO3 P0.21 (WB_IO3)
#define GPS_POWER 34 // P1.02 (WB_IO2, load switch for GPS power, HIGH to enable)

// LED pins
#define TX_BLUE_LED 36  // Blue TX LED on P1.04 (PIN_LED2)
#define GREEN_LED   35  // Green LED on P1.03 (PIN_LED1, reflects 1PPS)

// SX1262 Commands
#define SET_STANDBY          0x80
#define SET_RF_FREQUENCY     0x86
#define SET_PA_CONFIG        0x95
#define SET_TX_PARAMS        0x8E
#define SET_TX_CONTINUOUS_WAVE 0xD1
#define SET_DIO2_AS_RF_SW    0x9D
#define GET_STATUS           0xC0
#define SET_DIO3_AS_TCXO_CTRL 0x97

// CW Configuration Parameters
#define TX_POWER            22      // dBm
#define TX_STABILIZATION_DELAY 10   // ms (changed from 25)
#define WPM                 25      // Words per minute (changed from 20)
#define UNIT_TIME           (1200 / WPM) // ms

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C  // Common I2C address for 0.96" OLED
#ifdef ENABLE_LCD
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// GPS Configuration
#ifdef ENABLE_GPS
TinyGPSPlus gps;
bool gpsConnected = false;
bool gpsReceiving = false; // Tracks if NMEA sentences are received
#endif
bool ledState = false; // Tracks LED state for toggling
#define GPS_BAUD 9600
#define GPS_TIMEOUT 10000  // ms to wait for GPS data

// Create SPI instance for SX1262 using SPIM2
SPIClass SX1262_SPI(NRF_SPIM2, SPI_MISO, SPI_SCK, SPI_MOSI);

// Frequency list (in Hz) - add as many frequencies as needed, but SX1262 matching network will limit output power
const uint64_t FREQUENCIES[] = {
  1296200000ULL,  // 1296.2 MHz, 23cm amateur band, primary beacon frequency
  903300000ULL    // 903.3 MHz, 33cm amateur band, secondary frequency for testing
};
const int NUM_FREQUENCIES = sizeof(FREQUENCIES) / sizeof(FREQUENCIES[0]);
int currentFreqIndex = 0;

// Hardcoded Morse string
const char* MORSE_STRING = "BBB DE 3Y0X/KE83"; // Changed from "BBB DE N4IP / TEST &"

// Morse code lookup table (A-Z, 0-9, space, special characters)
const char* morseCode[] = {
  ".-",     // A
  "...-",   // B
  "-.-.",   // C
  "-..",    // D
  ".",      // E
  "..-.",   // F
  "--.",    // G
  "....",   // H
  "..",     // I
  ".---",   // J
  "-.-",    // K
  ".-..",   // L
  "--",     // M
  "-.",     // N
  "---",    // O
  ".--.",   // P
  "--.-",   // Q
  ".-.",    // R
  "...",    // S
  "-",      // T
  "..-",    // U
  "...-",   // V
  ".--",    // W
  "-..-",   // X
  "-.--",   // Y
  "--..",   // Z
  "-----",  // 0
  ".----",  // 1
  "..---",  // 2
  "...--",  // 3
  "....-",  // 4
  ".....",  // 5
  "-....",  // 6
  "--...",  // 7
  "---..",  // 8
  "----.",  // 9
  "",       // Space
  "-..-.",  // / (slash)
  ".-.-.-", // . (period)
  "--..--", // , (comma)
  "..--..", // ? (question mark)
  ".-.-.",  // AR (end of message prosign)
  "-...-",  // = or BT (break)
  "...-.-"  // SK (end of contact)
};

#ifdef ENABLE_GPS
// 1PPS interrupt handler
void ppsHandler() {
  ledState = !ledState; // Toggle LED state
  digitalWrite(GREEN_LED, ledState ? HIGH : LOW);
}

// Calculate 6-digit Maidenhead grid square
String calculateMaidenhead(double lat, double lon) {
  lon += 180.0;
  lat += 90.0;
  int lonField = lon / 20;
  int latField = lat / 10;
  double lonRemainder = lon - lonField * 20;
  double latRemainder = lat - latField * 10;
  int lonSquare = lonRemainder / 2;
  int latSquare = latRemainder;
  double lonSubRemainder = lonRemainder - lonSquare * 2;
  double latSubRemainder = latRemainder - latSquare;
  int lonSubsquare = lonSubRemainder * 12;
  int latSubsquare = latSubRemainder * 24;
  char grid[7];
  grid[0] = 'A' + lonField;
  grid[1] = 'A' + latField;
  grid[2] = '0' + lonSquare;
  grid[3] = '0' + latSquare;
  grid[4] = 'a' + lonSubsquare;
  grid[5] = 'a' + latSubsquare;
  grid[6] = '\0';
  return String(grid);
}
#endif

// Map characters to morseCode array indices
int charToIndex(char c) {
  c = toupper(c);
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= '0' && c <= '9') return c - '0' + 26;
  switch (c) {
    case ' ': return 36;
    case '/': return 37;
    case '.': return 38;
    case ',': return 39;
    case '?': return 40;
    case '+': return 41; // = or BT
    case '&': return 42; // SK
    default: return -1; // Invalid character
  }
}

float readBatteryVoltage() {
  int raw = analogRead(VBAT_PIN); // 0-1023 (10-bit ADC)
  // Assuming 3.3V reference, VBAT divider (1/2), and 3.6V max
  float voltage = (raw / 1023.0) * 3.3 * 2.0; // Adjust for divider
  return voltage;
}

void writeRegister(uint8_t cmd, const uint8_t* data, uint8_t len) {
  digitalWrite(NSS, LOW);
  uint8_t response = SX1262_SPI.transfer(cmd);
  for (uint8_t i = 0; i < len; i++) {
    SX1262_SPI.transfer(data[i]);
  }
  digitalWrite(NSS, HIGH);
  waitForBusy();
}

uint8_t getStatus() {
  digitalWrite(NSS, LOW);
  SX1262_SPI.transfer(GET_STATUS);
  uint8_t status = SX1262_SPI.transfer(0x00);
  digitalWrite(NSS, HIGH);
  return status;
}

void waitForBusy() {
  unsigned long timeout = millis() + 1000;
  while (digitalRead(BUSY) == HIGH) {
    if (millis() > timeout) {
      #ifdef ENABLE_LCD
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Error: BUSY stuck HIGH");
      display.display();
      #endif
      digitalWrite(TX_BLUE_LED, LOW);
      while (true);
    }
    delay(1);
  }
}

void setup() {
  // Initialize Serial for debugging (non-blocking)
  Serial.begin(115200);
  Serial.println("Serial initialized");
  delay(1000);

  // Initialize pins
  pinMode(NSS, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(BUSY, INPUT);
  pinMode(DIO1, INPUT);
  pinMode(DIO2, INPUT); // DIO2 as input (SX1262 drives TX switch)
  pinMode(ANT_SW, OUTPUT); // Antenna switch (PE4259)
  pinMode(TX_BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(NSS, HIGH);
  digitalWrite(RESET, HIGH);
  digitalWrite(ANT_SW, HIGH); // Power PE4259 RF switch
  digitalWrite(TX_BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW); // Green LED off until 1PPS

  #ifdef ENABLE_GPS
  // Initialize GPS power and 1PPS
  pinMode(GPS_POWER, OUTPUT);
  digitalWrite(GPS_POWER, HIGH);
  pinMode(GPS_1PPS, INPUT);
  Serial.println("GPS power enabled (P1.02 HIGH)");
  Serial1.begin(GPS_BAUD);
  Serial.println("UART1 initialized at 9600 baud");
  attachInterrupt(digitalPinToInterrupt(GPS_1PPS), ppsHandler, RISING);
  #else
  // GPS disabled, ensure power pin is off
  pinMode(GPS_POWER, OUTPUT);
  digitalWrite(GPS_POWER, LOW);
  #endif

  // Initialize VBAT pin
  pinMode(VBAT_PIN, INPUT);
  analogReference(AR_DEFAULT); // 3.3V reference

  // Initialize I2C and OLED display
  Wire.begin(); // I2C1 on SDA (P0.13), SCL (P0.14)
  #ifdef ENABLE_LCD
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED display initialized");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("RAK4631 Beacon");
    display.println("Initializing...");
    #ifdef ENABLE_GPS
    display.println("GPS Power: ON");
    #else
    display.println("GPS Disabled");
    #endif
    display.display();
  } else {
    Serial.println("OLED display not detected");
  }
  #else
  Serial.println("OLED disabled");
  #endif

  #ifdef ENABLE_GPS
  // Initialize UART1 for GPS
  unsigned long start = millis();
  gpsReceiving = false;
  while (millis() - start < GPS_TIMEOUT) {
    while (Serial1.available()) {
      gpsReceiving = true;
      if (gps.encode(Serial1.read())) {
        if (gps.location.isValid()) {
          gpsConnected = true;
          break;
        }
      }
    }
    if (gpsConnected) break;
  }
  if (gpsReceiving) {
    Serial.println("GPS module receiving data");
    if (gpsConnected) {
      Serial.println("GPS module initialized with valid fix");
    } else {
      Serial.println("GPS module: No valid fix");
    }
  } else {
    Serial.println("GPS module: No data received");
  }
  #endif

  #ifdef ENABLE_LCD
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("RAK4631 Beacon");
  #ifdef ENABLE_GPS
  if (gpsReceiving) {
    display.println("GPS: Receiving");
    if (gpsConnected) {
      display.println("Fix: Valid");
    } else {
      display.println("Fix: None");
    }
  } else {
    display.println("GPS: No data");
  }
  #else
  display.println("GPS Disabled");
  #endif
  float vbat = readBatteryVoltage();
  display.print("Batt: ");
  display.print(vbat, 1);
  display.println("V");
  if (vbat < VBAT_THRESHOLD) {
    display.println("LOW BATTERY!");
  }
  display.display();
  #endif

  // Initialize SPI
  SX1262_SPI.begin();
  SX1262_SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  Serial.println("SPI initialized");

  // Reset SX1262
  digitalWrite(RESET, LOW);
  delay(10);
  digitalWrite(RESET, HIGH);
  delay(10);
  waitForBusy();
  Serial.println("Reset triggered");
  Serial.println("Reset complete");

  // Configure SX1262 for CW
  configureCW();
  Serial.println("SX1262 CW Setup Complete");
  Serial.print("Transmitting: '");
  Serial.print(MORSE_STRING);
  Serial.print("' at ");
  Serial.print(WPM);
  Serial.println(" WPM on frequencies: 1296.2 MHz, 903.3 MHz");

  uint8_t status = getStatus();
  Serial.print("Initial SX1262 Status: 0x");
  Serial.println(status, HEX);
}

void configureCW() {
  // Set standby mode
  uint8_t standbyData[] = {0x00};
  writeRegister(SET_STANDBY, standbyData, 1);

  // Configure DIO3 to power 32 MHz TCXO (1.8V, 10 ms startup)
  uint8_t dio3Data[] = {0x01, 0x00, 0x01, 0x90}; // 1.8V, 10 ms (640 * 15.625 µs)
  writeRegister(SET_DIO3_AS_TCXO_CTRL, dio3Data, 4);

  // Configure DIO2 to control TX switch
  uint8_t dio2Data[] = {0x01};
  writeRegister(SET_DIO2_AS_RF_SW, dio2Data, 1);

  // Configure PA for 22 dBm
  uint8_t paConfig[] = {0x04, 0x07, 0x00, 0x01}; // 22 dBm, SX1262
  writeRegister(SET_PA_CONFIG, paConfig, 4);

  // Set TX parameters
  uint8_t txParams[] = {TX_POWER, 0x00}; // 22 dBm, no ramp time
  writeRegister(SET_TX_PARAMS, txParams, 2);
}

void setFrequency(uint64_t freq) {
  uint32_t freqReg = (uint32_t)(((uint64_t)freq << 25) / 32000000);
  uint8_t freqData[] = {(uint8_t)(freqReg >> 24), (uint8_t)(freqReg >> 16),
                        (uint8_t)(freqReg >> 8), (uint8_t)freqReg};
  writeRegister(SET_RF_FREQUENCY, freqData, 4);
}

void transmitMorse(const char* code) {
  for (int i = 0; code[i] != '\0'; i++) {
    digitalWrite(TX_BLUE_LED, HIGH);
    Serial.print(code[i]);
    uint8_t cwData[] = {0x00};
    writeRegister(SET_TX_CONTINUOUS_WAVE, cwData, 0); // RF ON
    delay(TX_STABILIZATION_DELAY);
    if (code[i] == '.') delay(UNIT_TIME - TX_STABILIZATION_DELAY); // Dit
    else if (code[i] == '-') delay(UNIT_TIME * 3 - TX_STABILIZATION_DELAY); // Dah
    uint8_t standbyData[] = {0x00};
    writeRegister(SET_STANDBY, standbyData, 1); // RF OFF
    digitalWrite(TX_BLUE_LED, LOW);
    delay(UNIT_TIME); // Intra-character space
  }
}

void sendMorseString(const char* str) {
  float vbat = readBatteryVoltage(); // Declare vbat once at the start

  for (int i = 0; str[i] != '\0'; i++) {
    int index = charToIndex(str[i]);
    if (index >= 0) {
      if (index == 36) { // Space
        Serial.print(" ");
        delay(UNIT_TIME * 7); // Word space
      } else {
        transmitMorse(morseCode[index]);
        Serial.print(" ");
        delay(UNIT_TIME * 3); // Inter-character space
      }
    }
  }
  Serial.println(" - Sent");

  // Update OLED display if enabled
  #ifdef ENABLE_LCD
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Freq: ");
  display.print((unsigned long)(FREQUENCIES[currentFreqIndex] / 1000000));
  display.print(".");
  display.print((unsigned long)(FREQUENCIES[currentFreqIndex] % 1000000) / 1000);
  display.println(" MHz");
  display.println("TX: BBB DE 3Y0X/KE83"); // Updated Morse message
  #ifdef ENABLE_GPS
  if (gpsReceiving) {
    display.print("Sats: ");
    display.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
    if (gps.time.isValid()) {
      display.print("UTC: ");
      if (gps.time.hour() < 10) display.print("0");
      display.print(gps.time.hour());
      display.print(":");
      if (gps.time.minute() < 10) display.print("0");
      display.print(gps.time.minute());
      display.print(":");
      if (gps.time.second() < 10) display.print("0");
      display.println(gps.time.second());
    }
    if (gpsConnected && gps.location.isValid()) {
      String grid = calculateMaidenhead(gps.location.lat(), gps.location.lng());
      display.print("Grid: ");
      display.println(grid);
    } else {
      display.println("Fix: None");
    }
  } else {
    display.println("GPS: No data");
  }
  #else
  display.println("GPS Disabled");
  #endif
  display.print("Batt: ");
  display.print(vbat, 1);
  display.println("V");
  if (vbat < VBAT_THRESHOLD) {
    display.println("LOW BATTERY!");
  }
  display.display();
  #endif

  // Update Serial with GPS and battery status
  #ifdef ENABLE_GPS
  Serial.println(gpsReceiving ? "GPS: Receiving" : "GPS: No data");
  if (gpsReceiving && gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }
  if (gps.time.isValid()) {
    Serial.print("UTC Time: ");
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());
  }
  if (gpsConnected && gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lon: ");
    Serial.println(gps.location.lng(), 6);
    String grid = calculateMaidenhead(gps.location.lat(), gps.location.lng());
    Serial.print("Grid Square: ");
    Serial.println(grid);
  } else {
    Serial.println("GPS: No fix");
  }
  #else
  Serial.println("GPS Disabled");
  #endif
  Serial.print("Battery Voltage: ");
  Serial.print(vbat, 1);
  Serial.println("V");
  if (vbat < VBAT_THRESHOLD) {
    Serial.println("Warning: Low Battery!");
  }
}

void loop() {
  #ifdef ENABLE_GPS
  // Read GPS data and check for serial activity
  gpsReceiving = false; // Reset flag each loop
  while (Serial1.available()) {
    gpsReceiving = true; // Any data received
    gps.encode(Serial1.read());
  }
  #endif

  Serial.print("Transmitting on ");
  Serial.print((unsigned long)(FREQUENCIES[currentFreqIndex] / 1000000));
  Serial.print(".");
  Serial.print((unsigned long)(FREQUENCIES[currentFreqIndex] % 1000000) / 1000);
  Serial.println(" MHz");
  setFrequency(FREQUENCIES[currentFreqIndex]);
  sendMorseString(MORSE_STRING);

  currentFreqIndex = (currentFreqIndex + 1) % NUM_FREQUENCIES; // Move to next frequency
  delay(1000); // Pause between frequency changes
}