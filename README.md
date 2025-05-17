# Amateur_Radio_CW_Beacon_RAK4630 (or RAK4631)
Transmits CW beacon at 1296.2 MHz, 903.3 MHz using SX1262, reads GPS data from RAK12500 (ZOE-M8Q), displays satellite count, UTC time, and Maidenhead grid square on an OLED display.

Works with RAK4630/RAK4631 boards using nRF52840
Tested on RAK WishMesh Board ONE
Will transmit 144-1300 MHz, but in order to achieve full power, the matching network for the TX path need to be change
RAK4630/RAK4631 does about 8dBm at 1296 MHz and 20dBm at 903 MHz

Installed these Arduino libraries:
Adafruit BusIO 1.17.1
Adafruit GFX Library 1.12.1
Adafruit SSD1306 2.5.14
TinyGPSPlus 1.0.3

