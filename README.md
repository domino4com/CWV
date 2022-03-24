<img src="assets/CWV.svg" width=200 align="right">

# CWS Core WiFi Extended
This is the extended core used in the Domino4 eco-system.
This core has 4Mb of PSRAM. Remember to enable the PSRAM in Arduino.

## Main ChipSet
The core is built around the [ESP32-WROOM-32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf) module from Espressif. Future versions of this core will be using newer version of the ESP32 chipset.

## Programming

### Programming in Arduino
- To program the Domino4 cores using Arduino, install the board files using the doumentation from [Espressif](https://github.com/espressif/arduino-esp32)
- Please read the notes regarding upload/transfer speed when using the [PPU](https://github.com/domino4com/PPU).
- Settings:
  - **Board:** Choose the ```ESP32 Dev Module```
  - **Speed:** Max 460800 bps
  - **Port:** Chose the port where your PPU is inserted. If you cannot see the port, the check out your [PPU installation](https://github.com/domino4com/PPU)

### Programming in Python.
- Download the MicroPython firmware from [micropython.org](https://micropython.org/download/esp32/)
- It is recommended to download and use the (Mu Editor)(https://codewith.mu/en/download)
- You can use the Mu Editor to upload the MicroPython Firmware.

## Pin Usage
### Extension Slot
The 10 pins on the Extension slot are configured on both side, making your extension board reversible.
| Pins| Function | Group |
|:-----------------------------:|:----:|:--:|
| :one:                 | Vcc | Power|
| :two:                 | MISO | SPI|
| :three:                    | MOSI | SPI|
| :four:                  | SCK | SPI|
| :five:            | IO15  | GPIO|
| :six: | IO33 | GPIO|
| :seven:                       | IO2 | GPIO|
| :eight:                       | SCL | I²C |
| :nine:                       | SDA | I²C |
| :keycap_ten:                       | GND | Power|

### SD Card
SD Card is used in 4 Pin SPI configuration.
| Pin | GPIO |
|:-----------------------------|:----:|
|  MISO |12| 
|  MOSI |13| 
|  SCK |14| 
|  CS |5| 

### Camera
| Postion | Color | GPIO | On when|
|:-----------------------------|:----:|:--:|:--:
|  Top |Red| IO25 | High |
|  Top |Blue| IO26 | High |
|  Middle |Yellow| IO19 | High |
|  Bottom |Red| IO17 | High |
|  Bottom |Green| IO18 | High |

### CAN Bus Interface
| Postion | Color | GPIO | On when|
|:-----------------------------|:----:|:--:|:--:
|  Vcc |Red| IO25 | High |
|  Vsrc |Blue| IO26 | High |
|  GPIO |Yellow| IO19 | High |
|  CAN H |Red| IO17 | High |
|  CAN L |Green| IO18 | High |
|  Control SDA |Red| IO17 | High |
|  Control SCL |Green| IO18 | High |
|  Reset |Red| IO25 | High |
|  Prog |Blue| IO26 | High |
|  TXD |Yellow| IO19 | High |
|  RXD |Red| IO17 | High |
|  SDA |Green| IO18 | High |
|  SCL |Red| IO17 | High |
|  GND |Green| IO18 | High |
### Other Pins
| Function |  GPIO | Notes|
|:-----------------------------|:----:|:--|
|  I²C SDA |21| |
|  I²C SCL |22| |
|  Serial TX |1| |
|  Serial RX |3| |
|  CAN Bus TX |2| |
|  CAN Bus RX |32| |

## Troubleshooting
- If you try to upload code and getting this message ```A fatal error occurred: Timed out waiting for packet content``` or ```A fatal error occurred: Invalid head of packet (0xE0)```, change the transfer speed to 460800 pbs.
- If you try to upload code and getting this message ```[7886] Failed to execute script esptool the selected serial port [7886] Failed to execute script esptool does not exist or your board is not connected```, your serial port is open by another application. Close the other application and try again.

# License: 
<img src="assets/CC-BY-NC-SA.svg" width=200 align="right">
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License

[View License Deed](https://creativecommons.org/licenses/by-nc-sa/4.0/) | [View Legal Code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

