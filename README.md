<img src="assets/CWV.svg" width=200 align="right">

# CWV Core WiFi Extended
This is the extended core used in the Domino4 eco-system.
> The core has 8Mb of PSRAM. Remember to enable the PSRAM in Arduino.

## Main ChipSet
The v1-4 core is built around the [ESP32-WROOM-32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf) module from Espressif. 
The v5 core is built around the [ESP32-S3FN8](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf) chipset from Espressif. 

## Pin Usage
### Extension Slot
The 10 pins on the Extension slot are configured on both side, making your extension board reversible.
| Pins| Function v1-4| v5+ | Group |Lora| SD Card Ext|
|:-----------------------------:|:----:|:----:|:--:|:--:|:--:|
| :one:                | Vcc  | Vcc | Power|Vcc|Vcc
| :two:                | MISO | MISO (IO38) | SPI|MISO|MMC D0
| :three:               | MOSI     | MOSI (IO37)| SPI|MOSI|MMC CMD
| :four:              | SCK |    SCK (IO36)| SPI|SCK|MMC CLK
| :five:         | IO15   | IO35  | GPIO|NSS|MMC D3
| :six: | IO33 | IO34 | GPIO|DIO0|MMC D1
| :seven:                    | n/a | IO33  | GPIO |  n/a|MMC D2
| :eight:                     | SCL |  SCL | I²C | n/a| n/a
| :nine:                   | SDA |   SDA | I²C | n/a| n/a
| :keycap_ten:                   | GND    | GND | Power| GND| GND

### SD Card
SD Card is used in 4 Pin SPI configuration. See [code example](#example-sd-card-over-spi)
> v1-4: Unlike the [Ai-Thinker ESP32-CAM](http://www.ai-thinker.com/pro_view-24.html), this core cannot use SD-MMC (only stadard SD), which requires DAT0 and DAT1 on the SD Card interface to be connected as well. There is simply not enough pins available on the ESP32.

| Pin | GPIO v1-4 | GPIO V5
|:-----------------------------|:----:|:----:|
|  MISO |12| 38
|  MOSI |13| 37
|  SCK |14| 36
|  CS |5†| 3

### Camera
The camera uses exactly the same pins as the ESP32-CAM from AI-Thinker. See [CWV Camera Repo](https://github.com/domino4com/CWV-Camera)
Not avalable on Flight version (CWN v5) of the CWV v5.
| Function | GPIO v1-4 | 
|:-----------------------------|:----:|
| PWDN_ |   32
|RESET |  -1
| XCLK |    0
| SIOD  |  26
| SIOC  |   27
| Y9  |    35
| Y8   |    34
| Y7   |    39
| Y6  |    36
|Y5   |   21
| Y4 |    19
| Y3   |    18‡
| Y2  |     5†
| VSYNC  |  25
| HREF   |  23
| PCLK  |  22

### CAN Bus Interface
10 of the pins are the same as the standard xBus interface, refered to as Intra circuitry pins, all controlled by this core. 4 Pins are used for Extra circuitry communication, controlled by an external core. The Control SDA/SCL are reserved for future use. Power is shared between both Intra- and Extra circutry.
| Pin | Intra/Extra | 
|:-----------------------------|:----:|
|  Vcc |Intra/Extra | 
|  Vsrc |Intra/Extra| 
|  GPIO |Intra|
|  CAN H |Extra| 
|  CAN L |Extra| 
|  CAN Enable |Extra| 
|  EXT Enable |Extra|
|:wavy_dash:
|  Reset |Intra| 
|  Prog |Intra| 
|  TXD |Intra| 
|  RXD |Intra| 
|  SDA |Intra| 
|  SCL |Intra| 
|  GND |Intra/Extra| 

### Other Pins
| Function |  GPIO v1-4|GPIO v5| Notes| Suggested Library |
|:-----------------------------|:----:|:----:|:--| :-- |
|  [I²C SDA](#ic) |26| 17||
|  [I²C SCL](#ic)  |27| 18||
|  Serial TX |1| 43||
|  Serial RX |3| 44||
|  CAN Bus TX |4|7 |[Suggested library](https://github.com/sandeepmistry/arduino-CAN)|
|  CAN Bus RX |2|6 ||
|  [Neopixel](#neopixel) |18‡  v4|39|[Suggested library](https://github.com/Freenove/Freenove_WS2812_Lib_for_ESP32)|
| Red LED | n/a |40
|  IO |33¶  v4 |1|
| Play button | n/a | 0
| 1-bit memory | n/a | 2 | Used for .uf2 bootloader

### Special pin use:
- Because of pin shortage Pin 5 (see † above), are being used as an output pin for the SD Card Chip Select (CS), and as a Data0 input pin for the camera. 
- Because of pin shortage Pin 18 (see ‡ above), are being used as an output pin for the Neopixel, and as a Data1 input pin for the camera.
- IO33 (see ¶ above) is tied to IO on the xBus as well as IO33 on the Extension slot. That pin could be used, such as with the LoRa extension, which means the pin is not available on the [xBus](https://github.com/domino4com/technology).

## Programming

### Programming in Arduino (all versions)
- To program the Domino4 cores using Arduino, install the board files using the doumentation from [Espressif](https://github.com/espressif/arduino-esp32)
  - Open "Preferences" or "Settings"
  - Insert this line ```https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json```
  - Into "Additional Boards Managers URLs"
  - Open Board Manager from the Tools menu
  - Install ESP32
  
### Programming in Arduino v1-4 (ESP32)
- Please read the notes regarding upload/transfer speed when using the [PPU](https://github.com/domino4com/PPU).
- Settings:
  - **Board:** Choose the ```ESP32 Dev Module```
  - **Speed:** Max 460800 bps
  - **PSRAM**: ```QSPI PSRAM```
  - **Port:** Chose the port where your PPU is inserted. If you cannot see the port, the check out your [PPU installation](https://github.com/domino4com/PPU)
  - **Partition Scheme**: Choose a suitable one for your project, such as ```8MB with spiffs...```

### Programming in Arduino v5 (ESP32-S3)

#### Using the onboard USB-C port
- Connect a USB-C data cable between the xChip and your computer. Notice that some USB cables only provides power. Make sure your cable is also ready for data transfer.
- Settings:
  - **Board:** Choose the ```ESP32S3 Dev Module```
  - **Speed:** Doesn't matter
  - **Port:** Chose the port where your xChip is inserted
  - **USB CDC on Boot**: ```Enable```
  - **PSRAM**: ```QSPI PSRAM```
  - **Flash Size**: ```8MB (64 Mb)```
  - **Partition Scheme**: Choose a suitable one for your project, such as ```8MB with spiffs...```
  
#### Using the PPU
- Please read the notes regarding upload/transfer speed when using the [PPU](https://github.com/domino4com/PPU).
- Settings:
  - **Board:** Choose the ```ESP32S3 Dev Module```
  - **Speed:** Max 460800 bps
  - **PSRAM**: ```QSPI PSRAM```
  - **Flash Size**: ```8MB (64 Mb)```
  - **Port:** Chose the port where your PPU is inserted. If you cannot see the port, the check out your [PPU installation](https://github.com/domino4com/PPU)
  - **Partition Scheme**: Choose a suitable one for your project, such as ```8MB with spiffs...```

### Programming in Python.
- Download the MicroPython firmware from [micropython.org](https://micropython.org/download/esp32/)
- It is recommended to download and use the (Mu Editor)(https://codewith.mu/en/download)
- You can use the Mu Editor to upload the MicroPython Firmware.
- Check out this [Quick Reference](https://docs.micropython.org/en/latest/esp32/quickref.html)

### Neopixel

#### Arduino
Install this [library](https://github.com/Freenove/Freenove_WS2812_Lib_for_ESP32)
and tryout the example below:
```C
#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT 1
// Uncomment below for your version
// #define LEDS_PIN 18 //v4
// #define LEDS_PIN 39 //v5
#define CHANNEL 0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

void setup() {
  strip.begin();
}

void loop() {
  strip.setLedColor(0,random(255), random(255), random(255));
  delay(50);
}
```

### I²C
I²C's SDA and SCL is not on the standard ESP32's Pin normally used.

#### Arduino
The pins has to be set before the `Wire.begin()` statement like this:
```C
// Uncomment below for your version
// #define I2C_SDA 26 //v4
// #define I2C_SCL 27 //v4
// #define I2C_SDA 17 //v5
// #define I2C_SCL 18 //v5
void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
}
```

#### MicroPython
SCL & SDA specified in MicroPython (notice the reversed order compared to Arduino). In this example also configured for `400 kbit/s Fast Mode FM`:

##### v1-4
```python
from machine import Pin, I2C
i2c = I2C(0, scl=Pin(27), sda=Pin(26), freq=400000)
```
##### v5
```python
from machine import Pin, I2C
i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)
```
### SPI
This core has two SPI busses. This core is using the HSPI bus. In certain libraries it is sufficient to set the SPI pins, in others using the HSPI has to be specified:

#### Example: LoRa over SPI
```C
#include <SPI.h>
#include <RadioLib.h>
// Uncomment below for your version
// v1-4
// #define SPI_MISO 12
// #define SPI_MOSI 13
// #define SPI_SCK 14
// #define LORA_CS 15 // NSS
// #define LORA_DIO0 33
// v5
// #define SPI_MISO 38
// #define SPI_MOSI 37
// #define SPI_SCK 36
// #define LORA_CS 35 // NSS
// #define LORA_DIO0 34
#define LORA_DIO1 -1
#define LORA_RESET -1

SPIClass mySpi (HSPI);
SPISettings spiSettings (2000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module (LORA_CS, LORA_DIO0, LORA_RESET, LORA_DIO1, mySpi, spiSettings);

void setup() {
  mySpi.begin();
}
```

#### Example: SD Card over SPI

#### Arduino
```C
// Uncomment below for your version
// v1-4
// #define SPI_MISO 12
// #define SPI_MOSI 13
// #define SPI_SCK 14
// #define SD_CS 5
// v5
// #define SPI_MISO 38
// #define SPI_MOSI 37
// #define SPI_SCK 36
// #define SD_CS 3

void setup() {
  SPIClass spi = SPIClass(HSPI);
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spi)) {
  // Error code
  }
}
```

#### MicroPython
##### v1-4
```python
import machine, sdcard, os
sd = sdcard.SDCard(machine.SPI(1), cs=machine.Pin(5))
```
##### v5 (not tested)
```python
import machine, sdcard, os
sd = sdcard.SDCard(machine.SPI(1), cs=machine.Pin(3))
```


## Troubleshooting
- If you try to upload code and getting this message ```A fatal error occurred: Timed out waiting for packet content``` or ```A fatal error occurred: Invalid head of packet (0xE0)```, change the transfer speed to 460800 pbs.
- If you try to upload code and getting this message ```[7886] Failed to execute script esptool the selected serial port [7886] Failed to execute script esptool does not exist or your board is not connected```, your serial port is open by another application. Close the other application and try again.

# License: 
<img src="assets/CC-BY-NC-SA.svg" width=200 align="right">
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License

[View License Deed](https://creativecommons.org/licenses/by-nc-sa/4.0/) | [View Legal Code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

