<img src="assets/CWV.svg" width=200 align="right">

# CWS Core WiFi Extended
This is the extended core used in the Domino4 eco-system.
> This core has 4Mb of PSRAM. Remember to enable the PSRAM in Arduino.

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
| Pins| Function | Group |Lora|
|:-----------------------------:|:----:|:--:|:--:|
| :one:                 | Vcc | Power|Vcc
| :two:                 | MISO | SPI|MISO
| :three:                    | MOSI | SPI|MOSI
| :four:                  | SCK | SPI|SCK
| :five:            | IO15  | GPIO|NSS
| :six: | IO33 | GPIO|DIO0
| :seven:                       | IO2 | GPIO| n/a
| :eight:                       | SCL | I²C | n/a
| :nine:                       | SDA | I²C | n/a
| :keycap_ten:                       | GND | Power| GND

### SD Card
SD Card is used in 4 Pin SPI configuration.
| Pin | GPIO |
|:-----------------------------|:----:|
|  MISO |12| 
|  MOSI |13| 
|  SCK |14| 
|  CS |5†| 

### Camera
The camera uses exactly the same pins as the ESP32-CAM from AI-Thinker
| Function | GPIO | 
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
| Y3   |    18
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
|  Control SDA |Extra| 
|  Control SCL |Extra|
|:wavy_dash:
|  Reset |Intra| 
|  Prog |Intra| 
|  TXD |Intra| 
|  RXD |Intra| 
|  SDA |Intra| 
|  SCL |Intra| 
|  GND |Intra/Extra| 

### Other Pins
| Function |  GPIO | Notes|
|:-----------------------------|:----:|:--|
|  I²C SDA |26| |
|  I²C SCL |27| |
|  Serial TX |1| |
|  Serial RX |3| |
|  CAN Bus TX |4| |
|  CAN Bus RX |2| |

### Special pin use:
- Because of pin shortage Pin 5 (see † above), are being used as an output pin for the SD Card Chip Select (CS), and as a Data0 input pin for the camera. 

## Programming

### I²C
I²C's SDA and SCL is not on the standard ESP32's Pin normally used in Arduino. The pins has to be set before the `Wire.begin()` statement like this:
```C
#define I2C_SDA 26
#define I2C_SCL 27
void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
}
```
### SPI
This core has two SPI busses. This core are using the HSPI. In certain libraries it is sufficient to set the SPI pins, in others using the HSPI has to be specified:

#### Example: LoRa over SPI
```C
#include <SPI.h>
#include <RadioLib.h>
#define SPI_MISO 12
#define SPI_MOSI 13
#define SPI_SCK 14
#define LORA_CS 15 // NSS
#define LORA_DIO0 33
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
```C
#define SPI_MISO 12
#define SPI_MOSI 13
#define SPI_SCK 14
#define SD_CS 5

void setup() {
  SPIClass spi = SPIClass(HSPI);
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spi)) {
  // Error code
  }
}
```

## Troubleshooting
- If you try to upload code and getting this message ```A fatal error occurred: Timed out waiting for packet content``` or ```A fatal error occurred: Invalid head of packet (0xE0)```, change the transfer speed to 460800 pbs.
- If you try to upload code and getting this message ```[7886] Failed to execute script esptool the selected serial port [7886] Failed to execute script esptool does not exist or your board is not connected```, your serial port is open by another application. Close the other application and try again.

# License: 
<img src="assets/CC-BY-NC-SA.svg" width=200 align="right">
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License

[View License Deed](https://creativecommons.org/licenses/by-nc-sa/4.0/) | [View Legal Code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

