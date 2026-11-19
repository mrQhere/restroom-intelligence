# Complete Pinout & Wiring Matrix

## 1. Arduino UNO (ATmega328P — 5V Domain)

| Arduino Pin | Connected Module / Signal | Direction | Electrical / Protocol Notes |
| :--- | :--- | :--- | :--- |
| **A0** | MQ-135 Gas Sensor (`AO`) | Analog Input | 0–1023 ADC; startup rail check flags `<=2` or `>=1021` |
| **A1** | Water Level Probe (`S`) | Analog Input | 0–1023 ADC; requires `CAL,WATER_DRY` & `CAL,WATER_WET` |
| **A4 (SDA)** | 16x2 I2C LCD + DS3231 RTC | Bidirectional I2C | Shared bus; LCD at `0x27` (or `0x3F`), DS3231 at `0x68` |
| **A5 (SCL)** | 16x2 I2C LCD + DS3231 RTC | I2C Clock | Shared clock line with 4.7kΩ pull-ups (on modules) |
| **D2** | Manual Reset Push Button | Digital Input (`PULLUP`) | Connect to GND; hold `>= 1.8s` to reset people count |
| **D3 (INT1)** | ZJ-S201 / YF-S201 Water Flow Sensor | Interrupt Input (`FALLING`) | Hall-effect pulse counter (`450 pulses/L` default) |
| **D4** | Exhaust Fan Relay (`IN`) | Digital Output | **Active-LOW** assumed (`LOW` = Fan ON, `HIGH` = Fan OFF) |
| **D5** | HC-SR04 Ultrasonic (`TRIG`) | Digital Output | 10 µs trigger pulse every 300 ms |
| **D6** | HC-SR04 Ultrasonic (`ECHO`) | Digital Input | Echo pulse width measurement (timeout 28 ms) |
| **D7** | NodeMCU `D1` (via Divider) | SoftwareSerial `TX` (9600) | **MUST pass through 1kΩ / 2.2kΩ voltage divider** |
| **D8** | NodeMCU `D2` (`TX`) | SoftwareSerial `RX` (9600) | Direct connection (3.3V HIGH is valid TTL HIGH on 5V UNO) |
| **D9** | Auto/Manual Toggle Switch | Digital Input (`PULLUP`) | Switch to GND = `MANUAL` mode; overrides dashboard |
| **D10** | Status LED (Green) | Digital Output | Use 220Ω resistor; mirrors Exhaust Fan state |
| **D11** | Fault LED (Yellow) | Digital Output | Use 220Ω resistor; illuminates on sensor/RTC/cal fault |

---

## 2. NodeMCU ESP8266 (ESP-12E — 3.3V Domain)

| NodeMCU Pin | Connected Module / Signal | Direction | Electrical / Protocol Notes |
| :--- | :--- | :--- | :--- |
| **D1 (GPIO5)** | Voltage Divider Junction | SoftwareSerial `RX` (9600) | Receives 3.44V attenuated telemetry from UNO `D7` |
| **D2 (GPIO4)** | Arduino UNO `D8` | SoftwareSerial `TX` (9600) | Transmits commands & `PACKET_ACK` frames |
| **D0 (GPIO16)** | MicroSD Module `CS` | SPI Chip Select | FAT32 formatted MicroSD card (`<= 32GB` recommended) |
| **D5 (GPIO14)** | MicroSD Module `SCK` | SPI Clock | Hardware SPI Clock |
| **D6 (GPIO12)** | MicroSD Module `MISO` | SPI Master-In Slave-Out | Hardware SPI MISO |
| **D7 (GPIO13)** | MicroSD Module `MOSI` | SPI Master-Out Slave-In | Hardware SPI MOSI |
| **VIN / 3V3** | Power Supply | Power Input | Power MicroSD module `VCC` per module regulator spec |
| **GND** | Common Ground | Ground Reference | **Must share common GND with Arduino UNO & sensors** |
