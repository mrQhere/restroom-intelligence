# Bill of Materials (BOM) — Microprocessor Lab Edition

| Qty | Component | Specification / Model | Purpose |
| :-: | :--- | :--- | :--- |
| 1 | **Arduino UNO R3** | ATmega328P (16 MHz, 5V) | Deterministic sensor acquisition, LCD & relay control |
| 1 | **NodeMCU ESP8266** | ESP-12E (LoLin / Amica v2/v3) | Wi-Fi SoftAP, Edge Analytics, Web UI & MicroSD logger |
| 1 | **Gas / Air Quality Sensor** | MQ-135 Module | Relative indoor air index & odor/ammonia trend detection |
| 1 | **Ultrasonic Distance Sensor** | HC-SR04 (Side-mounted) | Entry detection & two-position door state proxy |
| 1 | **Water Flow Sensor** | YF-S201 / ZJ-S201 (1/2") | Hall-effect turbine flow rate (L/min) & total volume (L) |
| 1 | **Water Level Sensor** | Analog Water Probe | Tank water presence and calibrated percentage |
| 1 | **Real-Time Clock (RTC)** | DS3231 + CR2032 Coin Cell | Battery-backed timestamping (`YYYY-MM-DD HH:MM:SS`) |
| 1 | **Character LCD Display** | 16x2 LCD (`JHD 162A`) + PCF8574 I2C | Live local telemetry (`P:xx AIR:GOOD / W:xx% F:x.xL/M`) |
| 1 | **MicroSD Card Module + Card** | SPI MicroSD Adapter + FAT32 Card | Persistent 8.3 CSV logging (`datalast.csv`, etc.) |
| 1 | **Relay Module + Exhaust Fan** | 1-Ch 5V Relay + 5V DC Fan (Sunon) | Switches exhaust ventilation fan (active-LOW) |
| 1 | **Resistors (Voltage Divider)** | 1x `1 kΩ`, 1x `2.2 kΩ` (1/4W) | Level-shifts UNO 5V TX (`D7`) to NodeMCU 3.3V RX (`D1`) |
| 1 | **Enclosure & Controls** | Foamboard Model (`320×180×150 mm`), Switch, Button | Tabletop physical prototype with hinged door & LCD cutout |
