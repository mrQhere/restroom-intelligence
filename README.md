<div align="center">

# 🚻 Restroom Intelligence
### Dual-Microcontroller Edge Telemetry, Predictive Hygiene Scoring & Zero-CDN iOS-Inspired Web Dashboard

[![Platform](https://img.shields.io/badge/MCU-ATmega328P%20%2B%20ESP8266-007aff?style=for-the-badge)](hardware/PINOUT.md)
[![Protocol](https://img.shields.io/badge/UART-v2%20CRC--8%20(0x07)-34c759?style=for-the-badge)](#uart-v2-telemetry-protocol)
[![Storage](https://img.shields.io/badge/MicroSD-8.3%20CSV%20Engine-ff9500?style=for-the-badge)](#sample-microsd-datasets-data)
[![Live Guide](https://img.shields.io/badge/Live%20Guide-Apple--Style%20Interactive%20Site-5e5ce6?style=for-the-badge)](https://mrqhere.github.io/restroom-intelligence/)

**An end-to-end Microprocessor Lab system pairing a deterministic Arduino UNO (`ATmega328P`) sensor controller with a NodeMCU (`ESP8266`) Wi-Fi edge analytics gateway, MicroSD CSV logger, and offline iOS-inspired web dashboard.**

[**🌐 Open Live Interactive Guide**](https://mrqhere.github.io/restroom-intelligence/) · [🔌 Complete Pinout](hardware/PINOUT.md) · [🛒 Bill of Materials](hardware/BOM.md) · [📊 Sample CSV Data](data/)

</div>

---

## 🎯 Core Objective: Edge Analytics & Predictive Hygiene Insights

The ESP8266 maintains **288 five-minute rolling buckets** and logs persistent 8.3 CSV files to the MicroSD card so the **Insights** and **Data** tabs compute correlations, occupancy heatmaps, water efficiency, and cleaning audits completely offline (`http://192.168.4.1`).

<table>
  <tr>
    <td align="center" width="33%"><img src="docs/images/ui/14_insights_entries_vs_air.png" height="270" alt="Entries vs Air"><br><sub><b>Entries vs. Air Index</b><br>Pearson correlation (r) & lag analysis</sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/15_insights_busy_hours.png" height="270" alt="Busy Hours Heatmap"><br><sub><b>Weekday × Hour Heatmap</b><br>Peak traffic slot detection (Fri 16:00)</sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/16_insights_entries_vs_water.png" height="270" alt="Entries vs Water Use"><br><sub><b>Entries vs. Water Use</b><br>0.46 L/entry & idle leak detection</sub></td>
  </tr>
  <tr>
    <td align="center" width="33%"><img src="docs/images/ui/17_insights_fan_effectiveness.png" height="270" alt="Fan Effectiveness"><br><sub><b>Exhaust Fan Effectiveness</b><br>5-min air recovery & 24h duty cycle</sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/18_insights_cleaning_history.png" height="270" alt="Cleaning History"><br><sub><b>Cleaning Audit Log</b><br>Air index before → 5 min after clean</sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/19_raw_uart_telemetry.png" height="270" alt="Raw UART Stream"><br><sub><b>Live CRC-8 UART Stream</b><br>Real-time T,2,... telemetry inspector</sub></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center" width="33%"><img src="docs/images/ui/19c_insights_overview_window.png" height="360" alt="Insights Overview"><br><sub><b>Insights Overview & Time Window</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/19b_insights_daily_report.png" height="360" alt="Printable Daily Report"><br><sub><b>Printable Restroom Daily Report</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/18b_data_sd_export.png" height="360" alt="MicroSD CSV Export"><br><sub><b>MicroSD 8.3 CSV & Date Export</b></sub></td>
  </tr>
</table>

---

## 📱 Website Experience Flow

### 1. Animated Dual-MCU Loading Screen & Guided Calibration
Joining Wi-Fi **`Restroom-Monitor`** (Password: **`restroom123`**) launches an animated UART circuit handshake between the **ATmega328P** and **ESP8266**, followed by a 3-step baseline calibration check.

<table>
  <tr>
    <td align="center" width="50%"><img src="docs/images/ui/01_loading_boot.png" height="380" alt="Loading Screen"><br><sub><b>Dual-MCU Boot & Handshake</b></sub></td>
    <td align="center" width="50%"><img src="docs/images/ui/02_system_calibrated.png" height="380" alt="System Calibrated"><br><sub><b>3-Step Guided Calibration Complete</b></sub></td>
  </tr>
</table>

### 2. Main Screen (`100 / 100` Cleanliness Score) & Dynamic Alert States
The Home screen computes a weighted **`0–100` Cleanliness Score** (40% Entries, 30% Air, 20% Time, 10% Water) and adapts live to poor air or sensor fault conditions.

<table>
  <tr>
    <td align="center" width="25%"><img src="docs/images/ui/03_main_dashboard_100.png" height="330" alt="100 of 100 Card"><br><sub><b>100 / 100 Cleanliness Ring</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/03d_home_clean_3h_countdown.png" height="330" alt="Home Screen Fresh"><br><sub><b>Full Home View (FRESH)</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/03b_home_poor_air_due_now.png" height="330" alt="Poor Air Alert"><br><sub><b>Poor Air → Clean Due Now</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/03c_home_probe_fault_alert.png" height="330" alt="Probe Fault Alert"><br><sub><b>Water Probe Fault Isolation</b></sub></td>
  </tr>
</table>

### 3. Live Trend Charts & Tactile Sensor Cards
<table>
  <tr>
    <td align="center" width="33%"><img src="docs/images/ui/04_entries_per_hour.png" height="270" alt="Entries Per Hour"><br><sub><b>Entries Per Hour (12h)</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/05_air_trend_10min.png" height="270" alt="Air Trend"><br><sub><b>Air Trend (10 min)</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/06_flow_trend_10min.png" height="270" alt="Flow Trend"><br><sub><b>Flow Trend (10 min)</b></sub></td>
  </tr>
  <tr>
    <td align="center" width="33%"><img src="docs/images/ui/07_exhaust_fan_manual.png" height="240" alt="Fan Manual"><br><sub><b>Exhaust Fan (MANUAL)</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/08_exhaust_fan_auto.png" height="240" alt="Fan Auto"><br><sub><b>Exhaust Fan (AUTO)</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/ui/09_air_quality_gauge.png" height="240" alt="Air Quality Gauge"><br><sub><b>MQ-135 Gauge (143 GOOD)</b></sub></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center" width="25%"><img src="docs/images/ui/10_water_tank_wet.png" height="210" alt="Water Tank"><br><sub><b>Water Tank (100% WET)</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/11_water_flow_active.png" height="210" alt="Water Flow Active"><br><sub><b>Flow Active (3.09 L/min)</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/12_water_flow_idle.png" height="210" alt="Water Flow Idle"><br><sub><b>Flow Idle (0.00 L/min)</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/13_door_people_counter.png" height="210" alt="Door People Counter"><br><sub><b>Door & People Counter</b></sub></td>
  </tr>
</table>

### 4. 12-Step Hardware Verification Wizard & Calibration Tab
<table>
  <tr>
    <td align="center" width="25%"><img src="docs/images/ui/20_verify_step1_gateway.png" height="270" alt="Step 1"><br><sub><b>1. ESP8266 Gateway</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/21_verify_step2_uart.png" height="270" alt="Step 2"><br><sub><b>2. ATmega328P UART</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/22_verify_step3_microsd.png" height="270" alt="Step 3"><br><sub><b>3. MicroSD Storage</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/23_verify_step4_ds3231_rtc.png" height="270" alt="Step 4"><br><sub><b>4. DS3231 Clock</b></sub></td>
  </tr>
  <tr>
    <td align="center" width="25%"><img src="docs/images/ui/24_verify_step5_mq135_slider.png" height="270" alt="Step 5"><br><sub><b>5. MQ-135 & Slider</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/25_verify_step6_water_probe.png" height="270" alt="Step 6"><br><sub><b>6. Water Probe</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/26_verify_step7_hcsr04.png" height="270" alt="Step 7"><br><sub><b>7. HC-SR04 Ultrasonic</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/27_verify_step8_flow_test.png" height="270" alt="Step 8"><br><sub><b>8. 18s Flow Test</b></sub></td>
  </tr>
  <tr>
    <td align="center" width="25%"><img src="docs/images/ui/28_verify_step9_people_counter.png" height="270" alt="Step 9"><br><sub><b>9. People Counter</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/29_verify_step10_door_closed.png" height="270" alt="Step 10"><br><sub><b>10. Door Closed Ref</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/30_verify_step11_door_open.png" height="270" alt="Step 11"><br><sub><b>11. Door Open Ref</b></sub></td>
    <td align="center" width="25%"><img src="docs/images/ui/31_verify_step12_report.png" height="270" alt="Step 12"><br><sub><b>12. Summary Report</b></sub></td>
  </tr>
</table>

<p align="center">
  <img src="docs/images/ui/32_calibrate_tab_full.png" height="420" alt="Calibrate Tab">
  <br><sub><b>Calibrate Tab — Adaptive Cleaning Rules & EEPROM Sensor Calibration</b></sub>
</p>

---

## 🛠️ Original Physical Prototype & CAD Blueprint

Tabletop lab enclosure (`320 × 180 × 150 mm`) with a hinged door, top cutout slot for the 16×2 I2C LCD (validated separately on the bench), side-mounted water flow sensor, water probe test cup, and rear exhaust fan + dual-MCU electronics bay.

<p align="center">
  <img src="docs/images/hardware/01_cad_blueprint.png" alt="CAD Isometric and Orthographic Blueprint" width="820">
</p>

<table>
  <tr>
    <td align="center" width="33%"><img src="docs/images/hardware/02_front_enclosure.jpg" height="240" alt="Front Enclosure"><br><sub><b>Front Enclosure & LCD Slot</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/hardware/03_door_entry_closed.jpg" height="240" alt="Door Closed"><br><sub><b>Door Closed Proximity Test</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/hardware/04_door_entry_open.jpg" height="240" alt="Door Open"><br><sub><b>Door Open Entry Test</b></sub></td>
  </tr>
  <tr>
    <td align="center" width="33%"><img src="docs/images/hardware/05_rear_electronics_fan.jpg" height="240" alt="Rear Electronics"><br><sub><b>Rear Electronics & Exhaust Fan</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/hardware/06_side_flow_sensor.jpg" height="240" alt="Side Flow Sensor"><br><sub><b>Side-Mounted Flow Sensor</b></sub></td>
    <td align="center" width="33%"><img src="docs/images/hardware/07_water_level_probe.jpg" height="240" alt="Water Probe Cup"><br><sub><b>Water Probe Calibration Cup</b></sub></td>
  </tr>
</table>

<p align="center">
  <img src="docs/images/hardware/08_lcd_i2c_bench_test.jpg" height="260" alt="16x2 I2C LCD Bench Test">
  <br><sub><b>16×2 I2C LCD (JHD 162A) & DS3231 RTC Bench Test</b></sub>
</p>

---

## 🏗️ System Architecture & Wiring Diagrams

<p align="center">
  <img src="docs/diagrams/architecture.svg" alt="Dual-MCU System Architecture">
</p>
<p align="center">
  <img src="docs/diagrams/connection_diagram.svg" alt="Complete Hardware Connection Diagram">
</p>
<p align="center">
  <img src="docs/diagrams/voltage_divider.svg" alt="5V to 3.3V UART Voltage Divider">
</p>

* 📄 **Pinout Matrix:** [`hardware/PINOUT.md`](hardware/PINOUT.md)
* 🛒 **Bill of Materials:** [`hardware/BOM.md`](hardware/BOM.md)
* 💻 **Arduino UNO Firmware:** [`firmware/arduino_uno/Restroom_Intelligence_ATmega328.ino`](firmware/arduino_uno/Restroom_Intelligence_ATmega328.ino)
* 🌐 **NodeMCU ESP8266 Firmware:** [`firmware/nodemcu_esp8266/Restroom_Intelligence_NodeMCU_FIXED_v3.ino`](firmware/nodemcu_esp8266/Restroom_Intelligence_NodeMCU_FIXED_v3.ino)

---

## 📊 Sample MicroSD Datasets (`data/`)
Real telemetry logs captured from the NodeMCU ESP8266 MicroSD card are included in [`data/`](data/):
* [`restroom_data_last.csv`](data/restroom_data_last.csv) — 1 Hz telemetry log (air, water, flow, distance, fault bits)
* [`restroom_entries_last.csv`](data/restroom_entries_last.csv) — Exact entry event timestamps
* [`restroom_events_last.csv`](data/restroom_events_last.csv) — System boot, fault, and cleaning audit log
* [`restroom_commands_last.csv`](data/restroom_commands_last.csv) — Control commands and ATmega ACK/NACK records
* [`restroom_buckets_last.csv`](data/restroom_buckets_last.csv) — 5-minute aggregated analytics buckets

---

## 📄 License & Author
**Author:** [`@mrQhere`](https://github.com/mrQhere)  
**License:** Distributed under the **MIT License**. See [`LICENSE`](LICENSE) for details.  
**Copyright:** `© 2026 mrQhere`