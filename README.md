<div align="center">

# 🚻 Restroom Intelligence
### Dual-Microcontroller Edge Telemetry, Predictive Hygiene Scoring & Zero-CDN iOS-Inspired Web Dashboard

[![Platform](https://img.shields.io/badge/MCU-ATmega328P%20%2B%20ESP8266-007aff?style=for-the-badge)](hardware/PINOUT.md)
[![Protocol](https://img.shields.io/badge/UART-v2%20CRC--8%20(0x07)-34c759?style=for-the-badge)](#uart-v2-telemetry-protocol)
[![Storage](https://img.shields.io/badge/MicroSD-8.3%20CSV%20Engine-ff9500?style=for-the-badge)](#microsd-csv-data-schema)
[![Guide](https://img.shields.io/badge/Guide-Apple--Style%20Interactive%20HTML-5e5ce6?style=for-the-badge)](docs/index.html)

**Complete Microprocessor Lab repository combining real-time embedded control on an Arduino UNO (`ATmega328P`) with edge analytics, MicroSD CSV logging, 12-step hardware verification, printable daily reports, and an offline iOS-inspired web UI on a NodeMCU (`ESP8266`).**

[Open the Apple-Style Illustrated Guide (`docs/index.html`)](docs/index.html) · [Complete Pinout](hardware/PINOUT.md) · [Bill of Materials](hardware/BOM.md)

</div>

---

## 🎯 Core Objective: Edge Analytics & Predictive Hygiene Insights

The ESP8266 maintains 288 five-minute rolling buckets and logs CSV data to the MicroSD card so the **Insights** and **Data** views compute correlations, occupancy patterns, water efficiency, cleaning impact, and printable daily reports offline (`http://192.168.4.1`):

<p align="center">
  <img src="docs/images/ui/14_insights_entries_vs_air.png" alt="Entries vs Air Correlation" width="32%">
  <img src="docs/images/ui/15_insights_busy_hours.png" alt="Weekday x Hour Heatmap" width="32%">
  <img src="docs/images/ui/16_insights_entries_vs_water.png" alt="Entries vs Water Use" width="32%">
</p>
<p align="center">
  <img src="docs/images/ui/17_insights_fan_effectiveness.png" alt="Exhaust Fan Effectiveness" width="32%">
  <img src="docs/images/ui/18_insights_cleaning_history.png" alt="Before and After Cleaning Audit" width="32%">
  <img src="docs/images/ui/19b_insights_daily_report.png" alt="Printable Restroom Daily Report" width="32%">
</p>
<p align="center">
  <img src="docs/images/ui/19c_insights_overview_window.png" alt="Insights Overview and Time Window Query" width="46%">
  <img src="docs/images/ui/18b_data_sd_export.png" alt="MicroSD Storage and Date Export" width="46%">
</p>

* **Entries vs. Air Index:** Computes Pearson correlation ($r$) across 0–15 minute lags to evaluate how foot traffic relates to air quality and support adaptive cleaning limits.
* **Busy Hours Heatmap:** Weekday × Hour traffic map highlighting peak windows (e.g., `Fri 16:00`) and cumulative entry counts.
* **Entries vs. Water Use:** Tracks water used per entry (`0.46 L/entry`) and flags continuous flow during zero-entry periods.
* **Fan Effectiveness & Cleaning History:** Compares 5-minute air index deltas with the fan ON vs. OFF and logs air index before/after cleaning events.
* **Printable Daily Report & MicroSD 8.3 Export:** Generates a printable summary report (`@media print`) and streams `datalast.csv`, `entrlast.csv`, `evntlast.csv`, `cmdslast.csv`, and `bktslast.csv` (`docs/images/ui/19_raw_uart_telemetry.png` & `docs/images/ui/19d_data_sd_stream_updated.png`).

---

## 📱 Website Experience Flow

### 1. Loading Screen & 3-Step Guided Calibration
Animated dual-MCU UART handshake screen followed by guided baseline capture (MQ-135 clean air, water probe dry/wet, and zero-flow baseline):

<p align="center">
  <img src="docs/images/ui/01_loading_boot.png" alt="Loading Screen UI" width="46%">
  <img src="docs/images/ui/02_system_calibrated.png" alt="System Calibrated Modal" width="46%">
</p>

### 2. Main Screen (`100 / 100` Cleanliness Score) & Alert States
Shows the weighted `0–100` cleanliness ring (`100 of 100 FRESH` in healthy conditions, and dynamic `35 of 100 Due now` alerts or probe/air fault warnings):

<p align="center">
  <img src="docs/images/ui/03_main_dashboard_100.png" alt="Main Dashboard 100 of 100 Cleanliness Score" width="24%">
  <img src="docs/images/ui/03d_home_clean_3h_countdown.png" alt="Home Screen 100 Score 3h Countdown" width="24%">
  <img src="docs/images/ui/03b_home_poor_air_due_now.png" alt="Poor Air Alert and Cleaning Due Now" width="24%">
  <img src="docs/images/ui/03c_home_probe_fault_alert.png" alt="Water Probe Fault Alert on Home Screen" width="24%">
</p>

### 3. Charts, Fan Control & Live Sensor Cards
<p align="center">
  <img src="docs/images/ui/04_entries_per_hour.png" alt="Entries Per Hour 12h Chart" width="32%">
  <img src="docs/images/ui/05_air_trend_10min.png" alt="10-Minute Air Trend" width="32%">
  <img src="docs/images/ui/06_flow_trend_10min.png" alt="10-Minute Flow Trend" width="32%">
</p>
<p align="center">
  <img src="docs/images/ui/07_exhaust_fan_manual.png" alt="Exhaust Fan Manual Control" width="32%">
  <img src="docs/images/ui/08_exhaust_fan_auto.png" alt="Exhaust Fan Auto Control" width="32%">
  <img src="docs/images/ui/09_air_quality_gauge.png" alt="Air Quality 143 GOOD" width="32%">
</p>
<p align="center">
  <img src="docs/images/ui/10_water_tank_wet.png" alt="Water Tank 100% WET" width="32%">
  <img src="docs/images/ui/11_water_flow_active.png" alt="Water Flow Active 3.09 L/min" width="32%">
  <img src="docs/images/ui/12_water_flow_idle.png" alt="Water Flow Idle 0.00 L/min" width="24%">
  <img src="docs/images/ui/13_door_people_counter.png" alt="Door and People Counter" width="24%">
</p>

### 4. 12-Step Hardware Verification Wizard & Calibration Tab
Step-by-step verification across all 12 hardware steps—from ESP8266 gateway, UART link, MicroSD, DS3231 RTC, MQ-135 alert slider, water probe, HC-SR04, 18s flow test, people counter, and door closed/open references to the final summary report and **Calibrate** tab:

<p align="center">
  <img src="docs/images/ui/20_verify_step1_gateway.png" alt="Step 1 Gateway" width="24%">
  <img src="docs/images/ui/21_verify_step2_uart.png" alt="Step 2 UART Link" width="24%">
  <img src="docs/images/ui/22_verify_step3_microsd.png" alt="Step 3 MicroSD" width="24%">
  <img src="docs/images/ui/23_verify_step4_ds3231_rtc.png" alt="Step 4 DS3231 RTC" width="24%">
</p>
<p align="center">
  <img src="docs/images/ui/24_verify_step5_mq135_slider.png" alt="Step 5 MQ-135 Slider" width="24%">
  <img src="docs/images/ui/25_verify_step6_water_probe.png" alt="Step 6 Water Probe" width="24%">
  <img src="docs/images/ui/26_verify_step7_hcsr04.png" alt="Step 7 HC-SR04 Ultrasonic" width="24%">
  <img src="docs/images/ui/27_verify_step8_flow_test.png" alt="Step 8 Flow 18s Test" width="24%">
</p>
<p align="center">
  <img src="docs/images/ui/28_verify_step9_people_counter.png" alt="Step 9 People Counter" width="24%">
  <img src="docs/images/ui/29_verify_step10_door_closed.png" alt="Step 10 Door Closed" width="24%">
  <img src="docs/images/ui/30_verify_step11_door_open.png" alt="Step 11 Door Open" width="24%">
  <img src="docs/images/ui/31_verify_step12_report.png" alt="Step 12 Verification Report" width="24%">
</p>
<p align="center">
  <img src="docs/images/ui/32_calibrate_tab_full.png" alt="Calibrate Tab Cleaning Rule and Sensors" width="46%">
</p>

---

## 🛠️ Original Physical Prototype & CAD Blueprint

Tabletop prototype (`320 × 180 × 150 mm`) with hinged door, top LCD cutout slot (tested separately on the I2C bench), side-mounted water flow sensor, water probe in test cup, and rear exhaust fan + dual-MCU electronics:

<p align="center">
  <img src="docs/images/hardware/01_cad_blueprint.png" alt="CAD Isometric and Orthographic Blueprint" width="820">
</p>
<p align="center">
  <img src="docs/images/hardware/02_front_enclosure.jpg" alt="Front Enclosure with Door and LCD Cutout" width="32%">
  <img src="docs/images/hardware/03_door_entry_closed.jpg" alt="Door Closed Entry Test" width="32%">
  <img src="docs/images/hardware/04_door_entry_open.jpg" alt="Door Open Entry Test" width="32%">
</p>
<p align="center">
  <img src="docs/images/hardware/05_rear_electronics_fan.jpg" alt="Rear Electronics and Exhaust Fan" width="24%">
  <img src="docs/images/hardware/06_side_flow_sensor.jpg" alt="Side-Mounted Flow Sensor" width="24%">
  <img src="docs/images/hardware/07_water_level_probe.jpg" alt="Water Level Probe Cup Test" width="24%">
  <img src="docs/images/hardware/08_lcd_i2c_bench_test.jpg" alt="16x2 I2C LCD Bench Test" width="24%">
</p>

---

## 🏗️ System Architecture & Voltage Level Shifting

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
