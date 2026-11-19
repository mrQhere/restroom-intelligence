/*
  RESTROOM INTELLIGENCE - Arduino UNO firmware
  USB Serial Monitor: 115200 baud
  UNO <-> NodeMCU UART: 9600 baud
  Libraries: RTClib by Adafruit, LiquidCrystal_I2C
  LCD address default: 0x27 (change to 0x3F if required)

  Startup self-check:
  - LCD/I2C device scan (checks LCD and DS3231 addresses on the shared bus)
  - DS3231 presence + lost-power check
  - MQ gas-sensor and water-sensor ADC rail checks
  - HC-SR04 echo check (no echo at boot is reported as a warning)
  - flow input electrical level check (flow itself is demand-dependent)
  Faults remain visible in Serial Monitor and are sent to the ESP dashboard.
  The LCD remains on the fixed two-line live display.
  Fixed LCD: P:xx AIR:GOOD / W:xx% F:x.xL/M (no rotation).
  UART v2 includes sequence number and CRC-8 (polynomial 0x07).
  Water percentage requires CAL,WATER_DRY then CAL,WATER_WET.
*/
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <EEPROM.h>

const byte PIN_MQ = A0;
const byte PIN_WATER = A1;
const byte PIN_FLOW = 3;        // ZJ-S201 flow sensor signal
const byte PIN_RELAY = 4;       // relay IN (active LOW assumed)
const byte PIN_TRIG = 5;
const byte PIN_ECHO = 6;
const byte PIN_ESP_RX = 8;      // UNO receives from ESP D2
const byte PIN_ESP_TX = 7;      // UNO sends to ESP D1 through divider
const byte PIN_SWITCH = 9;      // toggle to GND = manual mode
const byte PIN_BUTTON = 2;      // optional push button to GND; long press resets count
const byte PIN_LED_GREEN = 10;  // optional
const byte PIN_LED_YELLOW = 11; // optional

SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX);
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile unsigned long flowPulses = 0;
float waterLevelPct = -1.0f; // -1 means dry/wet calibration has not been completed
bool remoteModeOverride = false;
bool remoteManualMode = false;
bool previousSwitchLow = false;
uint32_t telemetrySequence = 0;
char pendingTelemetryFrame[260] = "";
uint32_t pendingTelemetrySequence = 0;
unsigned long pendingTelemetrySentAt = 0;
byte pendingTelemetryAttempts = 0;
bool telemetryAwaitingAck = false;
unsigned long telemetryAckedCount = 0;
unsigned long telemetryRetryCount = 0;
unsigned long telemetryDeliveryFailures = 0;

struct CalibrationData {
  uint16_t magic;
  int16_t airBaseline;
  int16_t waterDry;
  int16_t waterWet;
  float pulsesPerLitre;
};
const uint16_t CAL_MAGIC = 0x5249;

unsigned long prevFlowPulses = 0;
unsigned long lastSample = 0, lastLCD = 0, lastDistance = 0, lastDiagPrint = 0;
unsigned long buttonDownMs = 0;
bool buttonWasDown = false, buttonResetDone = false;

float flowLpm = 0, totalLitres = 0, distanceCm = -1;
float pulsesPerLitre = 450.0; // Calibrate against a measured volume.
float nearCm = 7.0, farCm = 15.0;
int airRaw = 0, waterRaw = 0, airThreshold = 400, waterThreshold = 500;
int airBaseline = -1, waterDry = -1, waterWet = -1;
long peopleCount = 0;
bool waterAvailable = false, fanOn = false, manualMode = false;
bool targetNear = false, rtcReady = false, lcdReady = false;
bool mqFault = false, waterFault = false, ultrasonicFault = false, flowPinFault = false;
bool rtcFault = true;
String airStatus = "UNKNOWN";
String commandBuffer = "";
String faultText = "";

void loadCalibration() {
  CalibrationData c;
  EEPROM.get(0, c);
  if (c.magic == CAL_MAGIC) {
    if (c.airBaseline >= 0 && c.airBaseline <= 1023) airBaseline = c.airBaseline;
    if (c.waterDry >= 0 && c.waterDry <= 1023) waterDry = c.waterDry;
    if (c.waterWet >= 0 && c.waterWet <= 1023) waterWet = c.waterWet;
    if (c.pulsesPerLitre >= 1.0f && c.pulsesPerLitre <= 100000.0f) pulsesPerLitre = c.pulsesPerLitre;
  }
}

void saveCalibration() {
  CalibrationData c;
  c.magic = CAL_MAGIC;
  c.airBaseline = airBaseline;
  c.waterDry = waterDry;
  c.waterWet = waterWet;
  c.pulsesPerLitre = pulsesPerLitre;
  EEPROM.put(0, c);
}

void updateWaterLevel() {
  if (waterDry < 0 || waterWet < 0 || abs(waterWet - waterDry) < 10) {
    waterLevelPct = -1.0f;
    return;
  }
  float pct = ((float)waterRaw - (float)waterDry) * 100.0f / ((float)waterWet - (float)waterDry);
  waterLevelPct = constrain(pct, 0.0f, 100.0f);
}

void flowISR() { flowPulses++; }

float readDistance() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 28000UL);
  if (us == 0) return -1.0;
  float cm = us * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f;
  return cm;
}

void setFan(bool on) {
  fanOn = on;
  digitalWrite(PIN_RELAY, on ? LOW : HIGH); // active-LOW relay module assumed
  digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW);
}

void updateAirStatus() {
  // Relative MQ sensor index only. These are operational thresholds, not certified AQI/ppm.
  if (mqFault) { airStatus = "ERROR"; return; }
  int limit = (airBaseline >= 0) ? airBaseline + 100 : airThreshold;
  if (airRaw < limit) airStatus = "GOOD";
  else if (airRaw < limit + 100) airStatus = "MODERATE";
  else if (airRaw < limit + 250) airStatus = "POOR";
  else airStatus = "BAD";
}

bool i2cFound(byte address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void addFault(const String &s) {
  if (faultText.length()) faultText += " | ";
  faultText += s;
  Serial.print(F("[FAULT] "));
  Serial.println(s);
}

void startupDiagnostics() {
  faultText = "";
  lcd.clear(); lcd.setCursor(0,0); lcd.print(F("SYSTEM CHECK"));
  lcd.setCursor(0,1); lcd.print(F("Please wait..."));
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F(" RESTROOM INTELLIGENCE / UNO STARTUP DIAGNOSTICS"));
  Serial.println(F("================================================"));

  lcdReady = i2cFound(0x27);
  if (lcdReady) Serial.println(F("[PASS] LCD detected at I2C address 0x27"));
  else if (i2cFound(0x3F)) addFault(F("LCD detected at 0x3F; change LiquidCrystal_I2C constructor from 0x27 to 0x3F"));
  else addFault(F("LCD/I2C display not detected; check SDA=A4, SCL=A5, VCC/GND"));

  rtcReady = rtc.begin();
  if (rtcReady) {
    Serial.println(F("[PASS] DS3231 detected"));
    if (rtc.lostPower()) addFault(F("RTC lost power/time may need setting"));
    else Serial.println(F("[PASS] RTC clock retained power"));
  } else addFault(F("DS3231 not detected; check A4/A5 and power"));
  rtcFault = !rtcReady || (rtcReady && rtc.lostPower());

  delay(250);
  airRaw = analogRead(PIN_MQ);
  if (airRaw <= 2 || airRaw >= 1021) {
    mqFault = true; addFault(F("MQ sensor AO at ADC rail; check sensor power/A0/wiring"));
  } else {
    mqFault = false; Serial.print(F("[PASS] MQ sensor ADC responds, raw=")); Serial.println(airRaw);
  }

  waterRaw = analogRead(PIN_WATER);
  if (waterRaw <= 2 || waterRaw >= 1021) {
    waterFault = true; addFault(F("Water sensor ADC at rail; check signal A1/power/wiring"));
  } else {
    waterFault = false; Serial.print(F("[PASS] Water sensor ADC responds, raw=")); Serial.println(waterRaw);
  }

  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  distanceCm = readDistance();
  if (distanceCm < 0) {
    ultrasonicFault = true; addFault(F("HC-SR04 no echo at startup; check TRIG D5/ECHO D6/power"));
  } else {
    ultrasonicFault = false; Serial.print(F("[PASS] HC-SR04 echo detected, cm=")); Serial.println(distanceCm,1);
  }

  pinMode(PIN_FLOW, INPUT_PULLUP);
  int flowLevel = digitalRead(PIN_FLOW);
  flowPinFault = false; // Static level cannot prove flow sensor operation.
  Serial.print(F("[INFO] ZJ-S201 input level=")); Serial.print(flowLevel);
  Serial.println(F("; no flow pulses at boot is not treated as a fault"));

  Serial.print(F("[INFO] ESP UART: software serial RX=D8 TX=D7 at 9600 baud"));
  Serial.println();
  if (faultText.length()) {
    Serial.println(F("[RESULT] Startup check completed with warnings:"));
    Serial.println(faultText);
    digitalWrite(PIN_LED_YELLOW, HIGH);
  } else {
    Serial.println(F("[RESULT] Startup checks passed; flow sensor is demand-dependent."));
    digitalWrite(PIN_LED_YELLOW, LOW);
  }
  if (rtcReady && rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println(F("Live output starts below. USB Serial Monitor = 115200 baud."));
  Serial.println(F("------------------------------------------------"));
}

uint8_t calculateCRC8(const char *data) {
  uint8_t crc = 0;
  while (*data) {
    crc ^= (uint8_t)*data++;
    for (byte i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void sendTelemetry() {
  DateTime t = rtcReady ? rtc.now() : DateTime(2000,1,1,0,0,0);
  char stamp[20];
  snprintf(stamp, sizeof(stamp), "%04d-%02d-%02d %02d:%02d:%02d",
           t.year(),t.month(),t.day(),t.hour(),t.minute(),t.second());

  // Version 2 packet fields:
  // T,2,sequence,timestamp,airRaw,airStatus,people,waterRaw,waterAvailable,
  // waterLevelPct,distanceCm,flowLpm,totalLitres,fanOn,mode,rtcOK,faultBits,faultText,CRC8
  unsigned int bits = (mqFault?1:0) | (waterFault?2:0) | (ultrasonicFault?4:0) | (rtcFault?8:0) | (waterLevelPct < 0?16:0);
  String safeFaults = faultText; safeFaults.replace(',', ';'); safeFaults.replace('\n',' ');
  char levelText[10], distanceText[12], flowText[12], totalText[12];
  if (waterLevelPct >= 0) dtostrf(waterLevelPct, 1, 1, levelText); else strcpy(levelText, "-1");
  dtostrf(distanceCm, 1, 1, distanceText);
  dtostrf(flowLpm, 1, 3, flowText);
  dtostrf(totalLitres, 1, 3, totalText);
  uint32_t sequence = ++telemetrySequence;
  char payload[240];
  int payloadLength = snprintf(payload, sizeof(payload),
      "2,%lu,%s,%d,%s,%ld,%d,%d,%s,%s,%s,%s,%d,%s,%d,%u,%s",
      (unsigned long)sequence, stamp, airRaw, airStatus.c_str(), peopleCount, waterRaw,
      waterAvailable ? 1 : 0, levelText, distanceText, flowText, totalText,
      fanOn ? 1 : 0, manualMode ? "MANUAL" : "AUTO",
      (rtcReady && !rtc.lostPower()) ? 1 : 0, bits, safeFaults.c_str());

  if (payloadLength > 0 && payloadLength < (int)sizeof(payload)) {
    uint8_t crc = calculateCRC8(payload);
    snprintf(pendingTelemetryFrame, sizeof(pendingTelemetryFrame), "T,%s,%u", payload, crc);
    pendingTelemetrySequence = sequence;
    pendingTelemetryAttempts = 1;
    pendingTelemetrySentAt = millis();
    telemetryAwaitingAck = true;
    espSerial.println(pendingTelemetryFrame);
  } else {
    Serial.println(F("[ERROR] Telemetry payload overflow; frame not sent"));
  }

  Serial.print(stamp);
  Serial.print(F(" | AIR=")); Serial.print(airStatus); Serial.print('('); Serial.print(airRaw); Serial.print(')');
  Serial.print(F(" | WATER=")); Serial.print(waterAvailable?F("AVAILABLE"):F("NOT DETECTED"));
  Serial.print(F("(raw ")); Serial.print(waterRaw); Serial.print(F(", level "));
  if (waterLevelPct >= 0) { Serial.print(waterLevelPct,1); Serial.print('%'); } else Serial.print(F("UNCALIBRATED"));
  Serial.print(')');
  Serial.print(F(" | PEOPLE=")); Serial.print(peopleCount);
  Serial.print(F(" | DIST=")); if(distanceCm<0) Serial.print(F("NO_ECHO")); else Serial.print(distanceCm,1);
  Serial.print(F("cm | FLOW=")); Serial.print(flowLpm,2); Serial.print(F("L/min | TOTAL=")); Serial.print(totalLitres,2);
  Serial.print(F("L | FAN=")); Serial.print(fanOn?F("ON"):F("OFF"));
  Serial.print(F(" | MODE=")); Serial.print(manualMode?F("MANUAL"):F("AUTO"));
  Serial.print(F(" | UART ACK/RETRY/FAIL=")); Serial.print(telemetryAckedCount); Serial.print('/'); Serial.print(telemetryRetryCount); Serial.print('/'); Serial.print(telemetryDeliveryFailures);
  Serial.print(F(" | RTC=")); Serial.print((rtcReady&&!rtc.lostPower())?F("OK"):F("CHECK"));
  if (faultText.length()) { Serial.print(F(" | WARN=")); Serial.print(faultText); }
  Serial.println();
}

void updateLCD() {
  if (!lcdReady) return;
  static char previousRow0[17] = "";
  static char previousRow1[17] = "";
  char row0[17];
  char row1[17];
  char flowText[8];

  // Fixed 16x2 display. Values change in place; no page rotation or scrolling.
  long shownPeople = constrain(peopleCount, 0L, 99L);
  const char *airLabel = airStatus.c_str();
  if (airStatus == "MODERATE") airLabel = "MOD"; // fits 16 columns
  if (airStatus == "UNKNOWN") airLabel = "WAIT";
  snprintf(row0, sizeof(row0), "P:%02ld AIR:%s", shownPeople, airLabel);

  float shownFlow = constrain(flowLpm, 0.0f, 99.9f);
  dtostrf(shownFlow, 1, 1, flowText); // avoids AVR printf float support requirement
  if (waterLevelPct >= 0.0f) {
    snprintf(row1, sizeof(row1), "W:%d%% F:%sL/M", (int)(waterLevelPct + 0.5f), flowText);
  } else {
    snprintf(row1, sizeof(row1), "W:--%% F:%sL/M", flowText);
  }

  // Pad each row to exactly 16 characters to remove stale trailing characters.
  for (byte i = strlen(row0); i < 16; i++) row0[i] = ' ';
  row0[16] = '\0';
  for (byte i = strlen(row1); i < 16; i++) row1[i] = ' ';
  row1[16] = '\0';

  if (strncmp(row0, previousRow0, 16) != 0) {
    lcd.setCursor(0, 0); lcd.print(row0);
    strncpy(previousRow0, row0, sizeof(previousRow0));
  }
  if (strncmp(row1, previousRow1, 16) != 0) {
    lcd.setCursor(0, 1); lcd.print(row1);
    strncpy(previousRow1, row1, sizeof(previousRow1));
  }
}

void refreshFaultText() {
  faultText = "";
  if (mqFault) faultText += "MQ_ADC_RANGE|";
  if (waterFault) faultText += "WATER_ADC_RANGE|";
  if (ultrasonicFault) faultText += "HC_SR04_NO_ECHO|";
  if (rtcFault) faultText += "RTC_CHECK|";
  if (waterLevelPct < 0.0f) faultText += "WATER_LEVEL_UNCALIBRATED|";
  if (faultText.length()) faultText.remove(faultText.length() - 1);
}

void serviceTelemetryRetry() {
  if (!telemetryAwaitingAck || millis() - pendingTelemetrySentAt < 250UL) return;
  if (pendingTelemetryAttempts < 3) {
    espSerial.println(pendingTelemetryFrame);
    pendingTelemetryAttempts++;
    telemetryRetryCount++;
    pendingTelemetrySentAt = millis();
    Serial.print(F("[UART] Retrying telemetry sequence "));
    Serial.println(pendingTelemetrySequence);
  } else {
    telemetryAwaitingAck = false;
    telemetryDeliveryFailures++;
    Serial.print(F("[UART ERROR] ESP did not acknowledge sequence "));
    Serial.println(pendingTelemetrySequence);
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  Serial.print(F("[ESP CMD] ")); Serial.println(cmd);
  if (cmd.startsWith("PACKET_ACK,")) {
    uint32_t seq = (uint32_t)cmd.substring(11).toInt();
    if (telemetryAwaitingAck && seq == pendingTelemetrySequence) {
      telemetryAwaitingAck = false;
      telemetryAckedCount++;
    }
    return;
  }
  bool ok = true;

  if (cmd == "FAN,1") {
    remoteModeOverride = true; remoteManualMode = true; manualMode = true; setFan(true);
  } else if (cmd == "FAN,0") {
    remoteModeOverride = true; remoteManualMode = true; manualMode = true; setFan(false);
  } else if (cmd == "MODE,AUTO") {
    remoteModeOverride = true; remoteManualMode = false; manualMode = false;
  } else if (cmd == "MODE,MANUAL") {
    remoteModeOverride = true; remoteManualMode = true; manualMode = true;
  } else if (cmd == "COUNT,RESET") {
    peopleCount = 0;
  } else if (cmd == "CAL,MQ") {
    airBaseline = analogRead(PIN_MQ); saveCalibration();
    Serial.print(F("[CAL] MQ baseline=")); Serial.println(airBaseline);
  } else if (cmd == "CAL,WATER_DRY") {
    waterDry = analogRead(PIN_WATER); updateWaterLevel(); saveCalibration();
    Serial.print(F("[CAL] water dry=")); Serial.println(waterDry);
  } else if (cmd == "CAL,WATER_WET") {
    waterWet = analogRead(PIN_WATER); updateWaterLevel(); saveCalibration();
    Serial.print(F("[CAL] water wet=")); Serial.println(waterWet);
  } else if (cmd.startsWith("TH,AIR,")) {
    airThreshold = constrain(cmd.substring(7).toInt(), 0, 1023);
  } else if (cmd.startsWith("TH,WATER,")) {
    waterThreshold = constrain(cmd.substring(9).toInt(), 0, 1023);
  } else if (cmd.startsWith("TH,DIST,")) {
    float v = cmd.substring(8).toFloat(); if (v >= 2 && v <= 100) nearCm = v; else ok = false;
  } else if (cmd.startsWith("CAL,FLOW,")) {
    float known = cmd.substring(9).toFloat();
    noInterrupts(); unsigned long p = flowPulses; interrupts();
    if (known > 0 && p > 0) { pulsesPerLitre = (float)p / known; saveCalibration(); }
    else ok = false;
    Serial.print(F("[CAL] pulses/L=")); Serial.println(pulsesPerLitre, 2);
  } else {
    ok = false;
  }

  if (ok) { espSerial.print(F("ACK,")); espSerial.println(cmd); }
  else { espSerial.print(F("NACK,")); espSerial.println(cmd); }
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  loadCalibration();
  pinMode(PIN_TRIG,OUTPUT); pinMode(PIN_ECHO,INPUT);
  pinMode(PIN_RELAY,OUTPUT); digitalWrite(PIN_RELAY,HIGH); // default fan OFF for active-low relay
  pinMode(PIN_SWITCH,INPUT_PULLUP);
  previousSwitchLow = (digitalRead(PIN_SWITCH) == LOW);
  manualMode = previousSwitchLow;
  pinMode(PIN_FLOW,INPUT_PULLUP);
  pinMode(PIN_BUTTON,INPUT_PULLUP);
  pinMode(PIN_LED_GREEN,OUTPUT); pinMode(PIN_LED_YELLOW,OUTPUT);
  digitalWrite(PIN_LED_GREEN,LOW); digitalWrite(PIN_LED_YELLOW,LOW);
  Wire.begin();
  lcd.init(); lcd.backlight();
  startupDiagnostics();
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW),flowISR,FALLING);
  lastSample=millis(); lastLCD=millis();
}

void loop() {
  while (espSerial.available()) {
    char c=(char)espSerial.read();
    if(c=='\n'){ handleCommand(commandBuffer); commandBuffer=""; }
    else if(c!='\r' && commandBuffer.length()<90) commandBuffer+=c;
    else if(commandBuffer.length()>=90) commandBuffer="";
  }
  serviceTelemetryRetry();

  airRaw=analogRead(PIN_MQ);
  waterRaw=analogRead(PIN_WATER);
  updateWaterLevel();
  mqFault=(airRaw<=2 || airRaw>=1021);
  waterFault=(waterRaw<=2 || waterRaw>=1021);
  updateAirStatus();
  waterAvailable=(waterDry>=0&&waterWet>=0)
    ? (abs(waterRaw-waterWet)<abs(waterRaw-waterDry))
    : (waterRaw>=waterThreshold);

  if(millis()-lastDistance>=300) {
    lastDistance=millis();
    float d=readDistance();
    // Smooth over isolated no-echo glitches: retain fault if repeated failures.
    static byte noEchoCount=0;
    if(d<0) {
      if(noEchoCount<10) noEchoCount++;
      if(noEchoCount>=5) { ultrasonicFault=true; distanceCm=-1.0f; targetNear=false; }
    } else { noEchoCount=0; distanceCm=d; ultrasonicFault=false; }
    bool nearNow=distanceCm>0 && distanceCm<=nearCm;
    if(nearNow&&!targetNear) peopleCount++;
    if(distanceCm>=farCm || distanceCm<0) targetNear=false;
    else if(nearNow) targetNear=true;
  }

  // The physical switch has priority when moved. Until it is moved again,
  // a dashboard MODE command remains in force.
  bool switchLowNow = (digitalRead(PIN_SWITCH) == LOW);
  if (switchLowNow != previousSwitchLow) {
    previousSwitchLow = switchLowNow;
    remoteModeOverride = false;
    manualMode = switchLowNow;
  }
  if (remoteModeOverride) manualMode = remoteManualMode;
  else manualMode = switchLowNow;

  bool btnDown=(digitalRead(PIN_BUTTON)==LOW);
  if(btnDown&&!buttonWasDown){ buttonDownMs=millis(); buttonResetDone=false; }
  if(btnDown&&!buttonResetDone&&millis()-buttonDownMs>=1800){ peopleCount=0; buttonResetDone=true; Serial.println(F("[ACTION] People count reset by button")); }
  buttonWasDown=btnDown;

  if(millis()-lastSample>=1000) {
    lastSample=millis();
    noInterrupts(); unsigned long p=flowPulses; interrupts();
    unsigned long delta=p-prevFlowPulses;
    flowLpm=(pulsesPerLitre>0)?(delta*60.0f/pulsesPerLitre):0;
    totalLitres=(pulsesPerLitre>0)?(p/pulsesPerLitre):0;
    prevFlowPulses=p;
    if(!manualMode) {
      int limit=(airBaseline>=0)?airBaseline+100:airThreshold;
      // Fail-safe: switch ventilation ON when the MQ analogue input is invalid.
      setFan(mqFault || airRaw>=limit);
    }
    rtcFault=!rtcReady || rtc.lostPower();
    refreshFaultText();
    digitalWrite(PIN_LED_YELLOW, (mqFault || waterFault || ultrasonicFault || rtcFault || waterLevelPct < 0.0f) ? HIGH : LOW);
    sendTelemetry();
  }
  if(millis()-lastLCD>=1000){ lastLCD=millis(); updateLCD(); }
}
