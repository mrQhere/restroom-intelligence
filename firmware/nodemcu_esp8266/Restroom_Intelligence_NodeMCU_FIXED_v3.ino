/*
  ================================================================
   RESTROOM INTELLIGENCE  -  NodeMCU (ESP8266) gateway  -  iOS-INSPIRED UI REWORK
  ================================================================
  Board   : NodeMCU 1.0 (ESP-12E Module)   | Core: "esp8266" by ESP8266 Community
  Flash   : upload this single file. No dashboard.h, no SPIFFS/LittleFS upload.
  Serial  : 115200 (USB debug)             | UART to ATmega: 9600

  WIRING (unchanged from your ATmega sketch)
    ATmega D7 (TX) -> 1k -> junction -> NodeMCU D1 (RX) ; junction -> 2.2k -> GND
    NodeMCU D2 (TX) -> ATmega D8 (RX)  ; GND common
    MicroSD : CS = D0, SCK = D5, MISO = D6, MOSI = D7, GND, VCC per your module
              (FAT32 card)

  WIFI    : SSID "Restroom-Monitor"  password "restroom123"  ->  http://192.168.4.1
            (access-point mode: no internet needed, so the page uses no CDN files)

  WHAT THIS FIRMWARE DOES
    * Sequential illustrated hardware-verification wizard with live pass/warn/fail results
    * Guided MQ baseline, water dry/wet, SD self-test, flow test, people-counter test and door captures
    * Parses ATmega UART protocol v2 (T,2,... with CRC-8 + sequence), ACKs every frame
    * Serves a responsive 5-tab dashboard (v5: score, predictions, analytics) with sensor details, charts, diagnostics and calibration
    * Logs everything to the MicroSD as CSV. If the card is missing or a write fails,
      lines wait in a RAM queue and are written when the card comes back (retry every 5 s)
    * Entry events are derived from increases in the ATmega PEOPLE counter and stored
    with exact timestamps (not rounded 5-minute buckets).
  * No LCD or fan/relay UI, checks, or commands are included.
  * Flow zero calibration and known-volume calibration are performed on the ESP8266
    and saved in EEPROM; they do not require undocumented ATmega commands.
  * Cleaning estimate (top-right) and ESP-side trend-based maintenance advisories
  * Two-position door calibration (closed/open) with a debounced door-cycle proxy
  * Manual 18-second water-demand test to flag no-flow only when a tap test is active
  * The single ultrasonic sensor cannot infer direction or count multiple people per cycle

  CSV FILES (download from the page, or /download?f=data|entries|events|commands)
    /datalast.csv     every telemetry frame (about 1 per second)
    /entrlast.csv     one row per detected entry event
    /evntlast.csv     boot, SD, link, fault and cleaning events
    /cmdslast.csv     supported commands and ATmega ACK/NACK

  IMPORTANT: the PEOPLE field must be a cumulative counter that increases once per
  entry. If ATmega sends current occupancy instead, ESP-side delta counting cannot
  produce a correct entry total; the ATmega firmware must expose a monotonic counter.
  Hardware tests are still required after compiling.
  ================================================================
*/

/* v9: SD names fixed (8.3: datalast/entrlast/evntlast/cmdslast/bktslast .csv), history restored from SD,
   boot loader + three-step calibration popup + refined iOS-style UI. ATmega protocol unchanged. */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <EEPROM.h>
#include <math.h>

// ---------------- configuration ----------------
#define SD_CS      D0
#define UART_RX    D1
#define UART_TX    D2
static const char*    AP_NAME        = "Restroom-Monitor";
static const char*    AP_PASS        = "restroom123";
static const uint32_t UART_BAUD      = 9600;
static const uint32_t LINK_TIMEOUT   = 5000;    // ms without a valid frame -> link lost
static const uint32_t LOG_INTERVAL   = 1000;    // one telemetry row per second
static const uint32_t SD_RETRY_MS    = 5000;
static const uint32_t CMD_TIMEOUT    = 2500;
static const uint16_t CLEAN_ENTRIES  = 25;      // clean after this many entries ...
static const uint16_t CLEAN_HOURS    = 3;       // ... or this many hours, or poor air
static const uint32_t MIN_VALID_EPOCH = 1704067200UL; // 2024-01-01: earlier = RTC not set
static const uint32_t FLOW_ZERO_WINDOW_MS = 60000UL;
static const uint32_t FLOW_CAL_MAGIC = 0x52494635UL; // "RIF5"
static const uint16_t ENTRY_EVENT_CAP = 256; // RAM ring; full history remains on SD

// ---------------- file table ----------------
// FINAL file names. The ESP8266 SD library only supports 8.3 names (max 8 characters before the dot),
// which is why the old "/entries_v8.csv" style names could never be created. These names are permanent:
// UI changes must never rename them, so the card is always the source for repopulating the dashboard.
enum { K_DATA = 0, K_ENT = 1, K_EVT = 2, K_CMD = 3, K_BKT = 4, K_N = 5 };
static const char* const PATHS[K_N]   = {"/datalast.csv", "/entrlast.csv", "/evntlast.csv", "/cmdslast.csv", "/bktslast.csv"};
static const char* const NAMES[K_N]   = {"data", "entries", "events", "commands", "buckets"};
static const char* const HEADERS[K_N] = {
  "sample_sequence,rtc_timestamp_local,air_quality_raw_adc_0_1023,air_quality_status,people_counter_value_as_reported,water_probe_raw_adc_0_1023,water_probe_wet_boolean_0_or_1,water_level_estimate_percent,ultrasonic_distance_cm,flow_raw_litres_per_minute,flow_calibrated_litres_per_minute,total_raw_litres,total_calibrated_litres,rtc_time_valid_boolean_0_or_1,atmega_fault_bits,atmega_fault_description,door_position_state,door_open_cycle_count_proxy,door_reference_calibrated_boolean_0_or_1,door_closed_reference_cm,door_open_reference_cm",
  "event_timestamp_local,event_epoch_unix_seconds,people_counter_value_after_event,positive_people_delta_assumed_entry_count",
  "esp_uptime_seconds,event_timestamp_local,event_description",
  "esp_uptime_seconds,command_timestamp_local,command_acknowledgement_or_event",
  "bucket_start_epoch_5min,air_avg_adc,entries,fan_on_percent,litres_x100"};

// ---------------- objects ----------------
SoftwareSerial Uno(UART_RX, UART_TX);
ESP8266WebServer server(80);
DNSServer dnsServer;
static const byte DNS_PORT = 53;
static IPAddress AP_IP(192, 168, 4, 1);

// ---------------- live telemetry ----------------
struct Live {
  bool     have;
  uint32_t seq;
  char     ts[24];
  int      air;
  char     airS[10];
  long     people;
  int      wRaw;
  bool     wet;
  float    wLvl, dist, flow, total;
  bool     fan;
  char     mode[8];
  bool     rtc;
  int      bits;
  char     fault[100];
};
static Live L = {false, 0, "--", 0, "WAITING", 0, 0, false, -1, -1, 0, 0, false, "--", false, 0, ""};

static uint32_t lastPacketMs = 0, lastLogMs = 0;
static uint32_t uartBytes = 0, validFrames = 0, badFrames = 0, crcErrors = 0, missedSeq = 0, dupFrames = 0, oooFrames = 0;
static bool     haveSeq = false;
static uint32_t lastSeq = 0;
static char     lastRaw[200] = "";
static uint32_t rawSeq = 0;
static int      lastBits = -1;
static bool     linkWasUp = false;

// ---------------- time ----------------
static bool     timeValid = false;
static uint32_t baseEpoch = 0, baseMs = 0;

// ---------------- SD / log queue ----------------
static bool     sdReady = false, downloading = false, stateRestored = false;
static const char* sdMsg = "not initialised";
static uint32_t lastSdTry = 0, logErrors = 0, qDropped = 0, dataBytes = 0;
#define QN 8   // RAM retry queue (SD holds the full history)
#define QL 300 // smaller RAM queue item; full records remain on SD
struct QItem { uint8_t k; char s[QL]; };
static QItem  q[QN];
static uint8_t qHead = 0, qCount = 0;

// ---------------- entries / cleaning ----------------
struct EntryEvent { uint32_t epoch; uint16_t count; };
static EntryEvent entryEvents[ENTRY_EVENT_CAP];
static uint16_t entryHead = 0, entryCount = 0;
static long     lastPeople = -1;
static uint32_t entriesSince = 0, cleanEpoch = 0;
static long     cleanMin = 0; static const char* cleanWhy = "TIME"; static bool cleanKnown = false;

// ---------------- flow calibration (ESP-side, persistent in EEPROM) ----------------
struct FlowCalStore {
  uint32_t magic;
  float zeroLpm;
  float scale;
  uint8_t flags;       // bit 0 = zero captured, bit 1 = volume calibrated
  uint8_t reserved[3];
  uint32_t checksum;
};
static float flowZero = 0.0f, flowScale = 1.0f;
static bool flowZeroSet = false, flowVolumeSet = false;
static bool flowZeroRunning = false, flowVolumeRunning = false;
static uint32_t flowZeroStartMs = 0, flowZeroSamples = 0;
static double flowZeroSum = 0.0;
static float flowVolumeStartTotal = 0.0f;
static uint32_t flowVolumeStartMs = 0;
static char flowCalMsg[100] = "Not calibrated: set no-flow zero, then test a known volume";

// ---------------- ESP-side door calibration / door-cycle estimator ----------------
// A single distance sensor can classify two calibrated door positions, but it cannot
// prove human direction or distinguish one person from a group. Cycles stay separate
// from the ATmega PEOPLE counter and are labelled as a proxy in the UI.
static const uint32_t DOOR_CAL_MAGIC = 0x444F4F36UL; // "DOO6"
static const int DOOR_EEPROM_ADDR = 48;
struct DoorCalStore {
  uint32_t magic;
  float closedCm;
  float openCm;
  uint32_t cycles;
  uint8_t flags;       // bit 0 = closed captured, bit 1 = open captured
  uint8_t reserved[3];
  uint32_t checksum;
};
static float doorClosedCm = -1.0f, doorOpenCm = -1.0f;
static bool doorClosedSet = false, doorOpenSet = false;
static bool doorCalRunning = false, doorCalTargetOpen = false;
static uint32_t doorCalStartMs = 0, doorCalLastSeq = 0;
static float doorCalValues[32];
static uint8_t doorCalSampleCount = 0;
static char doorCalMsg[120] = "Capture the fully closed and fully open door positions after system check";
static uint8_t doorState = 0, doorCandidate = 0; // 0 unknown, 1 closed, 2 open
static uint32_t doorCandidateSince = 0, doorCycles = 0;
static uint8_t doorCyclesUnsaved = 0;
static bool doorStateInitialised = false;

// ---------------- water-demand test + advisory/prediction engine ----------------
// No-flow is only considered a fault while a user-requested tap test is active.
static bool waterTestRunning = false, waterTestFlowSeen = false, waterTestFailed = false;
static uint32_t waterTestStartMs = 0;
static char waterTestResult[96] = "Not tested";
struct Advisory {
  uint8_t code;       // stable ID for pop-up deduplication
  uint8_t level;      // 1 info, 2 warning, 3 action required
  char title[52];
  char detail[108];
  char action[96];
};
static Advisory advisories[8];
static uint8_t advisoryCount = 0;
static float airTrendPerMin = 0.0f;
static bool airTrendReady = false;
static uint32_t lastAdvisoryMs = 0, lastAdvisorySignature = 0;

// ---------------- v5: adaptive clean rule, 24 h analytics, clean log ----------------
static const int CFG_ADDR = 96;
static const uint32_t CFG_MAGIC = 0x52494337UL; // "RIC7"
struct CfgStore { uint32_t magic; uint16_t ent; uint16_t mins; uint8_t adapt; uint8_t rsv[3]; uint32_t chk; };
static uint16_t cfgEnt = CLEAN_ENTRIES, cfgMin = CLEAN_HOURS * 60;   // stored clean thresholds
static bool     cfgAdapt = true;
static uint16_t effEnt = CLEAN_ENTRIES, effMin = CLEAN_HOURS * 60;   // limits actually in force
static float    airHighFrac = 0.0f;                                  // ~5 min moving share of "air not GOOD"
static bool     airHigh = false;
static float    cleanTotal = 0.0f;                                   // calibrated litres at last clean
#define BN 288                                                       // 24 h of 5-minute buckets
static uint16_t bAir[BN], bLit[BN]; static uint8_t bEnt[BN], bFan[BN];
static uint32_t curBk = 0;                                           // id = epoch/300 of the open bucket
static uint32_t bkAirSum = 0, bkTicks = 0, bkFan = 0, bkEnt = 0; static float bkLit = 0.0f, lastTot = -1.0f;
static bool     bkRestorePending = true;
static uint16_t heat[7][24];                                         // entries by weekday (Sun=0) x hour
struct CleanRec { uint32_t epoch; uint16_t airB, airA, ent; };
static CleanRec cleans[8]; static uint8_t cleanN = 0; static uint32_t cleanAfterMs = 0;

// ---------------- history (5 s steps) ----------------
#define HN 120
static uint16_t hAir[HN], hFlow[HN], hWaterRaw[HN];
static int16_t hWaterLevel[HN];
static uint16_t hHead = 0, hCount = 0;
static bool histRestorePending = true;

static float correctedFlow();

static void event(const char* fmt, ...);

// ---------------- command tracking ----------------
static String   pendingCmd = "", lastAck = "";
static const char* cmdRes = "NONE";
static uint32_t cmdStart = 0, cmdId = 0, ackCount = 0, nackCount = 0;

// ---------------- buffers ----------------
static char jb[4096];   // one shared JSON buffer (handlers never overlap)
#define hb jb

// ================================================================
//  small helpers
// ================================================================
struct JB {
  char* b; size_t cap; size_t n;
  JB(char* buf, size_t c) : b(buf), cap(c), n(0) { if (c) b[0] = 0; }
  void raw(const char* s) { while (*s && n + 1 < cap) b[n++] = *s++; b[n] = 0; }
  void f(const char* fmt, ...) {
    va_list a; va_start(a, fmt);
    int r = vsnprintf(b + n, cap - n, fmt, a);
    va_end(a);
    if (r > 0) n += ((size_t)r < cap - n) ? (size_t)r : cap - n - 1;
  }
  void esc(const char* s) {
    raw("\"");
    for (; *s && n + 8 < cap; s++) {
      unsigned char c = (unsigned char)*s;
      if (c == '"' || c == '\\') { b[n++] = '\\'; b[n++] = c; }
      else if (c < 32 || c > 126) b[n++] = ' ';
      else b[n++] = c;
    }
    b[n] = 0; raw("\"");
  }
};

static uint8_t crc8(const char* s) {
  uint8_t c = 0;
  while (*s) {
    c ^= (uint8_t)*s++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// "YYYY-MM-DD HH:MM:SS" (local RTC time, treated as UTC) -> seconds. Returns false if not plausible.
static bool epochFromStamp(const char* s, uint32_t* out) {
  int y, mo, d, h, mi, se;
  if (sscanf(s, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &se) != 6) return false;
  if (y < 2024 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || se > 59 || h < 0 || mi < 0 || se < 0) return false;
  y -= (mo <= 2);
  long era = y / 400;
  long yoe = y - era * 400;
  long doy = (153L * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468L;
  *out = (uint32_t)(days * 86400L + h * 3600L + mi * 60L + se);
  return true;
}

static uint32_t nowEpoch() {
  if (timeValid) return baseEpoch + (millis() - baseMs) / 1000UL;
  return millis() / 1000UL;                 // relative seconds when the RTC time is not valid
}

static bool linkUp() { return L.have && (millis() - lastPacketMs) < LINK_TIMEOUT; }

static uint32_t flowCalChecksum(const FlowCalStore& v) {
  uint32_t z = (uint32_t)(v.zeroLpm * 100000.0f);
  uint32_t sc = (uint32_t)(v.scale * 1000000.0f);
  return v.magic ^ z ^ (sc << 1) ^ (uint32_t)v.flags ^ 0xA55A3CC3UL;
}

static void loadFlowCalibration() {
  EEPROM.begin(128);
  FlowCalStore v;
  EEPROM.get(0, v);
  if (v.magic == FLOW_CAL_MAGIC && isfinite(v.zeroLpm) && isfinite(v.scale) &&
      v.zeroLpm >= 0.0f && v.zeroLpm <= 5.0f && v.scale >= 0.05f && v.scale <= 20.0f &&
      v.checksum == flowCalChecksum(v)) {
    flowZero = v.zeroLpm;
    flowScale = v.scale;
    flowZeroSet = (v.flags & 1) != 0;
    flowVolumeSet = (v.flags & 2) != 0;
    snprintf(flowCalMsg, sizeof(flowCalMsg), "Saved calibration loaded · zero %.3f L/min · scale %.4f", flowZero, flowScale);
  } else {
    flowZero = 0.0f; flowScale = 1.0f;
    flowZeroSet = false; flowVolumeSet = false;
    snprintf(flowCalMsg, sizeof(flowCalMsg), "Not calibrated · default scale 1.0000");
  }
}

static void saveFlowCalibration() {
  FlowCalStore v;
  memset(&v, 0, sizeof(v));
  v.magic = FLOW_CAL_MAGIC;
  v.zeroLpm = flowZero;
  v.scale = flowScale;
  v.flags = (flowZeroSet ? 1 : 0) | (flowVolumeSet ? 2 : 0);
  v.checksum = flowCalChecksum(v);
  EEPROM.put(0, v);
  EEPROM.commit();
}

static uint32_t doorCalChecksum(const DoorCalStore& v) {
  uint32_t c = (uint32_t)(v.closedCm * 1000.0f);
  uint32_t o = (uint32_t)(v.openCm * 1000.0f);
  return v.magic ^ c ^ (o << 1) ^ (v.cycles * 2654435761UL) ^ (uint32_t)v.flags ^ 0x6D52A19BUL;
}

static void saveDoorCalibration() {
  DoorCalStore v; memset(&v, 0, sizeof(v));
  v.magic = DOOR_CAL_MAGIC;
  // Unset positions use a zero sentinel so the EEPROM checksum never converts -1.0f to uint32_t.
  v.closedCm = doorClosedSet ? doorClosedCm : 0.0f;
  v.openCm = doorOpenSet ? doorOpenCm : 0.0f;
  v.cycles = doorCycles;
  v.flags = (doorClosedSet ? 1 : 0) | (doorOpenSet ? 2 : 0);
  v.checksum = doorCalChecksum(v);
  EEPROM.put(DOOR_EEPROM_ADDR, v); EEPROM.commit();
}

static void loadDoorCalibration() {
  DoorCalStore v; EEPROM.get(DOOR_EEPROM_ADDR, v);
  const bool closedStored = (v.flags & 1) != 0;
  const bool openStored = (v.flags & 2) != 0;
  const bool closedValid = !closedStored || (isfinite(v.closedCm) && v.closedCm >= 2.0f && v.closedCm <= 400.0f);
  const bool openValid = !openStored || (isfinite(v.openCm) && v.openCm >= 2.0f && v.openCm <= 400.0f);
  if (v.magic == DOOR_CAL_MAGIC && closedValid && openValid &&
      v.checksum == doorCalChecksum(v)) {
    doorClosedSet = closedStored; doorOpenSet = openStored;
    doorClosedCm = closedStored ? v.closedCm : -1.0f;
    doorOpenCm = openStored ? v.openCm : -1.0f;
    doorCycles = v.cycles;
    if (doorClosedSet && doorOpenSet && isfinite(doorClosedCm) && isfinite(doorOpenCm) && fabsf(doorOpenCm - doorClosedCm) >= 5.0f)
      snprintf(doorCalMsg, sizeof(doorCalMsg), "Saved door calibration loaded · closed %.1f cm · open %.1f cm", doorClosedCm, doorOpenCm);
    else if (doorClosedSet && doorOpenSet)
      snprintf(doorCalMsg, sizeof(doorCalMsg), "Saved positions too close (%.1f cm apart) · recapture one or both", fabsf(doorOpenCm - doorClosedCm));
    else if (doorClosedSet)
      snprintf(doorCalMsg, sizeof(doorCalMsg), "Saved CLOSED reference %.1f cm · now capture OPEN", doorClosedCm);
    else if (doorOpenSet)
      snprintf(doorCalMsg, sizeof(doorCalMsg), "Saved OPEN reference %.1f cm · now capture CLOSED", doorOpenCm);
    else
      snprintf(doorCalMsg, sizeof(doorCalMsg), "Not calibrated · capture closed and open positions");
  } else {
    doorClosedCm = doorOpenCm = -1.0f; doorClosedSet = doorOpenSet = false; doorCycles = 0;
    snprintf(doorCalMsg, sizeof(doorCalMsg), "Door calibration data invalid · capture both positions again");
  }
}

static bool doorCalReady() {
  return doorClosedSet && doorOpenSet && isfinite(doorClosedCm) && isfinite(doorOpenCm) && fabsf(doorOpenCm - doorClosedCm) >= 5.0f;
}

static void collectDoorCalibrationSample() {
  if (!doorCalRunning || L.seq == doorCalLastSeq) return;
  doorCalLastSeq = L.seq; // each accepted ATmega frame can contribute at most once
  if (!isfinite(L.dist) || L.dist < 2.0f || L.dist > 400.0f) return;
  if (doorCalSampleCount < (sizeof(doorCalValues) / sizeof(doorCalValues[0])))
    doorCalValues[doorCalSampleCount++] = L.dist;
}

static void serviceDoorCalibration() {
  // Ten seconds gives slower UART telemetry enough opportunity to provide five fresh frames.
  if (!doorCalRunning || millis() - doorCalStartMs < 10000UL) return;
  doorCalRunning = false;
  if (doorCalSampleCount < 5) {
    snprintf(doorCalMsg, sizeof(doorCalMsg), "Capture failed · received %u fresh valid readings; need 5 · check HC-SR04 and UART updates", (unsigned)doorCalSampleCount);
    event("DOOR_CAL_FAIL samples=%u", (unsigned)doorCalSampleCount);
    return;
  }
  // Sort samples and use the median so isolated HC-SR04 spikes cannot skew the reference.
  float sorted[32];
  memcpy(sorted, doorCalValues, doorCalSampleCount * sizeof(float));
  for (uint8_t i = 1; i < doorCalSampleCount; i++) {
    float key = sorted[i]; int j = (int)i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  // Reject a capture if the middle half of readings varies too much: the door may be moving.
  float spread = sorted[(doorCalSampleCount * 3) / 4] - sorted[doorCalSampleCount / 4];
  if (spread > 8.0f) {
    snprintf(doorCalMsg, sizeof(doorCalMsg), "Capture unstable · readings varied %.1f cm; hold the door still and retry", spread);
    event("DOOR_CAL_UNSTABLE spread=%.2f samples=%u", spread, (unsigned)doorCalSampleCount);
    return;
  }
  float measured = sorted[doorCalSampleCount / 2];
  if (doorCalTargetOpen) { doorOpenCm = measured; doorOpenSet = true; }
  else { doorClosedCm = measured; doorClosedSet = true; }
  if (doorCalReady()) {
    snprintf(doorCalMsg, sizeof(doorCalMsg), "Calibration ready · closed %.1f cm · open %.1f cm · separation %.1f cm", doorClosedCm, doorOpenCm, fabsf(doorOpenCm - doorClosedCm));
    doorStateInitialised = false; doorState = doorCandidate = 0;
    saveDoorCalibration(); event("DOOR_CAL_READY closed=%.2f open=%.2f", doorClosedCm, doorOpenCm);
  } else if (doorClosedSet && doorOpenSet) {
    saveDoorCalibration();
    snprintf(doorCalMsg, sizeof(doorCalMsg), "Positions too close (%.1f cm apart) · repeat with door fully open and fully closed", fabsf(doorOpenCm - doorClosedCm));
  } else {
    snprintf(doorCalMsg, sizeof(doorCalMsg), "%s position saved at %.1f cm · capture the other position", doorCalTargetOpen ? "OPEN" : "CLOSED", measured);
    saveDoorCalibration(); event("DOOR_CAL_CAPTURE pos=%s cm=%.2f", doorCalTargetOpen ? "OPEN" : "CLOSED", measured);
  }
}

static void updateDoorState() {
  // Starting position is CLOSED (the door is mostly closed); this state is never "unknown" once data flows.
  if (!doorStateInitialised) { doorState = 1; doorCandidate = 1; doorStateInitialised = true; event("DOOR_INITIAL_STATE CLOSED (default start)"); }
  if (!doorCalReady()) return;
  // Position is classified relative to the two saved distance references; mounting direction may vary.
  bool noEcho = !isfinite(L.dist) || L.dist < 0.0f || L.dist > 400.0f;   // no echo means beyond the measurable range
  uint8_t candidate = 0;
  if (noEcho) candidate = (doorClosedCm > doorOpenCm) ? 1 : 2; // no echo means beyond the far reference
  else if (L.dist >= 2.0f) {
    float sep = doorOpenCm - doorClosedCm;
    float t = (L.dist - doorClosedCm) / sep;
    float hh = 3.0f / fabsf(sep); if (hh < 0.12f) hh = 0.12f; if (hh > 0.30f) hh = 0.30f;
    if (t >= 0.5f + hh) candidate = 2; else if (t <= 0.5f - hh) candidate = 1;
  }
  if (!candidate) return; // between the two references: keep last stable state
  if (candidate == doorState) { doorCandidate = candidate; return; }
  if (candidate != doorCandidate) { doorCandidate = candidate; doorCandidateSince = millis(); return; }
  if (millis() - doorCandidateSince < 600UL) return;
  uint8_t oldState = doorState; doorState = candidate; doorCandidate = candidate;
  if (oldState == 1 && doorState == 2) {
    doorCycles++; doorCyclesUnsaved++;
    if (doorCyclesUnsaved >= 16) { saveDoorCalibration(); doorCyclesUnsaved = 0; }
    event("DOOR_OPEN_CYCLE proxy=%lu; not a verified person entry", (unsigned long)doorCycles);
  } else if (oldState == 2 && doorState == 1) {
    event("DOOR_CLOSED");
  }
}

static float calculateAirTrendPerMin() {
  if (hCount < 12) { airTrendReady = false; return 0.0f; }
  uint16_t k = hCount / 4; if (k < 3) k = 3;
  double first = 0.0, last = 0.0;
  for (uint16_t i = 0; i < k; i++) {
    uint16_t a = (hHead + HN - hCount + i) % HN;
    uint16_t b = (hHead + HN - k + i) % HN;
    first += hAir[a]; last += hAir[b];
  }
  double mins = ((double)(hCount - k) * 5.0) / 60.0;
  if (mins <= 0.0) { airTrendReady = false; return 0.0f; }
  airTrendReady = true;
  airTrendPerMin = (float)(((last / k) - (first / k)) / mins);
  return airTrendPerMin;
}

static void addAdvisory(uint8_t code, uint8_t level, const char* title, const char* detail, const char* action) {
  if (advisoryCount >= 8) return;
  Advisory& a = advisories[advisoryCount++]; a.code = code; a.level = level;
  snprintf(a.title, sizeof(a.title), "%s", title);
  snprintf(a.detail, sizeof(a.detail), "%s", detail);
  snprintf(a.action, sizeof(a.action), "%s", action);
}

static void serviceAdvisories() {
  if (millis() - lastAdvisoryMs < 2000UL) return;
  lastAdvisoryMs = millis(); advisoryCount = 0;
  bool up = linkUp();
  if (!up) {
    addAdvisory(1, 3, "ATmega data link offline", "Live sensor values are stale or unavailable.", "Check UART wiring, common ground and ATmega power.");
  } else {
    if (L.bits & 1) addAdvisory(2, 3, "Air sensor fault", "The ATmega reports an MQ sensor / ADC fault.", "Check sensor power, analog wiring and warm-up.");
    else if (!strcmp(L.airS, "BAD") || !strcmp(L.airS, "POOR"))
      addAdvisory(3, 3, "Air quality needs attention", "The air-quality category is POOR/BAD; the raw value is a relative index, not ppm.", "Inspect ventilation, clean the area and verify the sensor.");
    else if (calculateAirTrendPerMin() > 8.0f)
      addAdvisory(4, 2, "Air index rising", "Recent raw MQ trend is rising quickly; this is a trend estimate, not a gas concentration forecast.", "Check ventilation and inspect possible odour / cleaning sources.");

    if (L.bits & 2) addAdvisory(5, 3, "Water probe fault", "The ATmega reports a water-probe fault; tank level cannot be trusted.", "Inspect probe wiring, power and sensor placement.");
    else if (!L.wet || (L.wLvl >= 0.0f && L.wLvl <= 8.0f))
      addAdvisory(6, 3, "Tank water low or absent", "The water probe is dry or the reported level is at/below 8%.", "Refill the tank; then verify that the probe becomes wet.");

    if (waterTestRunning && correctedFlow() > 0.05f) waterTestFlowSeen = true;
    if (waterTestRunning && millis() - waterTestStartMs >= 18000UL) {
      waterTestRunning = false;
      if (waterTestFlowSeen) { waterTestFailed = false; snprintf(waterTestResult, sizeof(waterTestResult), "PASS · flow detected during the 18 s test"); }
      else { waterTestFailed = true; snprintf(waterTestResult, sizeof(waterTestResult), "FAIL · no flow detected; check supply, tank, valve, blockage or sensor"); }
      event("WATER_FLOW_TEST %s", waterTestFlowSeen ? "PASS" : "FAIL_NO_FLOW");
    }
    if (waterTestRunning && millis() - waterTestStartMs >= 7000UL && !waterTestFlowSeen && L.wet && !(L.bits & 2))
      addAdvisory(7, 3, "No water flow detected", "The manual test is active, the probe reports water present, but flow has not appeared.", "Open the test tap. If flow remains zero, inspect valve, filter, pipe blockage and flow sensor.");
    if (waterTestFailed && L.wet && !(L.bits & 2))
      addAdvisory(11, 3, "Possible blockage or no-flow fault", "No measurable flow was detected during the completed manual test while the tank probe reports water.", "Check water supply, tank outlet, valve, filter, pipe blockage and flow sensor wiring.");

    if (!waterTestRunning && hCount >= 60) {
      uint16_t flowing = 0;
      for (uint16_t i = 0; i < hCount; i++) { uint16_t ix = (hHead + HN - hCount + i) % HN; if (hFlow[ix] >= 10) flowing++; }
      if (flowing >= (uint16_t)(hCount * 0.92f))
        addAdvisory(8, 2, "Continuous water flow pattern", "Flow has remained above the 0.10 L/min reference for most of the recent history.", "Check whether a tap was left open or a leak is present.");
    }
    if (cleanKnown && cleanMin <= 20)
      addAdvisory(9, cleanMin <= 0 ? 3 : 2, "Cleaning forecast", cleanMin <= 0 ? "The current cleaning rule predicts cleaning is due." : "The current rule predicts cleaning within about 20 minutes.", "Inspect the restroom and record a clean after maintenance.");
    if (!doorCalReady()) addAdvisory(10, 2, "Door calibration incomplete", "Open/closed distance references are not both valid.", "After system check, capture the door fully CLOSED and fully OPEN.");
  }
  uint32_t signature = 0;
  for (uint8_t i = 0; i < advisoryCount; i++) signature = (signature * 33UL) ^ ((uint32_t)advisories[i].code * 7UL + advisories[i].level);
  if (signature != lastAdvisorySignature) {
    if (lastAdvisorySignature != 0 || signature != 0) event("ADVISORY_STATE count=%u signature=%lu", (unsigned)advisoryCount, (unsigned long)signature);
    lastAdvisorySignature = signature;
  }
}

static float correctedFlow() {
  float v = (L.flow - flowZero) * flowScale;
  if (!isfinite(v) || v < 0.05f) return 0.0f;
  return v;
}

static float correctedTotal() {
  float v = L.total * flowScale;
  return (!isfinite(v) || v < 0.0f) ? 0.0f : v;
}

static void collectFlowZeroSample() {
  if (!flowZeroRunning) return;
  if (isfinite(L.flow) && L.flow >= 0.0f && L.flow <= 1000.0f) {
    flowZeroSum += L.flow;
    flowZeroSamples++;
  }
}

static void serviceFlowCalibration() {
  if (flowZeroRunning && millis() - flowZeroStartMs >= FLOW_ZERO_WINDOW_MS) {
    flowZeroRunning = false;
    if (flowZeroSamples >= 5) {
      flowZero = (float)(flowZeroSum / flowZeroSamples);
      flowZeroSet = true;
      saveFlowCalibration();
      snprintf(flowCalMsg, sizeof(flowCalMsg), "No-flow zero saved · %.3f L/min raw baseline", flowZero);
      event("FLOW_ZERO_CAL zero=%.4f samples=%lu", flowZero, (unsigned long)flowZeroSamples);
    } else {
      snprintf(flowCalMsg, sizeof(flowCalMsg), "Zero calibration failed · not enough valid samples");
    }
  }
  if (flowVolumeRunning && millis() - flowVolumeStartMs > 180000UL) {
    flowVolumeRunning = false;
    snprintf(flowCalMsg, sizeof(flowCalMsg), "Volume test cancelled · timed out after 3 minutes");
  }
}

// ================================================================
//  SD card: init, write, queue
// ================================================================
static bool sdWrite(uint8_t k, const char* buf, size_t len) {
  File f = SD.open(PATHS[k], FILE_WRITE);
  if (!f) { sdReady = false; sdMsg = "write failed: cannot open file"; logErrors++; return false; }
  f.seek(f.size());                                   // always append, even if the core omits O_APPEND
  size_t w = f.write((const uint8_t*)buf, len);
  f.close();
  if (w != len) { sdReady = false; sdMsg = "write failed: short write"; logErrors++; return false; }
  if (k == K_DATA) dataBytes += len;
  return true;
}

static void enqueue(uint8_t k, const char* line) {
  if (qCount >= QN) { qHead = (qHead + 1) % QN; qCount--; qDropped++; }   // drop oldest, count it
  QItem& it = q[(qHead + qCount) % QN];
  it.k = k;
  strncpy(it.s, line, QL - 2); it.s[QL - 2] = 0;
  qCount++;
}

static void logLine(uint8_t k, const char* line) {
  char buf[QL];
  size_t n = strlen(line); if (n > QL - 2) n = QL - 2;
  memcpy(buf, line, n); buf[n++] = '\n'; buf[n] = 0;
  if (sdReady && !downloading && qCount == 0) {
    if (sdWrite(k, buf, n)) return;
  }
  enqueue(k, line);
}

static void drainQueue() {
  uint8_t guard = 4;
  while (qCount && sdReady && !downloading && guard--) {
    QItem& it = q[qHead];
    char buf[QL]; size_t n = strlen(it.s); memcpy(buf, it.s, n); buf[n++] = '\n'; buf[n] = 0;
    if (!sdWrite(it.k, buf, n)) return;            // keep it queued, retry later
    qHead = (qHead + 1) % QN; qCount--;
    yield();
  }
}

static void event(const char* fmt, ...) {
  char m[110]; va_list a; va_start(a, fmt); vsnprintf(m, sizeof(m), fmt, a); va_end(a);
  for (char* p = m; *p; p++) if (*p == ',' || *p == '\n' || *p == '\r') *p = ';';
  char line[QL];
  snprintf(line, sizeof(line), "%lu,%s,%s", (unsigned long)(millis() / 1000UL), L.have ? L.ts : "--", m);
  logLine(K_EVT, line);
  Serial.print(F("[EVT] ")); Serial.println(m);
}

static void commandLog(const char* what, const String& c) {
  char line[QL];
  snprintf(line, sizeof(line), "%lu,%s,\"%s %s\"", (unsigned long)(millis() / 1000UL), L.have ? L.ts : "--", what, c.c_str());
  for (char* p = line; *p; p++) if (*p == '\n' || *p == '\r') *p = ' ';
  logLine(K_CMD, line);
}

static void savePersist() {
  if (!sdReady || downloading) return;
  SD.remove("/cleanlst.txt");
  File f = SD.open("/cleanlst.txt", FILE_WRITE);
  if (f) { f.println((unsigned long)cleanEpoch); f.close(); }
}

static void rememberEntry(uint32_t epoch, uint16_t n) {
  if (!n) return;
  uint16_t idx;
  if (entryCount < ENTRY_EVENT_CAP) {
    idx = (entryHead + entryCount) % ENTRY_EVENT_CAP;
    entryCount++;
  } else {
    idx = entryHead;
    entryHead = (entryHead + 1) % ENTRY_EVENT_CAP;
  }
  entryEvents[idx].epoch = epoch;
  entryEvents[idx].count = n;
  if (epoch >= MIN_VALID_EPOCH) { uint32_t dd = epoch / 86400UL; uint16_t& hc = heat[(dd + 4) % 7][(epoch % 86400UL) / 3600UL]; hc = (uint16_t)((uint32_t)hc + n > 65535UL ? 65535UL : hc + n); }
  if (cleanEpoch && epoch >= cleanEpoch) entriesSince += n;
}

static uint32_t countEntries(uint32_t fromEpoch, uint32_t toEpoch) {
  if (toEpoch < fromEpoch) return 0;
  uint64_t total = 0;
  for (uint16_t i = 0; i < entryCount; i++) {
    uint16_t idx = (entryHead + i) % ENTRY_EVENT_CAP;
    const EntryEvent& e = entryEvents[idx];
    if (e.epoch >= fromEpoch && e.epoch <= toEpoch) total += e.count;
  }
  return total > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (uint32_t)total;
}

static bool parseEntryCsvLine(const String& s, uint32_t* epoch, uint16_t* added) {
  int c1 = s.indexOf(','); if (c1 < 0) return false;
  int c2 = s.indexOf(',', c1 + 1); if (c2 < 0) return false;
  int c3 = s.indexOf(',', c2 + 1); if (c3 < 0) return false;
  uint32_t e = (uint32_t)strtoul(s.c_str() + c1 + 1, NULL, 10);
  long n = s.substring(c3 + 1).toInt();
  if (e < MIN_VALID_EPOCH || n < 1 || n > 65535) return false;
  *epoch = e; *added = (uint16_t)n; return true;
}

static void loadEntryRows(File& f, bool copyToCurrent) {
  while (f.available()) {
    String s = f.readStringUntil('\n');
    if (!s.length()) continue;
    uint32_t e; uint16_t n;
    if (!parseEntryCsvLine(s, &e, &n)) continue;
    rememberEntry(e, n);
    if (copyToCurrent) {
      File dst = SD.open(PATHS[K_ENT], FILE_WRITE);
      if (dst) { dst.println(s); dst.close(); }
    }
    yield();
  }
}

static void pushClean(uint32_t e, uint16_t airB, uint16_t ent) {
  if (cleanN == 8) { memmove(&cleans[0], &cleans[1], 7 * sizeof(CleanRec)); cleanN = 7; }
  CleanRec& c = cleans[cleanN++]; c.epoch = e; c.airB = airB; c.airA = 0; c.ent = ent;
}

static void restoreCleanRow(const String& row) {
  int c1 = row.indexOf(','); int c2 = c1 < 0 ? -1 : row.indexOf(',', c1 + 1);
  if (c2 < 0) return;
  String ts = row.substring(c1 + 1, c2); const char* d = row.c_str() + c2 + 1; uint32_t e;
  const char* q;
  if (!strncmp(d, "CLEANED", 7)) {
    if (!epochFromStamp(ts.c_str(), &e)) return;
    int a = 0, n = 0; q = strstr(d, "air="); if (q) a = atoi(q + 4); q = strstr(d, " n="); if (q) n = atoi(q + 3);
    pushClean(e, (uint16_t)a, (uint16_t)n);
  } else if (!strncmp(d, "CLEAN_AFTER", 11)) {
    q = strstr(d, "air="); if (q && cleanN && !cleans[cleanN - 1].airA) cleans[cleanN - 1].airA = (uint16_t)atoi(q + 4);
  }
}

static void restoreState() {
  stateRestored = true;
  File c = SD.open("/cleanlst.txt", FILE_READ);
  if (c) { String s = c.readStringUntil('\n'); c.close(); cleanEpoch = (uint32_t)strtoul(s.c_str(), NULL, 10); }

  File f = SD.open(PATHS[K_ENT], FILE_READ);
  bool hasNewRows = false;
  if (f) {
    size_t sz = f.size();
    hasNewRows = sz > strlen(HEADERS[K_ENT]) + 3;
    // Entry logs are small relative to telemetry; read the complete file so
    // entries-since-clean can be rebuilt accurately. The ring keeps only newest N.
    f.readStringUntil('\n');
    loadEntryRows(f, false);
    f.close();
  }

  // Rebuild door-open-cycle proxy count from the append-only event log; EEPROM is a checkpoint.
  uint32_t cyclesFromEvents = 0;
  File ev = SD.open(PATHS[K_EVT], FILE_READ);
  if (ev) {
    while (ev.available()) {
      String row = ev.readStringUntil('\n');
      if (row.indexOf("DOOR_OPEN_CYCLE") >= 0) cyclesFromEvents++;
      else if (row.indexOf(",CLEAN") > 0) restoreCleanRow(row);
      yield();
    }
    ev.close();
  }
  if (cyclesFromEvents > doorCycles) { doorCycles = cyclesFromEvents; saveDoorCalibration(); }
  Serial.print(F("[SD] restored exact entry events: ")); Serial.println(entryCount);
  Serial.print(F("[SD] restored door-cycle proxy: ")); Serial.println((unsigned long)doorCycles);
}

static bool ensureHeader(uint8_t k) {
  if (!SD.exists(PATHS[k])) {
    File f = SD.open(PATHS[k], FILE_WRITE);
    if (!f) return false;
    f.println(HEADERS[k]); f.close();
    if (!SD.exists(PATHS[k])) return false;
  }
  return true;
}

static bool sdInit() {
  sdReady = false;
  bool mounted = SD.begin(SD_CS);
  if (!mounted) { delay(50); mounted = SD.begin(SD_CS, 1000000UL); }
  if (!mounted) { sdMsg = "card init failed at 1 MHz · check FAT32, CS D0, SCK D5, MISO D6, MOSI D7, GND and module supply"; return false; }
  SD.remove("/sdcheck.tmp");
  File w = SD.open("/sdcheck.tmp", FILE_WRITE);
  if (!w) { sdMsg = "write test failed"; return false; }
  w.println("RESTROOM_SD_OK"); w.close();
  File r = SD.open("/sdcheck.tmp", FILE_READ);
  if (!r) { sdMsg = "read test failed"; return false; }
  String chk = r.readStringUntil('\n'); r.close();
  bool removed = SD.remove("/sdcheck.tmp");
  if (chk.indexOf("RESTROOM_SD_OK") < 0) { sdMsg = "read-back mismatch"; return false; }
  if (!removed && SD.exists("/sdcheck.tmp")) { sdMsg = "temporary test-file cleanup failed"; return false; }
  for (uint8_t k = 0; k < K_N; k++) if (!ensureHeader(k)) { sdMsg = "cannot create CSV files"; return false; }
  File d = SD.open(PATHS[K_DATA], FILE_READ);
  if (d) { dataBytes = d.size(); d.close(); }
  sdReady = true; sdMsg = "OK - init/write/read/delete passed";
  Serial.println(F("[SD] mounted; files: datalast / entrlast / evntlast / cmdslast / bktslast"));
  if (!stateRestored) restoreState();
  return true;
}

// ================================================================
//  telemetry parsing
// ================================================================
static void onPeople(long p) {
  if (lastPeople < 0) { lastPeople = p; return; }
  if (p > lastPeople) {
    uint32_t add = (uint32_t)(p - lastPeople);
    if (add > 65535UL) add = 65535UL;
    uint32_t e = nowEpoch();
    rememberEntry(e, (uint16_t)add);
    bkEnt += add;
    char line[100];
    snprintf(line, sizeof(line), "%s,%lu,%ld,%lu", L.ts, (unsigned long)e, p, (unsigned long)add);
    logLine(K_ENT, line);
  } else if (p < lastPeople) {
    // A reset/rebase is not an entry. Count again only after the new baseline rises.
    event("PEOPLE_REBASE old=%ld new=%ld (decrease not counted)", lastPeople, p);
  }
  lastPeople = p;
}

static void buildDataCsv(char* out, size_t cap) {
  int n = snprintf(out, cap,
    "%lu,%s,%d,%s,%ld,%d,%d,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%d,%d,",
    (unsigned long)L.seq, L.ts, L.air, L.airS, L.people, L.wRaw, L.wet ? 1 : 0,
    L.wLvl, L.dist, L.flow, correctedFlow(), L.total, correctedTotal(), L.rtc ? 1 : 0, L.bits);
  if (n < 0 || (size_t)n >= cap - 4) { if (cap) out[0] = 0; return; }
  size_t pos = (size_t)n;
  out[pos++] = '"';
  for (const char* p = L.fault; *p && pos + 3 < cap; ++p) {
    char c = *p;
    if (c == '"') out[pos++] = '"';
    if (c == '\r' || c == '\n') c = ' ';
    out[pos++] = c;
  }
  out[pos++] = '"';
  int extra = snprintf(out + pos, cap - pos, ",%s,%lu,%d,%.1f,%.1f",
    doorState == 1 ? "CLOSED" : doorState == 2 ? "OPEN" : "UNKNOWN",
    (unsigned long)doorCycles, doorCalReady() ? 1 : 0, doorClosedCm, doorOpenCm);
  if (extra > 0) pos += ((size_t)extra < cap - pos) ? (size_t)extra : cap - pos - 1;
  out[pos] = 0;
}

static bool airOk(const char* s) {
  return !strcmp(s, "GOOD") || !strcmp(s, "MODERATE") || !strcmp(s, "POOR") || !strcmp(s, "BAD") || !strcmp(s, "UNKNOWN") || !strcmp(s, "ERROR");
}

static void sendPacketAck(uint32_t seq) { Uno.print(F("PACKET_ACK,")); Uno.println((unsigned long)seq); }

static bool onTelemetry(char* line) {
  if (strncmp(line, "T,2,", 4) != 0) return false;
  char* lastComma = strrchr(line, ',');
  if (!lastComma || lastComma < line + 6) return false;
  const char* crcTxt = lastComma + 1; size_t cl = strlen(crcTxt);
  if (cl < 1 || cl > 3) { crcErrors++; return false; }
  for (size_t i = 0; i < cl; i++) if (crcTxt[i] < '0' || crcTxt[i] > '9') { crcErrors++; return false; }
  int rx = atoi(crcTxt);
  *lastComma = 0;
  char* payload = line + 2;
  if (rx > 255 || (uint8_t)rx != crc8(payload)) { crcErrors++; return false; }

  char* f[17]; char* p = payload;
  for (int i = 0; i < 17; i++) {
    f[i] = p;
    char* c = strchr(p, ',');
    if (i < 16) { if (!c) return false; *c = 0; p = c + 1; }
    else if (c) return false;
  }
  if (strcmp(f[0], "2") != 0) return false;
  uint32_t seq = strtoul(f[1], NULL, 10); if (seq == 0) return false;
  if (strlen(f[2]) < 19) return false;
  if (!airOk(f[4])) return false;
  int air = atoi(f[3]), wraw = atoi(f[6]); long ppl = atol(f[5]);
  if (air < 0 || air > 1023 || wraw < 0 || wraw > 1023 || ppl < 0 || ppl > 1000000L) return false;

  if (haveSeq) {
    uint32_t delta = seq - lastSeq;
    if (delta == 0) { dupFrames++; sendPacketAck(seq); return true; }
    if (delta >= 0x80000000UL) {
      // Serial frames are ordered. A large backwards jump to a low sequence
      // is an ATmega restart, even if the ESP missed its first few frames.
      if (seq < 2000UL && lastSeq > seq && (lastSeq - seq) > 32UL) {
        lastSeq = seq; event("ATMEGA_REBOOT_DETECTED"); lastPeople = -1;
      } else { oooFrames++; return false; }
    } else { if (delta > 1) missedSeq += delta - 1; lastSeq = seq; }
  } else { haveSeq = true; lastSeq = seq; }

  L.have = true; L.seq = seq;
  strncpy(L.ts, f[2], sizeof(L.ts) - 1); L.ts[sizeof(L.ts) - 1] = 0;
  L.air = air; strncpy(L.airS, f[4], sizeof(L.airS) - 1); L.airS[sizeof(L.airS) - 1] = 0;
  L.people = ppl; L.wRaw = wraw; L.wet = atoi(f[7]) != 0;
  L.wLvl = atof(f[8]); L.dist = atof(f[9]); L.flow = atof(f[10]); L.total = atof(f[11]);
  L.fan = atoi(f[12]) != 0;
  if (!strcmp(f[13], "AUTO") || !strcmp(f[13], "MANUAL")) strcpy(L.mode, f[13]); else strcpy(L.mode, "--");
  L.rtc = atoi(f[14]) != 0; L.bits = atoi(f[15]);
  if (!isfinite(L.flow) || !isfinite(L.total) || L.flow < 0.0f || L.flow > 1000.0f || L.total < 0.0f || L.total > 100000000.0f) return false;
  strncpy(L.fault, f[16], sizeof(L.fault) - 1); L.fault[sizeof(L.fault) - 1] = 0;
  lastPacketMs = millis(); validFrames++;

  uint32_t e;
  if (epochFromStamp(L.ts, &e)) { baseEpoch = e; baseMs = millis(); timeValid = true; } else timeValid = false;

  onPeople(ppl);
  collectFlowZeroSample();
  collectDoorCalibrationSample();
  updateDoorState();
  if (L.bits != lastBits) { event("FAULT_BITS=%d %s", L.bits, L.fault[0] ? L.fault : "none"); lastBits = L.bits; }
  if (millis() - lastLogMs >= LOG_INTERVAL) {
    lastLogMs = millis();
    char csv[QL]; buildDataCsv(csv, sizeof(csv));
    if (csv[0]) logLine(K_DATA, csv);
  }
  sendPacketAck(seq);
  return true;
}

static void handleLine(char* line) {
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ' || line[n - 1] == '\r')) line[--n] = 0;
  if (!n) return;
  strncpy(lastRaw, line, sizeof(lastRaw) - 1); lastRaw[sizeof(lastRaw) - 1] = 0; rawSeq++;
  if (line[0] == 'T' && line[1] == ',') {
    if (!onTelemetry(line)) { badFrames++; }
  } else if (!strncmp(line, "ACK,", 4)) {
    String completed = pendingCmd;
    lastAck = String(line + 4); cmdRes = "ACKNOWLEDGED"; pendingCmd = ""; ackCount++; commandLog("ACK", lastAck);
    if (completed == "COUNT,RESET") lastPeople = -1;
  } else if (!strncmp(line, "NACK,", 5)) {
    lastAck = String(line + 5); cmdRes = "REJECTED"; pendingCmd = ""; nackCount++; commandLog("NACK", lastAck);
  } else badFrames++;
}

static void pollUart() {
  static char rx[300]; static size_t n = 0; static bool over = false;
  while (Uno.available()) {
    int c = Uno.read(); if (c < 0) break; uartBytes++;
    if (c == '\n') { rx[n] = 0; if (n && !over) handleLine(rx); else if (over) badFrames++; n = 0; over = false; }
    else if (c != '\r') { if (n < sizeof(rx) - 1) rx[n++] = (char)c; else over = true; }
  }
}

// ================================================================
//  periodic services
// ================================================================
static void serviceLink() {
  static uint32_t t = 0; if (millis() - t < 500) return; t = millis();
  bool up = linkUp();
  if (up != linkWasUp) { linkWasUp = up; event(up ? "ATMEGA_ONLINE" : "ATMEGA_OFFLINE"); }
  if (pendingCmd.length() && millis() - cmdStart > CMD_TIMEOUT) {
    cmdRes = "ACK_TIMEOUT"; commandLog("ACK_TIMEOUT", pendingCmd); pendingCmd = "";
  }
}

static void serviceSd() {
  if (!sdReady) {
    if (!downloading && millis() - lastSdTry >= SD_RETRY_MS) {
      lastSdTry = millis();
      if (sdInit()) event("SD_READY queued=%u dropped=%lu", (unsigned)qCount, (unsigned long)qDropped);
    }
    return;
  }
  drainQueue();
}

static void pushHist(int air, float flowCal, int wRaw, float wLvl) {
  float fl = flowCal * 100.0f; if (fl < 0) fl = 0; if (fl > 65000) fl = 65000;
  hAir[hHead] = (uint16_t)air; hFlow[hHead] = (uint16_t)fl; hWaterRaw[hHead] = (uint16_t)wRaw;
  hWaterLevel[hHead] = (wLvl >= 0.0f && wLvl <= 100.0f) ? (int16_t)(wLvl + 0.5f) : -1;
  hHead = (hHead + 1) % HN; if (hCount < HN) hCount++;
}

// Repopulate the short-term charts from the tail of datalast.csv (rows newer than the 10-minute window).
static void restoreHistory() {
  File f = SD.open(PATHS[K_DATA], FILE_READ); if (!f) return;
  size_t sz = f.size(); if (sz > 70000UL) { f.seek(sz - 70000UL); f.readStringUntil('\n'); }
  uint32_t now = nowEpoch(), lastTaken = 0; uint16_t taken = 0, rows = 0; char buf[330];
  while (f.available()) {
    int n = f.readBytesUntil('\n', buf, sizeof(buf) - 1); if (n <= 0) continue; buf[n] = 0;
    char* fld[11]; char* p = buf; bool ok = true;
    for (int i = 0; i < 11; i++) { fld[i] = p; char* c = strchr(p, ','); if (!c) { ok = (i == 10); break; } *c = 0; p = c + 1; }
    if (!ok) continue;
    uint32_t e; if (!epochFromStamp(fld[1], &e) || e + 600UL < now || e > now) continue;
    if (lastTaken && e < lastTaken + 5) continue;
    lastTaken = e; pushHist(atoi(fld[2]), atof(fld[10]), atoi(fld[5]), atof(fld[7])); taken++;
    if (!(++rows & 31)) { pollUart(); yield(); }
  }
  f.close();
  Serial.print(F("[SD] restored chart history points: ")); Serial.println(taken);
}

static void serviceHistory() {
  static uint32_t t = 0; if (millis() - t < 5000) return; t = millis();
  if (!linkUp()) return;
  if (histRestorePending && sdReady && !downloading && timeValid) { histRestorePending = false; restoreHistory(); }
  pushHist(L.air, correctedFlow(), L.wRaw, L.wLvl);
}

static uint32_t cfgChk(const CfgStore& v) { return v.magic ^ v.ent ^ ((uint32_t)v.mins << 16) ^ v.adapt ^ 0x3C5AA5C3UL; }
static void loadCfg() {
  CfgStore v; EEPROM.get(CFG_ADDR, v);
  if (v.magic == CFG_MAGIC && v.chk == cfgChk(v) && v.ent >= 5 && v.ent <= 200 && v.mins >= 20 && v.mins <= 1440) {
    cfgEnt = v.ent; cfgMin = v.mins; cfgAdapt = v.adapt != 0;
  }
}
static void saveCfg() {
  CfgStore v; memset(&v, 0, sizeof(v)); v.magic = CFG_MAGIC; v.ent = cfgEnt; v.mins = cfgMin; v.adapt = cfgAdapt ? 1 : 0;
  v.chk = cfgChk(v); EEPROM.put(CFG_ADDR, v); EEPROM.commit();
}

// Adaptive rule: when the air has been "not GOOD" for most of the last ~5 min the limits tighten
// (entries x0.6, time x0.5); otherwise the stored limits apply.
static void serviceCleaning() {
  static uint32_t t = 0; if (millis() - t < 1000) return; t = millis();
  bool notGood = linkUp() && !(L.bits & 1) && (!strcmp(L.airS, "MODERATE") || !strcmp(L.airS, "POOR") || !strcmp(L.airS, "BAD"));
  airHighFrac += ((notGood ? 1.0f : 0.0f) - airHighFrac) / 300.0f;
  airHigh = cfgAdapt && airHighFrac > 0.5f;
  effEnt = airHigh ? (uint16_t)max(5, (int)cfgEnt * 3 / 5) : cfgEnt;
  effMin = airHigh ? (uint16_t)max(10, (int)cfgMin / 2) : cfgMin;
  if (!timeValid) { cleanKnown = false; return; }
  uint32_t now = nowEpoch();
  if (cleanEpoch == 0 || cleanEpoch > now) { cleanEpoch = now; savePersist(); }
  long best = (long)(cleanEpoch + (uint32_t)effMin * 60UL) - (long)now; cleanWhy = "TIME";
  if (entriesSince >= effEnt) { best = 0; cleanWhy = "ENTRIES"; }
  else {
    uint32_t lastHour = countEntries(now > 3600UL ? now - 3600UL : 0UL, now);
    if (lastHour > 0) {
      long sec = (long)((effEnt - entriesSince) * 3600.0f / lastHour);
      if (sec < best) { best = sec; cleanWhy = "ENTRIES"; }
    }
  }
  if (linkUp() && !(L.bits & 1) && (!strcmp(L.airS, "POOR") || !strcmp(L.airS, "BAD"))) { best = 0; cleanWhy = "AIR"; }
  cleanMin = best <= 0 ? 0 : (best + 59) / 60;
  cleanKnown = true;
}

// ---- 24 h analytics: 5-minute buckets in RAM, one CSV line per bucket on the SD card ----
static void bkClear(uint32_t from, uint32_t to) {
  for (uint32_t i = from; i <= to && i - from < BN; i++) { uint16_t k = i % BN; bAir[k] = 0; bLit[k] = 0; bEnt[k] = 0; bFan[k] = 0; }
}

static void bkRestore(uint32_t nowId) {
  File f = SD.open(PATHS[K_BKT], FILE_READ); if (!f) return;
  size_t sz = f.size(); if (sz > 12000) f.seek(sz - 12000);
  f.readStringUntil('\n');                       // header or partial line
  uint8_t n = 0;
  while (f.available()) {
    String s = f.readStringUntil('\n');
    unsigned long ep = 0, lit = 0; int air = 0, ent = 0, fan = 0;
    if (sscanf(s.c_str(), "%lu,%d,%d,%d,%lu", &ep, &air, &ent, &fan, &lit) == 5) {
      uint32_t id = ep / 300UL;
      if (id < nowId && id + BN > nowId) {
        uint16_t k = id % BN;
        if (!bAir[k]) { bAir[k] = (uint16_t)air; bEnt[k] = (uint8_t)(ent > 255 ? 255 : ent); bFan[k] = (uint8_t)fan; bLit[k] = (uint16_t)(lit > 65535UL ? 65535UL : lit); }
      }
    }
    if (!(++n & 15)) { pollUart(); yield(); }
  }
  f.close();
}

static void serviceBuckets() {
  static uint32_t t = 0; if (millis() - t < 1000) return; t = millis();
  if (cleanAfterMs && (int32_t)(millis() - cleanAfterMs) >= 0) {       // air 5 min after a clean
    if (linkUp() && cleanN) { cleans[cleanN - 1].airA = (uint16_t)(L.air ? L.air : 1); event("CLEAN_AFTER air=%d", L.air); cleanAfterMs = 0; }
    else cleanAfterMs = millis() + 30000UL;
  }
  if (!timeValid) return;
  uint32_t id = nowEpoch() / 300UL;
  if (id < curBk) { memset(bAir, 0, sizeof(bAir)); memset(bLit, 0, sizeof(bLit)); memset(bEnt, 0, sizeof(bEnt)); memset(bFan, 0, sizeof(bFan)); curBk = 0; }
  if (id != curBk) {
    if (curBk) {
      uint16_t k = curBk % BN;
      if (bkTicks) {
        bAir[k] = (uint16_t)(bkAirSum / bkTicks); bEnt[k] = (uint8_t)(bkEnt > 255 ? 255 : bkEnt); bFan[k] = (uint8_t)(bkFan * 100UL / bkTicks);
        float l = bkLit * 100.0f; bLit[k] = (uint16_t)(l > 65535.0f ? 65535.0f : l);
        char ln[64]; snprintf(ln, sizeof(ln), "%lu,%u,%u,%u,%u", (unsigned long)(curBk * 300UL), (unsigned)bAir[k], (unsigned)bEnt[k], (unsigned)bFan[k], (unsigned)bLit[k]);
        logLine(K_BKT, ln);
      }
      if (id > curBk + 1) bkClear(curBk + 1, id - 1);
    }
    curBk = id; bkClear(id, id);
    bkAirSum = bkTicks = bkFan = bkEnt = 0; bkLit = 0.0f; lastTot = -1.0f;
  }
  if (bkRestorePending && sdReady && !downloading) { bkRestorePending = false; bkRestore(curBk); }
  if (linkUp()) {
    bkAirSum += (uint32_t)L.air; bkTicks++; if (L.fan) bkFan++;
    float tot = correctedTotal();
    if (lastTot >= 0.0f && tot >= lastTot && tot - lastTot < 5.0f) bkLit += tot - lastTot;
    lastTot = tot;
  }
}

static void serviceHeap() {
  static uint32_t t = 0, lowSince = 0; if (millis() - t < 2000) return; t = millis();
  if (ESP.getFreeHeap() < 5000) {
    if (!lowSince) lowSince = millis();
    else if (millis() - lowSince > 10000UL) { event("LOW_HEAP_RESTART %u", (unsigned)ESP.getFreeHeap()); if (sdReady) drainQueue(); delay(200); ESP.restart(); }
  } else lowSince = 0;
}

// Home-screen icon + manifest (the page links to both)
static const char MANIFEST[] PROGMEM = "{\"name\":\"Restroom Intelligence\",\"short_name\":\"Restroom\",\"start_url\":\"/\",\"display\":\"standalone\",\"background_color\":\"#f2f2f7\",\"theme_color\":\"#007aff\",\"icons\":[{\"src\":\"/icon.png\",\"sizes\":\"180x180\",\"type\":\"image/png\"}]}";
static const uint8_t ICON_PNG[] PROGMEM = {
  0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A,0x00,0x00,0x00,0x0D,0x49,0x48,0x44,0x52,
  0x00,0x00,0x00,0xB4,0x00,0x00,0x00,0xB4,0x08,0x03,0x00,0x00,0x00,0x0A,0x13,0xF6,
  0x00,0x00,0x00,0x00,0x48,0x50,0x4C,0x54,0x45,0xFF,0xFF,0xFF,0x90,0xF3,0xAE,0x34,
  0xE8,0x6C,0x34,0xE6,0x6F,0x34,0xE6,0x6E,0x34,0xE6,0x6D,0x31,0xE7,0x6A,0x2A,0xA8,
  0xCA,0x0D,0x81,0xFC,0x17,0x7B,0xF6,0x15,0x7C,0xF7,0x11,0x7E,0xF9,0x10,0x7F,0xFA,
  0x13,0x7D,0xF8,0x1C,0x77,0xF2,0x1E,0x76,0xF1,0x1F,0x75,0xF0,0x22,0x73,0xEE,0x20,
  0x74,0xEF,0x24,0x71,0xED,0x19,0x79,0xF4,0x18,0x7A,0xF5,0x17,0x7A,0xF5,0x19,0x77,
  0xF4,0x7A,0x7C,0xBF,0xD4,0x00,0x00,0x0B,0x2B,0x49,0x44,0x41,0x54,0x78,0xDA,0xD5,
  0xDC,0xEB,0x96,0x9B,0x20,0x10,0x00,0x60,0x13,0x93,0x25,0x6A,0x14,0xCD,0x1A,0x79,
  0xFF,0x37,0x2D,0xA0,0x31,0xC2,0xCC,0xC0,0x80,0xE9,0xD9,0xC8,0x9F,0x76,0xDB,0xD3,
  0xEC,0xD7,0xD9,0xE1,0x7E,0x29,0x6E,0xF9,0xE5,0x1E,0x2E,0x8D,0x5F,0x5A,0x58,0x6A,
  0x50,0x2A,0x50,0x26,0x53,0x9E,0xDB,0x52,0xFC,0x37,0x74,0xC3,0x40,0xB3,0xCD,0x0E,
  0x7A,0x2C,0x8E,0x17,0xE7,0x31,0x1F,0xFD,0x87,0xE6,0x6C,0xF4,0x5F,0x9A,0x73,0xD1,
  0x7F,0x6A,0xCE,0x44,0xFF,0xAD,0x39,0x0F,0xFD,0xC7,0xE6,0x2C,0xF4,0x5F,0x9B,0x73,
  0xD0,0x7F,0x6E,0xCE,0x40,0xFF,0xBD,0x39,0x1D,0xFD,0x05,0xE6,0x64,0xF4,0x37,0x98,
  0x53,0xD1,0x5F,0x61,0x4E,0x44,0x7F,0x87,0x39,0x0D,0xFD,0x25,0xE6,0x24,0xF4,0xB7,
  0x98,0x53,0xD0,0x5F,0x63,0x4E,0x40,0x7F,0x8F,0x99,0x8F,0xFE,0x22,0xB3,0x2A,0x0E,
  0x68,0xE6,0xA2,0xBF,0xCA,0xCC,0x44,0x7F,0x97,0x99,0x87,0xFE,0x32,0x33,0x0B,0xFD,
  0x6D,0x66,0x0E,0x3A,0xD1,0xBC,0x0A,0x17,0x6C,0x5B,0x7F,0xD8,0xDC,0x75,0xC5,0x47,
  0xCD,0xC6,0xA9,0x7F,0xB9,0x37,0xE6,0x1F,0x36,0xAD,0xF9,0xBA,0xF9,0x78,0x9C,0x19,
  0x68,0xAE,0xD9,0x7A,0xCD,0x3F,0xA8,0xAA,0x41,0x0C,0xE2,0x74,0x3A,0x89,0xB9,0x54,
  0x35,0x4C,0x90,0x5D,0x71,0x8E,0xA3,0x99,0x66,0x0B,0xBE,0x6B,0xEE,0xA9,0x58,0xCB,
  0xF9,0xE7,0x6C,0x4A,0x59,0x96,0x9A,0xFE,0xDC,0x26,0xC9,0x4E,0x73,0x0C,0xCD,0x32,
  0x5B,0x71,0xB5,0xF5,0xDA,0x52,0x5E,0x4A,0x53,0xCE,0x0B,0x5D,0xC3,0x17,0xF7,0x5E,
  0x73,0x04,0xCD,0x31,0xE3,0x62,0x83,0xFE,0x29,0x2F,0xBA,0x58,0x79,0x79,0x9D,0xE1,
  0xA3,0x76,0xEF,0x36,0x87,0xD1,0x0C,0x73,0x7B,0xBF,0x35,0x98,0xF8,0x8D,0x5E,0xCA,
  0x1C,0x73,0xED,0xAE,0x40,0xA8,0x53,0xCD,0x41,0x74,0xDC,0xAC,0xC9,0x95,0x28,0x88,
  0xE2,0xA2,0x67,0xB8,0x0E,0xB8,0x0E,0xB7,0xCB,0x4E,0x36,0x87,0xD0,0x51,0xB3,0x21,
  0x9F,0x8A,0x82,0x8F,0xB6,0xEE,0xF3,0xF9,0xE2,0xB0,0xD3,0xCD,0x01,0x74,0xCC,0xAC,
  0x73,0x39,0x44,0x26,0xD0,0x86,0x7D,0x39,0x9F,0x0D,0xFB,0x99,0x6B,0xA6,0xD1,0x51,
  0xF3,0xAD,0x0E,0x92,0x49,0xB4,0x71,0x5F,0x4C,0xB4,0x55,0xAE,0x99,0x44,0x47,0xCC,
  0x3A,0x33,0x44,0x51,0xE4,0xA2,0x2D,0x5B,0x57,0xC9,0x4C,0x33,0x85,0x8E,0x99,0x23,
  0x99,0x11,0x45,0x6B,0xF6,0xF5,0x5C,0x76,0x2A,0xCB,0x4C,0xA0,0x23,0xA9,0xC1,0x08,
  0x73,0x14,0x6D,0xAB,0xA4,0xD0,0xE4,0x29,0xD9,0x8C,0xA3,0x63,0x61,0x6E,0x4F,0xC5,
  0x7E,0xF4,0x9C,0x23,0xA3,0x4A,0x37,0xA3,0xE8,0x98,0x59,0x16,0xAC,0xB2,0x74,0xE3,
  0x41,0xB6,0xCE,0x6C,0x95,0x6C,0xC6,0xD0,0x91,0xDC,0x60,0xA5,0xC6,0x66,0xC0,0x74,
  0x0D,0xC9,0x4D,0x8A,0xA8,0x29,0xD1,0x8C,0xA0,0x23,0x3D,0xCA,0x8D,0x95,0x1A,0xC5,
  0x32,0x34,0x7D,0x8D,0x98,0x2E,0x94,0x5B,0xFF,0x75,0x39,0x4E,0x69,0x66,0x88,0x8E,
  0x55,0xC1,0x98,0xF9,0x24,0x64,0xD5,0xAE,0x23,0x6C,0xDD,0xF3,0x3D,0x8D,0xFD,0x62,
  0xC7,0x1D,0x54,0x8A,0xD8,0xC4,0xE6,0x9B,0x01,0x3A,0x62,0xBE,0x07,0xCD,0xA7,0xA1,
  0xB2,0xDA,0xB9,0x8F,0xAE,0xD7,0x69,0x57,0x5D,0x8D,0xC2,0x44,0x1C,0x67,0x5B,0xF5,
  0x98,0x60,0xF6,0xD1,0x11,0x73,0x15,0x12,0x4B,0xEB,0xC5,0xA7,0x56,0xFA,0x17,0xED,
  0xB6,0x09,0x8E,0xAA,0xA5,0x4A,0x30,0x7B,0xE8,0x58,0x9C,0x69,0xB2,0xA8,0x6F,0x8D,
  0x0D,0x2B,0x39,0x1F,0xD4,0xBF,0xD3,0x79,0xA2,0xD9,0xA8,0xBA,0x53,0x7C,0xB3,0x8B,
  0x8E,0xF4,0x83,0x64,0x3E,0xEB,0x20,0xDF,0xDA,0x75,0xFA,0x4D,0xCF,0x61,0xE7,0x70,
  0x5F,0x4A,0x46,0x86,0x84,0xCC,0x0E,0x3A,0xD6,0xD6,0x51,0x66,0x61,0x83,0xCC,0x5B,
  0xDF,0x50,0xA3,0xB8,0x22,0xB9,0xED,0xAB,0x83,0xE6,0x2D,0x3A,0xD6,0xA7,0x10,0x66,
  0x71,0xBF,0x2F,0x62,0xCE,0x5A,0xC1,0xB3,0x52,0x68,0x8E,0x68,0xB5,0xE2,0x9A,0x37,
  0xE8,0x98,0x19,0xEF,0x53,0x4E,0xD5,0x6D,0x25,0x33,0xD7,0x37,0xD4,0xA4,0xD9,0x88,
  0x5A,0x28,0xA6,0xF9,0x8D,0xCE,0xEB,0xBB,0x4D,0x66,0xA4,0xAF,0x7D,0xA9,0x2E,0xA8,
  0x8E,0x99,0x57,0x74,0x6C,0x5C,0xD7,0xA0,0x66,0x79,0xAB,0x73,0xD6,0xEB,0x74,0x67,
  0x22,0x40,0x62,0x97,0xD7,0xA5,0xE1,0x8B,0x9A,0x5F,0xE8,0x7B,0x4E,0x42,0x9F,0x9A,
  0xA6,0xCE,0x5C,0x63,0x9C,0x94,0x38,0xFB,0xC1,0x5E,0x2A,0x63,0xDC,0xBC,0xA0,0xEF,
  0x39,0x09,0x2D,0x9A,0xB6,0xCE,0x5F,0x17,0x55,0x30,0xB1,0x6D,0x82,0x30,0xCC,0x33,
  0x3A,0x36,0x1F,0x44,0x7B,0x42,0xB1,0x4D,0x8D,0x8C,0x35,0x46,0x24,0xB1,0x8D,0xBA,
  0x8B,0x9B,0x65,0xC1,0x9A,0xC3,0xFE,0x07,0xB3,0x56,0x4B,0x5F,0x6D,0x7B,0xC6,0x2E,
  0x6A,0x36,0xE8,0xB8,0x59,0xFC,0x0F,0xB3,0xAE,0x8E,0x88,0xDA,0x24,0x48,0xCC,0x1C,
  0x47,0xEB,0x51,0x50,0xFD,0x5F,0xCC,0x7A,0x2C,0xAA,0x6B,0xA3,0xDF,0xCD,0x98,0x04,
  0x89,0x99,0x65,0x11,0x5F,0x47,0x42,0x5A,0x8E,0xD3,0xBD,0xFE,0xC8,0xDE,0x84,0x56,
  0x83,0x04,0x51,0x51,0x73,0x0C,0x6D,0x96,0xF5,0x91,0x5A,0xD8,0x7C,0x6A,0x3F,0x05,
  0x51,0x07,0x43,0x2D,0x19,0xE8,0x06,0x0F,0xB4,0x6C,0xEA,0x4F,0xED,0x01,0x29,0x2F,
  0xAD,0xC3,0xA1,0x96,0x0C,0x74,0x83,0x07,0x5A,0xDC,0x3E,0x66,0x1E,0x41,0xC3,0x17,
  0x0A,0xB5,0x64,0xA0,0x1B,0x3C,0xD0,0xA7,0x9D,0xE6,0xD1,0x99,0x0E,0xFA,0x09,0x12,
  0x08,0xB5,0x64,0xA0,0x1B,0x22,0xD0,0x55,0xB3,0xCB,0x3C,0x35,0x66,0xC8,0x34,0x8D,
  0x74,0x82,0x10,0xA1,0x96,0x0C,0xF4,0x3C,0xA3,0x86,0x6D,0xF4,0x36,0x39,0xD2,0xCD,
  0x53,0x33,0x98,0x1F,0x9D,0x90,0xAB,0x5A,0xB7,0xD6,0x4E,0xBB,0x47,0x85,0x5A,0x32,
  0xD0,0xAF,0x7D,0x2B,0xD8,0x72,0xEC,0x32,0x8F,0xAF,0x6C,0x13,0x13,0x91,0x20,0x7A,
  0x61,0x4A,0xAA,0xA0,0x99,0x44,0x37,0x64,0xA0,0xEF,0xF5,0x1E,0xF3,0xA6,0x6A,0x4C,
  0x44,0x5D,0x44,0xF3,0x43,0x32,0xD0,0xAF,0xED,0x41,0x50,0x0D,0x37,0xB5,0x30,0xC3,
  0xFC,0x74,0xFE,0xFB,0x53,0x87,0x87,0x5A,0x8F,0x40,0x82,0x66,0x02,0xBD,0xEE,0xC3,
  0xD6,0x65,0x79,0xF2,0x9A,0xE8,0x1D,0x75,0xD0,0x5B,0x6D,0x5D,0xC7,0xFC,0xD1,0x50,
  0x4B,0x06,0x7A,0xDD,0x87,0x6D,0xC4,0xF9,0xE7,0xE7,0xB2,0x75,0x37,0xF5,0x8E,0x76,
  0xC3,0xFB,0xB1,0x69,0x9A,0x35,0xC3,0x50,0x7B,0x68,0xC9,0x40,0x37,0x5B,0xB4,0x9E,
  0x15,0xFD,0xFC,0x94,0xA7,0x93,0xD7,0x74,0xA4,0x9B,0xC7,0x06,0xB4,0xF8,0x1A,0x6D,
  0x87,0xFC,0x3A,0xD4,0xDB,0x06,0xC4,0xCF,0x0F,0xC9,0x40,0x6F,0x4F,0x15,0xD8,0x9F,
  0x9B,0xDE,0xFE,0x33,0x6E,0xF3,0x6D,0xDA,0xB6,0xCE,0x8D,0x73,0x03,0x07,0xB8,0xEB,
  0x3C,0x25,0x18,0x6A,0xC9,0x40,0x6F,0x8F,0x42,0xAC,0x9F,0x55,0x96,0x3F,0x36,0xDE,
  0xB7,0x3A,0x3B,0xCE,0xC8,0xA0,0xDC,0x4E,0x53,0x0C,0x2F,0x84,0x86,0xE6,0xBE,0x08,
  0xC5,0xB9,0xD9,0x7C,0x56,0x69,0xDC,0x53,0x5B,0xE7,0x9A,0x07,0x64,0x62,0xFC,0x9E,
  0x5B,0xB9,0xDD,0xE2,0xB6,0x7F,0x41,0xCC,0x00,0xED,0x9E,0x92,0xF1,0x3F,0x2A,0x3B,
  0xCE,0x2D,0x62,0x2E,0x06,0x83,0x9E,0x69,0x6E,0xA8,0xCD,0x32,0x6A,0xC0,0xEC,0xA3,
  0xBD,0x93,0x3D,0x1E,0x5A,0xD8,0x40,0xE7,0xB4,0x1B,0x63,0x41,0x04,0xBA,0xC3,0xD0,
  0xEF,0xFC,0x40,0xCD,0x1E,0xDA,0x3B,0x8E,0x04,0x3E,0x29,0xF3,0x3C,0x12,0x6A,0x2E,
  0xBA,0xCD,0x74,0xD0,0x6B,0xAA,0x5F,0x68,0xDC,0xEC,0xA2,0xBD,0x38,0x37,0x02,0x64,
  0x47,0x9D,0x65,0x9E,0xD0,0xD5,0xA9,0xED,0xBC,0x7B,0x54,0x18,0x9A,0x30,0x3B,0xE8,
  0x26,0x82,0x16,0x99,0x63,0x7E,0x74,0xDB,0x51,0xBA,0x6B,0x05,0x20,0xA9,0x03,0xE6,
  0x2D,0x1A,0xDE,0xC5,0x83,0x29,0xFD,0xBF,0xCC,0x58,0x4D,0x24,0xCD,0x1B,0x34,0x34,
  0xD7,0x0C,0x74,0x7C,0x9E,0x82,0x35,0xD0,0xB6,0xE1,0x70,0x87,0x17,0xBD,0x3B,0xAA,
  0xD6,0xF9,0x41,0x9A,0xDF,0x68,0xE4,0x1C,0xA3,0x83,0xD6,0x9F,0xF9,0x6C,0x31,0x73,
  0x1D,0x3E,0x27,0xD3,0xF2,0xCC,0x1D,0xAC,0x89,0xA4,0x79,0x45,0x23,0x66,0x9D,0xD2,
  0xEE,0x80,0xB1,0x02,0xE8,0xAA,0x6E,0xDA,0x67,0xF7,0x6C,0x9A,0xB4,0x38,0x0B,0xC4,
  0xAC,0x62,0xE8,0x1E,0xA0,0xB1,0xF3,0xA2,0xB0,0xF1,0x00,0xE6,0xA6,0x16,0xCB,0x02,
  0x2A,0x71,0x4E,0x06,0xED,0x54,0x30,0x33,0x48,0x6A,0x1F,0xDD,0x03,0x34,0x7A,0xC6,
  0x15,0xA0,0x5B,0x60,0x5E,0x49,0xA7,0x67,0x8B,0x99,0x27,0xD4,0xDC,0x60,0x6B,0x8C,
  0x11,0x74,0x0F,0xD0,0xF8,0xB9,0xDC,0x18,0xBA,0x72,0xC6,0x13,0x46,0x0D,0xCC,0x5D,
  0xA4,0x23,0x0C,0x35,0x1F,0x8A,0x34,0x5B,0x34,0x75,0x96,0x58,0x04,0x1B,0x0F,0x5D,
  0x05,0x1D,0xCC,0xB3,0x7D,0xF2,0xCC,0xF8,0xBA,0x68,0x10,0xDD,0x03,0x34,0x79,0xFE,
  0x39,0x88,0xAE,0x2A,0xBF,0x8E,0xE9,0x29,0xA0,0xBB,0x8E,0x84,0x76,0x84,0x8A,0x58,
  0x80,0x0E,0xA1,0x7B,0x80,0xA6,0xCF,0x6C,0x87,0xD0,0x3A,0x81,0x5B,0xDF,0xF3,0x74,
  0xCF,0xF6,0x4C,0x9C,0x4E,0x85,0x83,0xEE,0x01,0x3A,0x70,0xCE,0x3C,0x80,0x36,0xED,
  0xF3,0xB3,0x80,0xEA,0x71,0x33,0x53,0x49,0x32,0x07,0xD0,0x7D,0x1C,0xDD,0x72,0xD0,
  0x76,0xA3,0x1B,0xD9,0x59,0x1C,0xA7,0xD4,0x8E,0x30,0x8E,0x86,0xE6,0x47,0x11,0x38,
  0xCF,0x4F,0xA2,0xE7,0xDD,0xF9,0x67,0x41,0xAB,0x71,0xF3,0x6F,0x60,0x43,0x85,0x42,
  0x23,0x66,0x1F,0xDD,0x86,0xD0,0xDE,0x78,0x03,0xDD,0x0F,0x5D,0xD4,0x0D,0xBB,0x53,
  0x89,0xA0,0x31,0xB3,0x87,0x76,0x16,0xF8,0x5B,0xA2,0x47,0x7C,0xF5,0xD8,0x68,0x30,
  0x8D,0x7A,0x22,0x3A,0x95,0xE0,0xC6,0x15,0xDE,0xB9,0xA0,0x66,0x17,0xDD,0x72,0xD0,
  0xEF,0xB1,0x11,0xBE,0xF5,0x3C,0x4E,0x23,0x6A,0x3E,0x35,0xC1,0xCD,0x14,0x14,0x8D,
  0x9B,0x1D,0x74,0x1B,0x41,0x57,0xFE,0x99,0x93,0x27,0xAA,0xEE,0x5A,0xA2,0x23,0x0C,
  0xA3,0x91,0x01,0x13,0x61,0xDE,0xA2,0xFD,0xFB,0x34,0x70,0x68,0xEA,0x8F,0x9F,0x5B,
  0x5C,0x2D,0xF1,0x8E,0x30,0xB2,0xA7,0x09,0xD0,0x3D,0x65,0xDE,0xA0,0xDB,0x30,0xDA,
  0x36,0x1F,0xFE,0x80,0x9F,0x50,0xA3,0xB3,0xD8,0x88,0xB9,0x67,0xA0,0x1F,0x3E,0x1A,
  0xB9,0xBA,0xD4,0x46,0xD1,0x6C,0xB5,0x8C,0x99,0x95,0x3F,0x78,0x7F,0xD0,0xE6,0x15,
  0x8D,0xDD,0xB7,0x82,0x6D,0x1E,0x40,0x7B,0x4B,0xCE,0x54,0x19,0xA2,0xFB,0xDD,0xA0,
  0xC5,0x0B,0x98,0x5F,0x68,0xF4,0x8E,0x18,0x52,0x13,0xC1,0x7C,0x90,0xA5,0x8E,0x9B,
  0x21,0x5A,0xD2,0xE6,0x05,0x8D,0xDF,0x6B,0x6B,0xFD,0xED,0xD4,0xB1,0x9E,0xC0,0x1C,
  0x96,0x58,0x89,0xF1,0x3A,0xC2,0xF8,0x7E,0x77,0x2C,0xA5,0x1F,0x3E,0x9A,0xB8,0x8B,
  0xD7,0x3E,0xBD,0x65,0x63,0x2F,0x3F,0xE6,0x77,0x7B,0xC6,0x98,0x5A,0x30,0xCC,0x6E,
  0x4A,0x9B,0x6F,0x25,0x69,0xB3,0x45,0xD3,0xF7,0x07,0xC1,0x02,0x24,0x7E,0xB6,0x67,
  0xDC,0x6B,0xF6,0xB3,0xE3,0xEA,0xD7,0xC3,0x87,0x8F,0xA6,0xCD,0xAD,0xFF,0x51,0x63,
  0x8D,0xAE,0x15,0x84,0xD4,0x27,0x8E,0x19,0x64,0x47,0x19,0x32,0xE3,0xE8,0x1A,0x45,
  0xBB,0xF9,0xE1,0xCC,0x07,0x69,0x35,0xD3,0x3C,0x5C,0xFD,0x4D,0xDB,0x80,0xF9,0x51,
  0x04,0xEF,0x96,0xFA,0x49,0xFD,0xCE,0x0F,0xD6,0xBA,0xE8,0x3C,0xBB,0x62,0x98,0xE1,
  0x4A,0xAF,0x0C,0x98,0x31,0xF4,0x76,0x26,0x58,0x12,0xA1,0x06,0xE7,0x0A,0xD0,0x49,
  0x6C,0xBC,0x53,0x79,0xCD,0x4E,0xFC,0xEC,0x78,0x84,0xCC,0x08,0x7A,0x6B,0x86,0xF9,
  0x41,0x9E,0x85,0x50,0x5D,0xBE,0x19,0xEE,0x7E,0xCA,0x80,0x79,0x18,0x8A,0xF0,0xBD,
  0xE3,0xD1,0xCD,0x0F,0x5D,0x15,0xC9,0xF3,0x1B,0x48,0xAC,0x07,0x66,0x9C,0x65,0x49,
  0x67,0xC7,0x83,0x81,0xF6,0x57,0xBE,0xE0,0x5E,0x2A,0x79,0xE6,0x04,0xA8,0x87,0xC8,
  0x71,0x35,0x49,0x06,0x3A,0x18,0x67,0x80,0xF6,0x17,0xBE,0x40,0x7E,0xE8,0x50,0x93,
  0xE7,0x64,0x3C,0xB5,0xE0,0xC6,0x19,0x06,0x5A,0x05,0xCD,0x1E,0x1A,0x59,0x7F,0x86,
  0x1F,0x48,0x9F,0xED,0x71,0xD4,0x6C,0x33,0x3C,0x5B,0x23,0xE4,0x23,0x64,0x76,0xD1,
  0x88,0xB9,0xF6,0xC7,0x1F,0x57,0xA3,0xA6,0xEE,0x88,0xA9,0xEE,0x94,0x1E,0x67,0x10,
  0xE8,0xB5,0x1A,0x12,0x66,0x07,0x8D,0xAD,0xF3,0xD7,0xE3,0xE5,0xE2,0x7D,0xA4,0x7F,
  0x90,0xD1,0x3D,0xCA,0x6F,0x0F,0xCE,0xE8,0x5B,0x02,0x6C,0x33,0x1D,0x68,0xCA,0xBC,
  0x45,0x13,0x97,0x3D,0xE0,0x67,0xAA,0x31,0x70,0x47,0x4C,0xF7,0x26,0x1A,0xA2,0xF8,
  0xE6,0xE1,0xEC,0x1F,0x27,0xD4,0xDC,0xA0,0x79,0x83,0x26,0xF6,0x53,0xEA,0xF1,0xEC,
  0x9F,0xCD,0xEE,0x82,0x77,0xC4,0x3A,0xC6,0x59,0x62,0x49,0x26,0xC7,0xBB,0x1A,0xD2,
  0xE6,0x37,0x9A,0xBE,0x9F,0x02,0xCF,0xA0,0x85,0xEF,0xB5,0xE9,0x43,0x1C,0x7C,0x33,
  0x72,0xC2,0x6D,0x0E,0x74,0xC0,0xBC,0xA2,0xE9,0x7D,0x2B,0x93,0xD5,0xE0,0x64,0x65,
  0xEA,0xBD,0x36,0xAE,0x79,0xED,0x58,0x42,0xE6,0x17,0x3A,0xB4,0xD7,0x56,0x63,0xE7,
  0x41,0x3F,0x65,0x76,0x37,0xE2,0xD6,0x8E,0x25,0x68,0x5E,0xD0,0xE1,0xFD,0xC1,0xD1,
  0xFF,0xE4,0x45,0xBD,0xDB,0x2C,0x7B,0xE4,0x00,0xB5,0x49,0x8E,0xB0,0x79,0x46,0x47,
  0xF6,0x34,0xB1,0x03,0xC3,0x5D,0xDA,0xBD,0x36,0xE2,0x9C,0x0C,0x72,0x54,0x3D,0x1E,
  0xE7,0x19,0x1D,0xDF,0x87,0x45,0x3E,0x3C,0xED,0x5E,0x1B,0xD3,0x7C,0x2D,0x07,0x86,
  0xD9,0xA0,0x19,0x7B,0xC7,0x9D,0x7F,0x4B,0x0C,0x53,0xEF,0x36,0x2F,0xB5,0x30,0x6A,
  0xD6,0x68,0xCE,0x7E,0x37,0x48,0x90,0xF9,0x4A,0xEF,0xAE,0x3A,0xD8,0x23,0xE6,0x92,
  0x13,0x67,0x8D,0xE6,0x98,0xF5,0xC0,0x0E,0xF9,0x0E,0x67,0xA1,0x76,0xB5,0x1B,0x88,
  0x79,0x40,0x87,0xCF,0x0C,0x34,0x7E,0xAE,0x60,0x84,0xDF,0xE3,0xAA,0xD5,0xEA,0x73,
  0xE6,0x79,0xD0,0xC1,0x31,0xFF,0x16,0xCC,0xB3,0x10,0xA0,0xB1,0x9E,0x2F,0x22,0x77,
  0x2A,0xC7,0xAC,0x24,0xBC,0x76,0x31,0xF7,0xDF,0x2C,0x33,0x40,0x93,0xEF,0x3F,0xC3,
  0xB4,0x5E,0x12,0x5B,0xA5,0x9B,0x87,0x12,0xDE,0xA3,0x9C,0xFB,0x6F,0x96,0xD9,0x47,
  0x07,0xCE,0x9C,0x28,0xF4,0x3A,0x9E,0x09,0xF6,0x7C,0x90,0x91,0x6B,0x36,0x61,0x46,
  0xFF,0xFF,0x0F,0xAE,0xD9,0x43,0x87,0xCE,0xC9,0x20,0x69,0x3D,0xA7,0x88,0x30,0x6C,
  0xEE,0x18,0x49,0x93,0xB1,0x5B,0xB6,0x78,0x25,0x24,0xCC,0x2E,0x3A,0x7C,0xB6,0x07,
  0x55,0xDB,0x27,0x2F,0x44,0xC7,0x1C,0x3F,0x5B,0xF2,0x19,0xBD,0x18,0x9C,0x60,0x76,
  0xD0,0xD1,0x7B,0x6D,0xA8,0xDA,0xE6,0x48,0x84,0xED,0x90,0x4B,0xEC,0xF9,0x09,0xA4,
  0xE1,0x20,0xCD,0x5B,0x74,0xCE,0xBD,0xB6,0xF5,0xA5,0x0E,0xFD,0x36,0x14,0x9D,0xD8,
  0x06,0xAC,0xE4,0x40,0x90,0x6D,0x93,0x9F,0x10,0xE7,0x2D,0x9A,0x75,0xAF,0xAD,0x43,
  0xD5,0xAF,0xB7,0x8A,0x04,0x7E,0x99,0x50,0xFF,0xA1,0x9C,0xDF,0x43,0x40,0xDF,0x15,
  0x30,0xD5,0x22,0xC9,0xFC,0x46,0x33,0xEF,0xB5,0xE1,0x19,0xB2,0x84,0xDB,0x3E,0xC6,
  0x25,0x3B,0xE5,0x94,0x4E,0x9A,0x17,0x1C,0xE8,0x97,0x27,0x4C,0x3E,0xCB,0x24,0xF3,
  0x8A,0x66,0xDF,0x6B,0x1B,0x4B,0xF2,0xF9,0x88,0xF9,0x55,0x8C,0xF9,0x01,0xB4,0xB5,
  0xBC,0xFF,0x94,0x7C,0x97,0x64,0x48,0x8B,0xF3,0x8A,0x4E,0xB8,0xD7,0x86,0x5C,0x22,
  0xDC,0xBA,0x97,0x07,0xD0,0xB6,0xE5,0x42,0xBF,0x4A,0x52,0x26,0xB5,0xCF,0x2E,0x3A,
  0xED,0x8E,0x98,0xA0,0x52,0xE4,0x2D,0x77,0x4A,0xE8,0xF9,0x17,0x5D,0x05,0x93,0xCD,
  0x33,0x3A,0xF1,0x5D,0x35,0xDB,0x88,0x44,0x1E,0xD1,0xE1,0x14,0xDB,0x33,0xA9,0x74,
  0xB3,0x45,0x27,0xBF,0x05,0xA7,0x5F,0x61,0x38,0x9F,0x77,0xAB,0x4D,0xB7,0x34,0x30,
  0xC7,0x75,0x00,0x9D,0xF1,0x7E,0x9D,0x52,0xF8,0x93,0x17,0xA9,0x61,0xEE,0xFB,0x1C,
  0xB3,0x46,0x67,0xBE,0xB9,0xD7,0x89,0xF3,0x8E,0x1C,0xB1,0xFD,0xD1,0x20,0x33,0xF2,
  0xD9,0xA2,0x33,0xCD,0x73,0xB0,0x33,0xD9,0x73,0x17,0xDA,0xF7,0x99,0x66,0x04,0xCD,
  0x7C,0xA3,0xAC,0xCB,0x66,0xBF,0xC9,0x99,0x66,0x88,0x4E,0x79,0x5F,0x74,0x66,0x5F,
  0x52,0xDC,0xE5,0x4C,0x7E,0xC8,0x1D,0x66,0x80,0x4E,0x7C,0x0B,0xCE,0xB0,0xCB,0x40,
  0x7F,0x87,0xF5,0x99,0x3A,0xCA,0x72,0x4F,0x9C,0x01,0x3A,0xFD,0xFD,0x3A,0x35,0x0F,
  0x38,0xCF,0xB1,0x67,0xC4,0x56,0xB1,0x88,0xAD,0xF3,0x47,0xCD,0x1E,0x3A,0xEB,0xCD,
  0x3D,0xFD,0xD5,0x3C,0x22,0x3A,0x53,0xFD,0xDF,0xE6,0x8D,0xD0,0x5E,0xEE,0x37,0xBB,
  0xE8,0xBC,0x77,0x02,0x57,0x77,0xB9,0x8E,0x34,0xDC,0xB2,0xBE,0xC6,0xFA,0x90,0x32,
  0xBC,0xD7,0xC6,0x33,0x3B,0xE8,0x6C,0xF3,0x7B,0xD0,0x2C,0xB0,0x01,0x93,0x1D,0xF7,
  0x3D,0x7A,0x15,0xDC,0xEF,0x4E,0x30,0x6F,0xD1,0xBB,0xCC,0x2F,0xB8,0x1D,0x3D,0xCF,
  0xAF,0xB7,0x95,0xAF,0x41,0x6A,0xDF,0x4B,0x25,0x83,0x67,0x21,0xD2,0xCC,0x1B,0xF4,
  0x7E,0xF3,0x2A,0xF7,0x4A,0xE8,0x0C,0x55,0x8E,0xF9,0x8D,0xFE,0x94,0xB9,0xA3,0xAF,
  0xA7,0x7C,0xCA,0xBC,0xA2,0x3F,0x65,0xA6,0xEF,0x2E,0x7D,0xCE,0xFC,0x42,0x1F,0xCA,
  0xBC,0xA0,0x8F,0x65,0x9E,0xD1,0x07,0x33,0x5B,0xF4,0xD1,0xCC,0x06,0x7D,0x38,0xB3,
  0x46,0x1F,0xCF,0xFC,0x5B,0x1C,0xD0,0x3C,0xA3,0x0F,0x66,0xB6,0xE8,0xA3,0x99,0x0D,
  0xFA,0x70,0x66,0x1F,0x7D,0x08,0xF3,0x6F,0x71,0xBC,0x38,0x7B,0xE8,0x83,0x98,0x1D,
  0xF4,0x51,0xCC,0x5B,0xF4,0x61,0xCC,0x1B,0xF4,0x71,0xCC,0x6F,0xF4,0x81,0xCC,0x2B,
  0xFA,0x48,0xE6,0x17,0xFA,0x50,0xE6,0x05,0x7D,0x2C,0xF3,0x8C,0x3E,0x98,0xD9,0xA2,
  0xE9,0xB5,0xDC,0x8F,0x9B,0x1F,0x9F,0x30,0x1B,0xF4,0xD1,0xE2,0x6C,0xD0,0x87,0x8B,
  0xF3,0xEF,0xEF,0x3F,0x36,0x8E,0x04,0x05,0x2C,0x98,0x28,0x8D,0x00,0x00,0x00,0x00,
  0x49,0x45,0x4E,0x44,0xAE,0x42,0x60,0x82};

// ================================================================
//  DASHBOARD PAGE (stored in flash, served from PROGMEM)
// ================================================================
const char PAGE[] PROGMEM =
  "<!DOCTYPE html>\n"
  "<html lang=\"en\">\n"
  "<head>\n"
  "<meta charset=\"utf-8\">\n"
  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,viewport-fit=cover\">\n"
  "<meta name=\"color-scheme\" content=\"light dark\">\n"
  "<meta name=\"theme-color\" content=\"#007aff\">\n"
  "<meta name=\"apple-mobile-web-app-capable\" content=\"yes\">\n"
  "<meta name=\"mobile-web-app-capable\" content=\"yes\">\n"
  "<meta name=\"apple-mobile-web-app-title\" content=\"Restroom\">\n"
  "<meta name=\"apple-mobile-web-app-status-bar-style\" content=\"default\">\n"
  "<link rel=\"manifest\" href=\"/manifest.json\">\n"
  "<link rel=\"apple-touch-icon\" href=\"/icon.png\">\n"
  "<link rel=\"icon\" href=\"/icon.png\">\n"
  "<title>Restroom Intelligence</title>\n"
  "<script>(function(){var t=null;try{t=localStorage.getItem('ri-theme')}catch(e){}if(t!=='dark'&&t!=='light')t=(window.matchMedia&&matchMedia('(prefers-color-scheme:dark)').matches)?'dark':'light';document.documentElement.setAttribute('data-theme',t);})();</script>\n"
  "<style>\n"
  ":root{--bg:#f2f2f7;--card:rgba(255,255,255,.82);--solid:#fff;--ink:#1c1c1e;--mut:#6e6e73;--line:rgba(60,60,67,.14);--fill:rgba(120,120,128,.12);--blue:#007aff;--green:#34c759;--red:#ff3b30;--orange:#ff9500;--sh:0 1px 2px rgba(0,0,0,.04),0 8px 24px rgba(0,0,0,.06)}\n"
  "[data-theme=dark]{--bg:#000;--card:rgba(28,28,30,.86);--solid:#1c1c1e;--ink:#f5f5f7;--mut:#98989f;--line:rgba(84,84,88,.55);--fill:rgba(120,120,128,.26);--blue:#0a84ff;--green:#30d158;--red:#ff453a;--orange:#ff9f0a;--sh:0 1px 2px rgba(0,0,0,.4)}\n"
  "*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}\n"
  "html{background:var(--bg);overflow-x:hidden}\n"
  "body{background:var(--bg);color:var(--ink);font:15px/1.4 -apple-system,BlinkMacSystemFont,\"SF Pro Text\",\"SF Pro Display\",\"Helvetica Neue\",Segoe UI,Roboto,Arial,sans-serif;-webkit-font-smoothing:antialiased;min-height:100vh;transition:background .3s,color .3s}\n"
  "button{font:inherit;color:inherit;border:0;background:none;cursor:pointer}\n"
  ".hidden{display:none!important}\n"
  "/* intro */\n"
  "#intro{position:fixed;inset:0;z-index:100;background:var(--bg);transition:opacity .5s}#intro canvas{width:100%;height:100%;display:block}.out{opacity:0;pointer-events:none}\n"
  "/* shell */\n"
  "#app{max-width:980px;margin:0 auto;padding:calc(10px + env(safe-area-inset-top)) 16px calc(96px + env(safe-area-inset-bottom))}\n"
  ".top{display:flex;flex-wrap:wrap;align-items:flex-end;justify-content:space-between;gap:10px;margin:8px 2px 14px}\n"
  ".top h1{font-size:32px;font-weight:700;letter-spacing:-.8px;line-height:1.1}.top small{color:var(--mut);font-size:13px}\n"
  ".hr{display:flex;gap:8px;align-items:center}\n"
  ".pill{display:inline-flex;align-items:center;gap:6px;padding:7px 12px;border-radius:999px;background:var(--card);box-shadow:var(--sh);font-size:12.5px;font-weight:600;white-space:nowrap}\n"
  ".pill button{width:22px;height:22px;border-radius:50%;background:var(--fill);font-weight:700;font-size:12px}\n"
  ".pill.due{color:var(--red)}.pill.soon{color:var(--orange)}\n"
  "#themeBtn{width:38px;height:38px;border-radius:50%;background:var(--card);box-shadow:var(--sh);font-size:18px}\n"
  ".dot{width:8px;height:8px;border-radius:50%;background:var(--mut)}.dot.ok{background:var(--green)}.dot.bad{background:var(--red)}.dot.warn{background:var(--orange)}\n"
  "/* tabs */\n"
  ".tabs{position:fixed;left:0;right:0;bottom:0;z-index:40;display:flex;justify-content:center;gap:4px;padding:8px 12px calc(8px + env(safe-area-inset-bottom));background:var(--card);-webkit-backdrop-filter:blur(24px) saturate(180%);backdrop-filter:blur(24px) saturate(180%);border-top:1px solid var(--line)}\n"
  ".tabs button{flex:1;max-width:150px;display:flex;flex-direction:column;align-items:center;gap:2px;padding:6px 4px;border-radius:12px;font-size:11px;font-weight:600;color:var(--mut)}\n"
  ".tabs button svg{width:24px;height:24px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}\n"
  ".tabs button.on{color:var(--blue)}\n"
  "/* cards */\n"
  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px;margin-bottom:14px}\n"
  ".card{position:relative;background:var(--card);border-radius:22px;padding:18px;box-shadow:var(--sh);overflow:hidden;min-width:0;-webkit-backdrop-filter:blur(20px);backdrop-filter:blur(20px)}\n"
  ".card h3{font-size:12px;font-weight:600;letter-spacing:.6px;text-transform:uppercase;color:var(--mut);margin-bottom:12px;display:flex;justify-content:space-between;align-items:center;gap:8px}\n"
  ".big{font-size:42px;font-weight:700;letter-spacing:-1.4px;line-height:1}.sub{color:var(--mut);font-size:12.5px;margin-top:4px}\n"
  ".kv{display:flex;justify-content:space-between;gap:10px;padding:8px 0;border-top:1px solid var(--line);font-size:13.5px}.kv span{color:var(--mut)}.kv b{font-weight:600;text-align:right}\n"
  ".chip{display:inline-block;padding:3px 10px;border-radius:999px;font-size:11.5px;font-weight:700;background:var(--fill);color:var(--mut)}\n"
  ".chip.ok{background:rgba(52,199,89,.16);color:var(--green)}.chip.warn{background:rgba(255,149,0,.16);color:var(--orange)}.chip.bad{background:rgba(255,59,48,.14);color:var(--red)}\n"
  ".na{position:absolute;inset:0;z-index:5;display:none;flex-direction:column;align-items:center;justify-content:center;text-align:center;padding:20px;background:var(--card);-webkit-backdrop-filter:blur(10px);backdrop-filter:blur(10px)}\n"
  ".na b{color:var(--red);margin-bottom:4px}.na p{color:var(--mut);font-size:13px;max-width:280px}.card.bad .na{display:flex}\n"
  ".btn{padding:12px 16px;border-radius:14px;background:var(--fill);font-weight:600;font-size:14.5px;min-height:46px;transition:transform .12s,opacity .2s}\n"
  ".btn:active{transform:scale(.97)}.btn.pri{background:var(--blue);color:#fff}.btn.grn{background:var(--green);color:#fff}.btn.red{background:var(--red);color:#fff}\n"
  ".btn[disabled],.btn.dis{opacity:.38;pointer-events:none}\n"
  ".seg{display:flex;background:var(--fill);border-radius:12px;padding:3px;gap:2px}.seg button{flex:1;padding:9px 8px;border-radius:10px;font-weight:600;font-size:14px;color:var(--mut)}.seg button.on{background:var(--solid);color:var(--ink);box-shadow:0 2px 6px rgba(0,0,0,.14)}\n"
  ".seg.dis{opacity:.4;pointer-events:none}\n"
  "/* air */\n"
  "#airCv{width:100%;height:190px;display:block}\n"
  "/* tank */\n"
  ".tw{display:flex;align-items:center;gap:16px}#tk{width:104px;height:140px;flex:none}\n"
  ".tko{fill:var(--fill);stroke:var(--mut);stroke-width:3}.dry .tko{stroke:var(--red);animation:pr 1.6s ease-in-out infinite}.wet .tko{stroke:var(--green)}\n"
  "@keyframes pr{50%{opacity:.55}}\n"
  "#wfill{transition:transform 1s cubic-bezier(.3,.8,.3,1)}.wf{fill:var(--green)}.dry .wf{fill:var(--red)}\n"
  ".wv{animation:wave 2.4s linear infinite}.wv2{animation:wave 3.6s linear infinite reverse;opacity:.45}@keyframes wave{to{transform:translateX(-60px)}}\n"
  ".state{font-size:26px;font-weight:700;letter-spacing:-.5px}.state.ok{color:var(--green)}.state.bad{color:var(--red)}\n"
  "/* pipe */\n"
  "#pipe{width:100%;height:96px;display:block}.pb{fill:var(--fill);stroke:var(--line);stroke-width:2}\n"
  ".pf{stroke:var(--mut);stroke-width:8;stroke-linecap:round;stroke-dasharray:2 22;fill:none;animation:dash 1s linear infinite;animation-play-state:paused}.run .pf{stroke:var(--blue);animation-play-state:running}\n"
  "@keyframes dash{to{stroke-dashoffset:-24}}\n"
  "/* fan */\n"
  ".fr{display:flex;gap:16px;align-items:center}#fanSvg{width:104px;height:104px;flex:none}.fb{fill:var(--blue)}\n"
  ".fbtn{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}\n"
  "/* list rows */\n"
  ".lr{display:flex;gap:12px;align-items:center;padding:12px 0;border-top:1px solid var(--line)}.lr:first-of-type{border-top:0}\n"
  ".lr .t{flex:1;min-width:0}.lr .t b{display:block;font-size:15px}.lr .t span{display:block;color:var(--mut);font-size:12.5px}\n"
  ".ic{width:30px;height:30px;border-radius:50%;display:grid;place-items:center;font-weight:700;font-size:15px;color:#fff;background:var(--mut);flex:none}.ic.pass,.ic.ok{background:var(--green)}.ic.fail{background:var(--red)}.ic.warn{background:var(--orange)}\n"
  ".note{background:var(--fill);border-radius:12px;padding:10px 12px;font-size:13px;color:var(--mut);margin:8px 0}\n"
  ".al{display:flex;gap:10px;padding:10px 12px;border-radius:14px;background:var(--fill);margin-bottom:8px;font-size:13px}.al b{display:block}.al.l2{background:rgba(255,59,48,.13)}.al.l1{background:rgba(255,149,0,.14)}\n"
  "input[type=number],input[type=time]{font:inherit;color:var(--ink);background:var(--fill);border:0;border-radius:10px;padding:9px 10px;width:100%;min-width:0}\n"
  ".chips{display:flex;gap:6px;flex-wrap:wrap}.chips button{padding:7px 12px;border-radius:999px;background:var(--fill);font-size:13px;font-weight:600;color:var(--mut)}.chips button.on{background:var(--blue);color:#fff}\n"
  ".res{margin-top:12px;padding:12px 14px;border-radius:14px;background:var(--fill);display:flex;align-items:baseline;gap:8px}.res b{font-size:28px;color:var(--blue)}\n"
  "canvas.ch{width:100%;height:150px;display:block}\n"
  ".ins{list-style:none;margin-top:8px;display:grid;gap:4px}.ins li{font-size:12.8px;color:var(--mut);padding-left:14px;position:relative}.ins li:before{content:\"\";position:absolute;left:2px;top:.55em;width:6px;height:6px;border-radius:50%;background:var(--blue)}\n"
  ".dl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}.dl a{padding:12px;border-radius:12px;background:var(--fill);color:var(--blue);text-align:center;text-decoration:none;font-weight:600;font-size:13.5px}\n"
  "#raw{font:11.5px/1.5 ui-monospace,Menlo,Consolas,monospace;color:var(--mut);white-space:pre-wrap;word-break:break-all;max-height:170px;overflow:auto;background:var(--fill);border-radius:12px;padding:10px}\n"
  "/* wizard */\n"
  ".prog{display:flex;gap:4px;margin:6px 0 14px}.prog i{flex:1;height:5px;border-radius:5px;background:var(--fill)}.prog i.pass{background:var(--green)}.prog i.fail{background:var(--red)}.prog i.cur{background:var(--blue)}\n"
  ".stage{text-align:center;padding:10px 0}.stage .bigic{width:76px;height:76px;margin:6px auto 12px;border-radius:50%;display:grid;place-items:center;font-size:36px;font-weight:700;color:#fff;background:var(--mut)}\n"
  ".bigic.pass{background:var(--green)}.bigic.fail{background:var(--red)}.bigic.run{background:none;border:6px solid var(--fill);border-top-color:var(--blue);animation:sp .8s linear infinite;font-size:0}@keyframes sp{to{transform:rotate(360deg)}}\n"
  ".stage h2{font-size:21px;letter-spacing:-.3px}.stage p{color:var(--mut);font-size:14px;margin-top:4px}\n"
  ".row2{display:flex;gap:10px;margin-top:12px}.row2>*{flex:1}\n"
  "#toast{position:fixed;left:50%;bottom:calc(86px + env(safe-area-inset-bottom));transform:translate(-50%,20px);opacity:0;padding:11px 18px;border-radius:14px;background:rgba(40,40,44,.94);color:#fff;font-size:13.5px;z-index:90;transition:.3s;max-width:92vw;text-align:center;pointer-events:none}#toast.show{opacity:1;transform:translate(-50%,0)}\n"
  "#banner{margin-bottom:14px;padding:12px 14px;border-radius:16px;background:rgba(255,59,48,.12);color:var(--red);font-size:13.5px}\n"
  "\n"
  "/* ===== v5 ===== */\n"
  ":root{--s:12.5px;--m:15px;--l:32px}\n"
  ".big,.state,.top h1,.stage h2,.res b{font-size:var(--l)}\n"
  ".btn,.kv,.lr .t b,.seg button,.lr .t span,.note,.al,.chips button,.ins li,.pill,.card h3{font-size:var(--s)}\n"
  ".btn,.seg button,.lr .t b,.kv{font-size:var(--m)}\n"
  ".card h3{font-size:var(--s)}.sub,.kv span.s{font-size:var(--s)}\n"
  ".btn{transition:transform .12s,opacity .2s,filter .12s}.btn:active,.tabs button:active,.chips button:active,.seg button:active{transform:scale(.95);filter:brightness(.92)}\n"
  ".top{margin-top:6px}\n"
  ".strip{display:flex;gap:6px;overflow-x:auto;margin:0 0 14px;padding:2px;scrollbar-width:none}.strip::-webkit-scrollbar{display:none}\n"
  ".sc{display:inline-flex;align-items:center;gap:6px;padding:6px 11px;border-radius:999px;background:var(--card);box-shadow:var(--sh);font-size:var(--s);font-weight:600;white-space:nowrap;flex:none}\n"
  ".sc svg{width:14px;height:14px;stroke:currentColor;fill:none;stroke-width:2;stroke-linecap:round;stroke-linejoin:round;color:var(--mut)}\n"
  ".hero{display:flex;gap:18px;align-items:center;margin-bottom:14px;flex-wrap:wrap}\n"
  ".ring{position:relative;width:140px;height:140px;flex:none;margin:0 auto}\n"
  ".ring svg{width:100%;height:100%;transform:rotate(-90deg)}.ring .tr{stroke:var(--fill)}.ring .pg{stroke:var(--mut);transition:stroke-dashoffset 1.1s cubic-bezier(.3,.8,.3,1),stroke .5s}\n"
  ".rn{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}.rn b{font-size:var(--l);letter-spacing:-1px;line-height:1}.rn span{font-size:var(--s);color:var(--mut);margin-top:2px}\n"
  ".hm{flex:1;min-width:210px}.hm .m{font-size:var(--l);font-weight:700;letter-spacing:-.8px;line-height:1.1}\n"
  ".mb{display:grid;grid-template-columns:84px 1fr auto;gap:8px;align-items:center;font-size:var(--s);color:var(--mut);margin-top:7px}.mb i{display:block;height:6px;border-radius:6px;background:var(--fill);position:relative;overflow:hidden}.mb i:after{content:\"\";position:absolute;left:0;top:0;bottom:0;width:var(--w,0%);border-radius:6px;background:var(--c,var(--blue));transition:width .8s cubic-bezier(.3,.8,.3,1)}.mb b{color:var(--ink);font-weight:600;min-width:46px;text-align:right}\n"
  ".tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px}\n"
  ".tile{background:var(--fill);border-radius:16px;padding:12px}.tile span{display:block;font-size:var(--s);color:var(--mut);font-weight:600}.tile b{display:block;font-size:20px;letter-spacing:-.4px;margin:2px 0;font-weight:700}.tile i{display:block;font-style:normal;font-size:var(--s);color:var(--mut)}.tile.warn b{color:var(--orange)}.tile.bad b{color:var(--red)}.tile.ok b{color:var(--green)}\n"
  "input[type=date],select{font:inherit;color:var(--ink);background:var(--fill);border:0;border-radius:10px;padding:9px 10px;width:100%;min-width:0}\n"
  "input[type=number]{max-width:110px;text-align:right}\n"
  ".dl a{display:flex;flex-direction:column;gap:1px}.dl a small{color:var(--mut);font-weight:500;font-size:var(--s)}\n"
  ".rowx{display:flex;justify-content:space-between;gap:10px;padding:9px 0;border-top:1px solid var(--line);font-size:var(--s)}.rowx:first-child{border-top:0}.rowx b{font-size:var(--m)}.rowx span{color:var(--mut)}\n"
  "/* skeleton */\n"
  ".sk .big,.sk .state,.sk .kv b,.sk .tile b,.sk .rn b,.sk .hm .m,.sk .sc{color:transparent!important;background:linear-gradient(90deg,var(--fill),transparent,var(--fill));background-size:200% 100%;border-radius:8px;animation:shim 1.2s linear infinite}\n"
  "@keyframes shim{to{background-position:-200% 0}}\n"
  "/* pop-up */\n"
  "#pop{position:fixed;inset:0;z-index:95;display:none;align-items:center;justify-content:center;padding:16px;background:rgba(0,0,0,.5);-webkit-backdrop-filter:blur(10px);backdrop-filter:blur(10px)}#pop.show{display:flex}\n"
  ".sheet{width:min(460px,100%);background:var(--solid);border-radius:30px;padding:26px 22px calc(22px + env(safe-area-inset-bottom));box-shadow:0 20px 60px rgba(0,0,0,.4);animation:up .35s cubic-bezier(.2,.9,.3,1.1);text-align:center}\n"
  "@keyframes up{from{transform:translateY(40px) scale(.96);opacity:0}}\n"
  ".sheet .pi{width:84px;height:84px;margin:0 auto 12px;border-radius:50%;display:grid;place-items:center;font-size:40px;color:#fff;background:var(--orange);animation:pu 1.6s ease-in-out infinite}.sheet.due .pi{background:var(--red)}@keyframes pu{50%{box-shadow:0 0 0 14px rgba(255,149,0,.16)}}\n"
  ".sheet h2{font-size:var(--l);letter-spacing:-.8px;line-height:1.1}.sheet p{color:var(--mut);font-size:var(--m);margin:6px 0 14px}\n"
  ".why{text-align:left;margin-bottom:14px}.why .rowx b{font-size:var(--m)}.why .main{color:var(--red);font-weight:700}\n"
  ".sheet .btn{width:100%;margin-top:8px;min-height:54px;font-size:var(--m)}\n"
  "/* report + print */\n"
  "#rep{display:none}\n"
  "@media print{html,body{background:#fff!important}body>*:not(#rep){display:none!important}#rep{display:block!important;color:#000;padding:8px;font:13px/1.45 -apple-system,Segoe UI,Arial,sans-serif}#rep h1{font-size:24px;margin-bottom:2px}#rep h2{font-size:15px;margin:16px 0 6px;border-bottom:1px solid #999}#rep table{width:100%;border-collapse:collapse}#rep td,#rep th{padding:4px 6px;border-bottom:1px solid #ddd;text-align:left}#rep small{color:#555}}\n"
  "/* ===== v9 refined iOS system ===== */\n"
  ":root{--nav:rgba(242,242,247,.74);--ease:cubic-bezier(.32,.72,0,1)}\n"
  "[data-theme=dark]{--nav:rgba(0,0,0,.66)}\n"
  "body{background:radial-gradient(1100px 520px at 50% -8%,rgba(0,122,255,.09),transparent 62%),var(--bg);background-attachment:fixed}\n"
  "#app{padding-top:0;padding-bottom:calc(112px + env(safe-area-inset-bottom))}\n"
  ".top{position:sticky;top:0;z-index:30;margin:0 -16px 14px;padding:calc(10px + env(safe-area-inset-top)) 18px 10px;background:var(--nav);-webkit-backdrop-filter:saturate(180%) blur(22px);backdrop-filter:saturate(180%) blur(22px);border-bottom:.5px solid var(--line)}\n"
  ".top h1{font-size:30px;letter-spacing:-.9px}\n"
  ".card{border-radius:24px;border:.5px solid var(--line);box-shadow:0 1px 2px rgba(0,0,0,.04),0 10px 30px rgba(0,0,0,.05);transition:transform .35s var(--ease),box-shadow .35s}\n"
  ".tabs{left:50%;right:auto;transform:translateX(-50%);width:min(560px,calc(100% - 24px));bottom:calc(10px + env(safe-area-inset-bottom));border-radius:30px;padding:6px;border:.5px solid var(--line);box-shadow:0 12px 40px rgba(0,0,0,.2);-webkit-backdrop-filter:saturate(180%) blur(26px);backdrop-filter:saturate(180%) blur(26px)}\n"
  ".tabs button{border-radius:22px;transition:background .25s,color .25s}.tabs button.on{background:var(--fill)}\n"
  "#toast{bottom:calc(96px + env(safe-area-inset-bottom))}\n"
  ".btn.pri{background:linear-gradient(180deg,#3d94ff,#007aff);box-shadow:0 4px 14px rgba(0,122,255,.3)}\n"
  ".btn.grn{background:linear-gradient(180deg,#4cd964,#2fc04c)}.btn.red{background:linear-gradient(180deg,#ff5e55,#ff3b30)}\n"
  ".btn.flat{background:transparent;color:var(--blue)}\n"
  ".lr .ic{background:transparent;color:var(--mut);box-shadow:inset 0 0 0 1.5px var(--line);font-size:14px;width:28px;height:28px}\n"
  ".lr .ic.ok,.lr .ic.pass{background:var(--green);color:#fff;box-shadow:none}\n"
  ".lr .ic.warn{background:rgba(255,149,0,.14);color:var(--orange);box-shadow:inset 0 0 0 1.5px var(--orange)}\n"
  ".lr .ic.fail{background:var(--red);color:#fff;box-shadow:none}\n"
  ".chead{display:flex;align-items:center;gap:14px}.chead .mon{width:84px;height:auto;flex:none}\n"
  "@keyframes spin9{to{transform:rotate(360deg)}}@keyframes fi9{from{opacity:0}}@keyframes up9{from{opacity:0;transform:translateY(26px) scale(.96)}}\n"
  "/* monitor illustration */\n"
  ".mon .ecg{stroke-dasharray:100;animation:ecg9 2.2s ease-in-out infinite}\n"
  "@keyframes ecg9{0%{stroke-dashoffset:100}55%{stroke-dashoffset:0}100%{stroke-dashoffset:-100}}\n"
  ".mon .scan{animation:scan9 2.6s ease-in-out infinite}@keyframes scan9{0%{transform:translateY(-30px)}100%{transform:translateY(112px)}}\n"
  ".mon .led{animation:pr 1.6s ease-in-out infinite}\n"
  "/* boot */\n"
  "#boot{position:fixed;inset:0;z-index:100;display:flex;align-items:center;justify-content:center;background:var(--bg);transition:opacity .6s var(--ease),transform .6s var(--ease)}\n"
  "#boot.out{opacity:0;transform:scale(1.03);pointer-events:none}\n"
  ".bwrap{width:min(340px,86vw);text-align:center}\n"
  ".bwrap .mon{width:190px;height:auto;margin:0 auto 22px;display:block;filter:drop-shadow(0 18px 30px rgba(0,0,0,.25))}\n"
  ".bwrap h1{font-size:26px;letter-spacing:-.6px;font-weight:700}\n"
  ".bwrap p{color:var(--mut);font-size:14px;margin:6px 0 18px;min-height:20px}\n"
  ".bar,.cbar{height:5px;border-radius:5px;background:var(--fill);overflow:hidden}\n"
  ".bar i,.cbar i{display:block;height:100%;width:0;border-radius:5px;background:linear-gradient(90deg,#0a84ff,#5e5ce6);transition:width .7s var(--ease)}\n"
  ".bsteps{list-style:none;margin-top:18px;text-align:left;display:grid;gap:9px}\n"
  ".bsteps li{display:flex;align-items:center;gap:10px;font-size:13.5px;color:var(--mut);opacity:.45;transition:opacity .4s,color .4s}\n"
  ".bsteps li.run,.bsteps li.done{opacity:1;color:var(--ink)}\n"
  ".bsteps .tk{width:18px;height:18px;border-radius:50%;box-shadow:inset 0 0 0 1.5px var(--line);flex:none;position:relative}\n"
  ".bsteps .run .tk{box-shadow:none;border:2px solid var(--fill);border-top-color:var(--blue);animation:spin9 .8s linear infinite}\n"
  ".bsteps .done .tk{background:var(--green);box-shadow:none}\n"
  ".bsteps .done .tk:after{content:\"\";position:absolute;left:5.5px;top:2.5px;width:4px;height:8px;border:solid #fff;border-width:0 2px 2px 0;transform:rotate(45deg)}\n"
  "/* calibration popup */\n"
  "#calm{position:fixed;inset:0;z-index:97;display:none;align-items:center;justify-content:center;padding:16px;background:rgba(0,0,0,.42);-webkit-backdrop-filter:blur(14px);backdrop-filter:blur(14px)}\n"
  "#calm.show{display:flex;animation:fi9 .3s}\n"
  ".csheet{width:min(430px,100%);max-height:94vh;overflow:auto;background:var(--solid);border-radius:32px;padding:22px 20px calc(18px + env(safe-area-inset-bottom));box-shadow:0 30px 80px rgba(0,0,0,.45);text-align:center;animation:up9 .5s var(--ease)}\n"
  ".csheet .mon{width:124px;height:auto;margin:0 auto 6px;display:block}\n"
  ".csheet h2{font-size:24px;letter-spacing:-.6px}.csheet p{color:var(--mut);font-size:14px;margin:4px 0 14px}\n"
  ".cbar{margin:0 6px 14px}\n"
  ".cl{list-style:none;text-align:left;background:var(--fill);border-radius:18px;overflow:hidden}\n"
  ".cl li{display:flex;align-items:center;gap:12px;padding:13px 14px;border-top:.5px solid var(--line)}.cl li:first-child{border-top:0}\n"
  ".cic{width:30px;height:30px;flex:none}.cic svg{width:30px;height:30px;overflow:visible}\n"
  ".cring{fill:none;stroke:var(--line);stroke-width:2;transition:stroke .3s,fill .3s;transform-origin:12px 12px}\n"
  ".ctick{fill:none;stroke:#fff;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round;stroke-dasharray:20;stroke-dashoffset:20;transition:stroke-dashoffset .5s .12s var(--ease)}\n"
  ".cl li.run .cring{stroke:var(--blue);stroke-dasharray:22 42;animation:spin9 .9s linear infinite}\n"
  ".cl li.ok .cring{fill:var(--green);stroke:var(--green)}.cl li.ok .ctick{stroke-dashoffset:0}\n"
  ".cl li.need .cring{stroke:var(--orange)}\n"
  ".ct{flex:1;min-width:0}.ct b{display:block;font-size:15px}.ct span{display:block;color:var(--mut);font-size:12.5px}\n"
  ".cbtn{padding:8px 14px;border-radius:999px;background:var(--blue);color:#fff;font-weight:600;font-size:13px;white-space:nowrap}.cbtn[disabled]{opacity:.4}\n"
  ".csheet .btn{width:100%;margin-top:10px;min-height:50px}\n"
  "/* ===== v10 link animation, circuit icons, slider ===== */\n"
  "@keyframes show9{to{opacity:1}}@keyframes inL{from{transform:translateX(-150px);opacity:0}to{transform:none;opacity:1}}@keyframes inR{from{transform:translateX(150px);opacity:0}to{transform:none;opacity:1}}\n"
  "@keyframes pulse9{0%{opacity:1;stroke-dashoffset:14}100%{opacity:1;stroke-dashoffset:-100}}@keyframes dash9{to{stroke-dashoffset:-22}}\n"
  ".lnk{width:100%;max-width:340px;height:auto;margin:0 auto 18px;display:block;overflow:visible}\n"
  ".lnk .cL{animation:inL 1.5s var(--ease) both}.lnk .cR{animation:inR 1.5s var(--ease) both}\n"
  ".lnk .lw{fill:none;stroke:var(--line);stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round;opacity:0;animation:show9 .6s 1.3s forwards}\n"
  ".lnk .lp{fill:none;stroke-width:3.4;stroke-linecap:round;stroke-linejoin:round;stroke-dasharray:14 86;stroke-dashoffset:14;opacity:0;animation:pulse9 1.8s linear 1.7s infinite}\n"
  ".lnk.linked .lw{stroke:var(--green)}\n"
  ".lnk .lt{font:600 9px -apple-system,Helvetica,sans-serif;fill:var(--mut);text-anchor:middle;opacity:0;animation:show9 .6s 1.6s forwards}\n"
  ".circw{width:140px;margin:4px auto 10px}.cmini{width:54px;flex:none}\n"
  ".c9{width:100%;height:auto;overflow:visible}\n"
  ".c9 .c9b{fill:var(--fill);stroke:var(--mut);stroke-width:2}.c9 .c9s{fill:none;stroke:var(--mut);stroke-width:2;stroke-linecap:round;stroke-linejoin:round}\n"
  ".c9 .c9t{fill:none;stroke:var(--mut);stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round;opacity:.75}\n"
  ".c9 .c9p{fill:var(--solid);stroke:var(--mut);stroke-width:2}.c9 .c9l{fill:var(--mut)}.c9 .c9x{fill:none;stroke:var(--red);stroke-width:2.8;stroke-linecap:round}\n"
  ".st-pass .c9b,.st-pass .c9s,.st-pass .c9t,.st-pass .c9p{stroke:var(--green)}.st-pass .c9p,.st-pass .c9l{fill:var(--green)}.st-pass .c9l{filter:drop-shadow(0 0 4px var(--green))}\n"
  ".st-fail .c9b,.st-fail .c9s,.st-fail .c9p{stroke:var(--red)}.st-fail .c9t{stroke:var(--red);stroke-dasharray:3 5}.st-fail .c9l{fill:var(--red)}\n"
  ".st-run .c9t{stroke:var(--blue);stroke-dasharray:5 6;animation:dash9 .7s linear infinite;opacity:1}.st-run .c9b,.st-run .c9s{stroke:var(--blue)}.st-run .c9l{fill:var(--blue);animation:pr 1s infinite}\n"
  ".prog i{height:4px}.prog i.cur{box-shadow:0 0 10px var(--blue)}\n"
  "#vAct.wrap{flex-wrap:wrap}.slw{flex:1 1 100%;text-align:left;background:var(--fill);border-radius:16px;padding:12px 14px}\n"
  ".slh{display:flex;justify-content:space-between;font-size:14px;margin-bottom:10px}.slh span{font-weight:700;color:var(--blue)}\n"
  ".slt{position:relative;height:28px}.slt i{position:absolute;top:-2px;width:3px;height:32px;border-radius:2px;background:var(--ink);z-index:2;transition:left .6s var(--ease);margin-left:-1px}\n"
  ".slt input{-webkit-appearance:none;appearance:none;position:absolute;inset:0;width:100%;height:28px;background:none;margin:0}\n"
  ".slt input::-webkit-slider-runnable-track{height:8px;border-radius:8px;background:linear-gradient(90deg,#30d158,#ffd60a 55%,#ff3b30)}\n"
  ".slt input::-moz-range-track{height:8px;border-radius:8px;background:linear-gradient(90deg,#30d158,#ffd60a 55%,#ff3b30)}\n"
  ".slt input::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;margin-top:-9px;border-radius:50%;background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35)}\n"
  ".slt input::-moz-range-thumb{width:26px;height:26px;border:0;border-radius:50%;background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35)}\n"
  ".sls{display:flex;justify-content:space-between;font-size:11.5px;color:var(--mut);margin-top:6px}\n"
  "/* ===== v11 tap, pop-out cards, alerts ===== */\n"
  ".lnk .bd{stroke:#8e8e93;stroke-width:1.5;transition:stroke .9s}.lnk .bd.mega{fill:#334a70}.lnk .bd.esp{fill:#246b68}.lnk.linked .bd{stroke:#2fb457;stroke-width:2.7}.lnk .led{fill:#d1d1d6;transition:fill .9s}.lnk.linked .led{fill:#b8ffd0}\n"
  ".lnk .lp{stroke:#a1a1a8;transition:stroke .9s}.lnk.linked .lp{stroke:var(--green)}\n"
  "#pipe{height:130px}.tapm{fill:url(#tpg);stroke:var(--line);stroke-width:1}.taph{fill:var(--blue)}\n"
  ".tapH{transform-origin:124px 9px;transition:transform .7s var(--ease)}.run .tapH{transform:rotate(90deg)}\n"
  ".basin{fill:var(--fill);stroke:var(--line)}.bw{fill:#0a84ff;opacity:0;transition:opacity .6s}.run .bw{opacity:.45}\n"
  ".wsb{stroke:rgba(10,132,255,.28);stroke-width:7;stroke-linecap:round;opacity:0;transition:opacity .4s}\n"
  ".wst{stroke:#0a84ff;stroke-width:5;stroke-linecap:round;stroke-dasharray:7 9;opacity:0;animation:fall9 .45s linear infinite}\n"
  ".run .wsb,.run .wst{opacity:1}@keyframes fall9{to{stroke-dashoffset:-16}}\n"
  ".rip{fill:none;stroke:#0a84ff;stroke-width:1.6;opacity:0;transform-box:fill-box;transform-origin:center}\n"
  ".run .rip{animation:rip9 1.3s ease-out infinite}.run .rip.r2{animation-delay:.65s}@keyframes rip9{0%{opacity:.9;transform:scale(.4)}100%{opacity:0;transform:scale(1.9)}}\n"
  "#app .card{cursor:zoom-in}#app .card button,#app .card input,#app .card a{cursor:pointer}\n"
  "#cbd{position:fixed;inset:0;z-index:88;background:rgba(0,0,0,.42);-webkit-backdrop-filter:blur(10px);backdrop-filter:blur(10px);opacity:0;pointer-events:none;transition:opacity .3s}#cbd.show{opacity:1;pointer-events:auto}\n"
  ".card.pout{position:fixed!important;top:50%;left:50%;width:min(580px,94vw);max-height:86vh;overflow:auto;z-index:89;margin:0!important;background:var(--solid);cursor:default!important;animation:pop9 .38s var(--ease) both;box-shadow:0 30px 80px rgba(0,0,0,.45);padding:22px}\n"
  "@keyframes pop9{from{opacity:0;transform:translate(-50%,-47%) scale(.9)}to{opacity:1;transform:translate(-50%,-50%) scale(1)}}\n"
  ".card.pout .big{font-size:50px}.card.pout canvas.ch{height:260px}.card.pout #airCv{height:260px}\n"
  ".cx{position:absolute;top:12px;right:12px;width:32px;height:32px;border-radius:50%;background:var(--fill);font-weight:700;z-index:7}\n"
  "#alp{position:fixed;inset:0;z-index:96;display:none;align-items:center;justify-content:center;padding:18px;background:rgba(0,0,0,.52);-webkit-backdrop-filter:blur(14px);backdrop-filter:blur(14px)}\n"
  "#alp.show{display:flex;animation:fi9 .25s}\n"
  ".asheet{width:min(440px,100%);background:var(--solid);border-radius:34px;padding:30px 22px calc(22px + env(safe-area-inset-bottom));text-align:center;box-shadow:0 30px 80px rgba(0,0,0,.5);animation:up9 .5s var(--ease)}\n"
  ".aic{width:96px;height:96px;margin:0 auto 14px;border-radius:50%;display:grid;place-items:center;color:#fff;background:var(--blue);animation:pr 1.6s ease-in-out infinite}\n"
  ".aic svg{width:50px;height:50px;fill:none;stroke:currentColor;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}\n"
  ".asheet.crit .aic{background:var(--red)}.asheet.warn .aic{background:var(--orange)}\n"
  ".asheet h2{font-size:28px;letter-spacing:-.8px;line-height:1.1}.asheet p{color:var(--mut);font-size:15px;margin:8px 0 12px}\n"
  ".aw{display:inline-block;padding:6px 14px;border-radius:999px;background:var(--fill);font-size:12.5px;font-weight:600;margin-bottom:6px}\n"
  ".asheet .btn{width:100%;margin-top:8px;min-height:54px;font-size:16px}\n"
  "</style>\n"
  "</head>\n"
  "<body>\n"
  "<section id=\"boot\"><div class=\"bwrap\"><div id=\"bootMon\"></div><h1>Restroom Intelligence</h1><p id=\"bootS\">Starting…</p><div class=\"bar\"><i id=\"bootB\"></i></div><ul class=\"bsteps\" id=\"bootL\"></ul></div></section>\n"
  "<div id=\"app\" class=\"hidden sk\">\n"
  "  <div class=\"top\"><div><h1 id=\"vt\">Home</h1><small id=\"ts\">–</small></div>\n"
  "    <div class=\"hr\"><button id=\"themeBtn\" aria-label=\"Light or dark mode\">☾</button></div></div>\n"
  "  <div id=\"banner\" class=\"hidden\"></div>\n"
  "\n"
  "  <section id=\"v-home\">\n"
  "    <div class=\"strip\" id=\"strip\"></div>\n"
  "    <div id=\"alerts\"></div>\n"
  "    <div class=\"card hero\" id=\"cHero\">\n"
  "      <div class=\"ring\"><svg viewBox=\"0 0 120 120\"><circle class=\"tr\" cx=\"60\" cy=\"60\" r=\"52\" fill=\"none\" stroke-width=\"10\"/><circle class=\"pg\" id=\"rg\" cx=\"60\" cy=\"60\" r=\"52\" fill=\"none\" stroke-width=\"10\" stroke-linecap=\"round\" stroke-dasharray=\"326.7\" stroke-dashoffset=\"326.7\"/></svg><div class=\"rn\"><b id=\"sc\">–</b><span>of 100</span></div></div>\n"
  "      <div class=\"hm\"><h3 style=\"margin-bottom:6px\">Cleanliness <span class=\"chip\" id=\"scChip\">–</span></h3>\n"
  "        <div class=\"m\" id=\"nextT\">–</div><div class=\"sub\" id=\"nextS\" style=\"margin-bottom:6px\"></div>\n"
  "        <div id=\"why\"></div>\n"
  "        <button class=\"btn grn\" id=\"btnClean2\" style=\"width:100%;margin-top:14px\">Cleaned ✓</button></div>\n"
  "    </div>\n"
  "    <div class=\"card\" id=\"cPred\"><h3>Predictions <span class=\"chip\">live</span></h3><div class=\"tiles\">\n"
  "      <div class=\"tile\" id=\"tNext\"><span>Next clean</span><b id=\"pNext\">–</b><i id=\"pNextS\">&nbsp;</i></div>\n"
  "      <div class=\"tile\" id=\"tPeak\"><span>Expected peak</span><b id=\"pPeak\">–</b><i id=\"pPeakS\">&nbsp;</i></div>\n"
  "      <div class=\"tile\" id=\"tAir\"><span>Air in 10 min</span><b id=\"pAir\">–</b><i id=\"pAirS\">&nbsp;</i></div>\n"
  "      <div class=\"tile\" id=\"tWat\"><span>Water tank</span><b id=\"pWat\">–</b><i id=\"pWatS\">&nbsp;</i></div></div></div>\n"
  "    <div class=\"grid\" style=\"margin-top:14px\">\n"
  "      <div class=\"card\" id=\"gEnt\" data-na><h3>Entries per hour <span class=\"chip\">12 h</span></h3><canvas class=\"ch\" id=\"chEnt\"></canvas><ul class=\"ins\" id=\"insEnt\"></ul></div>\n"
  "      <div class=\"card\" id=\"gAir\" data-na><h3>Air trend <span class=\"chip\">10 min</span></h3><canvas class=\"ch\" id=\"chAir\"></canvas><ul class=\"ins\" id=\"insAir\"></ul></div>\n"
  "      <div class=\"card\" id=\"gFlow\" data-na><h3>Flow trend <span class=\"chip\">10 min</span></h3><canvas class=\"ch\" id=\"chFlow\"></canvas><ul class=\"ins\" id=\"insFlow\"></ul></div>\n"
  "    </div>\n"
  "    <div class=\"grid\">\n"
  "      <div class=\"card\" id=\"cFan\" data-na><h3>Exhaust fan <span class=\"chip\" id=\"fanChip\">–</span></h3>\n"
  "        <div class=\"fr\"><svg id=\"fanSvg\" viewBox=\"0 0 120 120\"><circle cx=\"60\" cy=\"60\" r=\"56\" fill=\"none\" stroke=\"var(--line)\" stroke-width=\"3\"/><g id=\"blades\"><path class=\"fb\" d=\"M60 60C52 38 56 14 70 14C84 18 78 44 60 60Z\" opacity=\".9\"/><path class=\"fb\" d=\"M60 60C82 52 106 56 106 70C102 84 76 78 60 60Z\" opacity=\".9\"/><path class=\"fb\" d=\"M60 60C68 82 64 106 50 106C36 102 42 76 60 60Z\" opacity=\".9\"/><path class=\"fb\" d=\"M60 60C38 68 14 64 14 50C18 36 44 42 60 60Z\" opacity=\".9\"/></g><circle cx=\"60\" cy=\"60\" r=\"9\" fill=\"var(--solid)\" stroke=\"var(--line)\" stroke-width=\"2\"/></svg>\n"
  "          <div style=\"flex:1\"><div class=\"state\" id=\"fanState\">–</div><div class=\"sub\" id=\"fanSub\"></div></div></div>\n"
  "        <div class=\"seg\" style=\"margin-top:14px\" id=\"modeSeg\"><button id=\"mAuto\">AUTO</button><button id=\"mMan\">MANUAL</button></div>\n"
  "        <div class=\"fbtn\" id=\"fanBtns\"><button class=\"btn grn\" id=\"fOn\">Fan ON</button><button class=\"btn red\" id=\"fOff\">Fan OFF</button></div>\n"
  "        <div class=\"sub\" id=\"fanHint\" style=\"margin-top:10px\">AUTO: the ATmega runs the fan from air quality. Choose MANUAL to switch it yourself.</div></div>\n"
  "      <div class=\"card\" id=\"cAir\" data-na><h3>Air quality <span class=\"chip\" id=\"airChip\">–</span></h3><canvas id=\"airCv\"></canvas><div class=\"kv\"><span>MQ-135 raw (relative index, not ppm)</span><b id=\"airRaw\">–</b></div></div>\n"
  "    </div>\n"
  "    <div class=\"grid\">\n"
  "      <div class=\"card\" id=\"cTank\" data-na><h3>Water tank <span class=\"chip\" id=\"tankChip\">–</span></h3>\n"
  "        <div class=\"tw\"><svg id=\"tk\" viewBox=\"0 0 120 160\" class=\"dry\"><defs><clipPath id=\"tkc\"><rect x=\"14\" y=\"14\" width=\"92\" height=\"132\" rx=\"14\"/></clipPath></defs><rect class=\"tko\" x=\"14\" y=\"14\" width=\"92\" height=\"132\" rx=\"14\"/><g clip-path=\"url(#tkc)\"><g id=\"wfill\" style=\"transform:translateY(160px)\"><g class=\"wv2\"><path class=\"wf\" id=\"wp2\"/></g><g class=\"wv\"><path class=\"wf\" id=\"wp1\"/></g><rect class=\"wf\" x=\"0\" y=\"8\" width=\"400\" height=\"200\"/></g></g></svg>\n"
  "          <div><div class=\"state\" id=\"tankState\">–</div><div class=\"sub\" id=\"tankSub\"></div></div></div>\n"
  "        <div class=\"kv\" style=\"margin-top:12px\"><span>Probe raw</span><b id=\"wRaw\">–</b></div><div class=\"kv\"><span>Level</span><b id=\"wLvl\">–</b></div></div>\n"
  "      <div class=\"card\" id=\"cFlow\" data-na><h3>Water flow <span class=\"chip\" id=\"flowChip\">–</span></h3>\n"
  "<svg id=\"pipe\" viewBox=\"0 0 300 130\" role=\"img\" aria-label=\"Water tap\"><defs><linearGradient id=\"tpg\" x1=\"0\" y1=\"0\" x2=\"0\" y2=\"1\"><stop offset=\"0\" stop-color=\"#dcdee4\"/><stop offset=\"1\" stop-color=\"#9a9eaa\"/></linearGradient></defs><ellipse class=\"basin\" cx=\"181\" cy=\"116\" rx=\"54\" ry=\"9\"/><ellipse class=\"bw\" cx=\"181\" cy=\"116\" rx=\"40\" ry=\"6\"/><ellipse class=\"rip\" cx=\"181\" cy=\"115\" rx=\"14\" ry=\"4\"/><ellipse class=\"rip r2\" cx=\"181\" cy=\"115\" rx=\"14\" ry=\"4\"/><rect class=\"tapm\" x=\"10\" y=\"30\" width=\"104\" height=\"18\" rx=\"7\"/><rect class=\"tapm\" x=\"104\" y=\"20\" width=\"44\" height=\"40\" rx=\"10\"/><path class=\"tapm\" d=\"M146 30h30a14 14 0 0 1 14 14v22h-18V48h-26z\"/><line class=\"wsb\" x1=\"181\" y1=\"66\" x2=\"181\" y2=\"113\"/><line class=\"wst\" id=\"pf\" x1=\"181\" y1=\"68\" x2=\"181\" y2=\"112\"/><rect class=\"tapm\" x=\"120\" y=\"11\" width=\"8\" height=\"10\"/><g class=\"tapH\"><rect class=\"taph\" x=\"100\" y=\"5\" width=\"48\" height=\"8\" rx=\"4\"/></g></svg>\n"
  "        <div style=\"display:flex;align-items:baseline;gap:8px\"><div class=\"big\" id=\"flowV\">0.00</div><div class=\"sub\" style=\"margin:0\">L/min</div></div>\n"
  "        <div class=\"kv\" style=\"margin-top:12px\"><span>Total</span><b id=\"flowT\">–</b></div><div class=\"kv\"><span>Flow calibration</span><b id=\"flowCs\">–</b></div></div>\n"
  "      <div class=\"card\" id=\"cDoor\" data-na><h3>Door · people <span class=\"chip\" id=\"doorChip\">–</span></h3>\n"
  "        <div style=\"display:flex;align-items:baseline;gap:8px\"><div class=\"big\" id=\"ppl\">–</div><div class=\"sub\" style=\"margin:0\">entries counted</div></div>\n"
  "        <div class=\"kv\" style=\"margin-top:12px\"><span>Distance (HC-SR04)</span><b id=\"dist\">–</b></div><div class=\"kv\"><span>Door calibration</span><b id=\"doorCalState\">not calibrated</b></div><div class=\"kv\"><span>Today / last hour</span><b id=\"eTH\">–</b></div><div class=\"kv\"><span>Since last clean</span><b id=\"eSince\">–</b></div><div class=\"kv\"><span>Door cycles</span><b id=\"dcyc\">–</b></div></div>\n"
  "    </div>\n"
  "  </section>\n"
  "\n"
  "  <section id=\"v-ins\" class=\"hidden\">\n"
  "    <div class=\"note\" id=\"insNote\">All figures are computed in your browser from the last 24 h of 5-minute summaries. The air value is a relative MQ-135 index, not ppm or a real odour measurement; entries are door-sensor events, not verified people. Correlation is shown only with 30+ samples.</div>\n"
  "    <div class=\"grid\">\n"
  "      <div class=\"card\"><h3>Entries vs air <span class=\"chip\" id=\"corChip\">–</span></h3><canvas class=\"ch\" id=\"cScat\" style=\"height:210px\"></canvas><ul class=\"ins\" id=\"insCor\"></ul></div>\n"
  "      <div class=\"card\"><h3>Busy hours <span class=\"chip\">weekday × hour</span></h3><canvas class=\"ch\" id=\"cHeat\" style=\"height:210px\"></canvas><ul class=\"ins\" id=\"insHeat\"></ul></div>\n"
  "      <div class=\"card\"><h3>Entries vs water use <span class=\"chip\" id=\"watChip\">–</span></h3><canvas class=\"ch\" id=\"cWat\" style=\"height:210px\"></canvas><ul class=\"ins\" id=\"insWat\"></ul></div>\n"
  "      <div class=\"card\"><h3>Fan effectiveness <span class=\"chip\" id=\"fanC\">–</span></h3><div class=\"tiles\" id=\"fanT\"></div><ul class=\"ins\" id=\"insFan\"></ul></div>\n"
  "      <div class=\"card\"><h3>Cleaning history <span class=\"chip\" id=\"clChip\">–</span></h3><div id=\"clHist\"></div></div>\n"
  "      <div class=\"card\"><h3>Entries in a time window</h3><div class=\"chips\" id=\"win\"><button data-w=\"15\">15 min</button><button data-w=\"60\" class=\"on\">1 hour</button><button data-w=\"180\">3 hours</button><button data-w=\"today\">Today</button><button data-w=\"cust\">Custom</button></div>\n"
  "        <div class=\"row2 hidden\" id=\"cust\" style=\"align-items:center\"><input type=\"time\" id=\"tFrom\" value=\"09:00\"><input type=\"time\" id=\"tTo\" value=\"12:00\"></div>\n"
  "        <div class=\"res\"><b id=\"winN\">–</b><span>entries</span></div></div>\n"
  "    </div>\n"
  "    <div class=\"row2\"><button class=\"btn pri\" id=\"btnPrint\">Print daily report</button></div>\n"
  "  </section>\n"
  "\n"
  "  <section id=\"v-cal\" class=\"hidden\">\n"
  "    <div class=\"card\" style=\"margin-bottom:14px\"><h3>Cleaning rule <span class=\"chip\" id=\"ruleChip\">–</span></h3>\n"
  "      <div class=\"sub\" style=\"margin:-4px 0 6px\">Stored on the ESP. Adaptive mode tightens it when the air stays high (entries ×0.6, time ×0.5).</div>\n"
  "      <div class=\"lr\"><div class=\"t\"><b>Clean after</b><span>entries since last clean</span></div><input type=\"number\" id=\"rE\" min=\"5\" max=\"200\" step=\"1\"></div>\n"
  "      <div class=\"lr\"><div class=\"t\"><b>or after</b><span>minutes since last clean</span></div><input type=\"number\" id=\"rM\" min=\"20\" max=\"1440\" step=\"5\"></div>\n"
  "      <div class=\"lr\"><div class=\"t\"><b>Adaptive</b><span id=\"rNow\">–</span></div><div class=\"seg\" id=\"rA\" style=\"width:120px\"><button data-a=\"1\">On</button><button data-a=\"0\">Off</button></div></div>\n"
  "      <div class=\"row2\"><button class=\"btn pri\" id=\"rSave\">Save rule</button></div></div>\n"
  "<div class=\"card\"><div class=\"chead\"><div id=\"calMon\"></div><div><h3 style=\"margin:0 0 4px\">Sensor calibration <span class=\"chip\">optional</span></h3><div class=\"sub\">Saved values are kept if you skip.</div></div></div>\n"
  "<button class=\"btn pri\" id=\"bGuide\" style=\"width:100%;margin:14px 0 8px\">Start guided calibration</button>\n"
  "      <div class=\"lr\"><div class=\"ic\" id=\"ciAir\">–</div><div class=\"t\"><b>Air (MQ-135)</b><span id=\"cnAir\">Fresh air, sensor warmed up 5 min, then tap Set.</span></div><button class=\"btn\" data-c=\"CAL,MQ\" id=\"bAir\">Set</button></div>\n"
  "      <div class=\"lr\"><div class=\"ic\" id=\"ciWat\">–</div><div class=\"t\"><b>Water probe</b><span id=\"cnWat\">Probe dry → Set DRY. Probe in water → Set WET.</span></div><div style=\"display:grid;gap:6px\"><button class=\"btn\" data-c=\"CAL,WATER_DRY\">DRY</button><button class=\"btn\" data-c=\"CAL,WATER_WET\">WET</button></div></div>\n"
  "      <div class=\"lr\"><div class=\"ic\" id=\"ciFlow\">–</div><div class=\"t\"><b>Water flow</b><span id=\"cnFlow\">Water off → Zero (60 s). Then Start, pass a measured volume, enter litres, Finish.</span>\n"
  "        <div class=\"row2\"><button class=\"btn\" id=\"fZero\">Zero</button><button class=\"btn\" id=\"fStart\">Start</button></div>\n"
  "        <div class=\"row2\"><input type=\"number\" id=\"litres\" step=\"0.01\" min=\"0.01\" placeholder=\"Litres\"><button class=\"btn pri\" id=\"fFin\">Finish</button></div></div></div>\n"
  "      <div class=\"lr\"><div class=\"ic\" id=\"ciDoor\">–</div><div class=\"t\"><b>Door distance</b><span id=\"cnDoor\">Capture the fully closed and fully open door separately. Hold each position still for 10 seconds; readings must differ by at least 5 cm.</span>\n"
  "        <div class=\"row2\"><button class=\"btn\" id=\"dClosed\">Closed</button><button class=\"btn\" id=\"dOpen\">Open</button></div></div></div>\n"
  "      <div class=\"row2\"><button class=\"btn\" id=\"bSkipCal\">Skip for now</button><button class=\"btn\" id=\"bReset\">Reset flow &amp; door</button></div>\n"
  "      <div class=\"row2\"><button class=\"btn\" id=\"bVer\">Run hardware check</button></div></div>\n"
  "  </section>\n"
  "\n"
  "  <section id=\"v-ver\" class=\"hidden\">\n"
  "    <div class=\"card\"><div style=\"display:flex;justify-content:space-between;align-items:center\"><h3 style=\"margin:0\">Hardware check</h3><span class=\"sub\" id=\"vStep\" style=\"margin:0\">Step 1 of 12</span></div>\n"
  "      <div class=\"prog\" id=\"vProg\"></div>\n"
  "      <div class=\"stage\" id=\"vStage\"><div class=\"bigic\" id=\"vIc\">–</div><h2 id=\"vTitle\"></h2><p id=\"vDesc\"></p><div class=\"note\" id=\"vNote\"></div><div class=\"note hidden\" id=\"vFix\"></div><div class=\"row2\" id=\"vAct\"></div></div>\n"
  "      <div class=\"row2\"><button class=\"btn\" id=\"vBack\">Back</button><button class=\"btn pri\" id=\"vNext\">Next</button></div>\n"
  "      <div class=\"row2\"><button class=\"btn\" id=\"vSkip\">Skip verification</button></div></div>\n"
  "    <div class=\"card hidden\" id=\"vRep\"><h3>Report</h3><div id=\"vRepList\"></div></div>\n"
  "  </section>\n"
  "\n"
  "  <section id=\"v-log\" class=\"hidden\">\n"
  "    <div class=\"grid\">\n"
  "      <div class=\"card\"><h3>MicroSD log <span class=\"chip\" id=\"sdChip\">–</span></h3><div class=\"kv\"><span>Status</span><b id=\"sdMsg\">–</b></div><div class=\"kv\"><span>Waiting in RAM</span><b id=\"sdQ\">–</b></div><div class=\"kv\"><span>Frames ok / bad / CRC</span><b id=\"fr\">–</b></div><div class=\"kv\"><span>Free heap · uptime</span><b id=\"hp\">–</b></div>\n"
  "        <div class=\"dl\"><a href=\"/download?f=data\" download><span>⬇ datalast.csv</span><small id=\"sz-data\">–</small></a><a href=\"/download?f=entries\" download><span>⬇ entrlast.csv</span><small id=\"sz-entries\">–</small></a><a href=\"/download?f=events\" download><span>⬇ evntlast.csv</span><small id=\"sz-events\">–</small></a><a href=\"/download?f=commands\" download><span>⬇ cmdslast.csv</span><small id=\"sz-commands\">–</small></a><a href=\"/download?f=buckets\" download style=\"grid-column:1/-1\"><span>⬇ daily summary (5-min buckets)</span><small id=\"sz-buckets\">–</small></a></div></div>\n"
  "      <div class=\"card\"><h3>Export a date range</h3>\n"
  "        <div class=\"chips\" id=\"exQ\"><button data-q=\"0\" class=\"on\">Today</button><button data-q=\"1\">Yesterday</button><button data-q=\"7\">Last 7 days</button></div>\n"
  "        <div class=\"row2\" style=\"margin-top:10px\"><select id=\"exF\"><option value=\"buckets\">Daily summary</option><option value=\"entries\">Entries</option><option value=\"events\">Events</option><option value=\"commands\">Commands</option><option value=\"data\">Telemetry (large)</option></select></div>\n"
  "        <div class=\"row2\"><input type=\"date\" id=\"exFrom\"><input type=\"date\" id=\"exTo\"></div>\n"
  "        <div class=\"row2\"><button class=\"btn pri\" id=\"exGo\">Download range</button></div>\n"
  "        <div class=\"note\">Dates use the controller’s RTC clock. Telemetry is ~15 MB per day: a one-day export takes a few minutes and pauses SD logging while it runs (records wait in RAM).</div></div>\n"
  "    </div>\n"
  "    <div class=\"card\"><h3>Raw sensor data <span class=\"chip\" id=\"rawN\">live</span></h3><div id=\"raw\">waiting…</div></div>\n"
  "  </section>\n"
  "</div>\n"
  "<nav class=\"tabs hidden\" id=\"tabs\">\n"
  "  <button data-v=\"home\" class=\"on\"><svg viewBox=\"0 0 24 24\"><path d=\"M3 11l9-8 9 8v9a1 1 0 01-1 1h-5v-6H9v6H4a1 1 0 01-1-1z\"/></svg>Home</button>\n"
  "  <button data-v=\"ins\"><svg viewBox=\"0 0 24 24\"><path d=\"M3 17l5-5 4 3 8-9\"/><path d=\"M15 6h5v5\"/></svg>Insights</button>\n"
  "  <button data-v=\"cal\"><svg viewBox=\"0 0 24 24\"><path d=\"M4 6h10M18 6h2M4 12h2M10 12h10M4 18h12M20 18h0\"/><circle cx=\"16\" cy=\"6\" r=\"2\"/><circle cx=\"8\" cy=\"12\" r=\"2\"/><circle cx=\"18\" cy=\"18\" r=\"2\"/></svg>Calibrate</button>\n"
  "  <button data-v=\"ver\"><svg viewBox=\"0 0 24 24\"><path d=\"M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6z\"/><path d=\"M8.5 12l2.5 2.5L16 9.5\"/></svg>Verify</button>\n"
  "  <button data-v=\"log\"><svg viewBox=\"0 0 24 24\"><ellipse cx=\"12\" cy=\"6\" rx=\"8\" ry=\"3\"/><path d=\"M4 6v12c0 1.7 3.6 3 8 3s8-1.3 8-3V6M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3\"/></svg>Data</button>\n"
  "</nav>\n"
  "<div id=\"cbd\"></div>\n"
  "<div id=\"alp\"><div class=\"asheet\" id=\"asheet\"><div class=\"aic\" id=\"aIc\"></div><h2 id=\"aT\"></h2><p id=\"aP\"></p><div class=\"aw hidden\" id=\"aW\"></div><div id=\"aB\"></div></div></div>\n"
  "<div id=\"calm\"><div class=\"csheet\" role=\"dialog\" aria-modal=\"true\" aria-labelledby=\"cmT\"><div id=\"cmMon\"></div><h2 id=\"cmT\">Calibration</h2><p id=\"cmP\">Checking your sensors…</p><div class=\"cbar\"><i id=\"cmB\"></i></div><ul class=\"cl\" id=\"cmL\"></ul><button class=\"btn pri hidden\" id=\"cmGo\">Done</button><button class=\"btn flat\" id=\"cmSkip\">Skip for now</button></div></div>\n"
  "<div id=\"pop\"><div class=\"sheet\" id=\"sheet\"><div class=\"pi\">🧹</div><h2 id=\"popT\">Cleaning due</h2><p id=\"popP\"></p><div class=\"why\" id=\"popW\"></div><button class=\"btn grn\" id=\"popOk\">Cleaned ✓</button><button class=\"btn\" id=\"popSz\">Snooze 10 min</button></div></div>\n"
  "<div id=\"rep\"></div>\n"
  "<div id=\"toast\"></div>\n"
  "<script>\n"
  "\"use strict\";\n"
  "const $=id=>document.getElementById(id),S={d:null,up:false,fails:0,hist:null,win:'60',wait:-1,rs:-1,raw:[],view:'home',step:0,ppl0:null,res:[],fs:0,fa:0},pad=n=>(n<10?'0':'')+n,avg=a=>a.length?a.reduce((x,y)=>x+y,0)/a.length:0;\n"
  "const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();\n"
  "function toast(m,ms){const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(toast.h);toast.h=setTimeout(()=>t.classList.remove('show'),ms||2600);}\n"
  "async function jget(u,ms){const c=new AbortController(),h=setTimeout(()=>c.abort(),ms||3000);try{const r=await fetch(u,{cache:'no-store',signal:c.signal});if(!r.ok)throw 0;return await r.json();}finally{clearTimeout(h);}}\n"
  "async function ctl(u){const c=new AbortController(),h=setTimeout(()=>c.abort(),4000);try{const r=await fetch(u,{cache:'no-store',signal:c.signal});let b={};try{b=await r.json();}catch(e){}return{st:r.status,b};}catch(e){return{st:0,b:{error:'No response from the ESP8266'}};}finally{clearTimeout(h);}}\n"
  "/* theme */\n"
  "function setTheme(t){document.documentElement.setAttribute('data-theme',t);try{localStorage.setItem('ri-theme',t);}catch(e){}$('themeBtn').textContent=t==='dark'?'☀︎':'☾';try{drawCharts();}catch(e){}}\n"
  "$('themeBtn').onclick=()=>setTheme(document.documentElement.getAttribute('data-theme')==='dark'?'light':'dark');\n"
  "$('themeBtn').textContent=document.documentElement.getAttribute('data-theme')==='dark'?'☀︎':'☾';\n"
  "/* navigation */\n"
  "const TITLES={home:'Home',ins:'Insights',cal:'Calibrate',ver:'Verify',log:'Data'};\n"
  "function nav(v){S.view=v;for(const k in TITLES)$('v-'+k).classList.toggle('hidden',k!==v);document.querySelectorAll('#tabs button').forEach(b=>b.classList.toggle('on',b.dataset.v===v));$('vt').textContent=TITLES[v];window.scrollTo(0,0);if(v==='home'){loadHist();loadAn();}if(v==='ins'){loadAn();qEnt();}if(v==='log')loadFiles();render();}\n"
  "document.querySelectorAll('#tabs button').forEach(b=>b.onclick=()=>nav(b.dataset.v));\n"
  "/* overlays */\n"
  "document.querySelectorAll('[data-na]').forEach(c=>{const o=document.createElement('div');o.className='na';o.innerHTML='<b>Function not available</b><p></p>';c.appendChild(o);});\n"
  "function setNA(id,m){const c=$(id);c.classList.toggle('bad',!!m);if(m)c.querySelector('.na p').textContent=m;}\n"
  "function avail(){const d=S.d,r={fan:'',air:'',tank:'',flow:'',door:'',gEnt:'',gAir:'',gFlow:''};\n"
  " if(!S.up||!d){const m='Lost connection to the ESP8266 (Wi-Fi).';for(const k in r)r[k]=m;return r;}\n"
  " if(!d.link){const m='No data from the ATmega (UART link lost).';for(const k in r)r[k]=m;return r;}\n"
  " const b=d.bits;if(b&1)r.air=r.gAir='MQ-135 air sensor malfunction.';if(b&2)r.tank='Water probe malfunction.';if(b&4)r.door=r.gEnt='Ultrasonic sensor malfunction (no echo).';if(!r.gEnt&&!d.tv)r.gEnt='RTC time not valid.';return r;}\n"
  "/* ---------- polling: slow down automatically when failing, pause when hidden ---------- */\n"
  "async function poll(){let ok=false;if(!document.hidden){try{S.d=await jget('/data',3000);S.fails=0;S.up=true;ok=true;}catch(e){if(++S.fails>=3)S.up=false;}try{render();}catch(e){console.error(e);}}\n"
  " setTimeout(poll,document.hidden?3000:(ok?1500:Math.min(8000,2000*Math.max(1,S.fails))));}\n"
  "async function loadHist(){if(!(S.up&&S.d&&S.d.link))return;try{S.hist=await jget('/history',4000);drawCharts();}catch(e){}qEnt();}\n"
  "/* ---------- render ---------- */\n"
  "const AIRC={GOOD:'--green',MODERATE:'--orange',POOR:'--orange',BAD:'--red'};let airT=0,airS=0,airCol='#8e8e93',airSt='WAITING';\n"
  "const fmtUp=s=>s>=3600?Math.floor(s/3600)+'h '+Math.floor(s%3600/60)+'m':s>=60?Math.floor(s/60)+'m':s+'s';\n"
  "function render(){const d=S.d,A=avail(),link=!!(S.up&&d&&d.link);\n"
  " setNA('cFan',A.fan);setNA('cAir',A.air);setNA('cTank',A.tank);setNA('cFlow',A.flow);setNA('cDoor',A.door);setNA('gEnt',A.gEnt);setNA('gAir',A.gAir);setNA('gFlow',A.gFlow);\n"
  " const bad=[...new Set(Object.values(A).filter(Boolean))],bn=$('banner');if(bad.length){bn.classList.remove('hidden');bn.innerHTML='<b>Some functions are not available</b><br>'+bad.join('<br>');}else bn.classList.add('hidden');\n"
  " if(!d){if(S.view==='ver')wizard();return;}\n"
  " $('app').classList.remove('sk');\n"
  " $('ts').textContent=d.tv?d.ts:'time not set (check RTC)';\n"
  " airT=d.air;airSt=d.airS;airCol=AIRC[d.airS]?css(AIRC[d.airS]):css('--mut');$('airRaw').textContent=d.air;const ac=$('airChip');ac.textContent=d.airS;ac.className='chip '+(d.airS==='GOOD'?'ok':d.airS==='MODERATE'?'warn':(d.airS==='POOR'||d.airS==='BAD'||d.airS==='ERROR')?'bad':'');\n"
  " const wet=!!d.wet;$('tk').setAttribute('class',wet?'wet':'dry');const lv=d.wLvl>=0?d.wLvl/100:(wet?.6:0);$('wfill').style.transform='translateY('+(wet?146-Math.max(.06,lv)*132:170)+'px)';\n"
  " $('tankState').textContent=wet?'WET':'NOT WET';$('tankState').className='state '+(wet?'ok':'bad');$('tankSub').textContent=wet?'Probe is in water':'No water at the probe';$('wRaw').textContent=d.wRaw;$('wLvl').textContent=d.wLvl>=0?d.wLvl.toFixed(0)+' %':'not calibrated';\n"
  " const tc=$('tankChip');tc.textContent=wet?'WATER':'NO WATER';tc.className='chip '+(wet?'ok':'bad');\n"
  " const fl=d.flow,run=fl>0.05;anim('flowV',fl,2);$('flowT').textContent=d.tot.toFixed(2)+' L';$('pipe').setAttribute('class',run?'run':'');$('pf').style.animationDuration=Math.max(.22,1.5/(1+fl*.6))+'s';\n"
  " const fc=$('flowChip');fc.textContent=run?'FLOWING':'NO FLOW';fc.className='chip '+(run?'ok':'');$('flowCs').textContent=d.flowVolumeSet?'calibrated':d.flowZeroSet?'zero set':'not calibrated';\n"
  " anim('ppl',d.ppl,0);$('dist').textContent=d.dist>=0?d.dist.toFixed(1)+' cm':'no echo';$('doorCalState').textContent=d.doorCalRun?'capturing '+(d.doorCalSamples||0)+'/5':d.doorCalReady?'calibrated':'not calibrated';$('doorCalState').title=d.doorCalMsg||'';$('eTH').textContent=(d.tv?d.eTod:'–')+' / '+d.eHr;$('eSince').textContent=d.tv?d.eSince:'–';$('dcyc').textContent=d.doorCycles;\n"
  " const dst=d.doorState===1?'CLOSED':d.doorState===2?'OPEN':'UNKNOWN',dc=$('doorChip');dc.textContent=dst;dc.className='chip '+(d.doorState===1?'ok':d.doorState===2?'warn':'');\n"
  " /* fan */\n"
  " const auto=d.mode==='AUTO',fo=!!d.fan;$('fanState').textContent=fo?'ON':'OFF';$('fanState').className='state '+(fo?'ok':'');$('fanSub').textContent=auto?'Automatic (air quality)':d.mode==='MANUAL'?'Manual control':'Mode unknown';\n"
  " const fch=$('fanChip');fch.textContent=d.mode;fch.className='chip '+(auto?'ok':d.mode==='MANUAL'?'warn':'');$('mAuto').className=auto?'on':'';$('mMan').className=d.mode==='MANUAL'?'on':'';\n"
  " $('fanBtns').classList.toggle('dis',!link||auto);$('fOn').disabled=$('fOff').disabled=!link||auto;$('fanHint').textContent=auto?'AUTO: the ATmega runs the fan from air quality. Choose MANUAL to switch it yourself.':'MANUAL: ON/OFF below control the relay. Choose AUTO to hand control back.';\n"
  " /* alerts */\n"
  " const al=d.alerts||[];$('alerts').innerHTML=al.map(a=>'<div class=\"al l'+(a.level>=2?2:1)+'\"><div><b>'+esc(a.title)+'</b>'+esc(a.detail)+' <i>'+esc(a.action)+'</i></div></div>').join('');\n"
  " /* logs */\n"
  " const sc=$('sdChip');sc.textContent=d.sd?'LOGGING':'OFFLINE';sc.className='chip '+(d.sd?'ok':'bad');$('sdMsg').textContent=d.sdMsg;$('sdQ').textContent=d.q+(d.qDrop?' (dropped '+d.qDrop+')':'');\n"
  " $('fr').textContent=d.ok+' / '+d.bad+' / '+d.crc;$('hp').textContent=Math.round(d.heap/1024*10)/10+' KB · '+fmtUp(d.up);\n"
  " if(d.rs!==S.rs&&d.raw){S.rs=d.rs;const t=new Date();S.raw.push('['+pad(t.getHours())+':'+pad(t.getMinutes())+':'+pad(t.getSeconds())+'] '+d.raw);if(S.raw.length>10)S.raw.shift();if(S.view==='log'){const r=$('raw');r.textContent=S.raw.join('\\n');r.scrollTop=r.scrollHeight;$('rawN').textContent='#'+d.rs;}}\n"
  " if(S.wait>=0&&d.cmdId===S.wait&&d.cmdRes!=='WAITING_FOR_ACK'){const ok=d.cmdRes==='ACKNOWLEDGED';toast(ok?'Done ✓ '+d.lastAck:'Command '+(d.cmdRes==='REJECTED'?'rejected by ATmega':'timed out'));if(ok&&S.lastCmd==='CAL,MQ'){try{localStorage.setItem('ri-air',Date.now());}catch(e){}}S.wait=-1;}\n"
  " calRender(d,link);render5(d,link);if(S.view==='ver')wizard();}\n"
  "function esc(s){return String(s==null?'':s).replace(/[&<>\"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]));}\n"
  "/* ---------- commands ---------- */\n"
  "async function sendCmd(c){S.lastCmd=c;const r=await ctl('/control?cmd='+encodeURIComponent(c));if(r.st===202){S.wait=r.b.id;toast('Sent '+c+' …',1200);return true;}toast(r.b&&r.b.error?r.b.error:'Command failed',3200);return false;}\n"
  "$('mAuto').onclick=()=>sendCmd('MODE,AUTO');$('mMan').onclick=()=>sendCmd('MODE,MANUAL');$('fOn').onclick=()=>sendCmd('FAN,1');$('fOff').onclick=()=>sendCmd('FAN,0');\n"
  "document.querySelectorAll('[data-c]').forEach(b=>b.onclick=()=>sendCmd(b.dataset.c));\n"
  "async function act(u){const r=await ctl(u);toast(r.b.message||r.b.error||(r.st<300?'OK':'Failed'),3200);return r;}\n"
  "/* ---------- calibration (one short note per sensor) ---------- */\n"
  "$('fZero').onclick=()=>act('/flow/zero/start');$('fStart').onclick=()=>act('/flow/volume/start');\n"
  "$('fFin').onclick=()=>{const v=parseFloat($('litres').value);if(!(v>0)){toast('Enter the measured litres first');return;}act('/flow/volume/finish?litres='+v);};\n"
  "$('dClosed').onclick=()=>act('/door/cal/start?pos=closed');$('dOpen').onclick=()=>act('/door/cal/start?pos=open');\n"
  "$('bSkipCal').onclick=()=>{nav('home');toast('Calibration skipped · saved values kept');};\n"
  "$('bReset').onclick=async()=>{if(!confirm('Clear the saved flow and door calibration?'))return;await act('/flow/reset');await act('/door/cal/reset');};\n"
  "function ci(id,s,t){const e=$(id);e.className='ic '+s;e.textContent=t;}\n"
  "function calRender(d,link){if(S.view!=='cal')return;ruleRender(d);['bAir'].forEach(i=>$(i).disabled=!link);document.querySelectorAll('[data-c]').forEach(b=>b.disabled=!link);\n"
  " let air=null;try{air=localStorage.getItem('ri-air');}catch(e){}\n"
  " ci('ciAir',air?'ok':'','✓');$('ciAir').textContent=air?'✓':'–';$('cnAir').textContent=air?'Baseline saved ✓ (set '+new Date(+air).toLocaleDateString()+'). Tap Set to redo.':'Fresh air, sensor warmed up 5 min, then tap Set.';\n"
  " const wok=d.wLvl>=0;ci('ciWat',wok?'ok':'',wok?'✓':'–');$('cnWat').textContent=wok?'DRY and WET saved ✓ · level '+d.wLvl.toFixed(0)+' %':'Probe dry → Set DRY. Probe in water → Set WET.';\n"
  " const fok=d.flowVolumeSet;ci('ciFlow',fok?'ok':d.flowZeroSet?'warn':'',fok?'✓':d.flowZeroSet?'!':'–');$('cnFlow').textContent=d.flowCalMsg||'';\n"
  " ['fZero','fStart','fFin'].forEach(i=>$(i).disabled=!link);const drefs=!!(d.doorClosedSet||d.doorOpenSet),dok=!!d.doorCalReady;ci('ciDoor',dok?'ok':drefs?'warn':'',dok?'✓':drefs?'!':'–');$('cnDoor').textContent=d.doorCalRun?d.doorCalMsg+' · '+(d.doorCalSamples||0)+'/5 fresh readings':dok?'Saved ✓ closed '+d.doorClosed.toFixed(1)+' cm · open '+d.doorOpen.toFixed(1)+' cm':d.doorCalMsg;\n"
  " $('dClosed').disabled=$('dOpen').disabled=!link||!!d.doorCalRun;}\n"
  "/* ---------- 12-step verification ---------- */\n"
  "const NT='nt';\n"
  "const STEPS=[\n"
  " {t:'ESP8266 gateway',d:'Wi-Fi access point and web server.',f:(d)=>S.up?{s:'pass',n:'Responding · free heap '+Math.round(d.heap/1024)+' KB'}:{s:'fail',n:'No response after 3 retries.',x:'Stay connected to “Restroom-Monitor” and check the NodeMCU power.'}},\n"
  " {t:'ATmega328P · UART link',d:'Telemetry frames from the sensor controller.',f:d=>d.link?{s:'pass',n:d.ok+' valid frames'+(d.crc?' · '+d.crc+' CRC errors':'')}:{s:'fail',n:'No frames received.',x:'Check ATmega power, D7→divider→NodeMCU D1, NodeMCU D2→ATmega D8 and a common GND.'}},\n"
  " {t:'MicroSD storage',d:'Card detected and CSV logging works.',f:d=>d.sd?{s:'pass',n:'Ready · '+d.sdMsg}:{s:'fail',n:d.sdMsg,x:'Use a FAT32 card, reseat it, check CS D0/SCK D5/MISO D6/MOSI D7 and module voltage.'}},\n"
  " {t:'DS3231 clock',d:'Real-time clock holds a valid time.',f:d=>(d.bits&8)?{s:'fail',n:'Clock missing or lost power.',x:'Check the DS3231 on A4/A5 and its coin cell, then set the time.'}:!d.tv?{s:'fail',n:'Time not valid yet.',x:'Set the DS3231 time (re-flash the ATmega after correcting the PC clock).'}:{s:'pass',n:d.ts}},\n"
  " {t:'MQ-135 air sensor',d:'ADC responds inside its valid range.',f:d=>(d.bits&1)?{s:'fail',n:'Reading stuck at the ADC limit.',x:'Check MQ-135 power, A0 wiring and allow warm-up.'}:{s:'pass',n:'Raw '+d.air+' · '+d.airS}},\n"
  " {t:'Water probe',d:'Probe ADC responds.',f:d=>(d.bits&2)?{s:'fail',n:'Reading stuck at the ADC limit.',x:'Check probe wiring on A1 and power.'}:{s:'pass',n:'Raw '+d.wRaw+(d.wLvl<0?' · level not calibrated (optional)':'')}},\n"
  " {t:'HC-SR04 ultrasonic',d:'A valid echo distance is reported.',f:d=>(d.bits&4)?{s:'fail',n:'No echo.',x:'Check TRIG D5, ECHO D6, 5 V and that nothing blocks the sensor.'}:{s:'pass',n:d.dist>=0?d.dist.toFixed(1)+' cm':'echo ok'}},\n"
  " {t:'Water flow sensor',d:'Run the 18 s test and open a tap.',a:'flow',f:d=>{const r=d.waterTestResult||'';return d.waterTestRunning?{s:'run',n:r}:/^PASS/.test(r)?{s:'pass',n:r}:/^FAIL/.test(r)?{s:'fail',n:r,x:'Check supply, valve, blockage and the sensor wiring on D3.'}:{s:NT,n:'Not tested yet.'};}},\n"
  " {t:'People counter',d:'Walk past the sensor once.',f:d=>{if(S.ppl0===null)S.ppl0=d.ppl;return d.ppl>S.ppl0?{s:'pass',n:'Counted '+(d.ppl-S.ppl0)+' entry'}:{s:NT,n:'Waiting for one pass · current count '+d.ppl,x:'Walk in front of the HC-SR04 once.'};}},\n"
  " {t:'Door · closed reference',d:'Close the door and capture the distance.',a:'dc',f:d=>d.doorClosedSet?{s:'pass',n:'Saved '+d.doorClosed.toFixed(1)+' cm'}:{s:NT,n:d.doorCalRun?d.doorCalMsg:'Not captured.'}},\n"
  " {t:'Door · open reference',d:'Open the door and capture the distance.',a:'do',f:d=>d.doorOpenSet?{s:'pass',n:'Saved '+d.doorOpen.toFixed(1)+' cm'}:{s:NT,n:d.doorCalRun?d.doorCalMsg:'Not captured.'}},\n"
  " {t:'Verification report',d:'Summary of all checks.',f:()=>({s:NT,n:''})}];\n"
  "function stepRes(i){const d=S.d;if(!d)return{s:NT,n:'Waiting for data…'};if(!S.up&&i>0)return{s:NT,n:'Wi-Fi lost.'};if(i>2&&i<8&&!d.link)return{s:'fail',n:'ATmega not reporting.',x:'Fix the UART link first (step 2).'};return STEPS[i].f(d);}\n"
  "function wizard(){const i=S.step,st=STEPS[i];$('vStep').textContent='Step '+(i+1)+' of 12';\n"
  " $('vProg').innerHTML=STEPS.map((_,k)=>{const r=k===i?null:S.res[k];return'<i class=\"'+(k===i?'cur':r==='pass'?'pass':r==='fail'?'fail':'')+'\"></i>';}).join('');\n"
  " $('vTitle').textContent=st.t;$('vDesc').textContent=st.d;$('vBack').disabled=i===0;$('vNext').textContent=i===11?'Done':'Next';\n"
  " if(i===11){$('vRep').classList.remove('hidden');let h='';let p=0,f=0,n=0;for(let k=0;k<11;k++){const r=stepRes(k);const s=(k===8||k===7||k===9||k===10)&&r.s===NT?(S.res[k]||NT):r.s;if(s==='pass')p++;else if(s==='fail')f++;else n++;h+='<div class=\"lr\"><div class=\"ic '+(s==='pass'?'pass':s==='fail'?'fail':'')+'\">'+(s==='pass'?'✓':s==='fail'?'✕':'–')+'</div><div class=\"t\"><b>'+STEPS[k].t+'</b><span>'+(s==='pass'?'PASS':s==='fail'?'FAIL':'NOT TESTED')+' · '+esc(r.n)+'</span></div></div>';}\n"
  "  $('vRepList').innerHTML=h;ic('bigic '+(f?'fail':n?'':'pass'),f?'✕':n?'–':'✓');$('vNote').textContent=p+' passed · '+f+' failed · '+n+' not tested';$('vFix').classList.add('hidden');$('vAct').innerHTML='';$('vAct').dataset.k='11';return;}\n"
  " $('vRep').classList.add('hidden');const r=stepRes(i);S.res[i]=r.s==='run'?S.res[i]:r.s;\n"
  " ic('bigic '+(r.s==='pass'?'pass':r.s==='fail'?'fail':r.s==='run'?'run':''),r.s==='pass'?'✓':r.s==='fail'?'✕':'–');\n"
  " $('vNote').textContent=(r.s==='pass'?'PASS · ':r.s==='fail'?'FAIL · ':r.s==='run'?'TESTING · ':'NOT TESTED · ')+r.n;const fx=$('vFix');if(r.x){fx.classList.remove('hidden');fx.textContent='How to fix: '+r.x;}else fx.classList.add('hidden');\n"
  " const a=$('vAct');if(a.dataset.k!==String(i)){a.dataset.k=i;a.innerHTML='';const mk=(l,fn)=>{const b=document.createElement('button');b.className='btn pri';b.textContent=l;b.onclick=fn;a.appendChild(b);};\n"
  "  if(st.a==='flow'){mk('Run 18 s test',()=>act('/water/test/start'));}if(st.a==='dc')mk('Capture closed',()=>act('/door/cal/start?pos=closed'));if(st.a==='do')mk('Capture open',()=>act('/door/cal/start?pos=open'));}}\n"
  "function ic(c,t){const e=$('vIc');e.className=c;e.textContent=t;}\n"
  "$('vNext').onclick=()=>{if(S.step===11){nav('home');return;}S.step++;S.ppl0=S.step===8&&S.d?S.d.ppl:S.ppl0;wizard();};\n"
  "$('vBack').onclick=()=>{if(S.step>0){S.step--;wizard();}};\n"
  "$('vSkip').onclick=()=>{nav('home');toast('Verification skipped · you can run it later from Verify');};\n"
  "/* ---------- entries window ---------- */\n"
  "document.querySelectorAll('#win button').forEach(b=>b.onclick=()=>{document.querySelectorAll('#win button').forEach(x=>x.classList.remove('on'));b.classList.add('on');S.win=b.dataset.w;$('cust').classList.toggle('hidden',S.win!=='cust');qEnt();});\n"
  "['tFrom','tTo'].forEach(i=>$(i).onchange=qEnt);const hm=v=>{const p=(v||'0:0').split(':');return(+p[0])*60+(+p[1]);};\n"
  "async function qEnt(){try{let u='/entries?';if(S.win==='cust')u+='from='+hm($('tFrom').value)+'&to='+hm($('tTo').value);else if(S.win==='today')u+='today=1';else u+='mins='+S.win;const r=await jget(u,3000);$('winN').textContent=r.ok?r.n:'–';}catch(e){}}\n"
  "/* ---------- air animation + fan ---------- */\n"
  "const parts=[];for(let i=0;i<40;i++)parts.push({x:Math.random(),y:Math.random(),s:.25+Math.random()*.75,r:1.6+Math.random()*3,ph:Math.random()*6.3});\n"
  "function cv2(cv){const dpr=Math.min(window.devicePixelRatio||1,2),w=cv.clientWidth,h=cv.clientHeight;if(!w||!h)return null;if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);}const c=cv.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);return{c,w,h};}\n"
  "function drawAir(dt,t){if(S.view!=='home')return;const k=cv2($('airCv'));if(!k)return;const{c,w,h}=k,cx=w/2,cy=h/2+4,R=Math.min(w,h)/2-16;airS+=(airT-airS)*Math.min(1,dt*3.5);const fr=Math.max(0,Math.min(1,airS/1023)),a1=.75*Math.PI;\n"
  " c.lineWidth=13;c.lineCap='round';c.strokeStyle=css('--fill');c.beginPath();c.arc(cx,cy,R,a1,a1+1.5*Math.PI);c.stroke();c.strokeStyle=airCol;c.beginPath();c.arc(cx,cy,R,a1,a1+.001+1.5*Math.PI*fr);c.stroke();\n"
  " c.save();c.beginPath();c.arc(cx,cy,R-14,0,6.3);c.clip();const sp=.05+Math.min(1,airS/700)*.35,n=Math.round(10+fr*30);\n"
  " for(let i=0;i<n;i++){const p=parts[i];p.y-=p.s*sp*dt;if(p.y<-.05){p.y=1.05;p.x=Math.random();}const px=cx-R+p.x*2*R+Math.sin(t/900+p.ph)*8,py=cy-R+p.y*2*R,g=c.createRadialGradient(px,py,0,px,py,p.r*3);g.addColorStop(0,airCol+'99');g.addColorStop(1,airCol+'00');c.fillStyle=g;c.beginPath();c.arc(px,py,p.r*3,0,6.3);c.fill();}\n"
  " c.restore();c.fillStyle=css('--ink');c.textAlign='center';c.textBaseline='middle';c.font='700 34px -apple-system,Segoe UI,sans-serif';c.fillText(Math.round(airS),cx,cy-6);c.fillStyle=airCol;c.font='700 12px -apple-system,Segoe UI,sans-serif';c.fillText(airSt,cx,cy+22);}\n"
  "let lt=performance.now();function tick(now){const dt=Math.min(.1,(now-lt)/1000);lt=now;if(!document.hidden&&!$('app').classList.contains('hidden')){try{drawAir(dt,now);}catch(e){}\n"
  " const tg=(S.d&&S.d.link&&S.d.fan)?640:0;S.fs+=(tg-S.fs)*Math.min(1,dt*1.6);S.fa=(S.fa+S.fs*dt)%360;$('blades').setAttribute('transform','rotate('+S.fa.toFixed(1)+' 60 60)');}requestAnimationFrame(tick);}\n"
  "(function(){let d='M0 8 ';for(let i=0;i<16;i++)d+=i?'t30 0 ':'q15 -8 30 0 ';d+='V200 H0Z';$('wp1').setAttribute('d',d);$('wp2').setAttribute('d','M-30 12 '+d.slice(5));})();\n"
  "/* ---------- charts ---------- */\n"
  "function line(cv,v,o){const k=cv2(cv);if(!k)return;const{c,w,h}=k;if(v.length<2){c.fillStyle=css('--mut');c.font='13px sans-serif';c.textAlign='center';c.fillText('collecting data…',w/2,h/2);return;}\n"
  " const L=34,R=8,T=10,B=20,pw=w-L-R,ph=h-T-B;let mn=Math.min(...v),mx=Math.max(...v);if(o.min!==undefined)mn=Math.min(mn,o.min);if(mx-mn<(o.span||1)){const m=(mx+mn)/2;mn=m-(o.span||1)/2;mx=m+(o.span||1)/2;}const pd=(mx-mn)*.12;mn-=pd;mx+=pd;if(o.min!==undefined&&mn<o.min)mn=o.min;\n"
  " c.font='11px -apple-system,sans-serif';c.fillStyle=css('--mut');c.strokeStyle=css('--line');c.lineWidth=1;c.textAlign='right';c.textBaseline='middle';\n"
  " for(let i=0;i<=3;i++){const y=T+ph*i/3;c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();const val=mx-(mx-mn)*i/3;c.fillText(o.dec?val.toFixed(o.dec):Math.round(val),L-5,y);}\n"
  " const X=i=>L+pw*i/(v.length-1),Y=x=>T+ph*(1-(x-mn)/(mx-mn)),g=c.createLinearGradient(0,T,0,T+ph);g.addColorStop(0,o.col+'55');g.addColorStop(1,o.col+'00');\n"
  " c.beginPath();c.moveTo(X(0),Y(v[0]));for(let i=1;i<v.length;i++)c.lineTo(X(i),Y(v[i]));c.lineTo(X(v.length-1),T+ph);c.lineTo(X(0),T+ph);c.fillStyle=g;c.fill();\n"
  " c.beginPath();c.moveTo(X(0),Y(v[0]));for(let i=1;i<v.length;i++)c.lineTo(X(i),Y(v[i]));c.strokeStyle=o.col;c.lineWidth=2.2;c.lineJoin='round';c.stroke();}\n"
  "function bars(cv,v,lab){const k=cv2(cv);if(!k)return;const{c,w,h}=k,L=26,R=6,T=14,B=20,pw=w-L-R,ph=h-T-B,mx=Math.max(3,...v),bw=pw/v.length;c.font='11px -apple-system,sans-serif';c.fillStyle=css('--mut');c.strokeStyle=css('--line');c.textAlign='right';c.textBaseline='middle';\n"
  " for(let i=0;i<=2;i++){const y=T+ph*i/2;c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();c.fillText(Math.round(mx-mx*i/2),L-5,y);}\n"
  " v.forEach((x,i)=>{const bh=ph*x/mx,xx=L+i*bw+bw*.18,ww=bw*.64,y=T+ph-bh,r=Math.min(5,ww/2);c.fillStyle=i===v.length-1?css('--blue'):css('--blue')+'88';c.beginPath();c.moveTo(xx,T+ph);c.lineTo(xx,y+r);c.arcTo(xx,y,xx+r,y,r);c.arcTo(xx+ww,y,xx+ww,y+r,r);c.lineTo(xx+ww,T+ph);c.fill();\n"
  "  if(i%2===(v.length-1)%2){c.fillStyle=css('--mut');c.textAlign='center';c.textBaseline='alphabetic';c.fillText(lab[i],xx+ww/2,h-4);}});}\n"
  "const ins=(id,a)=>$(id).innerHTML=a.map(t=>'<li>'+t+'</li>').join('');\n"
  "function drawCharts(){if(S.view==='ins'){try{drawIns();}catch(e){console.error(e);}return;}const h=S.hist;if(!h||S.view!=='home')return;const hr=h.hours||[],lab=hr.map((_,i)=>pad(((h.hNow-(hr.length-1-i))%24+24)%24));bars($('chEnt'),hr,lab);\n"
  " if(h.tv&&hr.length){const tot=hr.reduce((a,b)=>a+b,0),mx=Math.max(...hr),pk=hr.lastIndexOf(mx);ins('insEnt',[tot+' entries in the last 12 hours',mx>0?'Busiest hour: '+lab[pk]+':00 ('+mx+')':'No entries recorded yet','This hour '+hr[hr.length-1]+' vs previous '+(hr[hr.length-2]||0)]);}else ins('insEnt',['Waiting for a valid RTC time']);\n"
  " const a=(h.air||[]).map(Number);line($('chAir'),a,{col:css('--orange'),span:30});if(a.length>=6){const q=Math.max(2,Math.floor(a.length/4)),dl=avg(a.slice(-q))-avg(a.slice(0,q));ins('insAir',['Range '+Math.min(...a)+' – '+Math.max(...a)+' (avg '+Math.round(avg(a))+')',Math.abs(dl)<10?'Trend: stable':dl>0?'Trend: rising (+'+Math.round(dl)+')':'Trend: improving ('+Math.round(dl)+')']);}else ins('insAir',['Collecting samples…']);\n"
  " const f=(h.flow||[]).map(x=>x/100);line($('chFlow'),f,{col:css('--blue'),min:0,span:1,dec:1});if(f.length>=6){const ac=f.filter(x=>x>.05).length,pc=Math.round(100*ac/f.length),i2=['Average '+avg(f).toFixed(2)+' L/min · peak '+Math.max(...f).toFixed(1),'Flowing '+pc+'% of this window'];if(f.length>=60&&pc>=90&&avg(f)>.2)i2.push('Almost continuous flow — possible leak or tap left open');ins('insFlow',i2);}else ins('insFlow',['Collecting samples…']);}\n"
  "window.addEventListener('resize',()=>{try{drawCharts();}catch(e){}});\n"
  "/* ===================== v5: score, predictions, pop-up, analytics ===================== */\n"
  "const LPE=1.5/* assumed litres per entry for the water share of the score */,DAYS=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'],WHY={ENTRIES:'entries',TIME:'time since last clean',AIR:'air quality'};\n"
  "const fmtMin=m=>m>=90?Math.floor(m/60)+'h '+Math.round(m%60)+'m':Math.round(m)+' min',hh=h=>pad(h)+':00',sz=b=>b>1048576?(b/1048576).toFixed(2)+' MB':(b/1024).toFixed(1)+' KB';\n"
  "function vib(p){try{if(navigator.vibrate&&navigator.userActivation&&navigator.userActivation.hasBeenActive)navigator.vibrate(p);}catch(_){}}\n"
  "document.addEventListener('click',e=>{if(e.target.closest('button,a')){vib(8);}},true);\n"
  "function anim(id,v,dec){const e=$(id);if(!e)return;dec=dec||0;const a=e._v===undefined?v:e._v;e._v=v;cancelAnimationFrame(e._r);if(a===v||!isFinite(a)){e.textContent=v.toFixed(dec);return;}const t0=performance.now();(function f(n){const k=Math.min(1,(n-t0)/600),x=a+(v-a)*(1-Math.pow(1-k,3));e.textContent=x.toFixed(dec);if(k<1)e._r=requestAnimationFrame(f);})(t0);}\n"
  "const ico={link:'<svg viewBox=\"0 0 24 24\"><path d=\"M10 14a4 4 0 005.7 0l3-3a4 4 0 00-5.7-5.7l-1 1M14 10a4 4 0 00-5.7 0l-3 3a4 4 0 005.7 5.7l1-1\"/></svg>',sd:'<svg viewBox=\"0 0 24 24\"><path d=\"M6 3h9l4 4v14H6zM10 3v4M13 3v4\"/></svg>',clock:'<svg viewBox=\"0 0 24 24\"><circle cx=\"12\" cy=\"12\" r=\"9\"/><path d=\"M12 7v5l3 2\"/></svg>',fan:'<svg viewBox=\"0 0 24 24\"><circle cx=\"12\" cy=\"12\" r=\"2\"/><path d=\"M12 10c-1-5 1-7 3-6s1 4-3 6zM14 12c5-1 7 1 6 3s-4 1-6-3zM12 14c1 5-1 7-3 6s-1-4 3-6zM10 12c-5 1-7-1-6-3s4-1 6 3z\"/></svg>',door:'<svg viewBox=\"0 0 24 24\"><path d=\"M6 21V4a1 1 0 011-1h10a1 1 0 011 1v17M4 21h16\"/></svg>'};\n"
  "function strip(d,link){const c=[],p=(i,t,k)=>c.push('<span class=\"sc\">'+ico[i]+'<span class=\"dot '+k+'\"></span>'+t+'</span>'),rtc=d.tv&&!(d.bits&8);\n"
  " p('link',!S.up?'Wi-Fi lost':link?'Linked':'ATmega offline',!S.up?'bad':link?'ok':'warn');p('sd',d.sd?'SD logging':'SD offline',d.sd?'ok':'bad');p('clock',rtc?'RTC ok':'RTC check',rtc?'ok':'warn');\n"
  " p('fan','Fan '+(d.fan?'on':'off')+' · '+(d.mode==='AUTO'?'auto':d.mode==='MANUAL'?'manual':'?'),d.fan?'ok':'');p('door','Door '+(d.doorState===1?'closed':d.doorState===2?'open':'?'),d.doorState===1?'ok':d.doorState===2?'warn':'');$('strip').innerHTML=c.join('');}\n"
  "/* 0-100 cleanliness: 40 % entries, 30 % air, 20 % time, 10 % water used, relative to the rule in force */\n"
  "function score(d){if(!d.cKnown||!d.tv||d.effE===undefined)return null;const eN=d.effE,tN=d.effM,el=Math.max(0,(d.ep-d.cEp)/60),e=Math.min(1,d.eSince/eN),t=Math.min(1,el/tN),w=Math.min(1,(d.lSince||0)/(eN*LPE)),am={GOOD:0,MODERATE:.35,POOR:.75,BAD:1}[d.airS];let a=am===undefined?0:am;if(d.hi)a=Math.max(a,.5);\n"
  " let s=Math.round(100-100*Math.min(1,.4*e+.3*a+.2*t+.1*w));if(d.cMin<=0)s=Math.min(s,35);else if(d.cMin<=15)s=Math.min(s,49);return{s,e,a,t,w,el,eN,tN};}\n"
  "function waterEta(){const a=((S.hist&&S.hist.waterLevel)||[]).filter(x=>x>=0);if(a.length<24)return null;const q=Math.floor(a.length/4),per=(avg(a.slice(-q))-avg(a.slice(0,q)))/((a.length-q)*5);return per>=-0.0004?null:a[a.length-1]/(-per)/3600;}\n"
  "function peakHour(d){const H=S.an&&S.an.heat;if(!H||!d.tv)return null;const tot=H.reduce((s,r)=>s+r.reduce((x,y)=>x+y,0),0);if(tot<10)return{learn:1};const t=new Date(d.ep*1000),wd=t.getUTCDay(),hr=t.getUTCHours(),rs=H[wd].reduce((x,y)=>x+y,0),own=rs>=10,src=own?H[wd]:Array.from({length:24},(_,h)=>H.reduce((s,r)=>s+r[h],0));\n"
  " const am=(lo,hi)=>{let b=-1,v=0;for(let h=lo;h<hi;h++)if(src[h]>v){v=src[h];b=h;}return b<0?null:{h:b,v};};let p=am(hr,24),late=0;if(!p){p=am(0,24);late=1;}return p?{h:p.h,v:p.v,own,late}:{learn:1};}\n"
  "function tile(id,c){$(id).className='tile '+c;}\n"
  "function render5(d,link){strip(d,link);const r=score(d),rg=$('rg'),C=326.7,bar=(n,t,f)=>'<div class=\"mb\"><span>'+n+'</span><i style=\"--w:'+Math.round(Math.min(1,f)*100)+'%;--c:'+css(f>=1?'--red':f>=.7?'--orange':'--green')+'\"></i><b>'+t+'</b></div>';\n"
  " if(!r){$('sc').textContent='–';rg.style.strokeDashoffset=C;$('scChip').textContent=d.tv?'waiting':'no clock';$('scChip').className='chip warn';$('nextT').textContent=d.tv?'…':'Set the RTC';$('nextS').textContent='The cleaning forecast needs a valid RTC time.';$('why').innerHTML='';}\n"
  " else{anim('sc',r.s);rg.style.strokeDashoffset=C*(1-r.s/100);rg.style.stroke=css(r.s>=75?'--green':r.s>=50?'--blue':r.s>=36?'--orange':'--red');const ch=$('scChip');ch.textContent=r.s>=75?'Fresh':r.s>=50?'Good':r.s>=36?'Clean soon':'Clean now';ch.className='chip '+(r.s>=75?'ok':r.s>=50?'':r.s>=36?'warn':'bad');\n"
  "  $('nextT').textContent=d.cMin<=0?'Due now':'in '+fmtMin(d.cMin);$('nextS').textContent=(d.cMin<=0?'Reason: ':'Next clean, driven by ')+(WHY[d.cWhy]||'time')+(d.hi?' · air high, rule tightened':' · rule '+d.effE+' entries / '+fmtMin(d.effM));\n"
  "  $('why').innerHTML=bar('Entries',d.eSince+'/'+d.effE,r.e)+bar('Air',d.airS==='GOOD'?'good':d.airS.toLowerCase()+(d.hi?' ↑':''),Math.max(r.a,.01))+bar('Time',fmtMin(r.el),r.t)+bar('Water',(d.lSince||0).toFixed(1)+' L',r.w);}\n"
  " /* predictions */\n"
  " $('pNext').textContent=r?(d.cMin<=0?'Now':fmtMin(d.cMin)):'–';$('pNextS').textContent=r?'because of '+(WHY[d.cWhy]||'time'):'needs RTC time';tile('tNext',!r?'':d.cMin<=0?'bad':d.cMin<=20?'warn':'');\n"
  " if(d.airTrendReady&&link){const p=Math.max(0,Math.min(1023,Math.round(d.air+d.airTrendPerMin*10))),tr=d.airTrendPerMin;$('pAir').textContent=p;$('pAirS').textContent=Math.abs(tr)<.5?'steady':(tr>0?'▲ rising ':'▼ falling ')+Math.abs(tr).toFixed(1)+'/min';tile('tAir',tr>8?'bad':tr>3?'warn':'');}else{$('pAir').textContent='–';$('pAirS').textContent='learning trend…';tile('tAir','');}\n"
  " const pk=peakHour(d);if(!pk){$('pPeak').textContent='–';$('pPeakS').textContent='no data yet';}else if(pk.learn){$('pPeak').textContent='learning';$('pPeakS').textContent='needs about a day of entries';}else{$('pPeak').textContent=hh(pk.h);$('pPeakS').textContent=(pk.late?'passed today · ':'')+pk.v+' entries typical · '+(pk.own?'this weekday':'all days');}\n"
  " let wt='–',ws='',wc='';if(d.bits&2){wt='Probe fault';ws='check the probe';wc='bad';}else if(!d.wet){wt='Empty';ws='refill the tank';wc='bad';}else if(d.wLvl>=0){const h=waterEta();wt=Math.round(d.wLvl)+' %';ws=h===null?'level steady':'~'+(h<1?Math.round(h*60)+' min':h.toFixed(1)+' h')+' left at current use';wc=d.wLvl<=20||(h!==null&&h<2)?'bad':d.wLvl<=35||(h!==null&&h<6)?'warn':'ok';}else{wt='Present';ws='calibrate for level %';wc='ok';}\n"
  " $('pWat').textContent=wt;$('pWatS').textContent=ws;tile('tWat',wc);popCheck(d);}\n"
  "/* ---------- large clean pop-up ---------- */\n"
  "function showPop(d,due){const r=score(d);if(!r)return;S.popDue=due;$('sheet').className='sheet'+(due?' due':'');$('popT').textContent=due?'Time to clean':'Cleaning due in ~'+fmtMin(Math.max(1,d.cMin));$('popP').textContent=due?'The restroom reached its cleaning limit.':'Get ready – the next clean is approaching.';\n"
  " const m=d.cWhy,row=(k,t,v)=>'<div class=\"rowx\"><span class=\"'+(m===k?'main':'')+'\">'+t+(m===k?' · main reason':'')+'</span><b>'+v+'</b></div>';\n"
  " $('popW').innerHTML=row('ENTRIES','Entries',d.eSince+' / '+d.effE)+row('AIR','Air quality',esc(d.airS)+(d.hi?' · high':''))+row('TIME','Since last clean',fmtMin(r.el)+' / '+fmtMin(d.effM))+(d.hi?'<div class=\"note\">Air stayed high, so the rule tightened to '+d.effE+' entries / '+fmtMin(d.effM)+'.</div>':'');\n"
  " $('pop').classList.add('show');vib([60,40,60]);}\n"
  "const hidePop=()=>$('pop').classList.remove('show');\n"
  "function popCheck(d){if(!d.cKnown||!d.tv)return;if(S.pk!==d.cEp){S.pk=d.cEp;S.pf={pre:0,due:0};}const open=$('pop').classList.contains('show');if(open&&d.cMin>15){hidePop();return;}if(Date.now()<(S.snz||0))return;\n"
  " if(d.cMin<=0&&!S.pf.due){S.pf.due=1;showPop(d,true);}else if(d.cMin>0&&d.cMin<=15&&!S.pf.pre){S.pf.pre=1;showPop(d,false);}else if(open&&d.cMin<=0&&!S.popDue)showPop(d,true);}\n"
  "async function doClean(){const r=await ctl('/clean');toast(r.st===200?'Cleaning recorded ✓':(r.b.error||'Failed'));if(r.st===200){hidePop();S.snz=0;setTimeout(()=>{loadAn();},6e3);}}\n"
  "$('popOk').onclick=doClean;$('popSz').onclick=()=>{S.snz=Date.now()+6e5;if(S.popDue)S.pf.due=0;else S.pf.pre=0;hidePop();toast('Snoozed for 10 minutes');};\n"
  "$('btnClean2').onclick=()=>{if(confirm('Mark the restroom as cleaned now?'))doClean();};\n"
  "/* ---------- 24 h analytics ---------- */\n"
  "async function loadAn(){if(!S.up)return;try{S.an=await jget('/an',6000);if(S.view==='ins')drawIns();if(S.d)render5(S.d,!!S.d.link);}catch(e){}}\n"
  "function pear(X,Y){const n=X.length;if(n<3)return{r:0,n,m:0,b:0};const mx=avg(X),my=avg(Y);let a=0,b=0,c=0;for(let i=0;i<n;i++){const u=X[i]-mx,v=Y[i]-my;a+=u*v;b+=u*u;c+=v*v;}const m=b?a/b:0;return{r:b&&c?a/Math.sqrt(b*c):0,n,m,b:my-m*mx};}\n"
  "function pairs(a,L){const X=[],Y=[];for(let i=0;i+L<a.air.length;i++)if(a.air[i]>0&&a.air[i+L]>0){X.push(a.ent[i]);Y.push(a.air[i+L]);}return[X,Y];}\n"
  "const strength=r=>{r=Math.abs(r);return r<.3?'weak':r<.6?'moderate':'strong';};\n"
  "function frame(cv){const k=cv2(cv);if(!k)return null;k.c.font='11px -apple-system,sans-serif';return k;}\n"
  "function msg(k,t){k.c.fillStyle=css('--mut');k.c.textAlign='center';k.c.fillText(t,k.w/2,k.h/2);}\n"
  "function scatter(cv,X,Y,fit,xl,yl,dec){const k=frame(cv);if(!k)return;const{c,w,h}=k;if(X.length<3){msg(k,'collecting data…');return;}\n"
  " const L=42,R=10,T=10,B=32,pw=w-L-R,ph=h-T-B,mxX=Math.max(1,...X);let mn=Math.min(...Y),mx=Math.max(...Y);if(mx-mn<1)mx=mn+1;const pd=(mx-mn)*.1;mn=Math.max(0,mn-pd);mx+=pd;const gx=x=>L+pw*x/mxX,gy=y=>T+ph*(1-(y-mn)/(mx-mn));\n"
  " c.strokeStyle=css('--line');c.fillStyle=css('--mut');c.lineWidth=1;c.textAlign='right';c.textBaseline='middle';for(let i=0;i<=3;i++){const y=T+ph*i/3;c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();const v=mx-(mx-mn)*i/3;c.fillText(dec?v.toFixed(dec):Math.round(v),L-5,y);}\n"
  " c.textAlign='center';c.textBaseline='alphabetic';for(let i=0;i<=Math.min(5,mxX);i++){const x=Math.round(mxX*i/Math.min(5,mxX));c.fillText(x,gx(x),h-16);}c.fillText(xl,L+pw/2,h-3);\n"
  " c.save();c.translate(10,T+ph/2);c.rotate(-Math.PI/2);c.fillText(yl,0,0);c.restore();\n"
  " c.fillStyle=css('--blue')+'88';X.forEach((x,i)=>{c.beginPath();c.arc(gx(x),gy(Y[i]),3.2,0,6.3);c.fill();});\n"
  " if(fit){c.strokeStyle=css('--orange');c.lineWidth=2.4;c.beginPath();c.moveTo(gx(0),gy(fit.b));c.lineTo(gx(mxX),gy(fit.b+fit.m*mxX));c.stroke();}}\n"
  "function heatmap(cv,H,now){const k=frame(cv);if(!k)return;const{c,w,h}=k,L=30,T=4,B=18,cw=(w-L-2)/24,ch=(h-T-B)/7,mx=Math.max(1,...H.map(r=>Math.max(...r)));\n"
  " c.textBaseline='middle';c.textAlign='right';c.fillStyle=css('--mut');for(let d=0;d<7;d++){c.fillText(DAYS[d],L-5,T+ch*(d+.5));for(let q=0;q<24;q++){c.fillStyle=css('--blue');c.globalAlpha=H[d][q]?.1+.9*H[d][q]/mx:.05;c.beginPath();c.roundRect?c.roundRect(L+q*cw+1,T+d*ch+1,cw-2,ch-2,4):c.rect(L+q*cw+1,T+d*ch+1,cw-2,ch-2);c.fill();}}\n"
  " c.globalAlpha=1;c.textAlign='center';c.textBaseline='alphabetic';c.fillStyle=css('--mut');for(let q=0;q<24;q+=3)c.fillText(pad(q),L+(q+.5)*cw,h-4);\n"
  " if(now){c.strokeStyle=css('--ink');c.lineWidth=2;c.strokeRect(L+now.h*cw+.5,T+now.d*ch+.5,cw-1,ch-1);}}\n"
  "function drawIns(){const a=S.an,d=S.d||{};if(!a||!a.air)return;const n=a.air.length,ok=a.tv&&n>0;\n"
  " let best=null;for(let L=0;L<=3;L++){const[X,Y]=pairs(a,L),p=pear(X,Y);p.L=L;p.X=X;p.Y=Y;if(p.n>=30&&(!best||Math.abs(p.r)>Math.abs(best.r)))best=p;}\n"
  " const q=best||(()=>{const[X,Y]=pairs(a,0),p=pear(X,Y);p.L=0;p.X=X;p.Y=Y;return p;})(),has=!!best&&q.X.some(x=>x>0);\n"
  " scatter($('cScat'),q.X,q.Y,has?q:null,'entries per 5 min','air index');$('corChip').textContent=has?'r = '+q.r.toFixed(2):'n = '+q.n+' / 30';$('corChip').className='chip '+(has&&Math.abs(q.r)>=.3?'ok':'');\n"
  " ins('insCor',has?['r = '+q.r.toFixed(2)+' ('+strength(q.r)+') over '+q.n+' five-minute samples','Each extra entry in 5 min ≈ '+(q.m>=0?'+':'')+q.m.toFixed(1)+' air index'+(q.L?', about '+q.L*5+' min later':', in the same 5 min'),q.r>=.3&&q.m>0?'Supports the adaptive timer: more people → higher air → clean sooner':'Weak link so far — the adaptive timer still protects through the air reading','An association in sensor data, not proof of cause']:['Collecting samples: '+q.n+' of 30 needed','Keep the system running; the 24 h window fills automatically']);\n"
  " const H=a.heat||[],tot=H.reduce((s,r)=>s+r.reduce((x,y)=>x+y,0),0),t=new Date((d.ep||0)*1000);heatmap($('cHeat'),H.length?H:Array.from({length:7},()=>Array(24).fill(0)),d.tv?{d:t.getUTCDay(),h:t.getUTCHours()}:null);\n"
  " if(tot<5)ins('insHeat',['Not enough recorded entries yet','The map fills from the entries log on the SD card']);else{let b={v:0},wd=DAYS.map((_,i)=>H[i].reduce((x,y)=>x+y,0));H.forEach((r,i)=>r.forEach((v,h)=>{if(v>b.v)b={v,d:i,h};}));const bd=wd.indexOf(Math.max(...wd));ins('insHeat',['Busiest slot: '+DAYS[b.d]+' '+hh(b.h)+' ('+b.v+' entries)','Busiest weekday: '+DAYS[bd]+' ('+wd[bd]+')',tot+' entries recorded in total (totals, not daily averages)']);}\n"
  " /* water */\n"
  " const WX=[],WY=[];let sl=0,se=0,idle=0,run=0,lastRun=0;for(let i=0;i<n;i++){if(a.air[i]<=0)continue;const l=a.lit[i]/100;WX.push(a.ent[i]);WY.push(l);if(a.ent[i]>0){sl+=l;se+=a.ent[i];}else idle+=l;if(a.ent[i]===0&&l>=.5)run++;else run=0;if(i>=n-3)lastRun=Math.max(lastRun,run);}\n"
  " const wp=pear(WX,WY),wok=WX.length>=30&&WX.some(x=>x>0);scatter($('cWat'),WX,WY,wok?wp:null,'entries per 5 min','litres',1);\n"
  " const leak=run>=3&&lastRun>=3;$('watChip').textContent=leak?'possible leak':se?(sl/se).toFixed(2)+' L/entry':'–';$('watChip').className='chip '+(leak?'bad':se?'ok':'');\n"
  " ins('insWat',se?['≈ '+(sl/se).toFixed(2)+' litres per entry ('+se+' entries, '+sl.toFixed(1)+' L)','Flow with no entries: '+idle.toFixed(1)+' L in 24 h',leak?'Flow ≥ 0.5 L per 5 min with no entries for 15+ min — check for a leak or open tap':'No sustained flow without entries',wok?'r = '+wp.r.toFixed(2)+' between entries and litres':'Correlation needs 30+ samples']:['No water use recorded in this window yet']);\n"
  " /* fan */\n"
  " const on=[],off=[];for(let i=0;i+1<n;i++)if(a.air[i]>0&&a.air[i+1]>0){const dl=a.air[i+1]-a.air[i];if(a.fan[i]>=50)on.push(dl);else if(a.fan[i]<=10)off.push(dl);}\n"
  " const f=x=>x.length?(avg(x)>=0?'+':'')+avg(x).toFixed(1):'–',good=on.length>=8&&off.length>=8;\n"
  " $('fanT').innerHTML='<div class=\"tile\"><span>Next 5 min, fan ON</span><b>'+f(on)+'</b><i>'+on.length+' samples</i></div><div class=\"tile\"><span>Next 5 min, fan OFF</span><b>'+f(off)+'</b><i>'+off.length+' samples</i></div><div class=\"tile\"><span>Fan duty (24 h)</span><b>'+(n?Math.round(avg(a.fan.filter((_,i)=>a.air[i]>0))||0):0)+' %</b><i>average on-time</i></div>';\n"
  " $('fanC').textContent=good?(avg(on)<avg(off)-3?'lowers air':'no clear effect'):'collecting';$('fanC').className='chip '+(good&&avg(on)<avg(off)-3?'ok':'');\n"
  " ins('insFan',good?[avg(on)<avg(off)-3?'Air falls faster when the fan runs ('+f(on)+' vs '+f(off)+' index per 5 min)':'No clear drop while the fan runs — check fan placement and airflow','The fan mostly runs when air is already high, so this comparison is biased']:['Needs 8+ five-minute samples with the fan on and off']);\n"
  " /* cleaning history */\n"
  " const cl=(a.cl||[]).slice().reverse();$('clChip').textContent=cl.length?cl.length+' logged':'none';\n"
  " $('clHist').innerHTML=cl.length?cl.map((c,i)=>{const prev=cl[i+1],gap=prev?fmtMin((c.e-prev.e)/60):'–',t=new Date(c.e*1000),dl=c.a>1?c.a-c.b:null;return'<div class=\"rowx\"><span>'+pad(t.getUTCDate())+'/'+pad(t.getUTCMonth()+1)+' '+pad(t.getUTCHours())+':'+pad(t.getUTCMinutes())+' · gap '+gap+' · '+c.n+' entries</span><b>'+(c.b?c.b+' → '+(c.a>1?c.a:'…'):'–')+(dl!==null?' <small class=\"'+(dl<0?'ok':'')+'\" style=\"color:var(--'+(dl<0?'green':'orange')+')\">'+(dl>0?'+':'')+dl+'</small>':'')+'</b></div>';}).join('')+'<div class=\"sub\" style=\"margin-top:8px\">Air = relative index before → 5 min after cleaning. Last 8 cleans.</div>':'<div class=\"sub\">No cleans recorded yet — tap “Cleaned ✓” after each clean.</div>';}\n"
  "function printRep(){const a=S.an,d=S.d;if(!a||!d||!d.tv||!a.air){toast('Needs RTC time and analytics data');return;}const day0=d.ep-d.ep%86400,hrs=Array(24).fill(0);let ent=0,lit=0,as=0,an=0,mx=0,fan=0,first=0;\n"
  " for(let i=0;i<a.air.length;i++){const t=a.t0+i*300;if(t<day0||a.air[i]<=0)continue;if(!first)first=t;ent+=a.ent[i];lit+=a.lit[i]/100;as+=a.air[i];an++;mx=Math.max(mx,a.air[i]);fan+=a.fan[i];hrs[Math.floor((t-day0)/3600)]+=a.ent[i];}\n"
  " const pk=Math.max(...hrs),cl=(a.cl||[]).filter(c=>c.e>=day0),T=e=>{const t=new Date(e*1000);return pad(t.getUTCHours())+':'+pad(t.getUTCMinutes());},r=score(d);\n"
  " $('rep').innerHTML='<h1>Restroom daily report</h1><small>'+esc(d.ts)+' · covers '+(first?T(first)+'–'+T(d.ep):'no data')+' (up to 24 h of data are kept in memory)</small><h2>Summary</h2><table><tr><td>Entries (door-sensor events)</td><td>'+ent+'</td></tr><tr><td>Busiest hour</td><td>'+(pk>0?hh(hrs.indexOf(pk))+' ('+pk+' entries)':'–')+'</td></tr><tr><td>Air index avg / max (relative, not ppm)</td><td>'+(an?Math.round(as/an):'–')+' / '+mx+'</td></tr><tr><td>Water used</td><td>'+lit.toFixed(1)+' L'+(ent?' ('+(lit/ent).toFixed(2)+' L per entry)':'')+'</td></tr><tr><td>Fan on-time</td><td>'+(an?Math.round(fan/an):0)+' %</td></tr><tr><td>Cleanliness score now</td><td>'+(r?r.s+' / 100':'–')+'</td></tr></table><h2>Cleans today</h2>'+(cl.length?'<table><tr><th>Time</th><th>Entries since previous</th><th>Air before → after</th></tr>'+cl.map(c=>'<tr><td>'+T(c.e)+'</td><td>'+c.n+'</td><td>'+c.b+' → '+(c.a>1?c.a:'–')+'</td></tr>').join('')+'</table>':'<p>None recorded.</p>')+'<h2>Entries by hour</h2><table><tr>'+hrs.map((v,h)=>v?'<td>'+pad(h)+'h: <b>'+v+'</b></td>':'').join('')+'</tr></table><p><small>Entries are HC-SR04 proximity events; the air value is a relative MQ-135 index; both are estimates.</small></p>';window.print();}\n"
  "$('btnPrint').onclick=printRep;\n"
  "/* ---------- calibrate: cleaning rule, link to hardware check ---------- */\n"
  "S.ra=1;const ruleSeg=()=>$('rA').querySelectorAll('button').forEach(b=>b.classList.toggle('on',b.dataset.a===String(S.ra)));\n"
  "$('rA').querySelectorAll('button').forEach(b=>b.onclick=()=>{S.ra=b.dataset.a;S.ruleDirty=1;ruleSeg();});['rE','rM'].forEach(i=>$(i).oninput=()=>S.ruleDirty=1);\n"
  "$('rSave').onclick=async()=>{const e=+$('rE').value,m=+$('rM').value;if(!(e>=5&&e<=200&&m>=20&&m<=1440)){toast('Entries 5–200, minutes 20–1440');return;}const r=await act('/cfg?e='+e+'&m='+m+'&a='+S.ra);if(r.st<300)S.ruleDirty=0;};\n"
  "$('bVer').onclick=()=>nav('ver');\n"
  "function ruleRender(d){if(!S.ruleDirty&&d.cfgE){$('rE').value=d.cfgE;$('rM').value=d.cfgM;S.ra=d.adp;ruleSeg();}$('ruleChip').textContent=d.hi?'tightened':'normal';$('ruleChip').className='chip '+(d.hi?'warn':'ok');$('rNow').textContent=(d.hi?'Air stayed high → now ':'In force now: ')+d.effE+' entries / '+d.effM+' min';}\n"
  "/* ---------- data page: file sizes + date-range export ---------- */\n"
  "async function loadFiles(){try{const f=await jget('/files',4000);for(const k in f){const e=$('sz-'+k);if(e)e.textContent=sz(f[k]);}}catch(e){}}\n"
  "const ymd=t=>{const d=new Date(t);return d.getUTCFullYear()+'-'+pad(d.getUTCMonth()+1)+'-'+pad(d.getUTCDate());};\n"
  "function setRange(q){const day=864e5,base=S.d&&S.d.tv?S.d.ep*1000:Date.now()-new Date().getTimezoneOffset()*6e4,end=Math.floor(base/day)*day;$('exTo').value=ymd(q===1?end-day:end);$('exFrom').value=ymd(q===7?end-6*day:q===1?end-day:end);}\n"
  "document.querySelectorAll('#exQ button').forEach(b=>b.onclick=()=>{document.querySelectorAll('#exQ button').forEach(x=>x.classList.remove('on'));b.classList.add('on');setRange(+b.dataset.q);});setRange(0);\n"
  "$('exGo').onclick=()=>{const f=$('exF').value,a=$('exFrom').value,b=$('exTo').value;if(!a||!b){toast('Pick both dates');return;}const p=s=>{const q=s.split('-').map(Number);return Date.UTC(q[0],q[1]-1,q[2])/1000;},from=p(a),to=p(b)+86399;if(to<from){toast('The end date is before the start date');return;}const l=document.createElement('a');l.href='/export?f='+f+'&from='+from+'&to='+to;l.download='';document.body.appendChild(l);l.click();l.remove();toast('Export started…');};\n"
  "setInterval(()=>{if(document.hidden)return;if(S.view==='home')loadHist();if(S.view==='home'||S.view==='ins')loadAn();if(S.view==='log')loadFiles();},15000);\n"
  "setTimeout(()=>$('app').classList.remove('sk'),6000);\n"
  "/* ---------- v9: monitor illustration, boot sequence, calibration popup ---------- */\n"
  "const MONT='<svg viewBox=\"0 0 220 170\" class=\"mon\" aria-hidden=\"true\"><defs><linearGradient id=\"mg%K\" x1=\"0\" y1=\"0\" x2=\"0\" y2=\"1\"><stop offset=\"0\" stop-color=\"#2d313c\"/><stop offset=\"1\" stop-color=\"#15171d\"/></linearGradient><linearGradient id=\"mb%K\" x1=\"0\" y1=\"0\" x2=\"1\" y2=\"1\"><stop offset=\"0\" stop-color=\"#0a84ff\"/><stop offset=\"1\" stop-color=\"#5e5ce6\"/></linearGradient><clipPath id=\"mc%K\"><rect x=\"24\" y=\"16\" width=\"172\" height=\"104\" rx=\"8\"/></clipPath></defs><rect x=\"14\" y=\"6\" width=\"192\" height=\"122\" rx=\"16\" fill=\"url(#mg%K)\"/><rect x=\"24\" y=\"16\" width=\"172\" height=\"104\" rx=\"8\" fill=\"#0b0d12\"/><g clip-path=\"url(#mc%K)\"><rect class=\"scan\" x=\"24\" y=\"16\" width=\"172\" height=\"26\" fill=\"url(#mb%K)\" opacity=\".16\"/><rect x=\"34\" y=\"26\" width=\"52\" height=\"6\" rx=\"3\" fill=\"#fff\" opacity=\".2\"/><rect x=\"34\" y=\"38\" width=\"30\" height=\"5\" rx=\"2.5\" fill=\"#fff\" opacity=\".1\"/><rect x=\"150\" y=\"26\" width=\"36\" height=\"16\" rx=\"5\" fill=\"none\" stroke=\"#fff\" stroke-opacity=\".25\"/><circle class=\"led\" cx=\"160\" cy=\"34\" r=\"3\" fill=\"#30d158\"/><path class=\"ecg\" pathLength=\"100\" d=\"M32 84H76L86 60L98 106L108 72L116 84H188\" fill=\"none\" stroke=\"url(#mb%K)\" stroke-width=\"3.2\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/></g><circle cx=\"110\" cy=\"124\" r=\"1.8\" fill=\"#fff\" opacity=\".35\"/><path d=\"M92 128h36l6 24H86z\" fill=\"#2a2d36\"/><rect x=\"64\" y=\"152\" width=\"92\" height=\"8\" rx=\"4\" fill=\"#3b3f4b\"/></svg>';\n"
  "function mon(k){return MONT.replace(/%K/g,k);}\n"
  "function sleep(ms){return new Promise(r=>setTimeout(r,ms));}\n"
  "const BST=['Connecting to gateway','Linking sensor controller','Restoring history from MicroSD','Preparing dashboard'];\n"
  "async function boot(){\n"
  " $('bootMon').innerHTML=link('b');$('bootL').innerHTML=BST.map((t,i)=>'<li id=\"bs'+i+'\"><span class=\"tk\"></span>'+t+'</li>').join('');\n"
  " const mark=i=>{BST.forEach((t,j)=>{const e=$('bs'+j);e.className=j<i?'done':j===i?'run':'';});$('bootS').textContent=i<BST.length?BST[i]+'…':'Ready';$('bootB').style.width=(i/BST.length*100)+'%';};\n"
  " const until=async(f,ms)=>{const t=Date.now();while(Date.now()-t<ms&&!f())await sleep(150);};\n"
  " for(let i=0;i<BST.length;i++){mark(i);const t=Date.now();\n"
  "  if(i===0)await until(()=>S.d,6000);\n"
  "  else if(i===1)await until(()=>S.d&&S.d.link,4500);\n"
  "  else if(i===2)await Promise.allSettled([loadHist(),loadAn(),loadFiles()]);\n"
  "  await sleep(Math.max(0,1350-(Date.now()-t)));}\n"
  " mark(BST.length);{const sv=$('bootMon').firstChild;if(sv&&S.d&&S.d.link)sv.classList.add('linked');}await sleep(600);}\n"
  "let CM={busy:false,t:0,w:0,manual:false};\n"
  "const CR=[\n"
  " {k:'air',n:'Air quality',ok:d=>{try{return !!localStorage.getItem('ri-air');}catch(e){return false;}},okT:'Clean-air baseline saved',needT:'Clean air, then capture the baseline',btn:'Capture'},\n"
  " {k:'wat',n:'Water level',ok:d=>d.wLvl>=0,okT:'Dry and wet levels saved',needT:'Hold the probe dry, then capture',btn:'Capture dry'},\n"
  " {k:'flw',n:'Water flow',ok:d=>!!(d.flowZeroSet||d.flowVolumeSet),okT:'No-flow baseline saved',needT:'Close all taps, then capture',btn:'Capture zero'}];\n"
  "function crSet(k,c,t){$('cr'+k).className=c;$('crs'+k).textContent=t;}\n"
  "function crBtn(k,t,f){const b=$('crb'+k);b.classList.toggle('hidden',!t);if(t){b.textContent=t;b.disabled=false;b.onclick=f;}}\n"
  "function crProg(){const n=CR.filter(c=>$('cr'+c.k).classList.contains('ok')).length;$('cmB').style.width=(n/3*100)+'%';return n;}\n"
  "async function runCmd(c){if(!(await sendCmd(c)))return false;const id=S.wait,t=Date.now();while(Date.now()-t<5000){const d=S.d;if(d&&d.cmdId===id&&d.cmdRes!=='WAITING_FOR_ACK')return d.cmdRes==='ACKNOWLEDGED';await sleep(250);}return false;}\n"
  "function calFinish(tok){if(tok!==CM.t)return;const n=crProg(),all=n===3;$('cmT').textContent=all?'System calibrated':'Calibration needed';$('cmP').textContent=all?'All three checks passed.':n+' of 3 ready · skip now and calibrate later';$('cmGo').classList.toggle('hidden',!all);$('cmSkip').classList.toggle('hidden',all);if(all&&!CM.manual)setTimeout(()=>{if(CM.t===tok)calClose();},1700);}\n"
  "async function calScan(tok){\n"
  " for(const c of CR){if(tok!==CM.t)return;crBtn(c.k,'');if(!(await chk(c,tok)))return;\n"
  "  const d=S.d||{};\n"
  "  if(!(S.up&&d.link)){crSet(c.k,'need','Waiting for the sensor controller');continue;}\n"
  "  if(c.ok(d))crSet(c.k,'ok',c.okT);else{crSet(c.k,'need',c.needT);crBtn(c.k,c.btn,()=>calDo(c.k));}\n"
  "  crProg();}\n"
  " calFinish(tok);}\n"
  "async function calDo(k){if(CM.busy)return;CM.busy=true;const tok=CM.t;document.querySelectorAll('.cbtn').forEach(b=>b.disabled=true);\n"
  " const retry=(t)=>crBtn(k,t,()=>calDo(k));\n"
  " try{\n"
  " if(k==='air'){crSet(k,'run','Capturing baseline in clean air…');const[ok]=await Promise.all([runCmd('CAL,MQ'),sleep(2200)]);\n"
  "  if(ok){try{localStorage.setItem('ri-air',Date.now());}catch(e){}crSet(k,'ok','Clean-air baseline saved');}else{crSet(k,'need','Not confirmed · try again');retry('Retry');}}\n"
  " else if(k==='wat'){const dry=CM.w===0;crSet(k,'run',dry?'Capturing dry reference…':'Capturing wet reference…');\n"
  "  const[ok]=await Promise.all([runCmd(dry?'CAL,WATER_DRY':'CAL,WATER_WET'),sleep(1800)]);\n"
  "  if(ok&&dry){CM.w=1;crSet(k,'need','Dry saved · now place the probe in water');retry('Capture wet');}\n"
  "  else if(ok){CM.w=0;crSet(k,'ok','Dry and wet levels saved');}\n"
  "  else{crSet(k,'need',dry?'Keep the probe dry · try again':'Probe in water · try again');retry('Retry');}}\n"
  " else{crSet(k,'run','Measuring no-flow baseline…');const r=await act('/flow/zero/start');\n"
  "  if(r.st<300){for(let i=10;i>0;i--){crSet(k,'run','Measuring no-flow baseline · '+i+' s');await sleep(1000);}\n"
  "   for(let i=0;i<8&&!(S.d&&S.d.flowZeroSet);i++)await sleep(500);\n"
  "   if(S.d&&S.d.flowZeroSet)crSet(k,'ok','No-flow baseline saved');else{crSet(k,'need',(S.d&&S.d.flowCalMsg)||'Not enough samples · try again');retry('Retry');}}\n"
  "  else{crSet(k,'need','Could not start · try again');retry('Retry');}}\n"
  " }catch(e){crSet(k,'need','Something went wrong · try again');retry('Retry');}\n"
  " CM.busy=false;document.querySelectorAll('.cbtn').forEach(b=>b.disabled=false);calFinish(tok);}\n"
  "async function calOpen(manual){const tok=++CM.t;CM.manual=!!manual;CM.w=0;CM.busy=false;$('cmMon').innerHTML=mon('c');\n"
  " $('cmL').innerHTML=CR.map(c=>'<li id=\"cr'+c.k+'\" class=\"wait\"><span class=\"cic\"><svg viewBox=\"0 0 24 24\"><circle class=\"cring\" cx=\"12\" cy=\"12\" r=\"10\"/><path class=\"ctick\" d=\"M7 12.5l3.2 3.2L17 9\"/></svg></span><div class=\"ct\"><b>'+c.n+'</b><span id=\"crs'+c.k+'\">Waiting…</span></div><button class=\"cbtn hidden\" id=\"crb'+c.k+'\"></button></li>').join('');\n"
  " $('cmT').textContent='Calibration';$('cmP').textContent='Checking your sensors…';$('cmB').style.width='0';$('cmGo').classList.add('hidden');$('cmSkip').classList.remove('hidden');\n"
  " $('calm').classList.add('show');await calScan(tok);}\n"
  "function calClose(){CM.t++;CM.busy=false;$('calm').classList.remove('show');}\n"
  "$('cmGo').onclick=calClose;$('cmSkip').onclick=()=>{calClose();toast('Calibration skipped · saved values kept');};\n"
  "$('bGuide').onclick=()=>calOpen(true);$('calMon').innerHTML=mon('t');\n"
  "/* ---------- v10: link animation, circuit icons, air slider, long calibration ---------- */\n"
  "const LINKT='<svg viewBox=\"0 0 360 150\" class=\"lnk\" aria-hidden=\"true\"><path class=\"lw\" d=\"M116 75H168L174 61L180 89L186 61L192 89L198 61L204 75H210L214 66L220 98L226 52L230 75H244\"/><path class=\"lp\" pathLength=\"100\" d=\"M116 75H168L174 61L180 89L186 61L192 89L198 61L204 75H210L214 66L220 98L226 52L230 75H244\"/><text class=\"lt\" x=\"180\" y=\"122\">UART 9600 · 1 kΩ / 2.2 kΩ divider</text><text class=\"lt\" x=\"180\" y=\"40\">TX → RX</text><g class=\"cL\"><rect class=\"bd\" x=\"16\" y=\"26\" width=\"92\" height=\"98\" rx=\"7\"/><path d=\"M50 26a12 12 0 0 0 24 0\" fill=\"none\" stroke=\"#fff\" stroke-opacity=\".5\"/><g fill=\"#d1d1d6\"><rect x=\"8\" y=\"42\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"8\" y=\"58\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"8\" y=\"73\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"8\" y=\"90\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"8\" y=\"106\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"108\" y=\"42\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"108\" y=\"58\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"108\" y=\"73\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"108\" y=\"90\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"108\" y=\"106\" width=\"8\" height=\"5\" rx=\"1\"/></g><text x=\"62\" y=\"70\" font-size=\"10\" font-weight=\"700\" fill=\"#fff\" text-anchor=\"middle\" font-family=\"Helvetica,sans-serif\">ATmega</text><text x=\"62\" y=\"84\" font-size=\"10\" font-weight=\"700\" fill=\"#fff\" text-anchor=\"middle\" font-family=\"Helvetica,sans-serif\">328P</text><circle class=\"led\" cx=\"30\" cy=\"40\" r=\"3\"/></g><g class=\"cR\"><rect class=\"bd\" x=\"244\" y=\"28\" width=\"100\" height=\"94\" rx=\"10\"/><rect x=\"250\" y=\"34\" width=\"88\" height=\"82\" rx=\"6\" fill=\"none\" stroke=\"#fff\" stroke-opacity=\".35\"/><path d=\"M300 28v-12l6 6 6-10 6 10 6-6v12\" fill=\"none\" stroke=\"#fff\" stroke-opacity=\".8\" stroke-width=\"2.2\" stroke-linejoin=\"round\"/><rect x=\"268\" y=\"52\" width=\"46\" height=\"38\" rx=\"4\" fill=\"#000\" fill-opacity=\".28\" stroke=\"#fff\" stroke-opacity=\".4\"/><text x=\"291\" y=\"75\" font-size=\"9\" font-weight=\"700\" fill=\"#fff\" text-anchor=\"middle\" font-family=\"Helvetica,sans-serif\">ESP8266</text><g fill=\"#d1d1d6\"><rect x=\"236\" y=\"66\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"236\" y=\"79\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"336\" y=\"66\" width=\"8\" height=\"5\" rx=\"1\"/><rect x=\"336\" y=\"79\" width=\"8\" height=\"5\" rx=\"1\"/></g></g></svg>';\n"
  "function link(k){return LINKT.replace(/%K/g,k);}\n"
  "const GL={\"gw\": \"<rect class=\\\"c9b\\\" x=\\\"36\\\" y=\\\"26\\\" width=\\\"48\\\" height=\\\"46\\\" rx=\\\"5\\\"/><path class=\\\"c9s\\\" d=\\\"M44 36h16M44 44h28M44 52h12M44 60h22M42 26v-8l4 4 4-8 4 8 4-4v8\\\"/>\", \"mc\": \"<rect class=\\\"c9b\\\" x=\\\"42\\\" y=\\\"22\\\" width=\\\"36\\\" height=\\\"52\\\" rx=\\\"3\\\"/><path class=\\\"c9s\\\" d=\\\"M54 22a6 6 0 0 0 12 0M36 32h6M36 42h6M36 52h6M36 62h6M78 32h6M78 42h6M78 52h6M78 62h6\\\"/>\", \"sd\": \"<path class=\\\"c9b\\\" d=\\\"M42 24h26l12 12v34H42z\\\"/><path class=\\\"c9s\\\" d=\\\"M50 30v9M56 30v9M62 30v9\\\"/>\", \"rtc\": \"<circle class=\\\"c9b\\\" cx=\\\"60\\\" cy=\\\"48\\\" r=\\\"23\\\"/><path class=\\\"c9s\\\" d=\\\"M60 48V33M60 48l11 7\\\"/>\", \"mq\": \"<circle class=\\\"c9b\\\" cx=\\\"60\\\" cy=\\\"46\\\" r=\\\"21\\\"/><path class=\\\"c9s\\\" d=\\\"M46 38h28M44 46h32M46 54h28M52 67v6M68 67v6\\\"/>\", \"wp\": \"<rect class=\\\"c9b\\\" x=\\\"38\\\" y=\\\"24\\\" width=\\\"44\\\" height=\\\"30\\\" rx=\\\"4\\\"/><path class=\\\"c9s\\\" d=\\\"M48 54v18M60 54v18M72 54v18M42 40q4-4 8 0t8 0t8 0t8 0\\\"/>\", \"us\": \"<rect class=\\\"c9b\\\" x=\\\"34\\\" y=\\\"30\\\" width=\\\"52\\\" height=\\\"36\\\" rx=\\\"5\\\"/><circle class=\\\"c9s\\\" cx=\\\"48\\\" cy=\\\"48\\\" r=\\\"10\\\"/><circle class=\\\"c9s\\\" cx=\\\"72\\\" cy=\\\"48\\\" r=\\\"10\\\"/><circle class=\\\"c9s\\\" cx=\\\"48\\\" cy=\\\"48\\\" r=\\\"3\\\"/><circle class=\\\"c9s\\\" cx=\\\"72\\\" cy=\\\"48\\\" r=\\\"3\\\"/>\", \"fl\": \"<circle class=\\\"c9b\\\" cx=\\\"60\\\" cy=\\\"48\\\" r=\\\"22\\\"/><path class=\\\"c9s\\\" d=\\\"M60 48V32M60 48l14 8M60 48l-14 8\\\"/>\", \"pp\": \"<circle class=\\\"c9b\\\" cx=\\\"56\\\" cy=\\\"34\\\" r=\\\"8\\\"/><path class=\\\"c9s\\\" d=\\\"M42 70q0-18 14-18t14 18M76 32q7 8 0 16M82 28q10 12 0 24\\\"/>\", \"dc\": \"<rect class=\\\"c9b\\\" x=\\\"42\\\" y=\\\"22\\\" width=\\\"36\\\" height=\\\"50\\\" rx=\\\"3\\\"/><circle class=\\\"c9s\\\" cx=\\\"71\\\" cy=\\\"48\\\" r=\\\"2.5\\\"/><path class=\\\"c9s\\\" d=\\\"M42 72h36\\\"/>\", \"do\": \"<path class=\\\"c9b\\\" d=\\\"M42 22l26 7v36l-26 7z\\\"/><path class=\\\"c9s\\\" d=\\\"M42 22h36v50H42M62 48h2\\\"/>\", \"rep\": \"<rect class=\\\"c9b\\\" x=\\\"38\\\" y=\\\"24\\\" width=\\\"44\\\" height=\\\"46\\\" rx=\\\"5\\\"/><path class=\\\"c9s\\\" d=\\\"M48 48l8 8 16-16\\\"/>\"};\n"
  "const GK=['gw','mc','sd','rtc','mq','wp','us','fl','pp','dc','do','rep'];\n"
  "function circ(g,st){return '<svg class=\"c9 st-'+st+'\" viewBox=\"0 0 120 96\" aria-hidden=\"true\"><path class=\"c9t\" d=\"M6 34H22L30 42H38\"/><path class=\"c9t\" d=\"M6 62H22L30 54H38\"/><path class=\"c9t\" d=\"M114 34H98L90 42H82\"/><path class=\"c9t\" d=\"M114 62H98L90 54H82\"/><circle class=\"c9p\" cx=\"6\" cy=\"34\" r=\"3.5\"/><circle class=\"c9p\" cx=\"6\" cy=\"62\" r=\"3.5\"/><circle class=\"c9p\" cx=\"114\" cy=\"34\" r=\"3.5\"/><circle class=\"c9p\" cx=\"114\" cy=\"62\" r=\"3.5\"/>'+(GL[g]||'')+'<circle class=\"c9l\" cx=\"90\" cy=\"22\" r=\"3\"/>'+(st==='fail'?'<path class=\"c9x\" d=\"M16 26l12 16M28 26l-12 16\"/>':'')+'</svg>';}\n"
  "ic=function(c,t){const e=$('vIc');const st=/pass/.test(c)?'pass':/fail/.test(c)?'fail':/run/.test(c)?'run':'nt';e.className='circw';e.innerHTML=circ(GK[S.step]||'rep',st);};\n"
  "function upRows(){document.querySelectorAll('#vRepList .lr').forEach((r,k)=>{const o=r.querySelector('.ic');if(!o)return;const s=o.classList.contains('pass')?'pass':o.classList.contains('fail')?'fail':'nt';const m=document.createElement('div');m.className='cmini';m.innerHTML=circ(GK[k],s);o.replaceWith(m);});}\n"
  "function ensureSl(){if($('airSl'))return;const a=$('vAct');a.classList.add('wrap');const w=document.createElement('div');w.className='slw';\n"
  " w.innerHTML='<div class=\"slh\"><b>Air alert level</b><span id=\"slV\"></span></div><div class=\"slt\"><i id=\"slLive\"></i><input type=\"range\" id=\"airSl\" min=\"50\" max=\"1000\" step=\"10\" aria-label=\"Air alert level\"></div><div class=\"sls\"><span>Clean</span><span>raw ADC</span><span>Heavy</span></div><div class=\"note\" id=\"slN\"></div>';\n"
  " a.appendChild(w);const s=$('airSl');let v=600;try{v=+localStorage.getItem('ri-airlvl')||600;}catch(e){}s.value=v;s.oninput=()=>{try{localStorage.setItem('ri-airlvl',s.value);}catch(e){}updSl();};}\n"
  "function updSl(){const s=$('airSl'),d=S.d;if(!s||!d)return;const v=+s.value,r=d.air;$('slV').textContent=v;$('slLive').style.left=(Math.max(50,Math.min(1000,r))-50)/950*100+'%';$('slN').textContent='Live reading '+r+(r>=v?' is above':' is below')+' your alert level · ATmega class '+d.airS+' (alert level is a dashboard-side setting)';}\n"
  "const _wz=wizard;wizard=function(){_wz();if(S.step===11)upRows();if(S.step===4&&S.d){ensureSl();updSl();}};\n"
  "const CD={air:120,wat:30,flw:60};\n"
  "async function settle(k,tok,secs,txt,live){for(let s=secs;s>0;s--){if(tok!==CM.t)return false;const d=S.d||{};crSet(k,'run',txt+' · '+Math.floor(s/60)+':'+String(s%60).padStart(2,'0')+(live?' · raw '+live(d):''));await sleep(1000);}return tok===CM.t;}\n"
  "async function calDo(k){if(CM.busy)return;CM.busy=true;const tok=CM.t;document.querySelectorAll('.cbtn').forEach(b=>b.disabled=true);\n"
  " const retry=t=>crBtn(k,t,()=>calDo(k));$('cmP').textContent='Calibrating · keep conditions steady';\n"
  " try{\n"
  " if(k==='air'){if(!(await settle(k,tok,CD.air,'Stabilising in clean air',d=>d.air)))return;crSet(k,'run','Saving baseline…');const ok=await runCmd('CAL,MQ');if(tok!==CM.t)return;\n"
  "  if(ok){try{localStorage.setItem('ri-air',Date.now());}catch(e){}crSet(k,'ok','Clean-air baseline saved');}else{crSet(k,'need','Not confirmed · try again');retry('Retry');}}\n"
  " else if(k==='wat'){const dry=CM.w===0;if(!(await settle(k,tok,CD.wat,dry?'Settling dry probe':'Settling wet probe',d=>d.wRaw)))return;crSet(k,'run','Saving reference…');\n"
  "  const ok=await runCmd(dry?'CAL,WATER_DRY':'CAL,WATER_WET');if(tok!==CM.t)return;\n"
  "  if(ok&&dry){CM.w=1;crSet(k,'need','Dry saved · now place the probe in water');retry('Capture wet');}\n"
  "  else if(ok){CM.w=0;crSet(k,'ok','Dry and wet levels saved');}\n"
  "  else{crSet(k,'need',dry?'Keep the probe dry · try again':'Probe in water · try again');retry('Retry');}}\n"
  " else{crSet(k,'run','Starting no-flow baseline…');const r=await act('/flow/zero/start');\n"
  "  if(r.st<300){if(!(await settle(k,tok,CD.flw,'Measuring no-flow baseline',d=>(+d.flow||0).toFixed(2))))return;\n"
  "   for(let i=0;i<8&&!(S.d&&S.d.flowZeroSet);i++)await sleep(500);if(tok!==CM.t)return;\n"
  "   if(S.d&&S.d.flowZeroSet)crSet(k,'ok','No-flow baseline saved');else{crSet(k,'need',(S.d&&S.d.flowCalMsg)||'Not enough samples · try again');retry('Retry');}}\n"
  "  else{crSet(k,'need','Could not start · try again');retry('Retry');}}\n"
  " }catch(e){crSet(k,'need','Something went wrong · try again');retry('Retry');}\n"
  " finally{if(tok===CM.t){CM.busy=false;document.querySelectorAll('.cbtn').forEach(b=>b.disabled=false);calFinish(tok);}}}\n"
  "/* ---------- v11: 5 s checks, pop-out cards, big alert popups ---------- */\n"
  "async function chk(c,tok){const m=['Reading sensor…','Comparing with saved values…','Verifying…'];for(let i=0;i<3;i++){if(tok!==CM.t)return false;crSet(c.k,'run',m[i]);await sleep(1670);}return tok===CM.t;}\n"
  "let POP=null;\n"
  "function cardOpen(c){if(POP)return;const r=c.getBoundingClientRect(),ph=document.createElement('div');ph.style.height=r.height+'px';ph.style.gridColumn=getComputedStyle(c).gridColumn;c.parentNode.insertBefore(ph,c);c.classList.add('pout');const x=document.createElement('button');x.className='cx';x.textContent='✕';x.setAttribute('aria-label','Close');x.onclick=e=>{e.stopPropagation();cardClose();};c.appendChild(x);POP={c,ph,x};$('cbd').classList.add('show');setTimeout(()=>{try{drawCharts();}catch(e){}},80);}\n"
  "function cardClose(){if(!POP)return;const{c,ph,x}=POP;POP=null;x.remove();c.classList.remove('pout');ph.remove();$('cbd').classList.remove('show');setTimeout(()=>{try{drawCharts();}catch(e){}},80);}\n"
  "document.addEventListener('click',e=>{if(POP)return;const c=e.target.closest('#app .card');if(!c||e.target.closest('button,a,input,select,textarea,label,.seg,.chips'))return;cardOpen(c);});\n"
  "$('cbd').onclick=cardClose;document.addEventListener('keydown',e=>{if(e.key==='Escape')cardClose();});\n"
  "const AI={air:'<path d=\"M3 9h11a3 3 0 1 0-3-3M3 14h15a3 3 0 1 1-3 3M3 19h7\"/>',water:'<path d=\"M12 3s6 6.5 6 11a6 6 0 0 1-12 0c0-4.5 6-11 6-11z\"/>',link:'<path d=\"M2 9a15 15 0 0 1 20 0M5.5 12.5a10 10 0 0 1 13 0M9 16a5 5 0 0 1 6 0\"/><path d=\"M4 4l16 16\"/>',sd:'<path d=\"M6 3h8l4 4v14H6z\"/><path d=\"M10 7v3M13 7v3\"/>',chip:'<rect x=\"7\" y=\"7\" width=\"10\" height=\"10\" rx=\"1\"/><path d=\"M10 3v4M14 3v4M10 17v4M14 17v4M3 10h4M3 14h4M17 10h4M17 14h4\"/>'};\n"
  "const AL={},ALQ=[];let ALO=null;\n"
  "function alShow(o){ALQ.push(o);alNext();}\n"
  "function alNext(){if(ALO||!ALQ.length)return;if($('pop').classList.contains('show')||$('calm').classList.contains('show')){setTimeout(alNext,2500);return;}\n"
  " const o=ALO=ALQ.shift();$('asheet').className='asheet '+o.lvl;$('aIc').innerHTML='<svg viewBox=\"0 0 24 24\">'+AI[o.ic]+'</svg>';$('aT').textContent=o.t;$('aP').textContent=o.p;$('aW').textContent=o.w||'';$('aW').classList.toggle('hidden',!o.w);\n"
  " const b=$('aB');b.innerHTML='';(o.btns||[]).forEach(x=>{const e=document.createElement('button');e.className='btn '+(x.c||'');e.textContent=x.l;e.onclick=()=>{alClose();x.f();};b.appendChild(e);});\n"
  " const k=document.createElement('button');k.className='btn flat';k.textContent='Dismiss';k.onclick=alClose;b.appendChild(k);$('alp').classList.add('show');if(o.lvl==='crit')vib([200,80,200]);}\n"
  "function alClose(){$('alp').classList.remove('show');ALO=null;setTimeout(alNext,400);}\n"
  "function alRaise(k,o){const n=Date.now(),a=AL[k]||(AL[k]={on:false,last:0});if(a.on)return;a.on=true;if(!a.last||n-a.last>600000){a.last=n;alShow(o);}}\n"
  "function alClear(k){if(AL[k])AL[k].on=false;}\n"
  "function alertCheck(){const d=S.d,bt=$('boot');if(!d||(bt&&!bt.classList.contains('hidden')))return;\n"
  " if(!S.up||!d.link){S.dn=S.dn||Date.now();if(Date.now()-S.dn>8000)alRaise('link',{lvl:'crit',ic:'link',t:!S.up?'Gateway connection lost':'Sensor controller offline',p:!S.up?'This device lost the Restroom-Monitor Wi-Fi. Move closer to the NodeMCU and reconnect.':'No valid frames from the ATmega. Check its power and the UART wiring (D7 → divider → D1, D2 → D8, common GND).'});}else{S.dn=0;alClear('link');}\n"
  " if(!(S.up&&d.link))return;\n"
  " const fan=d.fan?'Exhaust fan running':'Exhaust fan off',w=fan+' · '+d.mode;\n"
  " const bb=[];if(d.mode==='MANUAL'&&!d.fan)bb.push({l:'Turn fan ON',c:'pri',f:()=>sendCmd('FAN,1')});if(d.mode==='MANUAL')bb.push({l:'Switch to AUTO',c:'',f:()=>sendCmd('MODE,AUTO')});\n"
  " if(d.airS==='BAD')alRaise('airBad',{lvl:'crit',ic:'air',t:'Air quality very bad',p:'The sensor reads '+d.air+' (raw). Ventilate the room and clean it as soon as possible.',w:w,btns:bb});else alClear('airBad');\n"
  " if(d.airS==='POOR')alRaise('airPoor',{lvl:'warn',ic:'air',t:'Air quality poor',p:'The sensor reads '+d.air+' (raw). Check the room and the exhaust fan.',w:w,btns:bb});else alClear('airPoor');\n"
  " if(!(d.bits&2)&&!d.wet)alRaise('wat',{lvl:'warn',ic:'water',t:'Water tank empty',p:'The probe reports no water. Refill the tank.'});else alClear('wat');\n"
  " [[1,'Air sensor fault','MQ-135 reading is stuck at the ADC limit. Check power and the A0 wiring.'],[2,'Water probe fault','Probe reading is stuck. Check the A1 wiring and power.'],[4,'Ultrasonic sensor fault','No echo from the HC-SR04. Check TRIG D5, ECHO D6 and 5 V.'],[8,'Clock fault','The DS3231 clock is missing or lost power. Check A4/A5 and the coin cell.']].forEach(f=>{if(d.bits&f[0])alRaise('f'+f[0],{lvl:'crit',ic:'chip',t:f[1],p:f[2]});else alClear('f'+f[0]);});\n"
  " if(!d.sd){S.sdn=S.sdn||Date.now();if(Date.now()-S.sdn>20000)alRaise('sd',{lvl:'warn',ic:'sd',t:'MicroSD not available',p:(d.sdMsg||'Card not ready')+'. Logging is paused and records are queued in RAM.'});}else{S.sdn=0;alClear('sd');}}\n"
  "const _rd=render;render=function(){_rd();try{alertCheck();}catch(e){}};\n"
  "/* ---------- start ---------- */\n"
  "poll();requestAnimationFrame(tick);let go=false;\n"
  "function after(){if(go)return;go=true;$('boot').classList.add('out');setTimeout(()=>$('boot').classList.add('hidden'),620);$('app').classList.remove('hidden');$('tabs').classList.remove('hidden');nav('home');setTimeout(()=>calOpen(false),650);}\n"
  "function startMainBoot(){const guard=setTimeout(()=>{console.warn('Boot guard timeout · opening dashboard');after();},18000);boot().then(()=>{clearTimeout(guard);after();}).catch(e=>{console.error('Startup sequence failed:',e);$('bootS').textContent='Opening dashboard…';$('bootB').style.width='100%';after();});}\n"
  "startMainBoot();\n"
  "</script>\n"
  "</body>\n"
  "</html>\n"
  "\n";

// ================================================================
//  HTTP handlers
// ================================================================
static void noCache() { server.sendHeader(F("Cache-Control"), F("no-store")); }

static void handleRoot() { noCache(); server.send_P(200, "text/html", PAGE); }

static void handleData() {
  JB j(jb, sizeof(jb));
  bool up = linkUp();
  uint32_t now = nowEpoch();
  uint32_t dayStart = now - (now % 86400UL);
  uint32_t eTod = timeValid ? countEntries(dayStart, now) : 0;
  uint32_t eHr = countEntries(now > 3600UL ? now - 3600UL : 0UL, now);
  j.f("{\"up\":%lu,\"heap\":%u,\"cl\":%d,\"link\":%d,\"age\":%lu,\"seq\":%lu,",
      (unsigned long)(millis() / 1000UL), (unsigned)ESP.getFreeHeap(), WiFi.softAPgetStationNum(), up ? 1 : 0,
      L.have ? (unsigned long)(millis() - lastPacketMs) : 0UL, (unsigned long)L.seq);
  j.raw("\"ts\":"); j.esc(L.ts);
  j.f(",\"tv\":%d,\"ep\":%lu,\"air\":%d,", timeValid ? 1 : 0, (unsigned long)now, L.air);
  j.raw("\"airS\":"); j.esc(L.airS);
  j.f(",\"ppl\":%ld,\"wRaw\":%d,\"wet\":%d,\"wLvl\":%.1f,\"dist\":%.1f,\"flowRaw\":%.3f,\"flow\":%.3f,\"totRaw\":%.3f,\"tot\":%.3f,",
      L.people, L.wRaw, L.wet ? 1 : 0, L.wLvl, L.dist, L.flow, correctedFlow(), L.total, correctedTotal());
  j.f("\"fan\":%d,", L.fan ? 1 : 0); j.raw("\"mode\":"); j.esc(L.mode);
  j.f(",\"rtc\":%d,\"bits\":%d,", L.rtc ? 1 : 0, L.bits);
  j.raw("\"ftxt\":"); j.esc(L.fault);
  j.f(",\"eTod\":%lu,\"eHr\":%lu,\"eSince\":%lu,\"cMin\":%ld,",
      (unsigned long)eTod, (unsigned long)eHr, (unsigned long)entriesSince, cleanMin);
  j.raw("\"cWhy\":"); j.esc(cleanWhy);
  float lSince = correctedTotal() - cleanTotal; if (lSince < 0.0f) { cleanTotal = correctedTotal(); lSince = 0.0f; }
  j.f(",\"cKnown\":%d,\"cEp\":%lu,\"cfgE\":%u,\"cfgM\":%u,\"adp\":%d,\"hi\":%d,\"effE\":%u,\"effM\":%u,\"lSince\":%.2f,\"sd\":%d,",
      cleanKnown ? 1 : 0, (unsigned long)cleanEpoch, (unsigned)cfgEnt, (unsigned)cfgMin, cfgAdapt ? 1 : 0, airHigh ? 1 : 0, (unsigned)effEnt, (unsigned)effMin, lSince, sdReady ? 1 : 0);
  j.raw("\"sdMsg\":"); j.esc(sdMsg);
  j.f(",\"q\":%u,\"qDrop\":%lu,\"lerr\":%lu,\"dBytes\":%lu,\"ok\":%lu,\"bad\":%lu,\"crc\":%lu,\"miss\":%lu,\"cmdId\":%lu,",
      (unsigned)qCount, (unsigned long)qDropped, (unsigned long)logErrors, (unsigned long)dataBytes,
      (unsigned long)validFrames, (unsigned long)badFrames, (unsigned long)crcErrors, (unsigned long)missedSeq, (unsigned long)cmdId);
  j.raw("\"cmdRes\":"); j.esc(cmdRes);
  j.raw(",\"lastAck\":"); j.esc(lastAck.c_str());
  j.f(",\"rs\":%lu,\"flowZero\":%.4f,\"flowScale\":%.5f,\"flowZeroSet\":%d,\"flowVolumeSet\":%d,\"flowZeroRun\":%d,\"flowTestRun\":%d,",
      (unsigned long)rawSeq, flowZero, flowScale, flowZeroSet ? 1 : 0, flowVolumeSet ? 1 : 0, flowZeroRunning ? 1 : 0, flowVolumeRunning ? 1 : 0);
  j.raw("\"flowCalMsg\":"); j.esc(flowCalMsg);
  j.f(",\"doorClosed\":%.2f,\"doorOpen\":%.2f,\"doorClosedSet\":%d,\"doorOpenSet\":%d,\"doorCalReady\":%d,\"doorCalRun\":%d,\"doorCalSamples\":%u,\"doorCalOpenTarget\":%d,\"doorState\":%u,\"doorCycles\":%lu,",
      doorClosedCm, doorOpenCm, doorClosedSet ? 1 : 0, doorOpenSet ? 1 : 0, doorCalReady() ? 1 : 0, doorCalRunning ? 1 : 0, (unsigned)doorCalSampleCount, doorCalTargetOpen ? 1 : 0, (unsigned)doorState, (unsigned long)doorCycles);
  j.raw("\"doorCalMsg\":"); j.esc(doorCalMsg);
  j.raw(",\"waterTestRunning\":"); j.raw(waterTestRunning ? "true" : "false");
  j.raw(",\"waterTestSeen\":"); j.raw(waterTestFlowSeen ? "true" : "false");
  j.raw(",\"waterTestResult\":"); j.esc(waterTestResult);
  j.f(",\"airTrendPerMin\":%.2f,\"airTrendReady\":%d,\"alertCount\":%u,\"alerts\":[", airTrendPerMin, airTrendReady ? 1 : 0, (unsigned)advisoryCount);
  for (uint8_t i = 0; i < advisoryCount; i++) {
    if (i) j.raw(",");
    j.f("{\"code\":%u,\"level\":%u,\"title\":", (unsigned)advisories[i].code, (unsigned)advisories[i].level);
    j.esc(advisories[i].title); j.raw(",\"detail\":"); j.esc(advisories[i].detail);
    j.raw(",\"action\":"); j.esc(advisories[i].action); j.raw("}");
  }
  j.raw("],\"countStatus\":"); j.esc("positive ATmega PEOPLE deltas; assumes PEOPLE is a cumulative entry counter");
  j.raw(",\"raw\":"); j.esc(lastRaw);
  j.raw("}");
  noCache(); server.send(200, "application/json", jb);
}

static void handleHistory() {
  JB j(hb, sizeof(hb));
  uint32_t now = nowEpoch();
  j.f("{\"tv\":%d,\"hNow\":%u,\"air\":[", timeValid ? 1 : 0, (unsigned)((now % 86400UL) / 3600UL));
  for (uint16_t i = 0; i < hCount; i++) { uint16_t idx = (hHead + HN - hCount + i) % HN; j.f(i ? ",%u" : "%u", (unsigned)hAir[idx]); }
  j.raw("],\"flow\":[");
  for (uint16_t i = 0; i < hCount; i++) { uint16_t idx = (hHead + HN - hCount + i) % HN; j.f(i ? ",%u" : "%u", (unsigned)hFlow[idx]); }
  j.raw("],\"waterRaw\":[");
  for (uint16_t i = 0; i < hCount; i++) { uint16_t idx = (hHead + HN - hCount + i) % HN; j.f(i ? ",%u" : "%u", (unsigned)hWaterRaw[idx]); }
  j.raw("],\"waterLevel\":[");
  for (uint16_t i = 0; i < hCount; i++) { uint16_t idx = (hHead + HN - hCount + i) % HN; j.f(i ? ",%d" : "%d", (int)hWaterLevel[idx]); }
  j.raw("],\"hours\":[");
  uint32_t curHour = now / 3600UL;
  for (int h = 0; h < 12; h++) {
    uint32_t hourIndex = curHour >= (uint32_t)(11 - h) ? curHour - (uint32_t)(11 - h) : 0UL;
    uint32_t hourStart = hourIndex * 3600UL;
    uint32_t hourEnd = hourStart + 3599UL;
    if (hourEnd > now) hourEnd = now;
    uint32_t c = countEntries(hourStart, hourEnd);
    j.f(h ? ",%lu" : "%lu", (unsigned long)c);
  }
  j.raw("]}");
  noCache(); server.send(200, "application/json", hb);
}

static void handleEntries() {
  uint32_t now = nowEpoch(), from = 0, to = now;
  if (server.hasArg("mins")) {
    long m = server.arg("mins").toInt(); if (m < 1) m = 1; if (m > 1440) m = 1440;
    from = ((uint32_t)m * 60UL <= now) ? now - (uint32_t)m * 60UL : 0;
  } else {
    if (!timeValid) { noCache(); server.send(200, "application/json", "{\"ok\":0,\"err\":\"RTC time not valid\"}"); return; }
    uint32_t midnight = now - now % 86400UL;
    if (server.hasArg("today")) { from = midnight; to = now; }
    else {
      long fm = server.arg("from").toInt(), tm = server.arg("to").toInt();
      if (fm < 0) fm = 0; if (fm > 1439) fm = 1439; if (tm < 0) tm = 0; if (tm > 1439) tm = 1439;
      from = midnight + (uint32_t)fm * 60UL;
      to = midnight + (uint32_t)tm * 60UL + 59UL;
      if (tm < fm) from -= 86400UL;
      if (to > now) to = now;
    }
  }
  uint32_t n = countEntries(from, to);
  char o[100]; snprintf(o, sizeof(o), "{\"ok\":1,\"n\":%lu,\"resolution\":\"event timestamp\"}", (unsigned long)n);
  noCache(); server.send(200, "application/json", o);
}

static bool allowed(const String& c) {
  return c == "FAN,1" || c == "FAN,0" || c == "MODE,AUTO" || c == "MODE,MANUAL" || c == "COUNT,RESET" || c == "CAL,MQ" || c == "CAL,WATER_DRY" || c == "CAL,WATER_WET";
}

static void handleControl() {
  if (!server.hasArg("cmd")) { server.send(400, "application/json", "{\"error\":\"missing cmd\"}"); return; }
  String c = server.arg("cmd"); c.trim();
  if (!allowed(c)) { server.send(400, "application/json", "{\"error\":\"unsupported command\"}"); return; }
  if (!linkUp()) { server.send(503, "application/json", "{\"error\":\"ATmega offline - command not sent\"}"); return; }
  if (pendingCmd.length()) { server.send(409, "application/json", "{\"error\":\"previous command still waiting for ACK\"}"); return; }
  Uno.println(c);
  pendingCmd = c; cmdRes = "WAITING_FOR_ACK"; cmdStart = millis(); cmdId++;
  commandLog("SEND", c);
  char o[72]; snprintf(o, sizeof(o), "{\"queued\":true,\"id\":%lu}", (unsigned long)cmdId);
  noCache(); server.send(202, "application/json", o);
}

static void handleClean() {
  if (!timeValid) { server.send(409, "application/json", "{\"error\":\"RTC time not valid\"}"); return; }
  uint16_t airB = linkUp() ? (uint16_t)L.air : 0;
  pushClean(nowEpoch(), airB, (uint16_t)(entriesSince > 65535UL ? 65535UL : entriesSince));
  cleanAfterMs = millis() + 300000UL; if (!cleanAfterMs) cleanAfterMs = 1;
  cleanTotal = correctedTotal();
  event("CLEANED air=%u n=%lu", (unsigned)airB, (unsigned long)entriesSince);
  cleanEpoch = nowEpoch(); entriesSince = 0; savePersist(); airHighFrac = 0.0f;
  noCache(); server.send(200, "application/json", "{\"ok\":true}");
}

static void handleDoorCalibrationStart() {
  if (!linkUp()) { server.send(503, "application/json", "{\"error\":\"ATmega link offline\"}"); return; }
  if (doorCalRunning) { server.send(409, "application/json", "{\"error\":\"door capture already running\"}"); return; }
  String pos = server.arg("pos"); pos.toLowerCase();
  if (pos != "open" && pos != "closed") { server.send(400, "application/json", "{\"error\":\"pos must be open or closed\"}"); return; }
  doorCalTargetOpen = (pos == "open");
  doorCalRunning = true; doorCalStartMs = millis();
  doorCalLastSeq = L.seq; doorCalSampleCount = 0;
  snprintf(doorCalMsg, sizeof(doorCalMsg), "Capturing %s position for 10 seconds · hold still for at least 5 fresh readings", doorCalTargetOpen ? "OPEN" : "CLOSED");
  noCache(); server.send(202, "application/json", "{\"ok\":true,\"message\":\"Door distance capture started for 10 seconds; hold still\"}");
}

static void handleDoorCalibrationReset() {
  doorClosedCm = doorOpenCm = -1.0f; doorClosedSet = doorOpenSet = false;
  doorCalRunning = false; doorCalSampleCount = 0; doorCalLastSeq = L.seq;
  doorState = doorCandidate = 0; doorStateInitialised = false;
  snprintf(doorCalMsg, sizeof(doorCalMsg), "Door calibration cleared · capture both positions again");
  saveDoorCalibration(); event("DOOR_CAL_RESET");
  noCache(); server.send(200, "application/json", "{\"ok\":true,\"message\":\"Door calibration cleared\"}");
}

static void handleWaterTestStart() {
  if (!linkUp()) { server.send(503, "application/json", "{\"error\":\"ATmega link offline\"}"); return; }
  if (waterTestRunning) { server.send(409, "application/json", "{\"error\":\"water test already running\"}"); return; }
  waterTestRunning = true; waterTestFlowSeen = false; waterTestFailed = false; waterTestStartMs = millis();
  snprintf(waterTestResult, sizeof(waterTestResult), "RUNNING · open the test tap and keep it open for 18 seconds");
  event("WATER_FLOW_TEST_START"); noCache();
  server.send(202, "application/json", "{\"ok\":true,\"message\":\"Test started · open a tap now and keep it open for 18 seconds\"}");
}

static void handleWaterTestCancel() {
  waterTestRunning = false; waterTestFlowSeen = false; waterTestFailed = false;
  snprintf(waterTestResult, sizeof(waterTestResult), "Cancelled by operator");
  event("WATER_FLOW_TEST_CANCEL"); noCache();
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Water flow test cancelled\"}");
}

static void handleFlowZeroStart() {
  if (!linkUp()) { server.send(503, "application/json", "{\"error\":\"ATmega link offline\"}"); return; }
  if (flowZeroRunning) { server.send(409, "application/json", "{\"error\":\"zero calibration already running\"}"); return; }
  flowZeroRunning = true; flowZeroStartMs = millis(); flowZeroSamples = 0; flowZeroSum = 0.0;
  snprintf(flowCalMsg, sizeof(flowCalMsg), "Sampling no-flow baseline · keep water OFF for 10 seconds");
  noCache(); server.send(202, "application/json", "{\"ok\":true,\"message\":\"No-flow zero sampling started for 10 seconds\"}");
}

static void handleFlowVolumeStart() {
  if (!linkUp()) { server.send(503, "application/json", "{\"error\":\"ATmega link offline\"}"); return; }
  if (flowVolumeRunning) { server.send(409, "application/json", "{\"error\":\"volume test already running\"}"); return; }
  flowVolumeStartTotal = L.total; flowVolumeStartMs = millis(); flowVolumeRunning = true;
  snprintf(flowCalMsg, sizeof(flowCalMsg), "Volume test active · pass a measured amount of water, then enter actual litres");
  noCache(); server.send(202, "application/json", "{\"ok\":true,\"message\":\"Volume test started\"}");
}

static void handleFlowVolumeFinish() {
  if (!linkUp()) { flowVolumeRunning = false; server.send(503, "application/json", "{\"error\":\"ATmega link offline\"}"); return; }
  if (!flowVolumeRunning) { server.send(409, "application/json", "{\"error\":\"start a volume test first\"}"); return; }
  float actual = server.arg("litres").toFloat();
  float rawDelta = L.total - flowVolumeStartTotal;
  if (!isfinite(actual) || actual < 0.01f || actual > 100.0f) { server.send(400, "application/json", "{\"error\":\"measured volume must be 0.01 to 100 litres\"}"); return; }
  if (millis() - flowVolumeStartMs < 3000UL || rawDelta <= 0.0001f || !isfinite(rawDelta)) {
    flowVolumeRunning = false;
    snprintf(flowCalMsg, sizeof(flowCalMsg), "Volume calibration failed · no positive raw total change detected");
    server.send(409, "application/json", "{\"error\":\"no volume change detected; check flow sensor and ATmega total-litre field\"}"); return;
  }
  float factor = actual / rawDelta;
  if (!isfinite(factor) || factor < 0.05f || factor > 20.0f) {
    flowVolumeRunning = false;
    server.send(400, "application/json", "{\"error\":\"calibration factor outside safe range (0.05 to 20)\"}"); return;
  }
  flowScale = factor; flowVolumeSet = true; flowVolumeRunning = false;
  saveFlowCalibration();
  snprintf(flowCalMsg, sizeof(flowCalMsg), "Volume calibrated · raw %.3f L → measured %.3f L · factor %.5f", rawDelta, actual, flowScale);
  event("FLOW_VOLUME_CAL factor=%.5f raw=%.3f actual=%.3f", flowScale, rawDelta, actual);
  noCache(); server.send(200, "application/json", "{\"ok\":true,\"message\":\"Volume calibration saved to EEPROM\"}");
}

static void handleFlowReset() {
  flowZero = 0.0f; flowScale = 1.0f; flowZeroSet = false; flowVolumeSet = false;
  flowZeroRunning = false; flowVolumeRunning = false;
  saveFlowCalibration();
  snprintf(flowCalMsg, sizeof(flowCalMsg), "Calibration reset · default scale 1.0000");
  event("FLOW_CAL_RESET");
  noCache(); server.send(200, "application/json", "{\"ok\":true,\"message\":\"Flow calibration reset\"}");
}

static void handleSdTest() {
  if (downloading) { server.send(409, "application/json", "{\"message\":\"download in progress\"}"); return; }
  bool ok = sdInit();
  char o[180]; snprintf(o, sizeof(o), "{\"ok\":%s,\"message\":\"SD: %s\"}", ok ? "true" : "false", sdMsg);
  noCache(); server.send(ok ? 200 : 503, "application/json", o);
}

static void handlePhoto() {
  if (!sdReady) { server.send(503, "text/plain", "MicroSD unavailable"); return; }
  String name = server.arg("f"); name.trim();
  const char* allowedNames[] = {"nodemcu.jpg", "atmega328p.jpg", "ds3231.jpg", "mq135.jpg", "water_probe.jpg", "hcsr04.jpg", "flow_sensor.jpg"};
  const char* selected = NULL;
  for (uint8_t i = 0; i < 7; i++) if (name == allowedNames[i]) { selected = allowedNames[i]; break; }
  if (!selected) { server.send(400, "text/plain", "Photo name not allowed"); return; }
  char path[32]; snprintf(path, sizeof(path), "/%s", selected);
  File photo = SD.open(path, FILE_READ);
  if (!photo) { server.send(404, "text/plain", "Photo not found on MicroSD"); return; }
  noCache();
  server.sendHeader(F("Content-Disposition"), F("inline"));
  server.streamFile(photo, "image/jpeg");
  photo.close();
}

static void handleDownload() {
  String a = server.arg("f"); int k = -1;
  for (int i = 0; i < K_N; i++) if (a == NAMES[i]) k = i;
  if (k < 0) { server.send(404, "text/plain", "unknown file"); return; }
  if (!sdReady) { server.send(503, "text/plain", String("MicroSD unavailable: ") + sdMsg); return; }
  // Flush pending records before opening the read handle and taking the snapshot.
  for (uint8_t i = 0; i < 20 && qCount && sdReady; ++i) drainQueue();
  if (!sdReady || qCount) { server.send(503, "text/plain", "Log queue is not empty or the MicroSD is not ready; retry after SD recovers"); return; }
  File fl = SD.open(PATHS[k], FILE_READ);
  if (!fl) { server.send(404, "text/plain", "CSV file not found on MicroSD"); return; }
  downloading = true;
  size_t left = fl.size(), total = left, sent = 0;
  char disp[88]; snprintf(disp, sizeof(disp), "attachment; filename=\"restroom_%s_last.csv\"", NAMES[k]);
  server.sendHeader(F("Content-Disposition"), disp);
  server.sendHeader(F("Cache-Control"), F("no-store, no-cache, must-revalidate"));
  server.sendHeader(F("Pragma"), F("no-cache"));
  server.setContentLength(total);
  server.send(200, "text/csv; charset=utf-8", "");
  uint8_t buf[512];
  WiFiClient& client = server.client();
  while (left && client.connected()) {
    size_t want = left < sizeof(buf) ? left : sizeof(buf);
    int n = fl.read(buf, want);
    if (n <= 0) break;

    // Write this RAM buffer directly. ESP8266WebServer's sendContent(const char*,size)
    // overload is a PROGMEM path, so it must not be used with this stack buffer.
    size_t off = 0;
    uint32_t stalledAt = millis();
    while (off < (size_t)n && client.connected()) {
      size_t w = client.write(buf + off, (size_t)n - off);
      if (w) { off += w; stalledAt = millis(); }
      else {
        pollUart(); serviceLink(); yield();
        if (millis() - stalledAt > 5000UL) break;
      }
    }
    if (off != (size_t)n) break;   // disconnected or socket stalled; browser sees incomplete length
    left -= (size_t)n; sent += (size_t)n;
    pollUart();                    // drain SoftwareSerial while the browser downloads
    serviceLink();
    yield();
  }
  fl.close();
  downloading = false;
  Serial.printf("[HTTP] CSV %s streamed: %u/%u bytes\n", NAMES[k], (unsigned)sent, (unsigned)total);
}

static void sendChunk(JB& j) { server.sendContent(String(j.b)); j.n = 0; j.b[0] = 0; }

// 24 h analytics: 5-minute buckets (oldest first), weekday x hour heat-map, clean log. Streamed in chunks.
static void handleAn() {
  char buf[512]; JB j(buf, sizeof(buf));
  noCache(); server.setContentLength(CONTENT_LENGTH_UNKNOWN); server.send(200, "application/json", "");
  uint32_t last = curBk ? curBk - 1 : 0; uint16_t n = curBk ? BN : 0;
  j.f("{\"tv\":%d,\"now\":%lu,\"t0\":%lu,\"n\":%u,\"air\":[", timeValid ? 1 : 0, (unsigned long)nowEpoch(), (unsigned long)(n ? (last - (BN - 1)) * 300UL : 0UL), (unsigned)n);
  for (uint8_t a = 0; a < 4; a++) {
    for (uint16_t i = 0; i < n; i++) {
      uint16_t k = (last - (BN - 1) + i) % BN;
      unsigned v = a == 0 ? bAir[k] : a == 1 ? bEnt[k] : a == 2 ? bFan[k] : bLit[k];
      j.f(i ? ",%u" : "%u", v);
      if (j.n > 440) sendChunk(j);
    }
    j.raw(a == 0 ? "],\"ent\":[" : a == 1 ? "],\"fan\":[" : a == 2 ? "],\"lit\":[" : "],\"heat\":[");
  }
  for (uint8_t d = 0; d < 7; d++) {
    j.raw(d ? ",[" : "[");
    for (uint8_t h = 0; h < 24; h++) j.f(h ? ",%u" : "%u", (unsigned)heat[d][h]);
    j.raw("]"); if (j.n > 400) sendChunk(j);
  }
  j.raw("],\"cl\":[");
  for (uint8_t i = 0; i < cleanN; i++) j.f("%s{\"e\":%lu,\"b\":%u,\"a\":%u,\"n\":%u}", i ? "," : "", (unsigned long)cleans[i].epoch, (unsigned)cleans[i].airB, (unsigned)cleans[i].airA, (unsigned)cleans[i].ent);
  j.raw("]}");
  sendChunk(j); server.sendContent("");
}

static void handleFiles() {
  JB j(jb, sizeof(jb)); j.raw("{");
  for (uint8_t k = 0; k < K_N; k++) {
    unsigned long sz = 0;
    if (sdReady && !downloading) { File f = SD.open(PATHS[k], FILE_READ); if (f) { sz = f.size(); f.close(); } }
    j.f("%s\"%s\":%lu", k ? "," : "", NAMES[k], sz);
  }
  j.raw("}"); noCache(); server.send(200, "application/json", jb);
}

static void handleCfg() {
  long e = server.arg("e").toInt(), m = server.arg("m").toInt();
  if (e < 5 || e > 200 || m < 20 || m > 1440) { server.send(400, "application/json", "{\"error\":\"entries 5-200 and minutes 20-1440 required\"}"); return; }
  cfgEnt = (uint16_t)e; cfgMin = (uint16_t)m; cfgAdapt = server.arg("a") != "0"; saveCfg();
  event("CLEAN_RULE entries=%u minutes=%u adaptive=%d", (unsigned)cfgEnt, (unsigned)cfgMin, cfgAdapt ? 1 : 0);
  noCache(); server.send(200, "application/json", "{\"ok\":true,\"message\":\"Cleaning rule saved\"}");
}

// Epoch (seconds) of a CSV row; 0 when it has no usable timestamp (header, "--").
static uint32_t csvLineEpoch(uint8_t k, const char* ln) {
  if (k == K_BKT) return (uint32_t)strtoul(ln, NULL, 10);
  const char* c = strchr(ln, ','); if (!c) return 0;
  if (k == K_ENT) return (uint32_t)strtoul(c + 1, NULL, 10);
  uint32_t e; return epochFromStamp(c + 1, &e) ? e : 0;
}

// Date-range export: binary-search the (time-ordered) CSV, then stream only rows from..to.
static void handleExport() {
  String a = server.arg("f"); int k = -1;
  for (int i = 0; i < K_N; i++) if (a == NAMES[i]) k = i;
  if (k < 0) { server.send(404, "text/plain", "unknown file"); return; }
  if (!sdReady) { server.send(503, "text/plain", String("MicroSD unavailable: ") + sdMsg); return; }
  uint32_t from = strtoul(server.arg("from").c_str(), NULL, 10), to = strtoul(server.arg("to").c_str(), NULL, 10);
  if (!to) to = 0xFFFFFFFFUL;
  if (to < from) { server.send(400, "text/plain", "bad range"); return; }
  for (uint8_t i = 0; i < 20 && qCount && sdReady; ++i) drainQueue();
  File fl = SD.open(PATHS[k], FILE_READ);
  if (!fl) { server.send(404, "text/plain", "CSV file not found on MicroSD"); return; }
  downloading = true;
  String hdr = fl.readStringUntil('\n'); size_t hdrEnd = fl.position();
  size_t lo = hdrEnd, hi = fl.size();
  while (hi - lo > 4096) {
    size_t mid = lo + (hi - lo) / 2;
    fl.seek(mid); fl.readStringUntil('\n');
    String ln = fl.readStringUntil('\n');
    uint32_t e = csvLineEpoch(k, ln.c_str());
    if (e && e < from) lo = mid; else hi = mid;
    pollUart(); yield();
  }
  fl.seek(lo); if (lo != hdrEnd) fl.readStringUntil('\n');
  char disp[112]; snprintf(disp, sizeof(disp), "attachment; filename=\"restroom_%s_%lu_%lu.csv\"", NAMES[k], (unsigned long)from, (unsigned long)(to == 0xFFFFFFFFUL ? 0UL : to));
  server.sendHeader(F("Content-Disposition"), disp); noCache();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); server.send(200, "text/csv; charset=utf-8", "");
  String out; out.reserve(700); out += hdr; out += '\n';
  uint32_t rows = 0;
  while (fl.available() && server.client().connected()) {
    String ln = fl.readStringUntil('\n'); if (!ln.length()) continue;
    uint32_t e = csvLineEpoch(k, ln.c_str());
    if (!e || e < from) continue;
    if (e > to) break;
    out += ln; out += '\n'; rows++;
    if (out.length() >= 512) { server.sendContent(out); out = ""; pollUart(); serviceLink(); yield(); }
  }
  if (out.length()) server.sendContent(out);
  server.sendContent("");
  fl.close(); downloading = false;
  Serial.printf("[HTTP] export %s rows=%lu\n", NAMES[k], (unsigned long)rows);
}

static void handleDiag() {
  JB j(jb, sizeof(jb));
  j.f("{\"ap\":\"%s\",\"ssid\":\"%s\",\"clients\":%d,\"uptime\":%lu,\"heap\":%u,\"chipId\":%lu,\"flash\":%lu,\"uartBytes\":%lu,\"valid\":%lu,\"bad\":%lu,\"crc\":%lu,\"missed\":%lu,\"dup\":%lu,\"ooo\":%lu,\"ack\":%lu,\"nack\":%lu,\"sd\":%d,\"queued\":%u,\"dropped\":%lu,\"logErrors\":%lu,\"doorCycles\":%lu,\"doorCal\":%d,\"waterTest\":%d}",
      WiFi.softAPIP().toString().c_str(), AP_NAME, WiFi.softAPgetStationNum(), (unsigned long)(millis() / 1000UL), (unsigned)ESP.getFreeHeap(), (unsigned long)ESP.getChipId(), (unsigned long)ESP.getFlashChipSize(),
      (unsigned long)uartBytes, (unsigned long)validFrames, (unsigned long)badFrames, (unsigned long)crcErrors, (unsigned long)missedSeq, (unsigned long)dupFrames, (unsigned long)oooFrames,
      (unsigned long)ackCount, (unsigned long)nackCount, sdReady ? 1 : 0, (unsigned)qCount, (unsigned long)qDropped, (unsigned long)logErrors, (unsigned long)doorCycles, doorCalReady() ? 1 : 0, waterTestRunning ? 1 : 0);
  noCache(); server.send(200, "application/json", jb);
}

// ================================================================
//  setup / loop
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // Start the access point before peripherals. This isolates Wi-Fi startup
  // from SoftwareSerial, SPI, SD initialization and EEPROM-dependent work.
  Serial.println();
  Serial.println(F("[WIFI] Starting AP..."));
  Serial.printf("[MEM] free heap before WiFi: %u bytes\n", (unsigned)ESP.getFreeHeap());
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(250);
  WiFi.mode(WIFI_AP);
  delay(500);
  Serial.printf("[MEM] free heap after WIFI_AP mode: %u bytes\n", (unsigned)ESP.getFreeHeap());
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(17.0f);   // conservative, stable output-power setting

  bool cfgOk = WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255,255,255,0));
  Serial.print(F("[WIFI] softAPConfig="));
  Serial.println(cfgOk ? F("OK") : F("FAILED"));
  Serial.printf("[MEM] free heap after softAPConfig: %u bytes\n", (unsigned)ESP.getFreeHeap());

  bool apOk = false;
  const uint8_t channels[] = {1, 6, 11};
  for (uint8_t i = 0; i < 3 && !apOk; ++i) {
    Serial.print(F("[WIFI] AP attempt channel "));
    Serial.println(channels[i]);
    apOk = WiFi.softAP(AP_NAME, AP_PASS, channels[i], false, 4);
    delay(700);
    Serial.printf("[MEM] free heap after AP attempt: %u bytes\n", (unsigned)ESP.getFreeHeap());
    IPAddress checkIP = WiFi.softAPIP();
    bool ipValid = (checkIP[0] == 192 && checkIP[1] == 168 &&
                    checkIP[2] == 4 && checkIP[3] == 1);
    if (apOk && ipValid && WiFi.getMode() == WIFI_AP) {
      Serial.print(F("[WIFI] AP READY SSID="));
      Serial.print(AP_NAME);
      Serial.print(F(" IP="));
      Serial.print(checkIP);
      Serial.print(F(" channel="));
      Serial.println(channels[i]);
    } else {
      Serial.print(F("[WIFI] attempt failed; softAP="));
      Serial.print(apOk);
      Serial.print(F(" IP="));
      Serial.println(checkIP);
      WiFi.softAPdisconnect(false);
      delay(300);
      apOk = false;
    }
  }

  if (apOk) {
    dnsServer.start(DNS_PORT, "*", AP_IP);
    Serial.println(F("[WIFI] Captive DNS started"));
  } else {
    Serial.println(F("[WIFI] ERROR: AP FAILED. Check power, board and ESP8266 core."));
    Serial.printf("[MEM] free heap after AP failure: %u bytes\n", (unsigned)ESP.getFreeHeap());
  }

  // Initialize peripherals only after the AP has had first chance to start.
  Uno.begin(UART_BAUD);
  SPI.begin();
  loadFlowCalibration();
  loadDoorCalibration();
  loadCfg();

  // SD errors are reported in the dashboard; Wi-Fi remains available.
  // Do not add SD-library allocations when heap is already critically low.
  if (ESP.getFreeHeap() >= 12000) sdInit();
  else { sdReady = false; sdMsg = "deferred: insufficient free heap"; Serial.printf("[SD] skipped; free heap=%u bytes\n", (unsigned)ESP.getFreeHeap()); }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.on("/entries", handleEntries);
  server.on("/control", handleControl);
  server.on("/clean", handleClean);
  server.on("/an", handleAn);
  server.on("/files", handleFiles);
  server.on("/cfg", handleCfg);
  server.on("/export", handleExport);
  server.on("/manifest.json", []() { server.send_P(200, "application/manifest+json", MANIFEST); });
  server.on("/icon.png", []() { server.sendHeader(F("Cache-Control"), F("max-age=86400")); server.send_P(200, "image/png", (PGM_P)ICON_PNG, sizeof(ICON_PNG)); });
  server.on("/sdtest", handleSdTest);
  server.on("/photo", handlePhoto);
  server.on("/download", handleDownload);
  server.on("/flow/zero/start", handleFlowZeroStart);
  server.on("/flow/volume/start", handleFlowVolumeStart);
  server.on("/flow/volume/finish", handleFlowVolumeFinish);
  server.on("/flow/reset", handleFlowReset);
  server.on("/door/cal/start", handleDoorCalibrationStart);
  server.on("/door/cal/reset", handleDoorCalibrationReset);
  server.on("/water/test/start", handleWaterTestStart);
  server.on("/water/test/cancel", handleWaterTestCancel);
  server.on("/logs", []() { server.sendHeader("Location", "/download?f=data"); server.send(302, "text/plain", ""); });
  server.on("/diag", handleDiag);
  server.on("/wifi", handleDiag);
  server.on("/ap-status", []() {
    String body = String("{\"ap\":") +
      ((WiFi.getMode() == WIFI_AP && WiFi.softAPIP()[0] != 0) ? "true" : "false") +
      ",\"ssid\":\"" + AP_NAME + "\",\"ip\":\"" +
      WiFi.softAPIP().toString() + "\",\"stations\":" +
      String(WiFi.softAPgetStationNum()) + "}";
    server.send(200, "application/json", body);
  });
  server.on("/health", handleDiag);
  server.on("/favicon.ico", []() { server.send(204, "text/plain", ""); });
  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();

  event("BOOT sd=%d ap=%d cfg=%d", sdReady ? 1 : 0, apOk ? 1 : 0, cfgOk ? 1 : 0);
  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" RESTROOM INTELLIGENCE - iOS-INSPIRED UI"));
  Serial.println(F("=============================================="));
  Serial.print(F("[AP]  "));
  if (WiFi.getMode() == WIFI_AP && WiFi.softAPIP()[0] != 0) {
    Serial.print(AP_NAME); Serial.print(F(" / ")); Serial.print(AP_PASS);
    Serial.print(F(" -> http://")); Serial.println(WiFi.softAPIP().toString());
  } else {
    Serial.println(F("FAILED - no access point active"));
  }
  Serial.print(F("[SD]  ")); Serial.println(sdMsg);
  Serial.println(F("[UART] D1 RX / D2 TX @ 9600"));
  Serial.print(F("[DOOR] ready=")); Serial.print(doorCalReady() ? F("yes") : F("no")); Serial.print(F(" closed=")); Serial.print(doorClosedCm, 1); Serial.print(F(" open=")); Serial.print(doorOpenCm, 1); Serial.print(F(" cycles=")); Serial.println((unsigned long)doorCycles);
  Serial.print(F("[FLOW] zero=")); Serial.print(flowZero, 4); Serial.print(F(" scale=")); Serial.println(flowScale, 5);
}

void loop() {
  dnsServer.processNextRequest();
  pollUart();
  server.handleClient();
  pollUart();
  serviceLink();
  serviceSd();
  serviceHistory();
  serviceCleaning();
  serviceBuckets();
  serviceFlowCalibration();
  serviceDoorCalibration();
  serviceAdvisories();
  serviceHeap();
  yield();
}
