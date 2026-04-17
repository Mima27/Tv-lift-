// ============================================================
//  mega2560_logic.ino
//  Logica principale per ATmega2560 — scheda Mega2560 WiFi R3
//
//  CONFIGURAZIONE DIP SWITCH (8 switch sulla scheda)
//  ┌────────┬──────┬──────┬──────────────────────────────────┐
//  │ Fase   │ ON   │ OFF  │ Note                             │
//  ├────────┼──────┼──────┼──────────────────────────────────┤
//  │ Flash  │ 1,2  │ 3-8  │ Programma ATmega2560 via USB     │
//  │ Mega   │      │      │                                  │
//  ├────────┼──────┼──────┼──────────────────────────────────┤
//  │ Flash  │ 3,4, │ 1,2, │ Programma ESP8266 via USB        │
//  │ ESP    │ 7,8  │ 5,6  │ (GPIO0=LOW per boot mode)        │
//  ├────────┼──────┼──────┼──────────────────────────────────┤
//  │ Usoper │ 1,2, │ 3,4, │ Funzionamento normale:           │
//  │ norma  │ 5,6  │ 7,8  │ USB→Mega(debug), Mega↔ESP via   │
//  │        │      │      │ Serial3 (pin 14/15)              │
//  └────────┴──────┴──────┴──────────────────────────────────┘
//
//  Porte seriali:
//    Serial  (TX0/RX0, pin 1/0) → USB/CH340, debug a 115200 baud
//    Serial3 (TX3/RX3, pin 14/15) → ESP8266 integrato, 57600 baud
//
//  I2C sensore VL53: pin 20 (SDA), 21 (SCL) — standard Mega2560
//  ADC : riferimento 5 V, 10 bit (0-1023)
//  Relè: pin 2-7
//  Sensore corrente ACS712-5A: pin A0 (segnale diretto 0-5V, no partitore)
// ============================================================

#include <EEPROM.h>
#include <stdlib.h>
#include <Wire.h>
#include <avr/wdt.h>

#include <Adafruit_VL53L0X.h>
#define USE_VL53L0X 1

// Porta seriale verso l'ESP8266 integrato (Serial3 su Mega2560)
#define ESP_SERIAL      Serial3
#define ESP_SERIAL_BAUD 57600

// Pin relè
const int relay1 = 2;  // Apertura sportello anteriore
const int relay2 = 3;  // Chiusura sportello anteriore
const int relay3 = 4;  // Alzata TV
const int relay4 = 5;  // Abbassamento TV
const int relay5 = 6;  // Apertura sportello posteriore
const int relay6 = 7;  // Chiusura sportello posteriore

const uint8_t relayActiveState   = LOW;
const uint8_t relayInactiveState = HIGH;
const unsigned long defaultDoorTimeMs = 500UL;
const unsigned long defaultTvTravelTimeMs = 48000UL;
const float defaultTvOnCurrentThreshold = 0.180f;
const float tvOffCurrentHysteresis = 0.020f;
const unsigned long defaultTvOnConfirmMs = 5000UL;
const unsigned long defaultTvOffConfirmMs = 20000UL;
const int defaultMinHeightMm = 200;
const int defaultMaxHeightMm = 800;
const int heightClosedWindowMm = 40;
const int heightOpenWindowMm   = 40;
const uint8_t requiredHeightConsensus = 3;

// Parametri ACS712-5A (segnale diretto 0-5 V, ADC 5 V, no partitore)
const int   currentSensorPin       = A0;
const float ADC_REFERENCE_VOLTAGE  = 5.0f;
const float ACS712_ZERO_VOLTAGE    = 2.5f;   // uscita a zero corrente
const float ACS712_SENSITIVITY     = 0.185f; // V/A per modello 5A

bool  tvOn         = false;
int   currentHeight = 0;   // mm (VL53)
float currentRms   = 0.0f; // A

enum LiftState {
  IDLE, OPENING_FRONT, OPENING_REAR, RAISING,
  CLOSING_FRONT, LOWERING, CLOSING_REAR
};
LiftState liftState = IDLE;
unsigned long stateStartTime = 0;
bool isClosingSequence = false;

struct PersistentConfig {
  uint32_t magic;
  uint32_t T1, T2, T3, T4, T5, T6;
  uint32_t tvOnConfirmMs;
  uint32_t tvOffConfirmMs;
  int  minHeight, maxHeight;
  float tvOnCurrentThreshold;
};
PersistentConfig cfg;
const uint32_t EEPROM_MAGIC = 0x544C4633; // 'T','L','F','3'

enum TvPosition {
  TV_POSITION_UNKNOWN,
  TV_POSITION_CLOSED,
  TV_POSITION_OPEN
};

bool          heightSensorOk = false;
unsigned long lastSensorRead  = 0;
unsigned long lastStatusSent  = 0;
bool          statusDirty     = true;
String        serialBuffer;
bool          manualModeActive = false;
int           manualRelayPin   = -1;
unsigned long manualUntil      = 0;
unsigned long currentAboveThresholdSince = 0;
unsigned long currentBelowThresholdSince = 0;
const unsigned long relayDeadTimeMs = 80;
unsigned long lastHeightSensorInitAttempt = 0;
const unsigned long heightSensorRetryMs = 5000;
const char*   heightSensorModel = "VL53L0X";
const char*   heightSensorStatus = "BOOT";
uint8_t       heightSensorAddress = 0;
TvPosition    tvPosition = TV_POSITION_UNKNOWN;
TvPosition    pendingTvPosition = TV_POSITION_UNKNOWN;
uint8_t       pendingTvPositionSamples = 0;
bool          fwupActive = false;
bool          fwupSawEof = false;
unsigned long fwupExpectedLine = 1;
unsigned long fwupAcceptedLines = 0;

#if defined(USE_VL53L1X)
Adafruit_VL53L1X heightSensor = Adafruit_VL53L1X();
#elif defined(USE_VL53L0X)
Adafruit_VL53L0X heightSensor = Adafruit_VL53L0X();
#endif

// ──────────────────────────────────────────────────────────
//  Config EEPROM
// ──────────────────────────────────────────────────────────
String normalizeNumericString(const String& raw) {
  String normalized = raw;
  normalized.trim();
  normalized.replace(',', '.');
  return normalized;
}

bool parseUnsignedLongValue(const String& raw, unsigned long& result) {
  String normalized = normalizeNumericString(raw);
  int decimalSeparator = normalized.indexOf('.');
  if (decimalSeparator >= 0) normalized = normalized.substring(0, decimalSeparator);
  if (normalized.length() == 0) return false;

  char* endPtr = NULL;
  unsigned long parsed = strtoul(normalized.c_str(), &endPtr, 10);
  if (endPtr == normalized.c_str() || *endPtr != '\0') return false;
  result = parsed;
  return true;
}

bool parseIntValue(const String& raw, int& result) {
  String normalized = normalizeNumericString(raw);
  int decimalSeparator = normalized.indexOf('.');
  if (decimalSeparator >= 0) normalized = normalized.substring(0, decimalSeparator);
  if (normalized.length() == 0) return false;

  char* endPtr = NULL;
  long parsed = strtol(normalized.c_str(), &endPtr, 10);
  if (endPtr == normalized.c_str() || *endPtr != '\0') return false;
  result = (int)parsed;
  return true;
}

bool parseFloatValue(const String& raw, float& result) {
  String normalized = normalizeNumericString(raw);
  if (normalized.length() == 0) return false;

  char* endPtr = NULL;
  float parsed = (float)strtod(normalized.c_str(), &endPtr);
  if (endPtr == normalized.c_str() || *endPtr != '\0') return false;
  result = parsed;
  return true;
}

bool parseHexNibble(char c, uint8_t& out) {
  if (c >= '0' && c <= '9') { out = (uint8_t)(c - '0'); return true; }
  if (c >= 'A' && c <= 'F') { out = (uint8_t)(10 + c - 'A'); return true; }
  if (c >= 'a' && c <= 'f') { out = (uint8_t)(10 + c - 'a'); return true; }
  return false;
}

bool parseHexByteAt(const String& text, int pos, uint8_t& out) {
  if (pos < 0 || (pos + 1) >= (int)text.length()) return false;
  uint8_t hi = 0, lo = 0;
  if (!parseHexNibble(text[pos], hi) || !parseHexNibble(text[pos + 1], lo)) return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

bool validateIntelHexRecord(const String& rawLine, bool& eofRecord, String& errorOut) {
  eofRecord = false;
  String line = rawLine;
  line.trim();

  if (line.length() < 11) { errorOut = "HEX_SHORT"; return false; }
  if (line[0] != ':') { errorOut = "HEX_NO_COLON"; return false; }
  if (((line.length() - 1) % 2) != 0) { errorOut = "HEX_LEN"; return false; }

  int recordBytes = (line.length() - 1) / 2;
  if (recordBytes < 5) { errorOut = "HEX_RECORD_SHORT"; return false; }

  uint8_t sum = 0;
  uint8_t byteCount = 0;
  uint8_t recordType = 0;

  for (int i = 0; i < recordBytes; i++) {
    uint8_t b = 0;
    if (!parseHexByteAt(line, 1 + i * 2, b)) { errorOut = "HEX_CHAR"; return false; }
    if (i == 0) byteCount = b;
    if (i == 3) recordType = b;
    sum = (uint8_t)(sum + b);
  }

  if (recordBytes != (int)(byteCount + 5)) { errorOut = "HEX_RECORD_LEN"; return false; }
  if (sum != 0) { errorOut = "HEX_CHECKSUM"; return false; }

  if (recordType == 0x01) eofRecord = true;
  return true;
}

void fwupResetState() {
  fwupActive = false;
  fwupSawEof = false;
  fwupExpectedLine = 1;
  fwupAcceptedLines = 0;
}

void sendFwup(const String& value) {
  ESP_SERIAL.println("FWUP," + value);
}

void rebootMegaViaWatchdog() {
  delay(40);
  wdt_enable(WDTO_120MS);
  while (true) {}
}

void processFwupCommand(const String& line) {
  if (line == "FWUP,BEGIN") {
    stopLift();
    fwupResetState();
    fwupActive = true;
    sendFwup("ACK,BEGIN");
    return;
  }

  if (line == "FWUP,ABORT") {
    fwupResetState();
    sendFwup("ACK,ABORT");
    return;
  }

  if (line == "FWUP,STATUS?") {
    sendFwup("STATUS,ACTIVE=" + String(fwupActive ? 1 : 0) +
             ",LINES=" + String(fwupAcceptedLines) +
             ",EOF=" + String(fwupSawEof ? 1 : 0));
    return;
  }

  if (line == "FWUP,RESET_BOOT") {
    sendFwup("ACK,RESET_BOOT");
    rebootMegaViaWatchdog();
    return;
  }

  if (line == "FWUP,END") {
    if (!fwupActive) { sendFwup("ERR,NOT_ACTIVE"); return; }
    if (!fwupSawEof) { sendFwup("ERR,EOF_MISSING"); return; }
    fwupActive = false;
    sendFwup("ACK,END,LINES=" + String(fwupAcceptedLines));
    return;
  }

  if (line.startsWith("FWUP,LINE,")) {
    if (!fwupActive) { sendFwup("ERR,NOT_ACTIVE"); return; }

    int c2 = line.indexOf(',', 10);
    if (c2 < 0) { sendFwup("ERR,BAD_LINE"); return; }

    String seqStr = line.substring(10, c2);
    String hexRec = line.substring(c2 + 1);

    unsigned long seq = 0;
    if (!parseUnsignedLongValue(seqStr, seq)) { sendFwup("ERR,BAD_SEQ"); return; }
    if (seq != fwupExpectedLine) {
      sendFwup("ERR,SEQ_MISMATCH,EXPECTED=" + String(fwupExpectedLine) + ",GOT=" + String(seq));
      return;
    }

    bool eofRecord = false;
    String err;
    if (!validateIntelHexRecord(hexRec, eofRecord, err)) {
      sendFwup("ERR,HEX," + String(seq) + "," + err);
      return;
    }

    fwupAcceptedLines++;
    fwupExpectedLine++;
    if (eofRecord) fwupSawEof = true;

    sendFwup("ACK,LINE," + String(seq));
    return;
  }

  sendFwup("ERR,UNKNOWN");
}

void normalizeConfig() {
  if (cfg.T1 < 100UL || cfg.T1 > 120000UL) cfg.T1 = defaultDoorTimeMs;
  if (cfg.T2 < 100UL || cfg.T2 > 120000UL) cfg.T2 = defaultDoorTimeMs;
  if (cfg.T3 < 100UL || cfg.T3 > 600000UL) cfg.T3 = defaultTvTravelTimeMs;
  if (cfg.T4 < 100UL || cfg.T4 > 600000UL) cfg.T4 = defaultTvTravelTimeMs;
  if (cfg.T5 < 100UL || cfg.T5 > 120000UL) cfg.T5 = defaultDoorTimeMs;
  if (cfg.T6 < 100UL || cfg.T6 > 120000UL) cfg.T6 = defaultDoorTimeMs;
  if (cfg.tvOnConfirmMs < 500UL || cfg.tvOnConfirmMs > 120000UL) cfg.tvOnConfirmMs = defaultTvOnConfirmMs;
  if (cfg.tvOffConfirmMs < 1000UL || cfg.tvOffConfirmMs > 300000UL) cfg.tvOffConfirmMs = defaultTvOffConfirmMs;
  if (cfg.minHeight < 0) cfg.minHeight = defaultMinHeightMm;
  if (cfg.maxHeight <= cfg.minHeight) cfg.maxHeight = cfg.minHeight + 100;
  if (cfg.tvOnCurrentThreshold < 0.01f || cfg.tvOnCurrentThreshold > 5.0f) {
    cfg.tvOnCurrentThreshold = defaultTvOnCurrentThreshold;
  }
}

void setDefaultConfig() {
  cfg.magic               = EEPROM_MAGIC;
  cfg.T1 = defaultDoorTimeMs; cfg.T2 = defaultDoorTimeMs;
  cfg.T3 = defaultTvTravelTimeMs; cfg.T4 = defaultTvTravelTimeMs;
  cfg.T5 = defaultDoorTimeMs; cfg.T6 = defaultDoorTimeMs;
  cfg.tvOnConfirmMs       = defaultTvOnConfirmMs;
  cfg.tvOffConfirmMs      = defaultTvOffConfirmMs;
  cfg.minHeight           = defaultMinHeightMm;
  cfg.maxHeight           = defaultMaxHeightMm;
  cfg.tvOnCurrentThreshold = defaultTvOnCurrentThreshold;
}

void saveConfig() {
  EEPROM.put(0, cfg);
}

void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != EEPROM_MAGIC) {
    setDefaultConfig();
    saveConfig();
    return;
  }
  normalizeConfig();
}

// ──────────────────────────────────────────────────────────
//  Sensore VL53
// ──────────────────────────────────────────────────────────
bool isI2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void setupHeightSensor() {
  Wire.begin(); // SDA=20, SCL=21 (Mega2560)
  delay(10);
  lastHeightSensorInitAttempt = millis();
  heightSensorAddress = 0;

#if defined(USE_VL53L1X)
  heightSensorModel = "VL53L1X";
  if (!isI2cDevicePresent(0x29)) {
    heightSensorOk = false;
    heightSensorStatus = "I2C_ADDR_0x29_NOT_FOUND";
    Serial.println(F("VL53L1X non rilevato su I2C 0x29"));
    return;
  }
  heightSensorAddress = 0x29;
  if (!heightSensor.begin(0x29, &Wire)) {
    heightSensorOk = false;
    heightSensorStatus = "INIT_FAILED";
    Serial.println(F("VL53L1X trovato su I2C ma inizializzazione fallita"));
    return;
  }
  heightSensor.setTimingBudget(50);
  heightSensor.startRanging();
  heightSensorOk = true;
  heightSensorStatus = "OK";
  Serial.println(F("VL53L1X inizializzato su I2C 0x29"));
#elif defined(USE_VL53L0X)
  heightSensorModel = "VL53L0X";
  if (!isI2cDevicePresent(0x29)) {
    heightSensorOk = false;
    heightSensorStatus = "I2C_ADDR_0x29_NOT_FOUND";
    Serial.println(F("VL53L0X non rilevato su I2C 0x29"));
    return;
  }
  heightSensorAddress = 0x29;
  if (!heightSensor.begin()) {
    heightSensorOk = false;
    heightSensorStatus = "INIT_FAILED";
    Serial.println(F("VL53L0X trovato su I2C ma inizializzazione fallita"));
    return;
  }
  heightSensorOk = true;
  heightSensorStatus = "OK";
  Serial.println(F("VL53L0X inizializzato su I2C 0x29"));
#endif
}

int readHeight() {
#if defined(USE_VL53L1X)
  if (!heightSensorOk || !heightSensor.dataReady()) return currentHeight;
  int d = heightSensor.distance();
  if (d > 0) { heightSensor.clearInterrupt(); return d; }
  return currentHeight;
#elif defined(USE_VL53L0X)
  if (!heightSensorOk) return currentHeight;
  VL53L0X_RangingMeasurementData_t m;
  heightSensor.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? m.RangeMilliMeter : currentHeight;
#else
  return currentHeight;
#endif
}

// ──────────────────────────────────────────────────────────
//  Sensore corrente ACS712
// ──────────────────────────────────────────────────────────
float readCurrentRms() {
  const int sampleCount = 120;
  float sumSquared = 0.0f;
  for (int i = 0; i < sampleCount; i++) {
    int raw = analogRead(currentSensorPin);
    float voltage = raw * (ADC_REFERENCE_VOLTAGE / 1023.0f);
    float current = (voltage - ACS712_ZERO_VOLTAGE) / ACS712_SENSITIVITY;
    sumSquared += current * current;
    delayMicroseconds(800);
  }
  return sqrtf(sumSquared / sampleCount);
}

void updateTvStateFromCurrent(unsigned long now) {
  bool aboveThreshold = currentRms >= cfg.tvOnCurrentThreshold;
  bool belowThreshold = currentRms < (cfg.tvOnCurrentThreshold - tvOffCurrentHysteresis);

  if (aboveThreshold) {
    if (currentAboveThresholdSince == 0) currentAboveThresholdSince = now;
  } else {
    currentAboveThresholdSince = 0;
  }

  if (belowThreshold) {
    if (currentBelowThresholdSince == 0) currentBelowThresholdSince = now;
  } else {
    currentBelowThresholdSince = 0;
  }

  if (!tvOn && currentAboveThresholdSince > 0 &&
      (now - currentAboveThresholdSince) >= cfg.tvOnConfirmMs) {
    tvOn = true;
    currentBelowThresholdSince = 0;
    markStatusDirty();
  }

  if (tvOn && currentBelowThresholdSince > 0 &&
      (now - currentBelowThresholdSince) >= cfg.tvOffConfirmMs) {
    tvOn = false;
    currentAboveThresholdSince = 0;
    markStatusDirty();
  }
}

// ──────────────────────────────────────────────────────────
//  Stato macchina
// ──────────────────────────────────────────────────────────
const char* getStateCode() {
  switch (liftState) {
    case IDLE:          return "IDLE";
    case OPENING_FRONT: return "OPENING_FRONT";
    case OPENING_REAR:  return "OPENING_REAR";
    case RAISING:       return "RAISING";
    case CLOSING_FRONT: return "CLOSING_FRONT";
    case LOWERING:      return "LOWERING";
    case CLOSING_REAR:  return "CLOSING_REAR";
    default:            return "UNKNOWN";
  }
}

const char* getStateLabel() {
  switch (liftState) {
    case IDLE:          return "Fermo";
    case OPENING_FRONT: return "Apertura anteriore";
    case OPENING_REAR:  return "Apertura posteriore";
    case RAISING:       return "Alzando TV";
    case CLOSING_FRONT: return "Chiusura anteriore";
    case LOWERING:      return "Abbassando TV";
    case CLOSING_REAR:  return "Chiusura posteriore";
    default:            return "Sconosciuto";
  }
}

const char* getTvPositionCode() {
  switch (tvPosition) {
    case TV_POSITION_CLOSED: return "CLOSED";
    case TV_POSITION_OPEN:   return "OPEN";
    default:                 return "UNKNOWN";
  }
}

void markStatusDirty() { statusDirty = true; }

void updateTvPositionFromHeight(int measuredHeight) {
  if (!heightSensorOk) return;

  TvPosition candidate = TV_POSITION_UNKNOWN;
  if (measuredHeight <= (cfg.minHeight + heightClosedWindowMm)) {
    candidate = TV_POSITION_CLOSED;
  } else if (measuredHeight >= (cfg.maxHeight - heightOpenWindowMm)) {
    candidate = TV_POSITION_OPEN;
  } else {
    pendingTvPosition = TV_POSITION_UNKNOWN;
    pendingTvPositionSamples = 0;
    return;
  }

  if (candidate != pendingTvPosition) {
    pendingTvPosition = candidate;
    pendingTvPositionSamples = 1;
    return;
  }

  if (pendingTvPositionSamples < 255) pendingTvPositionSamples++;
  if (pendingTvPositionSamples >= requiredHeightConsensus && tvPosition != candidate) {
    tvPosition = candidate;
    markStatusDirty();
  }
}

// ──────────────────────────────────────────────────────────
//  Gestione relè
// ──────────────────────────────────────────────────────────
bool isRelayActive(int pin) {
  return digitalRead(pin) == relayActiveState;
}

void setRelayState(int pin, bool active) {
  digitalWrite(pin, active ? relayActiveState : relayInactiveState);
}

int oppositeRelayPin(int pin) {
  if (pin == relay1) return relay2;
  if (pin == relay2) return relay1;
  if (pin == relay3) return relay4;
  if (pin == relay4) return relay3;
  if (pin == relay5) return relay6;
  if (pin == relay6) return relay5;
  return -1;
}

void activateRelaySafe(int pin) {
  int opp = oppositeRelayPin(pin);
  if (opp >= 0 && isRelayActive(opp)) {
    setRelayState(opp, false);
    delay(relayDeadTimeMs);
  }
  if (!isRelayActive(pin)) setRelayState(pin, true);
}

void stopLift() {
  liftState = IDLE; isClosingSequence = false;
  manualModeActive = false; manualRelayPin = -1; manualUntil = 0;
  tvPosition = TV_POSITION_UNKNOWN;
  pendingTvPosition = TV_POSITION_UNKNOWN;
  pendingTvPositionSamples = 0;
  const int relayPins[] = {relay1, relay2, relay3, relay4, relay5, relay6};
  for (uint8_t i = 0; i < 6; i++) setRelayState(relayPins[i], false);
  markStatusDirty();
}

void startOpening() {
  if (liftState != IDLE) return;
  manualModeActive = false; manualRelayPin = -1;
  isClosingSequence = false;
  tvPosition = TV_POSITION_UNKNOWN;
  liftState = OPENING_FRONT;
  stateStartTime = millis();
  activateRelaySafe(relay1);
  markStatusDirty();
}

void startClosing() {
  if (liftState != IDLE) return;
  manualModeActive = false; manualRelayPin = -1;
  isClosingSequence = true;
  tvPosition = TV_POSITION_UNKNOWN;
  liftState = OPENING_FRONT;
  stateStartTime = millis();
  activateRelaySafe(relay1);
  markStatusDirty();
}

int relayPinFromCode(const String& code) {
  if (code == "R1") return relay1;
  if (code == "R2") return relay2;
  if (code == "R3") return relay3;
  if (code == "R4") return relay4;
  if (code == "R5") return relay5;
  if (code == "R6") return relay6;
  return -1;
}

void startManualRelay(int pin, unsigned long durationMs) {
  stopLift();
  manualModeActive = true;
  manualRelayPin   = pin;
  manualUntil      = millis() + durationMs;
  activateRelaySafe(pin);
  markStatusDirty();
}

void updateManualMode() {
  if (!manualModeActive) return;
  if ((long)(millis() - manualUntil) >= 0) {
    if (manualRelayPin >= 0) setRelayState(manualRelayPin, false);
    manualModeActive = false; manualRelayPin = -1; manualUntil = 0;
    markStatusDirty();
  }
}

// ──────────────────────────────────────────────────────────
//  Comunicazione con ESP8266 via Serial3
// ──────────────────────────────────────────────────────────
void sendStatus() {
  String line = "STATUS,TV=" + String(tvOn ? 1 : 0) +
                ",I="   + String(currentRms, 3) +
                ",THR=" + String(cfg.tvOnCurrentThreshold, 3) +
                ",H="   + String(currentHeight) +
                ",POS=" + String(getTvPositionCode()) +
                ",SOK=" + String(heightSensorOk ? 1 : 0) +
                ",SMODEL=" + String(heightSensorModel) +
                ",SSTAT=" + String(heightSensorStatus) +
                ",SADDR=" + String(heightSensorAddress > 0 ? "0x29" : "N/A") +
                ",MIN=" + String(cfg.minHeight) +
                ",MAX=" + String(cfg.maxHeight) +
                ",T1="  + String(cfg.T1) +
                ",T2="  + String(cfg.T2) +
                ",T3="  + String(cfg.T3) +
                ",T4="  + String(cfg.T4) +
                ",T5="  + String(cfg.T5) +
                ",T6="  + String(cfg.T6) +
                ",TON=" + String(cfg.tvOnConfirmMs) +
                ",TOFF=" + String(cfg.tvOffConfirmMs) +
                ",STATE=" + String(getStateCode()) +
                ",LABEL=" + String(getStateLabel());
  ESP_SERIAL.println(line);
  lastStatusSent = millis();
  statusDirty    = false;
}

void sendAck(const String& value)   { ESP_SERIAL.println("ACK," + value); }
void sendError(const String& value) { ESP_SERIAL.println("ERR," + value); }

bool applySetting(const String& key, const String& value) {
  if (key == "T1" || key == "T2" || key == "T3" || key == "T4" || key == "T5" || key == "T6" ||
      key == "TON" || key == "TOFF") {
    unsigned long parsed = 0;
    if (!parseUnsignedLongValue(value, parsed)) return false;
    if      (key == "T1") cfg.T1 = parsed;
    else if (key == "T2") cfg.T2 = parsed;
    else if (key == "T3") cfg.T3 = parsed;
    else if (key == "T4") cfg.T4 = parsed;
    else if (key == "T5") cfg.T5 = parsed;
    else if (key == "T6") cfg.T6 = parsed;
    else if (key == "TON") cfg.tvOnConfirmMs = parsed;
    else                    cfg.tvOffConfirmMs = parsed;
  } else if (key == "MIN" || key == "MAX") {
    int parsed = 0;
    if (!parseIntValue(value, parsed)) return false;
    if (key == "MIN") cfg.minHeight = parsed;
    else               cfg.maxHeight = parsed;
  } else if (key == "THR") {
    float parsed = 0.0f;
    if (!parseFloatValue(value, parsed)) return false;
    cfg.tvOnCurrentThreshold = parsed;
  } else {
    return false;
  }

  normalizeConfig();
  saveConfig();
  markStatusDirty();
  return true;
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("FWUP,")) {
    processFwupCommand(line);
    return;
  }

  if (line == "OPEN")    { startOpening(); sendAck("OPEN");  return; }
  if (line == "CLOSE")   { startClosing(); sendAck("CLOSE"); return; }
  if (line == "STOP")    { stopLift();     sendAck("STOP");  return; }
  if (line == "STATUS?") { sendStatus();   return; }
  if (line == "PING")    { ESP_SERIAL.println("PONG"); return; }

  if (line.startsWith("SET,")) {
    int c2 = line.indexOf(',', 4);
    if (c2 < 0) { sendError("BAD_SET"); return; }
    String key = line.substring(4, c2);
    String val = line.substring(c2 + 1);
    key.trim(); val.trim();
    if (applySetting(key, val)) { sendAck("SET," + key); sendStatus(); }
    else                         sendError("UNKNOWN_KEY");
    return;
  }

  if (line.startsWith("JOG,")) {
    int c2 = line.indexOf(',', 4);
    if (c2 < 0) { sendError("BAD_JOG"); return; }
    String code = line.substring(4, c2);
    String dur  = line.substring(c2 + 1);
    code.trim(); dur.trim();
    int pin = relayPinFromCode(code);
    unsigned long ms = 0;
    if (!parseUnsignedLongValue(dur, ms) || pin < 0 || ms < 100UL || ms > 3000UL) {
      sendError("BAD_JOG");
      return;
    }
    startManualRelay(pin, ms);
    sendAck("JOG," + code);
    sendStatus();
    return;
  }

  sendError("UNKNOWN_CMD");
}

void processSerial() {
  while (ESP_SERIAL.available()) {
    char ch = (char)ESP_SERIAL.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += ch;
      if (serialBuffer.length() > 120) {
        serialBuffer = "";
        sendError("LINE_TOO_LONG");
      }
    }
  }
}

// ──────────────────────────────────────────────────────────
//  Aggiornamento periodico sensori
// ──────────────────────────────────────────────────────────
void updateSensors() {
  unsigned long now = millis();
  if (!heightSensorOk && (now - lastHeightSensorInitAttempt >= heightSensorRetryMs)) {
    setupHeightSensor();
    markStatusDirty();
  }
  if (now - lastSensorRead < 250) return;
  lastSensorRead = now;

  currentRms = readCurrentRms();
  bool prevTv = tvOn;
  int  prevH  = currentHeight;
  updateTvStateFromCurrent(now);
  int measuredHeight = readHeight();
  if (prevH == 0) currentHeight = measuredHeight;
  else currentHeight = (prevH * 3 + measuredHeight) / 4;

  if (liftState == IDLE) updateTvPositionFromHeight(currentHeight);
  if (tvOn != prevTv || currentHeight != prevH) markStatusDirty();
}

// ──────────────────────────────────────────────────────────
//  Macchina a stati della sequenza
// ──────────────────────────────────────────────────────────
void updateLiftState() {
  if (manualModeActive) return;
  unsigned long t = millis();

  switch (liftState) {
    case OPENING_FRONT:
      if (t - stateStartTime >= (unsigned long)cfg.T1) {
        setRelayState(relay1, false);
        if (isClosingSequence) {
          liftState = LOWERING; stateStartTime = t; activateRelaySafe(relay4);
        } else {
          liftState = OPENING_REAR; stateStartTime = t; activateRelaySafe(relay5);
        }
        markStatusDirty();
      }
      break;

    case OPENING_REAR:
      if (t - stateStartTime >= (unsigned long)cfg.T5) {
        setRelayState(relay5, false);
        liftState = RAISING; stateStartTime = t; activateRelaySafe(relay3);
        markStatusDirty();
      }
      break;

    case RAISING:
      if (t - stateStartTime >= (unsigned long)cfg.T3) {
        setRelayState(relay3, false);
        liftState = CLOSING_FRONT; stateStartTime = t; activateRelaySafe(relay2);
        markStatusDirty();
      }
      break;

    case CLOSING_FRONT:
      if (t - stateStartTime >= (unsigned long)cfg.T2) {
        setRelayState(relay2, false);
        if (isClosingSequence) {
          liftState = CLOSING_REAR; stateStartTime = t; activateRelaySafe(relay6);
        } else {
          tvPosition = TV_POSITION_OPEN;
          isClosingSequence = false;
          liftState = IDLE;
        }
        markStatusDirty();
      }
      break;

    case LOWERING:
      if (t - stateStartTime >= (unsigned long)cfg.T4) {
        setRelayState(relay4, false);
        liftState = CLOSING_FRONT; stateStartTime = t; activateRelaySafe(relay2);
        markStatusDirty();
      }
      break;

    case CLOSING_REAR:
      if (t - stateStartTime >= (unsigned long)cfg.T6) {
        setRelayState(relay6, false);
        tvPosition = TV_POSITION_CLOSED;
        liftState = IDLE; isClosingSequence = false;
        markStatusDirty();
      }
      break;

    case IDLE:
      if (tvOn && tvPosition == TV_POSITION_CLOSED) startOpening();
      if (!tvOn && tvPosition == TV_POSITION_OPEN) startClosing();
      break;
  }
}

void maybeSendStatus() {
  unsigned long now = millis();
  if (statusDirty || (now - lastStatusSent >= 1000)) sendStatus();
}

// ──────────────────────────────────────────────────────────
//  Setup & Loop
// ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);           // USB/CH340 — monitor seriale per debug
  Serial3.begin(ESP_SERIAL_BAUD); // Collegamento verso ESP8266 integrato

  const int relayPins[] = {relay1, relay2, relay3, relay4, relay5, relay6};
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(relayPins[i], relayInactiveState);
    pinMode(relayPins[i], OUTPUT);
  }

  loadConfig();
  stopLift();
  setupHeightSensor();

  Serial.println(F("=== Mega2560 TV Lift pronto ==="));
  ESP_SERIAL.println("READY");
  sendStatus();
}

void loop() {
  processSerial();
  if (!fwupActive) {
    updateSensors();
    updateManualMode();
    updateLiftState();
  }
  maybeSendStatus();
}
