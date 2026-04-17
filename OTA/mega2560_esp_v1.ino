// ============================================================
//  mega2560_esp.ino
//  Gateway WiFi/MQTT per ESP8266 integrato — Mega2560 WiFi R3
//
//  CONFIGURAZIONE DIP SWITCH prima di caricare questo sketch:
//    SW3=ON, SW4=ON, SW7=ON, SW8=ON — modalità FLASH ESP8266
//    (tutti gli altri OFF; GPIO0 deve essere LOW al boot)
//
//  CONFIGURAZIONE DIP SWITCH per il funzionamento normale:
//    SW1=ON, SW2=ON (USB→Mega debug)
//    SW5=ON, SW6=ON (Mega Serial3 ↔ ESP8266 Serial)
//    SW3=OFF, SW4=OFF, SW7=OFF, SW8=OFF
//
//  L'ESP8266 comunica con l'ATmega2560 via Serial (57600 baud),
//  che sulla scheda è fisicamente collegato a Serial3 del Mega
//  (pin TX3/14 e RX3/15) attraverso i DIP switch SW5/SW6.
//
//  Scheda Arduino IDE: "Generic ESP8266 Module"
//    oppure "NodeMCU 1.0" — Flash size: 4M (o 32Mb per la tua scheda)
// ============================================================

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>

// ── Credenziali WiFi ─────────────────────────────────────
const char* ssid     = "myhome-2G";
const char* password = "crispyjungle618";

// ── MQTT ─────────────────────────────────────────────────
const char* mqtt_server   = "192.168.0.22";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "openhabian";
const char* mqtt_password = "z6RE6ysp$1aFnCE4h6059TQ$Bo*#7!4";
const char* mqtt_client_id          = "TVLiftMega2560";
const char* mqtt_topic_status       = "domotica/tv_lift/status";
const char* mqtt_topic_command      = "domotica/tv_lift/command";
const char* mqtt_topic_availability = "domotica/tv_lift/availability";
const char* mqtt_topic_ip           = "domotica/tv_lift/ip";
const char* ota_update_user         = "admin";
const char* ota_update_password     = "tvlift_ota";

struct RemoteState {
  bool   tvOn          = false;
  bool   arduinoOnline = false;
  bool   heightSensorOk = false;
  float  currentRms    = 0.0f;
  float  currentThreshold = 0.180f;
  int    height    = 0;
  int    minHeight = 200;
  int    maxHeight = 800;
  unsigned long T1 = 500, T2 = 500;
  unsigned long T3 = 48000, T4 = 48000;
  unsigned long T5 = 500, T6 = 500;
  unsigned long tvOnConfirmMs = 5000;
  unsigned long tvOffConfirmMs = 20000;
  String sensorModel = "VL53L1X";
  String sensorStatus = "BOOT";
  String sensorAddress = "N/A";
  String tvPosition = "UNKNOWN";
  String stateCode  = "UNKNOWN";
  String stateLabel = "N/D";
  unsigned long lastUpdate = 0;
};

RemoteState    remoteState;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient     espClient;
PubSubClient   client(espClient);

String        serialBuffer;
unsigned long lastPublish           = 0;
unsigned long lastStatusRequest     = 0;
int           manualPulseMs         = 500;
const size_t  serialLineMaxLen      = 384;
const unsigned long arduinoOfflineTimeoutMs = 12000;
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
bool          apFallbackActive      = false;

const char* fallbackApSsid     = "TVLift-Mega2560-Setup";
const char* fallbackApPassword = "tvlift123";

bool          megaUploadInProgress = false;
bool          megaHexValid         = false;
bool          megaHexSawEof        = false;
size_t        megaHexBytes         = 0;
unsigned long megaHexLines         = 0;
unsigned int  megaUploadProgressPct = 0;
unsigned long megaUploadStartedAt = 0;
bool          megaForwardingActive = false;
bool          megaBootloaderDetected = false;
unsigned long megaBootloaderProbeAt = 0;
String        megaUploadFileName;
String        megaUploadMessage = "Nessun file caricato";
String        megaUploadError;
String        megaHexLineBuffer;
String        megaUploadLog;

void appendMegaUploadLog(const String& line) {
  megaUploadLog += line + "\n";
  const size_t maxLogLen = 2200;
  if (megaUploadLog.length() > maxLogLen) {
    megaUploadLog.remove(0, megaUploadLog.length() - maxLogLen);
  }
}

String jsonEscape(const String& src) {
  String out;
  out.reserve(src.length() + 16);
  for (size_t i = 0; i < src.length(); i++) {
    char c = src[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

bool waitForMegaFwupReply(const String& expectedPrefix, unsigned long timeoutMs, String& matchedLine) {
  String line;
  unsigned long start = millis();

  while ((millis() - start) < timeoutMs) {
    while (Serial.available()) {
      char ch = (char)Serial.read();
      if (ch == '\r') continue;
      if (ch == '\n') {
        line.trim();
        if (line.length() == 0) { line = ""; continue; }

        remoteState.arduinoOnline = true;
        remoteState.lastUpdate = millis();
        handleArduinoLine(line);
        if (line.startsWith("FWUP,")) appendMegaUploadLog("RX " + line);

        if (line.startsWith(expectedPrefix)) {
          matchedLine = line;
          return true;
        }
        if (line.startsWith("FWUP,ERR")) {
          matchedLine = line;
          return false;
        }
        line = "";
      } else {
        if (line.length() < serialLineMaxLen) line += ch;
      }
    }
    yield();
  }

  matchedLine = "TIMEOUT";
  return false;
}

bool sendFwupCommandAndWait(const String& command,
                            const String& expectedPrefix,
                            unsigned long timeoutMs) {
  appendMegaUploadLog("TX " + command);
  sendArduinoCommand(command);

  String reply;
  bool ok = waitForMegaFwupReply(expectedPrefix, timeoutMs, reply);
  if (ok) return true;

  if (reply == "TIMEOUT") {
    megaUploadError = "Timeout attesa risposta: " + expectedPrefix;
  } else {
    megaUploadError = reply;
  }
  megaUploadMessage = "Errore protocollo con Mega";
  return false;
}

void flushSerialInput() {
  while (Serial.available()) Serial.read();
}

bool probeMegaBootloaderStk500v1(unsigned long timeoutMs) {
  // Optiboot/STK500v1 risponde tipicamente a GET_SYNC (0x30 0x20) con 0x14 0x10.
  flushSerialInput();
  uint8_t state = 0;
  unsigned long start = millis();

  while ((millis() - start) < timeoutMs) {
    Serial.write((uint8_t)0x30);
    Serial.write((uint8_t)0x20);

    unsigned long burstStart = millis();
    while ((millis() - burstStart) < 140UL) {
      while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        if (state == 0) {
          state = (b == 0x14) ? 1 : 0;
        } else {
          if (b == 0x10) return true;
          state = (b == 0x14) ? 1 : 0;
        }
      }
      yield();
    }

    yield();
  }

  return false;
}

bool parseHexByte(const String& text, int pos, uint8_t& value) {
  if (pos < 0 || (pos + 1) >= (int)text.length()) return false;
  auto nibble = [&](char c, uint8_t& out) -> bool {
    if (c >= '0' && c <= '9') { out = (uint8_t)(c - '0'); return true; }
    if (c >= 'A' && c <= 'F') { out = (uint8_t)(10 + c - 'A'); return true; }
    if (c >= 'a' && c <= 'f') { out = (uint8_t)(10 + c - 'a'); return true; }
    return false;
  };

  uint8_t hi = 0, lo = 0;
  if (!nibble(text[pos], hi) || !nibble(text[pos + 1], lo)) return false;
  value = (uint8_t)((hi << 4) | lo);
  return true;
}

bool validateIntelHexLine(const String& line, String& errorOut, bool& eofRecord) {
  eofRecord = false;
  if (line.length() < 11) {
    errorOut = "Riga HEX troppo corta";
    return false;
  }
  if (line[0] != ':') {
    errorOut = "Riga HEX senza ':' iniziale";
    return false;
  }
  if (((line.length() - 1) % 2) != 0) {
    errorOut = "Lunghezza HEX non valida";
    return false;
  }

  int bytesCount = (line.length() - 1) / 2;
  if (bytesCount < 5) {
    errorOut = "Record HEX incompleto";
    return false;
  }

  uint8_t sum = 0;
  uint8_t byteCount = 0;
  uint8_t recordType = 0;

  for (int i = 0; i < bytesCount; i++) {
    uint8_t value = 0;
    if (!parseHexByte(line, 1 + i * 2, value)) {
      errorOut = "Carattere HEX non valido";
      return false;
    }
    if (i == 0) byteCount = value;
    if (i == 3) recordType = value;
    sum = (uint8_t)(sum + value);
  }

  if (bytesCount != (int)(byteCount + 5)) {
    errorOut = "Lunghezza record HEX incoerente";
    return false;
  }
  if (sum != 0) {
    errorOut = "Checksum HEX non valido";
    return false;
  }

  if (recordType == 0x01) eofRecord = true;
  return true;
}

void processMegaHexLine(const String& line) {
  if (!megaHexValid) return;
  String trimmed = line;
  trimmed.trim();
  if (trimmed.length() == 0) return;

  bool eofRecord = false;
  String error;
  if (!validateIntelHexLine(trimmed, error, eofRecord)) {
    megaHexValid = false;
    megaUploadError = "Riga " + String(megaHexLines + 1) + ": " + error;
    return;
  }

  megaHexLines++;
  if (eofRecord) megaHexSawEof = true;

  if (megaForwardingActive) {
    String seq = String(megaHexLines);
    if (!sendFwupCommandAndWait("FWUP,LINE," + seq + "," + trimmed,
                                "FWUP,ACK,LINE," + seq,
                                1600UL)) {
      megaHexValid = false;
      return;
    }
  }
}

void handleMegaUpdatePage() {
  String statusClass = megaHexValid ? "ok" : "warn";
  if (!megaUploadError.isEmpty()) statusClass = "err";

  String html = "<!doctype html><html><head><title>Mega2560 Update</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>";
  html += "body{margin:0;font-family:'Trebuchet MS','Segoe UI',Tahoma,sans-serif;background:linear-gradient(155deg,#f7fbff 0%,#e7f0f8 55%,#deebf6 100%);color:#10202d}";
  html += ".wrap{max-width:860px;margin:0 auto;padding:16px}.card{background:#fff;border:1px solid #d5e1ec;border-radius:14px;padding:14px;box-shadow:0 10px 22px rgba(15,35,55,.07)}";
  html += ".k{font-size:.85rem;color:#5a6b79}.v{margin-top:6px;font-size:1.05rem;font-weight:700}.row{margin:10px 0}.btn{display:inline-block;text-align:center;padding:11px 12px;border-radius:11px;color:#fff;text-decoration:none;font-weight:800;border:0;cursor:pointer;background:#1d5876}";
  html += ".ok{color:#0c724b}.warn{color:#8b6409}.err{color:#8f2222}input[type=file]{width:100%;padding:8px;border:1px solid #c6d5e3;border-radius:10px;background:#f9fcff}";
  html += ".bar{height:14px;border-radius:999px;background:#d8e6f2;overflow:hidden}.bar>span{display:block;height:100%;background:#1d7ca3;width:0%}pre{white-space:pre-wrap;background:#f7fbff;border:1px solid #d5e1ec;border-radius:10px;padding:10px;max-height:240px;overflow:auto}";
  html += "</style></head><body><div class='wrap'>";
  html += "<div class='card'><h2 style='margin:0 0 8px'>Aggiornamento Mega2560 via Web</h2>";
  html += "<div class='k'>Pipeline attiva: upload HEX, validazione checksum, invio record con ACK al Mega e reset controllato a fine trasferimento.</div>";
  html += "<div class='row'><a class='btn' href='/'>Torna alla dashboard</a></div>";
  html += "<form method='POST' action='/mega-update/upload' enctype='multipart/form-data'>";
  html += "<div class='row'><label>Firmware Mega (.hex)</label><input type='file' name='firmware' accept='.hex,text/plain'></div>";
  html += "<button class='btn' type='submit'>Carica e valida</button></form></div>";

  html += "<div class='card' style='margin-top:12px'><div class='k'>Ultimo upload</div>";
  html += "<div id='up-file' class='v'>File: " + (megaUploadFileName.length() ? megaUploadFileName : String("N/D")) + "</div>";
  html += "<div id='up-bytes' class='v'>Byte: " + String(megaHexBytes) + "</div>";
  html += "<div id='up-lines' class='v'>Record validati: " + String(megaHexLines) + "</div>";
  html += "<div id='up-eof' class='v'>EOF record: " + String(megaHexSawEof ? "SI" : "NO") + "</div>";
  html += "<div id='up-bl' class='v'>Bootloader rilevato: " + String(megaBootloaderDetected ? "SI" : "NO") + "</div>";
  html += "<div class='row'><div class='k'>Progresso trasferimento</div><div class='bar'><span id='up-bar' style='width:" + String(megaUploadProgressPct) + "%'></span></div><div id='up-pct' class='v'>" + String(megaUploadProgressPct) + "%</div></div>";
  html += "<div id='up-state' class='v " + statusClass + "'>Stato: " + megaUploadMessage + "</div>";
  if (!megaUploadError.isEmpty()) {
    html += "<div id='up-err' class='v err'>Errore: " + megaUploadError + "</div>";
  } else {
    html += "<div id='up-err' class='v'></div>";
  }
  html += "<div class='k' style='margin-top:8px'>Log protocollo</div><pre id='up-log'>" + megaUploadLog + "</pre>";
  html += "<script>";
  html += "function setText(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}";
  html += "async function pollMega(){try{var r=await fetch('/api/mega-update',{cache:'no-store'});if(!r.ok)return;var d=await r.json();";
  html += "setText('up-file','File: '+(d.fileName||'N/D'));setText('up-bytes','Byte: '+d.bytes);setText('up-lines','Record validati: '+d.lines);setText('up-eof','EOF record: '+(d.eof?'SI':'NO'));setText('up-bl','Bootloader rilevato: '+(d.bootloaderDetected?'SI':'NO'));setText('up-pct',d.progressPct+'%');";
  html += "var b=document.getElementById('up-bar');if(b)b.style.width=d.progressPct+'%';";
  html += "setText('up-state','Stato: '+d.message);var s=document.getElementById('up-state');if(s){s.classList.remove('ok','warn','err');s.classList.add(d.statusClass);}setText('up-err',d.error?('Errore: '+d.error):'');setText('up-log',d.log||'');";
  html += "}catch(e){}}pollMega();setInterval(pollMega,1000);";
  html += "</script>";
  html += "</div></div></body></html>";

  server.send(200, "text/html", html);
}

void handleApiMegaUpdate() {
  String statusClass = megaHexValid ? "ok" : "warn";
  if (!megaUploadError.isEmpty()) statusClass = "err";

  String json = "{";
  json += "\"inProgress\":" + String(megaUploadInProgress ? "true" : "false");
  json += ",\"hexValid\":" + String(megaHexValid ? "true" : "false");
  json += ",\"eof\":" + String(megaHexSawEof ? "true" : "false");
  json += ",\"bytes\":" + String(megaHexBytes);
  json += ",\"lines\":" + String(megaHexLines);
  json += ",\"progressPct\":" + String(megaUploadProgressPct);
  json += ",\"bootloaderDetected\":" + String(megaBootloaderDetected ? "true" : "false");
  json += ",\"bootloaderProbeAt\":" + String(megaBootloaderProbeAt);
  json += ",\"fileName\":\"" + jsonEscape(megaUploadFileName) + "\"";
  json += ",\"message\":\"" + jsonEscape(megaUploadMessage) + "\"";
  json += ",\"error\":\"" + jsonEscape(megaUploadError) + "\"";
  json += ",\"statusClass\":\"" + statusClass + "\"";
  json += ",\"log\":\"" + jsonEscape(megaUploadLog) + "\"";
  json += "}";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", json);
}

void handleMegaUploadFinish() {
  if (megaUploadInProgress) {
    server.send(409, "text/plain", "Upload ancora in corso");
    return;
  }
  server.sendHeader("Location", "/mega-update");
  server.send(303);
}

void handleMegaUploadStream() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    megaUploadInProgress = true;
    megaHexValid = true;
    megaHexSawEof = false;
    megaHexBytes = 0;
    megaHexLines = 0;
    megaUploadProgressPct = 0;
    megaUploadStartedAt = millis();
    megaForwardingActive = true;
    megaBootloaderDetected = false;
    megaBootloaderProbeAt = 0;
    megaUploadFileName = upload.filename;
    megaUploadError = "";
    megaUploadMessage = "Upload in corso";
    megaHexLineBuffer = "";
    megaUploadLog = "";
    appendMegaUploadLog("Inizio upload file: " + megaUploadFileName);

    if (!sendFwupCommandAndWait("FWUP,ABORT", "FWUP,ACK,ABORT", 800UL)) {
      megaUploadError = "Mega non risponde ad ABORT preliminare";
      megaHexValid = false;
      megaForwardingActive = false;
      return;
    }
    if (!sendFwupCommandAndWait("FWUP,BEGIN", "FWUP,ACK,BEGIN", 1800UL)) {
      megaHexValid = false;
      megaForwardingActive = false;
      return;
    }
    megaUploadMessage = "Upload e trasferimento a Mega in corso";
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    megaHexBytes += upload.currentSize;
    if (upload.totalSize > 0) {
      megaUploadProgressPct = (unsigned int)((megaHexBytes * 100UL) / upload.totalSize);
      if (megaUploadProgressPct > 100) megaUploadProgressPct = 100;
    }
    for (size_t i = 0; i < upload.currentSize; i++) {
      char c = (char)upload.buf[i];
      if (c == '\r') continue;
      if (c == '\n') {
        processMegaHexLine(megaHexLineBuffer);
        megaHexLineBuffer = "";
      } else {
        megaHexLineBuffer += c;
        if (megaHexLineBuffer.length() > 520) {
          megaHexValid = false;
          megaUploadError = "Riga HEX troppo lunga";
          megaForwardingActive = false;
        }
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (megaHexLineBuffer.length() > 0) {
      processMegaHexLine(megaHexLineBuffer);
      megaHexLineBuffer = "";
    }

    if (megaHexValid && !megaHexSawEof) {
      megaUploadMessage = "File HEX senza record EOF";
      megaUploadError = "Manca record di fine file (tipo 01)";
      megaHexValid = false;
      megaForwardingActive = false;
    }

    if (megaHexValid && megaForwardingActive) {
      if (!sendFwupCommandAndWait("FWUP,END", "FWUP,ACK,END", 2000UL)) {
        megaHexValid = false;
      } else if (!sendFwupCommandAndWait("FWUP,RESET_BOOT", "FWUP,ACK,RESET_BOOT", 1200UL)) {
        megaHexValid = false;
      }
    }

    megaUploadInProgress = false;
    megaForwardingActive = false;
    megaUploadProgressPct = 100;

    if (!megaHexValid) {
      megaUploadMessage = "Upload completato ma trasferimento Mega fallito";
    } else {
      megaUploadMessage = "Trasferimento completato: Mega riavviato, verifica bootloader in corso";
      appendMegaUploadLog("Trasferimento terminato in " + String(millis() - megaUploadStartedAt) + " ms");

      megaBootloaderDetected = probeMegaBootloaderStk500v1(2200UL);
      megaBootloaderProbeAt = millis();
      if (megaBootloaderDetected) {
        megaUploadMessage = "Bootloader Mega rilevato: target pronto al flashing seriale";
        appendMegaUploadLog("Probe bootloader OK (STK500v1 sync)");
      } else {
        megaUploadMessage = "Reset eseguito ma bootloader non rilevato su Serial3";
        megaUploadError = "Serve bootloader compatibile su UART collegata all'ESP";
        appendMegaUploadLog("Probe bootloader FALLITA su Serial3");
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    megaUploadInProgress = false;
    megaForwardingActive = false;
    megaHexValid = false;
    megaUploadMessage = "Upload annullato";
    megaUploadError = "Trasferimento interrotto";
    megaHexLineBuffer = "";
    appendMegaUploadLog("Upload annullato dal client");
  }
}

// ──────────────────────────────────────────────────────────
//  Comando verso ATmega2560 via Serial (→ Mega Serial3)
// ──────────────────────────────────────────────────────────
void sendArduinoCommand(const String& command) {
  Serial.println(command);
}

// ──────────────────────────────────────────────────────────
//  Gestione WiFi
// ──────────────────────────────────────────────────────────
void startFallbackAp() {
  if (apFallbackActive) return;
  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(fallbackApSsid, fallbackApPassword);
  apFallbackActive = ok;
}

void setupWifi() {
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  lastWifiReconnectAttempt = millis();
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000UL) {
    delay(250);
    yield();
  }

  if (WiFi.status() != WL_CONNECTED) startFallbackAp();
}

void publishStatus(bool force);

bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    if (apFallbackActive) {
      WiFi.softAPdisconnect(true);
      apFallbackActive = false;
      WiFi.mode(WIFI_STA);
    }
    return true;
  }

  unsigned long now = millis();
  if ((now - lastWifiReconnectAttempt) < 10000UL) return false;
  lastWifiReconnectAttempt = now;
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  if (!apFallbackActive) startFallbackAp();
  return false;
}

// ──────────────────────────────────────────────────────────
//  MQTT
// ──────────────────────────────────────────────────────────
void reconnectMqtt() {
  if (WiFi.status() != WL_CONNECTED || client.connected()) return;

  unsigned long now = millis();
  if ((now - lastMqttReconnectAttempt) < 5000UL) return;
  lastMqttReconnectAttempt = now;

  if (client.connect(mqtt_client_id, mqtt_user, mqtt_password,
                     mqtt_topic_availability, 1, true, "offline")) {
    client.subscribe(mqtt_topic_command);
    client.publish(mqtt_topic_availability, "online", true);
    String ip = WiFi.localIP().toString();
    client.publish(mqtt_topic_ip, ip.c_str(), true);
    publishStatus(true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if      (msg == "open")   sendArduinoCommand("OPEN");
  else if (msg == "close")  sendArduinoCommand("CLOSE");
  else if (msg == "stop")   sendArduinoCommand("STOP");
  else if (msg == "status") sendArduinoCommand("STATUS?");
}

// ──────────────────────────────────────────────────────────
//  Parser messaggi dal Mega2560
// ──────────────────────────────────────────────────────────
String normalizeNumericString(String value) {
  value.trim();
  value.replace(',', '.');
  return value;
}

void updateRemoteField(const String& key, const String& value) {
  if      (key == "TV")    remoteState.tvOn             = (value == "1");
  else if (key == "I")     remoteState.currentRms       = normalizeNumericString(value).toFloat();
  else if (key == "THR")   remoteState.currentThreshold = normalizeNumericString(value).toFloat();
  else if (key == "H")     remoteState.height           = value.toInt();
  else if (key == "SOK")   remoteState.heightSensorOk   = (value == "1");
  else if (key == "SMODEL") remoteState.sensorModel     = value;
  else if (key == "SSTAT") remoteState.sensorStatus     = value;
  else if (key == "SADDR") remoteState.sensorAddress    = value;
  else if (key == "POS")   remoteState.tvPosition       = value;
  else if (key == "MIN")   remoteState.minHeight        = value.toInt();
  else if (key == "MAX")   remoteState.maxHeight        = value.toInt();
  else if (key == "T1")    remoteState.T1               = value.toInt();
  else if (key == "T2")    remoteState.T2               = value.toInt();
  else if (key == "T3")    remoteState.T3               = value.toInt();
  else if (key == "T4")    remoteState.T4               = value.toInt();
  else if (key == "T5")    remoteState.T5               = value.toInt();
  else if (key == "T6")    remoteState.T6               = value.toInt();
  else if (key == "TON")   remoteState.tvOnConfirmMs    = value.toInt();
  else if (key == "TOFF")  remoteState.tvOffConfirmMs   = value.toInt();
  else if (key == "STATE") remoteState.stateCode        = value;
  else if (key == "LABEL") remoteState.stateLabel       = value;
}

void handleArduinoLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "READY" || line == "PONG" ||
      line.startsWith("ACK,") || line.startsWith("ERR,")) {
    remoteState.arduinoOnline = true;
    remoteState.lastUpdate    = millis();
    return;
  }

  if (!line.startsWith("STATUS,")) return;

  int start = 7;
  while (start < (int)line.length()) {
    int comma = line.indexOf(',', start);
    String token = (comma >= 0) ? line.substring(start, comma) : line.substring(start);
    int eq = token.indexOf('=');
    if (eq > 0) updateRemoteField(token.substring(0, eq), token.substring(eq + 1));
    if (comma < 0) break;
    start = comma + 1;
  }

  remoteState.arduinoOnline = true;
  remoteState.lastUpdate    = millis();
  publishStatus(false);
}

void processSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      handleArduinoLine(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += ch;
      if (serialBuffer.length() > serialLineMaxLen) serialBuffer = "";
    }
  }
}

// ──────────────────────────────────────────────────────────
//  Pubblicazione MQTT
// ──────────────────────────────────────────────────────────
void publishStatus(bool force) {
  if (!client.connected()) return;

  unsigned long now = millis();
  if (!force && (now - lastPublish) < 5000) return;

  String status =
    "TV:"    + String(remoteState.tvOn ? "1" : "0") +
    ",I_RMS:"  + String(remoteState.currentRms, 3) +
    ",I_THR:"  + String(remoteState.currentThreshold, 3) +
    ",H:"      + String(remoteState.height) +
    ",S_OK:"   + String(remoteState.heightSensorOk ? "1" : "0") +
    ",S_MODEL:" + remoteState.sensorModel +
    ",S_STAT:" + remoteState.sensorStatus +
    ",S_ADDR:" + remoteState.sensorAddress +
    ",POS:"    + remoteState.tvPosition +
    ",H_MIN:"  + String(remoteState.minHeight) +
    ",H_MAX:"  + String(remoteState.maxHeight) +
    ",State:"  + remoteState.stateLabel +
    ",Mega:"   + String(remoteState.arduinoOnline ? "1" : "0") +
    ",IP:"     + WiFi.localIP().toString();

  client.publish(mqtt_topic_status, status.c_str(), true);
  lastPublish = now;
}

// ──────────────────────────────────────────────────────────
//  Route HTTP
// ──────────────────────────────────────────────────────────
void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleOpen()  { sendArduinoCommand("OPEN");  redirectHome(); }
void handleClose() { sendArduinoCommand("CLOSE"); redirectHome(); }
void handleStop()  { sendArduinoCommand("STOP");  redirectHome(); }

void handleSetTimes() {
  if (server.hasArg("t1")) sendArduinoCommand("SET,T1," + server.arg("t1"));
  if (server.hasArg("t2")) sendArduinoCommand("SET,T2," + server.arg("t2"));
  if (server.hasArg("t3")) sendArduinoCommand("SET,T3," + server.arg("t3"));
  if (server.hasArg("t4")) sendArduinoCommand("SET,T4," + server.arg("t4"));
  if (server.hasArg("t5")) sendArduinoCommand("SET,T5," + server.arg("t5"));
  if (server.hasArg("t6")) sendArduinoCommand("SET,T6," + server.arg("t6"));
  if (server.hasArg("ton")) sendArduinoCommand("SET,TON," + server.arg("ton"));
  if (server.hasArg("toff")) sendArduinoCommand("SET,TOFF," + server.arg("toff"));
  sendArduinoCommand("STATUS?");
  redirectHome();
}

void handleSetLimits() {
  if (server.hasArg("currentThreshold")) {
    sendArduinoCommand("SET,THR," + normalizeNumericString(server.arg("currentThreshold")));
  }
  if (server.hasArg("minHeight"))        sendArduinoCommand("SET,MIN," + server.arg("minHeight"));
  if (server.hasArg("maxHeight"))        sendArduinoCommand("SET,MAX," + server.arg("maxHeight"));
  sendArduinoCommand("STATUS?");
  redirectHome();
}

void handleSetPulse() {
  if (server.hasArg("pulseMs")) {
    int p = server.arg("pulseMs").toInt();
    if (p >= 100 && p <= 3000) manualPulseMs = p;
  }
  redirectHome();
}

void handleManual() {
  if (!server.hasArg("r")) { redirectHome(); return; }
  String rc = server.arg("r");
  rc.trim(); rc.toUpperCase();
  if (rc != "R1" && rc != "R2" && rc != "R3" &&
      rc != "R4" && rc != "R5" && rc != "R6") { redirectHome(); return; }
  sendArduinoCommand("JOG," + rc + "," + String(manualPulseMs));
  redirectHome();
}

void handleApiStatus() {
  String json = "{";
  json += "\"tvOn\":"           + String(remoteState.tvOn       ? "true" : "false");
  json += ",\"arduinoOnline\":" + String(remoteState.arduinoOnline ? "true" : "false");
  json += ",\"mqttOnline\":"    + String(client.connected()     ? "true" : "false");
  json += ",\"heightSensorOk\":" + String(remoteState.heightSensorOk ? "true" : "false");
  json += ",\"ip\":\""          + WiFi.localIP().toString() + "\"";
  json += ",\"currentRms\":"    + String(remoteState.currentRms, 4);
  json += ",\"currentThreshold\":" + String(remoteState.currentThreshold, 4);
  json += ",\"height\":"        + String(remoteState.height);
  json += ",\"sensorModel\":\"" + remoteState.sensorModel + "\"";
  json += ",\"sensorStatus\":\"" + remoteState.sensorStatus + "\"";
  json += ",\"sensorAddress\":\"" + remoteState.sensorAddress + "\"";
  json += ",\"tvPosition\":\"" + remoteState.tvPosition + "\"";
  json += ",\"minHeight\":"     + String(remoteState.minHeight);
  json += ",\"maxHeight\":"     + String(remoteState.maxHeight);
  json += ",\"t1\":"  + String(remoteState.T1);
  json += ",\"t2\":"  + String(remoteState.T2);
  json += ",\"t3\":"  + String(remoteState.T3);
  json += ",\"t4\":"  + String(remoteState.T4);
  json += ",\"t5\":"  + String(remoteState.T5);
  json += ",\"t6\":"  + String(remoteState.T6);
  json += ",\"tvOnConfirmMs\":" + String(remoteState.tvOnConfirmMs);
  json += ",\"tvOffConfirmMs\":" + String(remoteState.tvOffConfirmMs);
  json += ",\"state\":\"" + remoteState.stateLabel + "\"";
  json += ",\"manualPulseMs\":" + String(manualPulseMs);
  json += "}";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", json);
}

void handleRoot() {
  String tvClass  = remoteState.tvOn          ? "ok"  : "warn";
  String ardClass = remoteState.arduinoOnline ? "ok"  : "err";
  String mqClass  = client.connected()        ? "ok"  : "err";
  String sensorClass = remoteState.heightSensorOk ? "ok" : "err";

  String html = "<!doctype html><html><head><title>TV Lift - Mega2560</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>";
  html += ":root{--bg:#e8f1f7;--ink:#10202d;--muted:#5a6b79;--card:#ffffff;--line:#d5e1ec;--ok:#0b8457;--warn:#ad7a09;--err:#b42323;--a:#0f4c81;--b:#2a7f97;--btn:#1d5876;--btn2:#0f7a55;--btn3:#8b4f10;--btn4:#a12727;}";
  html += "*{box-sizing:border-box}body{margin:0;color:var(--ink);font-family:'Trebuchet MS','Segoe UI',Tahoma,sans-serif;background:linear-gradient(155deg,#f7fbff 0%,#e7f0f8 55%,#deebf6 100%)}";
  html += ".wrap{max-width:1100px;margin:0 auto;padding:16px}.hero{background:linear-gradient(125deg,var(--a),var(--b));color:#fff;border-radius:18px;padding:20px 22px;box-shadow:0 16px 36px rgba(13,57,88,.24)}";
  html += ".hero h1{margin:0 0 5px;font-size:1.55rem;letter-spacing:.2px}.hero p{margin:0;opacity:.95}";
  html += ".badges{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}.badge{padding:7px 11px;border-radius:999px;font-weight:700;font-size:.86rem;border:1px solid rgba(255,255,255,.25)}";
  html += ".ok{background:rgba(11,132,87,.16);color:#0c724b}.warn{background:rgba(173,122,9,.16);color:#8b6409}.err{background:rgba(180,35,35,.16);color:#8f2222}";
  html += ".grid{display:grid;gap:12px;margin-top:13px}.stats{grid-template-columns:repeat(auto-fit,minmax(170px,1fr))}.panels{grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}";
  html += ".card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px;box-shadow:0 10px 22px rgba(15,35,55,.07)}";
  html += ".k{font-size:.8rem;color:var(--muted)}.v{margin-top:5px;font-size:1.17rem;font-weight:800}";
  html += ".actions,.manual{display:grid;gap:8px}.actions{grid-template-columns:repeat(3,minmax(0,1fr))}.manual{grid-template-columns:repeat(2,minmax(0,1fr));margin-top:8px}";
  html += ".btn{display:block;text-align:center;padding:11px 10px;border-radius:11px;color:#fff;text-decoration:none;font-weight:800;border:0;cursor:pointer}.btn-main{background:var(--btn)}.btn-open{background:var(--btn2)}.btn-close{background:var(--btn3)}.btn-stop{background:var(--btn4)}";
  html += ".btn-man-up{background:#23704a}.btn-man-down{background:#7d5626}";
  html += ".row{display:grid;grid-template-columns:1fr 128px;gap:10px;align-items:center;margin:8px 0}.row label{font-size:.92rem;color:var(--muted)}";
  html += "input{width:100%;padding:9px 10px;border:1px solid #c6d5e3;border-radius:10px;background:#f9fcff;color:var(--ink)}.submit{margin-top:8px;background:var(--btn);color:#fff;border:0;border-radius:10px;padding:10px 12px;font-weight:800;width:100%;cursor:pointer}";
  html += ".manual-head{display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap}.mini{font-size:.82rem;color:var(--muted)}.foot{margin-top:12px;text-align:center;color:var(--muted);font-size:.84rem}";
  html += "@media (max-width:760px){.actions{grid-template-columns:1fr}.row{grid-template-columns:1fr}}";
  html += "</style></head><body><div class='wrap'>";

  html += "<section class='hero'><h1>TV Lift — Mega2560 WiFi R3</h1><p>Gateway ESP8266 integrato &bull; ponte Serial3 con ATmega2560</p><div class='badges'>";
  html += "<span id='badge-tv' class='badge " + tvClass  + "'>TV: <span id='tv-state'>"      + String(remoteState.tvOn          ? "Accesa"  : "Spenta")  + "</span></span>";
  html += "<span id='badge-arduino' class='badge " + ardClass + "'>Mega: <span id='arduino-state'>" + String(remoteState.arduinoOnline ? "Online"  : "Offline") + "</span></span>";
  html += "<span id='badge-mqtt' class='badge " + mqClass  + "'>MQTT: <span id='mqtt-state'>" + String(client.connected()        ? "Online"  : "Offline") + "</span></span>";
  html += "<span id='badge-sensor' class='badge " + sensorClass + "'>Sensore H: <span id='sensor-state'>" + String(remoteState.heightSensorOk ? "OK" : "Errore") + "</span></span>";
  html += "<span class='badge'>IP: <span id='device-ip'>" + WiFi.localIP().toString() + "</span></span></div></section>";

  html += "<section class='grid stats'>";
  html += "<div class='card'><div class='k'>Corrente RMS</div><div id='current-rms' class='v'>"  + String(remoteState.currentRms, 3)       + " A</div></div>";
  html += "<div class='card'><div class='k'>Soglia TV ON</div><div id='current-thr' class='v'>"  + String(remoteState.currentThreshold, 3) + " A</div></div>";
  html += "<div class='card'><div class='k'>Altezza corrente</div><div id='height-now' class='v'>"   + String(remoteState.height)    + " mm</div></div>";
  html += "<div class='card'><div class='k'>Posizione TV</div><div id='tv-position' class='v'>" + remoteState.tvPosition + "</div></div>";
  html += "<div class='card'><div class='k'>Target min/max</div><div id='height-target' class='v'>"  + String(remoteState.minHeight) + " / " + String(remoteState.maxHeight) + " mm</div></div>";
  html += "<div class='card'><div class='k'>Diagnostica sensore</div><div id='sensor-diag' class='v'>" + remoteState.sensorModel + " @ " + remoteState.sensorAddress + "</div><div class='mini' id='sensor-status'>" + remoteState.sensorStatus + "</div></div>";
  html += "<div class='card'><div class='k'>Stato sequenza</div><div id='state-text' class='v'>"     + remoteState.stateLabel + "</div></div>";
  html += "<div class='card'><div class='k'>Pulse manuale</div><div id='pulse-now' class='v'>"       + String(manualPulseMs) + " ms</div></div>";
  html += "</section>";

  html += "<section class='card'><div class='k'>Comandi automatici</div><div class='actions'>";
  html += "<a class='btn btn-open'  href='/open'>Apri sequenza</a>";
  html += "<a class='btn btn-close' href='/close'>Chiudi sequenza</a>";
  html += "<a class='btn btn-stop'  href='/stop'>Stop immediato</a>";
  html += "</div></section>";

  html += "<section class='card'><div class='k'>Manutenzione ESP8266</div><div class='mini' style='margin:8px 0'>Aggiorna il firmware del gateway via rete caricando un file .bin</div>";
  html += "<a class='btn btn-main' href='/update'>Aggiorna firmware ESP (OTA)</a></section>";

  html += "<section class='card'><div class='k'>Manutenzione Mega2560</div><div class='mini' style='margin:8px 0'>Upload firmware .hex e validazione web (fase flashing in integrazione)</div>";
  html += "<a class='btn btn-main' href='/mega-update'>Sezione update Mega2560</a></section>";

  html += "<section class='card'><div class='manual-head'><div class='k'>Pilotaggio manuale pistoni</div><div class='mini'>Ogni click attiva il rele per il tempo impostato</div></div><div class='manual'>";
  html += "<a class='btn btn-man-up'   href='/manual?r=R1'>Pistone Anteriore Su</a>";
  html += "<a class='btn btn-man-down' href='/manual?r=R2'>Pistone Anteriore Giu</a>";
  html += "<a class='btn btn-man-up'   href='/manual?r=R3'>TV Su</a>";
  html += "<a class='btn btn-man-down' href='/manual?r=R4'>TV Giu</a>";
  html += "<a class='btn btn-man-up'   href='/manual?r=R5'>Pistone Posteriore Su</a>";
  html += "<a class='btn btn-man-down' href='/manual?r=R6'>Pistone Posteriore Giu</a>";
  html += "</div><form action='/setpulse' method='POST' style='margin-top:10px'>";
  html += "<div class='row'><label>Durata impulso manuale (100-3000 ms)</label><input type='number' min='100' max='3000' name='pulseMs' value='" + String(manualPulseMs) + "'></div>";
  html += "<button class='submit' type='submit'>Salva durata impulso</button></form></section>";

  html += "<section class='grid panels'>";
  html += "<article class='card'><h3 style='margin:0 0 8px'>Tempi attuatori (ms)</h3><div class='mini' style='margin-bottom:8px'>I campi restano fermi mentre li modifichi</div><form action='/settimes' method='POST' data-live-lock='1'>";
  html += "<div class='row'><label>T1 Apertura anteriore</label><input id='t1' type='number' name='t1' value='" + String(remoteState.T1) + "'></div>";
  html += "<div class='row'><label>T2 Chiusura anteriore</label><input id='t2' type='number' name='t2' value='" + String(remoteState.T2) + "'></div>";
  html += "<div class='row'><label>T3 Alzata TV</label><input id='t3' type='number' name='t3' value='"          + String(remoteState.T3) + "'></div>";
  html += "<div class='row'><label>T4 Abbassamento TV</label><input id='t4' type='number' name='t4' value='"    + String(remoteState.T4) + "'></div>";
  html += "<div class='row'><label>T5 Apertura posteriore</label><input id='t5' type='number' name='t5' value='" + String(remoteState.T5) + "'></div>";
  html += "<div class='row'><label>T6 Chiusura posteriore</label><input id='t6' type='number' name='t6' value='" + String(remoteState.T6) + "'></div>";
  html += "<div class='row'><label>Conferma TV ON (ms sopra soglia)</label><input id='ton' type='number' min='500' max='120000' name='ton' value='" + String(remoteState.tvOnConfirmMs) + "'></div>";
  html += "<div class='row'><label>Conferma TV OFF (ms sotto soglia)</label><input id='toff' type='number' min='1000' max='300000' name='toff' value='" + String(remoteState.tvOffConfirmMs) + "'></div>";
  html += "<button class='submit' type='submit'>Salva tempi</button></form></article>";

  html += "<article class='card'><h3 style='margin:0 0 8px'>Sicurezza sensori</h3><div class='mini' style='margin-bottom:8px'>Anche questi valori non vengono sovrascritti durante la modifica</div><form action='/setlimits' method='POST' data-live-lock='1'>";
  html += "<div class='row'><label>Soglia corrente TV ON (A)</label><input id='thr' step='0.01' type='number' name='currentThreshold' value='" + String(remoteState.currentThreshold, 2) + "'></div>";
  html += "<div class='row'><label>Altezza minima (mm)</label><input id='hmin' type='number' name='minHeight' value='"  + String(remoteState.minHeight) + "'></div>";
  html += "<div class='row'><label>Altezza massima (mm)</label><input id='hmax' type='number' name='maxHeight' value='" + String(remoteState.maxHeight) + "'></div>";
  html += "<button class='submit' type='submit'>Salva limiti</button></form></article>";
  html += "</section>";

  html += "<div class='foot'>Funzionamento normale: SW1,SW2,SW5,SW6=ON &mdash; tutti gli altri OFF</div>";
  html += "<script>";
  html += "function txt(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}";
  html += "function isLockedField(e){return !!(e&&e.form&&e.form.dataset.liveLock==='1'&&e.form.dataset.editing==='1');}";
  html += "function val(id,v){var e=document.getElementById(id);if(!e)return;if(document.activeElement===e||isLockedField(e))return;e.value=v;}";
  html += "function badge(id,k){var e=document.getElementById(id);if(!e)return;e.classList.remove('ok','warn','err');e.classList.add(k);}";
  html += "function bindLiveLockedForms(){var forms=document.querySelectorAll(\"form[data-live-lock='1']\");for(var i=0;i<forms.length;i++){(function(form){form.dataset.editing='0';form.addEventListener('input',function(){form.dataset.editing='1';});form.addEventListener('submit',function(){form.dataset.editing='0';});})(forms[i]);}}";
  html += "async function poll(){try{var r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)return;var d=await r.json();";
  html += "txt('tv-state',d.tvOn?'Accesa':'Spenta');txt('arduino-state',d.arduinoOnline?'Online':'Offline');txt('mqtt-state',d.mqttOnline?'Online':'Offline');txt('sensor-state',d.heightSensorOk?'OK':'Errore');txt('device-ip',d.ip);";
  html += "txt('current-rms',d.currentRms.toFixed(3)+' A');txt('current-thr',d.currentThreshold.toFixed(3)+' A');txt('height-now',Math.round(d.height)+' mm');txt('tv-position',d.tvPosition);txt('height-target',Math.round(d.minHeight)+' / '+Math.round(d.maxHeight)+' mm');txt('sensor-diag',d.sensorModel+' @ '+d.sensorAddress);txt('sensor-status',d.sensorStatus);txt('state-text',d.state);txt('pulse-now',d.manualPulseMs+' ms');";
  html += "badge('badge-tv',d.tvOn?'ok':'warn');badge('badge-arduino',d.arduinoOnline?'ok':'err');badge('badge-mqtt',d.mqttOnline?'ok':'err');badge('badge-sensor',d.heightSensorOk?'ok':'err');";
  html += "val('t1',d.t1);val('t2',d.t2);val('t3',d.t3);val('t4',d.t4);val('t5',d.t5);val('t6',d.t6);val('ton',d.tvOnConfirmMs);val('toff',d.tvOffConfirmMs);val('thr',d.currentThreshold.toFixed(2));val('hmin',d.minHeight);val('hmax',d.maxHeight);";
  html += "}catch(e){}}bindLiveLockedForms();poll();setInterval(poll,2000);";
  html += "</script>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// ──────────────────────────────────────────────────────────
//  Setup & Loop
// ──────────────────────────────────────────────────────────
void setup() {
  // Serial comunica con ATmega2560 via DIP SW5/SW6 (Mega Serial3)
  Serial.begin(57600);

  setupWifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  server.on("/",           handleRoot);
  server.on("/api/status", HTTP_GET,  handleApiStatus);
  server.on("/open",       handleOpen);
  server.on("/close",      handleClose);
  server.on("/stop",       handleStop);
  server.on("/settimes",   HTTP_POST, handleSetTimes);
  server.on("/setlimits",  HTTP_POST, handleSetLimits);
  server.on("/setpulse",   HTTP_POST, handleSetPulse);
  server.on("/manual",     HTTP_GET,  handleManual);
  server.on("/mega-update", HTTP_GET, handleMegaUpdatePage);
  server.on("/api/mega-update", HTTP_GET, handleApiMegaUpdate);
  server.on("/mega-update/upload", HTTP_POST, handleMegaUploadFinish, handleMegaUploadStream);
  httpUpdater.setup(&server, "/update", ota_update_user, ota_update_password);
  server.begin();

  delay(500);
  sendArduinoCommand("PING");
  sendArduinoCommand("STATUS?");
}

void loop() {
  bool wifiOk = ensureWifiConnected();
  if (wifiOk && !client.connected()) reconnectMqtt();
  if (client.connected()) client.loop();

  server.handleClient();
  processSerial();

  unsigned long now = millis();
  if (!megaUploadInProgress && (now - lastStatusRequest >= 1000)) {
    lastStatusRequest = now;
    sendArduinoCommand("STATUS?");
  }

  if (now - remoteState.lastUpdate > arduinoOfflineTimeoutMs) {
    remoteState.arduinoOnline = false;
  }

  publishStatus(false);
}
