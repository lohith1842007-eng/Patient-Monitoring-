/*
  PulseGrid Bluetooth Patient Monitor
  Board: ESP32 Dev Module (ESP32-WROOM-32)

  Libraries:
  - SparkFun MAX3010x Pulse and Proximity Sensor Library
  - TinyGPSPlus by Mikal Hart

  Android & PC Connection: Bluetooth Classic SPP & Web Serial
  Bluetooth Device Name: PulseGrid
*/

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <BluetoothSerial.h>
#include "MAX30105.h"
#include "heartRate.h"

// ---------------- USER SETTINGS ----------------

const char PHONE_NUMBER[] = "+917010191275";

// Enable only after verifying GSM supply and UART levels.
const bool ENABLE_GSM_SMS = false;

// ---------------- PIN CONFIGURATION ----------------

#define SDA_PIN       21
#define SCL_PIN       22

#define GPS_RX_PIN    16
#define GPS_TX_PIN    17

#define GSM_RX_PIN    26
#define GSM_TX_PIN    27

#define SOS_PIN       18
#define BUZZER_PIN    19

// ---------------- MODULES ----------------

MAX30105 particleSensor;
TinyGPSPlus gps;
BluetoothSerial SerialBT;

HardwareSerial gpsSerial(1);
HardwareSerial gsmSerial(2);

// ---------------- VARIABLES ----------------

bool sensorFound = false;
bool heartRateAvailable = false;
bool spo2Available = false;
bool gpsFix = false;
bool sosStatus = false;
bool smsBusy = false;

float heartRateBPM = 0;
float spo2 = 0;
double latitude = 0;
double longitude = 0;

String smsStatus = "DISABLED";

unsigned long alertCount = 0;
unsigned long lastBeatTime = 0;
unsigned long lastTelemetry = 0;
unsigned long lastDebug = 0;
unsigned long lastGPSDebug = 0;
unsigned long lastButtonChange = 0;

int lastButtonReading = HIGH;
int stableButtonState = HIGH;

// Heart Rate averaging buffer
const byte RATE_SIZE = 4;
float rates[RATE_SIZE] = {0};
byte rateIndex = 0;
byte rateCount = 0;

// SpO2 buffer
const int SPO2_BUF_LEN = 50;
uint32_t irBuffer[SPO2_BUF_LEN];
uint32_t redBuffer[SPO2_BUF_LEN];
int spo2BufIndex = 0;
bool spo2BufFull = false;

String alertHistory[5];
int historyCount = 0;

// ---------------- ALERT HISTORY ----------------

void addHistory(const String &message) {
  if (historyCount < 5) {
    alertHistory[historyCount++] = message;
  } else {
    for (int i = 0; i < 4; i++) {
      alertHistory[i] = alertHistory[i + 1];
    }
    alertHistory[4] = message;
  }
}

// ---------------- BUZZER ----------------

void beepBuzzer(unsigned long durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}

// ---------------- GSM SMS ----------------

bool sendSMS(String message) {
  if (!ENABLE_GSM_SMS) {
    smsStatus = "DISABLED";
    Serial.println("SMS disabled in settings.");
    return false;
  }

  smsBusy = true;
  smsStatus = "SENDING";

  while (gsmSerial.available()) {
    gsmSerial.read();
  }

  gsmSerial.println("AT");
  delay(1000);
  gsmSerial.println("AT+CMGF=1");
  delay(1000);

  while (gsmSerial.available()) {
    gsmSerial.read();
  }

  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println("\"");

  String response = "";
  unsigned long start = millis();
  bool promptReceived = false;

  while (millis() - start < 5000) {
    while (gsmSerial.available()) {
      char c = (char)gsmSerial.read();
      response += c;

      if (c == '>') {
        promptReceived = true;
        break;
      }
    }
    if (promptReceived) break;
    delay(10);
  }

  if (!promptReceived) {
    smsStatus = "ERROR";
    smsBusy = false;
    Serial.println("No SMS prompt from SIM800L.");
    return false;
  }

  gsmSerial.print(message);
  gsmSerial.write(26);

  response = "";
  start = millis();

  while (millis() - start < 20000) {
    while (gsmSerial.available()) {
      response += (char)gsmSerial.read();
    }

    if (response.indexOf("+CMGS:") >= 0) {
      smsStatus = "SUBMITTED";
      smsBusy = false;
      Serial.println("SMS submitted by SIM800L.");
      return true;
    }

    if (response.indexOf("ERROR") >= 0) {
      smsStatus = "ERROR";
      smsBusy = false;
      Serial.println(response);
      return false;
    }

    delay(10);
  }

  smsStatus = "UNCONFIRMED";
  smsBusy = false;
  Serial.println("SMS result not confirmed.");
  return false;
}

// ---------------- SOS ----------------

void triggerSOS() {
  if (sosStatus || smsBusy) {
    Serial.println("SOS already active or busy.");
    return;
  }

  sosStatus = true;
  alertCount++;
  addHistory("SOS alert triggered");

  Serial.println("SOS ALERT!");

  // Notify connected clients immediately
  Serial.println("EVENT=SOS");
  if (SerialBT.hasClient()) {
    SerialBT.println("EVENT=SOS");
  }

  beepBuzzer(1000);

  String message = "SOS ALERT! Assistance requested.";

  if (gpsFix) {
    message += " Location: https://maps.google.com/?q=";
    message += String(latitude, 6);
    message += ",";
    message += String(longitude, 6);
  } else {
    message += " GPS fix unavailable.";
  }

  if (heartRateAvailable) {
    message += " Last HR estimate: ";
    message += String(heartRateBPM, 1);
    message += " BPM.";
  }

  if (spo2Available && spo2 > 0) {
    message += " SpO2: ";
    message += String(spo2, 1);
    message += "%.";
  }

  Serial.print("SMS recipient: ");
  Serial.println(PHONE_NUMBER);

  sendSMS(message);
}

// ---------------- SPO2 CALCULATION ----------------

void calculateSpO2() {
  if (!spo2BufFull) return;

  uint32_t irMin = 0xFFFFFFFF, irMax = 0, irSum = 0;
  uint32_t redMin = 0xFFFFFFFF, redMax = 0, redSum = 0;

  for (int i = 0; i < SPO2_BUF_LEN; i++) {
    irSum += irBuffer[i];
    redSum += redBuffer[i];
    if (irBuffer[i] < irMin) irMin = irBuffer[i];
    if (irBuffer[i] > irMax) irMax = irBuffer[i];
    if (redBuffer[i] < redMin) redMin = redBuffer[i];
    if (redBuffer[i] > redMax) redMax = redBuffer[i];
  }

  float irDC = (float)irSum / SPO2_BUF_LEN;
  float redDC = (float)redSum / SPO2_BUF_LEN;
  float irAC = (float)(irMax - irMin);
  float redAC = (float)(redMax - redMin);

  if (irDC > 0 && redDC > 0 && irAC > 0) {
    float R = (redAC / redDC) / (irAC / irDC);
    float calcSpO2 = 110.0 - (25.0 * R);

    if (calcSpO2 >= 70.0 && calcSpO2 <= 100.0) {
      if (spo2 <= 0) {
        spo2 = calcSpO2;
      } else {
        spo2 = (spo2 * 0.8) + (calcSpO2 * 0.2);
      }
      spo2Available = true;
    }
  }
}

// ---------------- HEART RATE & SPO2 ----------------

void readHeartRate() {
  if (!sensorFound) return;

  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  if (millis() - lastDebug >= 1000) {
    lastDebug = millis();

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(" | HR=");

    if (heartRateAvailable) {
      Serial.print(heartRateBPM, 1);
      Serial.print(" BPM | SpO2=");
      if (spo2Available) Serial.print(spo2, 1); else Serial.print("Calc");
      Serial.println("%");
    } else {
      Serial.println("Waiting for finger / valid beats");
    }
  }

  if (irValue < 50000) {
    heartRateAvailable = false;
    spo2Available = false;
    rateIndex = 0;
    rateCount = 0;
    lastBeatTime = 0;
    return;
  }

  // Populate SpO2 buffers
  irBuffer[spo2BufIndex] = irValue;
  redBuffer[spo2BufIndex] = redValue;
  spo2BufIndex = (spo2BufIndex + 1) % SPO2_BUF_LEN;
  if (spo2BufIndex == 0) spo2BufFull = true;

  if (checkForBeat(irValue)) {
    unsigned long now = millis();

    if (lastBeatTime != 0) {
      float elapsedSeconds = (now - lastBeatTime) / 1000.0;

      if (elapsedSeconds > 0) {
        float bpm = 60.0 / elapsedSeconds;

        if (bpm >= 40 && bpm <= 200) {
          rates[rateIndex] = bpm;
          rateIndex = (rateIndex + 1) % RATE_SIZE;

          if (rateCount < RATE_SIZE) rateCount++;

          float total = 0;
          for (byte i = 0; i < rateCount; i++) {
            total += rates[i];
          }

          heartRateBPM = total / rateCount;
          heartRateAvailable = true;

          calculateSpO2();
        }
      }
    }

    lastBeatTime = now;
  }
}

// ---------------- GPS ----------------

void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid() && gps.location.age() < 5000) {
    gpsFix = true;
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    gpsFix = false;
  }

  if (millis() - lastGPSDebug >= 5000) {
    lastGPSDebug = millis();

    Serial.print("GPS: ");
    if (gpsFix) {
      Serial.print(latitude, 6);
      Serial.print(", ");
      Serial.println(longitude, 6);
    } else {
      Serial.println("Waiting for GPS fix");
    }
  }
}

// ---------------- TELEMETRY DISPATCH ----------------

void sendTelemetry() {
  // Semicolon-separated line format:
  // HR=72.0;SPO2=98.0;GPS=FIX;LAT=12.971598;LON=77.594562;SOS=NORMAL;SMS=DISABLED;ALERTS=0
  String data = "HR=";

  if (heartRateAvailable) {
    data += String(heartRateBPM, 1);
  } else {
    data += "NA";
  }

  data += ";SPO2=";
  if (spo2Available && spo2 > 0) {
    data += String(spo2, 1);
  } else {
    data += "NA";
  }

  data += ";GPS=";
  data += gpsFix ? "FIX" : "SEARCHING";

  data += ";LAT=";
  data += gpsFix ? String(latitude, 6) : "NA";

  data += ";LON=";
  data += gpsFix ? String(longitude, 6) : "NA";

  data += ";SOS=";
  data += sosStatus ? "ACTIVE" : "NORMAL";

  data += ";SMS=";
  data += smsStatus;

  data += ";ALERTS=";
  data += String(alertCount);

  // Send over USB Serial for PC Web Dashboard / Debugging
  Serial.println(data);

  // Send over Bluetooth Classic SPP for Android App / PC Bluetooth
  if (SerialBT.hasClient()) {
    SerialBT.println(data);
  }
}

// ---------------- COMMAND PROCESSOR ----------------

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "SOS") {
    triggerSOS();
  } else if (command == "RESET_SOS") {
    sosStatus = false;
    Serial.println("EVENT=SOS_RESET");
    if (SerialBT.hasClient()) {
      SerialBT.println("EVENT=SOS_RESET");
    }
  } else if (command == "STATUS") {
    sendTelemetry();
  }
}

void readIncomingCommands() {
  // Read from Bluetooth
  static String btCommand = "";
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (btCommand.length() > 0) {
        handleCommand(btCommand);
        btCommand = "";
      }
    } else if (btCommand.length() < 40) {
      btCommand += c;
    }
  }

  // Read from USB Serial
  static String serialCommand = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        handleCommand(serialCommand);
        serialCommand = "";
      }
    } else if (serialCommand.length() < 40) {
      serialCommand += c;
    }
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("   PulseGrid Bluetooth Patient Monitor        ");
  Serial.println("   ESP32-WROOM-32 Telemetry Station           ");
  Serial.println("==============================================");

  pinMode(SOS_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  Serial.println("Starting Bluetooth Classic SPP...");

  if (SerialBT.begin("PulseGrid")) {
    Serial.println(">> Bluetooth Device: PulseGrid");
    Serial.println(">> Pair from Android or PC Bluetooth settings.");
  } else {
    Serial.println(">> Bluetooth initialization failed.");
  }

  Serial.println("Initializing MAX30102 pulse oximeter...");

  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    sensorFound = true;

    particleSensor.setup(
      60,    // LED brightness
      4,     // Sample average
      2,     // Red + IR
      100,   // Sample rate
      411,   // Pulse width
      4096   // ADC range
    );

    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);

    Serial.println(">> MAX30102 online.");
  } else {
    sensorFound = false;
    Serial.println(">> WARNING: MAX30102 not found; verify I2C wiring.");
  }

  Serial.println(">> Telemetry stream ready.");
  Serial.println("==============================================\n");
}

// ---------------- MAIN LOOP ----------------

void loop() {
  readGPS();
  readHeartRate();
  readIncomingCommands();

  // Debounce the physical SOS button
  int reading = digitalRead(SOS_PIN);

  if (reading != lastButtonReading) {
    lastButtonChange = millis();
  }

  if (millis() - lastButtonChange > 40) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        triggerSOS();
      }
    }
  }

  lastButtonReading = reading;

  if (millis() - lastTelemetry >= 1000) {
    lastTelemetry = millis();
    sendTelemetry();
  }

  delay(5);
}
