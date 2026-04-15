/*
  ev_bms_ina226.ino
  Arduino BMS for 36V VRLA pack using INA226 (I2C) and 3x DS18B20.
  - Pack voltage on A0 (Rtop=80k, Rbot=10k)
  - INA226 on I2C measures shunt (0.001 ohm)
  - DS18B20 1-wire on D2 (4.7k pull-up to 5V)
  - Precharge relay driver on D3 (MOSFET gate)
  - Contactor driver on D4 (MOSFET gate)
  - Buzzer on D5, Green LED D6, Red LED D7
  - Start/Reset button on D8 (INPUT_PULLUP)
*/

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------------- Configuration ----------------
const uint8_t INA226_ADDR = 0x40; // INA226 I2C address (change if different)
const float R_SHUNT = 0.001f;     // shunt in ohms (1 mΩ)
const float MAX_EXPECTED_CURRENT_A = 150.0f;
const float PREFERRED_CURRENT_LSB = 0.005f; // 5 mA LSB if sufficient

// Voltage divider constants (Rtop=80k, Rbot=10k)
const float ADC_REF = 5.00f;
const int ADC_MAX = 1023;
const float DIVIDER_RATIO = 9.0f; // (Rtop+Rbot)/Rbot = 90k/10k

// Pins
const int PIN_VPACK_ADC = A0;
const int PIN_DS18B20 = 2;
const int PIN_PRECHARGE = 3;
const int PIN_CONTACTOR = 4;
const int PIN_BUZZER = 5;
const int PIN_LED_GREEN = 6;
const int PIN_LED_RED = 7;
const int PIN_BUTTON = 8;

// Protection thresholds
const float PACK_CUTOFF_V = 35.0f;
const float PACK_RECONNECT_V = 36.8f;
const float PACK_RECONNECT_TIME = 10.0f; // seconds
const float OVERCURRENT_THRESHOLD_A = 80.0f;
const float OVERCURRENT_INSTANT_A = 120.0f;
const unsigned long OVERCURRENT_TIME_MS = 250;
const float OVERTEMP_TRIP_C = 55.0f;
const float OVERTEMP_WARN_C = 50.0f;

// Precharge timing
const unsigned long PRECHARGE_MIN_MS = 1200;
const unsigned long PRECHARGE_TIMEOUT_MS = 8000;

// smoothing
const float ALPHA = 0.2f;

// ---------------- INA226 register addresses ----------------
#define INA226_REG_CONFIG     0x00
#define INA226_REG_SHUNTVOLT  0x01
#define INA226_REG_BUSVOLT    0x02
#define INA226_REG_POWER      0x03
#define INA226_REG_CURRENT    0x04
#define INA226_REG_CALIB      0x05

// ---------------- Globals & state ----------------
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);
DeviceAddress tempAddresses[8];
int tempCount = 0;

enum State { STATE_OFF, STATE_PRECHARGE, STATE_RUNNING, STATE_FAULT_UNDERVOLT, STATE_FAULT_OVERCURRENT, STATE_FAULT_OVERTEMP, STATE_MANUAL_LOCK};
State state = STATE_OFF;

unsigned long prechargeStart = 0;
unsigned long overcurrentStartMs = 0;
unsigned long lastReconnectStart = 0;
unsigned long lastSerialMs = 0;
unsigned long lastButtonMillis = 0;
bool lastButtonState = HIGH;

float packVoltageFilt = 0.0f;
float currentFilt = 0.0f;

// INA226 calibration values
uint16_t ina226_cal_reg = 0;
float ina_current_lsb = 0.0f; // A per bit

// ---------------- Prototypes ----------------
void ina226WriteRegister(uint8_t reg, uint16_t value);
uint16_t ina226ReadRegister(uint8_t reg);
void ina226Init();
float ina226ReadShuntVoltage_V();
float ina226ReadBusVoltage_V();
float ina226ReadCurrent_A();

float readPackVoltage();
float readMaxTemperature();
void handleButtonPress();
void tripToFault(State f);
const char* stateToString(State s);
void printAddress(DeviceAddress deviceAddress);

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("EV BMS w/ INA226 starting..."));

  pinMode(PIN_PRECHARGE, OUTPUT);
  pinMode(PIN_CONTACTOR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);

  sensors.begin();
  tempCount = sensors.getDeviceCount();
  Serial.print(F("DS18B20 count: "));
  Serial.println(tempCount);
  for (int i=0;i<tempCount;i++){
    if (sensors.getAddress(tempAddresses[i], i)) {
      Serial.print(F("Found temp sensor: "));
      printAddress(tempAddresses[i]);
      Serial.println();
    }
  }

  Wire.begin(); // I2C
  ina226Init();

  state = STATE_OFF;
  packVoltageFilt = readPackVoltage();
  currentFilt = ina226ReadCurrent_A();
  lastSerialMs = millis();
}

// ---------------- Loop ----------------
void loop() {
  unsigned long now = millis();

  float vpack = readPackVoltage();
  packVoltageFilt = (1.0f - ALPHA) * packVoltageFilt + ALPHA * vpack;

  float currentA = ina226ReadCurrent_A();
  currentFilt = (1.0f - ALPHA) * currentFilt + ALPHA * currentA;

  float maxTemp = readMaxTemperature();

  bool btn = digitalRead(PIN_BUTTON);
  if (btn == LOW && lastButtonState == HIGH && (now - lastButtonMillis) > 200) {
    lastButtonMillis = now;
    handleButtonPress();
  }
  lastButtonState = btn;

  switch(state) {
    case STATE_OFF:
      digitalWrite(PIN_CONTACTOR, LOW);
      digitalWrite(PIN_PRECHARGE, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
      if (btn == LOW) {
        Serial.println(F("Start pressed -> PRECHARGE"));
        state = STATE_PRECHARGE;
        digitalWrite(PIN_PRECHARGE, HIGH);
        prechargeStart = now;
        digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW);
      }
      break;

    case STATE_PRECHARGE:
      digitalWrite(PIN_LED_RED, HIGH);
      if (now - prechargeStart >= PRECHARGE_MIN_MS) {
        Serial.println(F("Precharge done -> close contactor"));
        digitalWrite(PIN_CONTACTOR, HIGH);
        delay(200);
        digitalWrite(PIN_PRECHARGE, LOW);
        state = STATE_RUNNING;
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_BUZZER, HIGH); delay(50); digitalWrite(PIN_BUZZER, LOW);
      } else if (now - prechargeStart >= PRECHARGE_TIMEOUT_MS) {
        Serial.println(F("Precharge timeout, forcing contactor close"));
        digitalWrite(PIN_CONTACTOR, HIGH);
        delay(200);
        digitalWrite(PIN_PRECHARGE, LOW);
        state = STATE_RUNNING;
      }
      break;

    case STATE_RUNNING:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);

      if (packVoltageFilt <= PACK_CUTOFF_V) {
        Serial.println(F("UNDERVOLT TRIP"));
        tripToFault(STATE_FAULT_UNDERVOLT);
        break;
      }
      if (maxTemp >= OVERTEMP_TRIP_C) {
        Serial.println(F("OVERTEMP TRIP"));
        tripToFault(STATE_FAULT_OVERTEMP);
        break;
      }
      if (currentFilt >= OVERCURRENT_INSTANT_A) {
        Serial.println(F("INSTANT OVERCURRENT TRIP"));
        tripToFault(STATE_FAULT_OVERCURRENT);
        break;
      } else if (currentFilt >= OVERCURRENT_THRESHOLD_A) {
        if (overcurrentStartMs == 0) overcurrentStartMs = now;
        if ((now - overcurrentStartMs) >= OVERCURRENT_TIME_MS) {
          Serial.println(F("SUSTAINED OVERCURRENT TRIP"));
          tripToFault(STATE_FAULT_OVERCURRENT);
          overcurrentStartMs = 0;
          break;
        }
      } else {
        overcurrentStartMs = 0;
      }
      break;

    case STATE_FAULT_UNDERVOLT:
      digitalWrite(PIN_CONTACTOR, LOW);
      digitalWrite(PIN_PRECHARGE, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_BUZZER, HIGH);
      if (packVoltageFilt >= PACK_RECONNECT_V) {
        if (lastReconnectStart == 0) lastReconnectStart = now;
        if ((now - lastReconnectStart)/1000.0 >= PACK_RECONNECT_TIME) {
          Serial.println(F("Voltage stable - attempting auto reconnect"));
          lastReconnectStart = 0;
          state = STATE_PRECHARGE;
          digitalWrite(PIN_PRECHARGE, HIGH);
          prechargeStart = now;
          digitalWrite(PIN_BUZZER, LOW);
        }
      } else {
        lastReconnectStart = 0;
      }
      break;

    case STATE_FAULT_OVERCURRENT:
    case STATE_FAULT_OVERTEMP:
      digitalWrite(PIN_CONTACTOR, LOW);
      digitalWrite(PIN_PRECHARGE, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_BUZZER, HIGH);
      break;

    default:
      break;
  }

  if (millis() - lastSerialMs >= 1000) {
    lastSerialMs = millis();
    float shV = ina226ReadShuntVoltage_V();
    float busV = ina226ReadBusVoltage_V();
    Serial.print("Vpack: "); Serial.print(packVoltageFilt,3);
    Serial.print(" V, I(INA): "); Serial.print(currentFilt,2); Serial.print(" A");
    Serial.print(", ShuntV: "); Serial.print(shV*1000.0,3); Serial.print(" mV");
    Serial.print(", BusVraw: "); Serial.print(busV,3); Serial.print(" V");
    Serial.print(", T_max: "); Serial.print(readMaxTemperature(),2); Serial.print(" C");
    Serial.print(", State: "); Serial.println(stateToString(state));
  }

  delay(50);
}

// ---------------- INA226 utils ----------------
void ina226WriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t ina226ReadRegister(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(INA226_ADDR, (uint8_t)2);
  uint16_t val = 0;
  if (Wire.available() >= 2) {
    val = (Wire.read() << 8) | Wire.read();
  }
  return val;
}

void ina226Init() {
  // compute minimal current LSB and choose preferred if possible
  float min_lsb = MAX_EXPECTED_CURRENT_A / 32767.0f;
  ina_current_lsb = (PREFERRED_CURRENT_LSB >= min_lsb) ? PREFERRED_CURRENT_LSB : min_lsb;

  // cal = trunc(0.00512 / (current_LSB * R_shunt))
  float cal_f = 0.00512f / (ina_current_lsb * R_SHUNT);
  ina226_cal_reg = (uint16_t)cal_f;
  if (ina226_cal_reg == 0) ina226_cal_reg = 1;

  Serial.print("INA226: current_lsb A/bit = ");
  Serial.println(ina_current_lsb, 6);
  Serial.print("INA226: calib reg = ");
  Serial.println(ina226_cal_reg);

  // example config: averaging 16, bus conv 4.17 ms, shunt conv 4.17 ms, mode shunt+bus cont
  uint16_t config = 0x4127;
  ina226WriteRegister(INA226_REG_CONFIG, config);
  ina226WriteRegister(INA226_REG_CALIB, ina226_cal_reg);
}

float ina226ReadShuntVoltage_V() {
  uint16_t raw = ina226ReadRegister(INA226_REG_SHUNTVOLT);
  int16_t sraw = (int16_t)raw;
  float v = (float)sraw * 2.5e-6f; // 2.5 uV per bit
  return v;
}

float ina226ReadBusVoltage_V() {
  uint16_t raw = ina226ReadRegister(INA226_REG_BUSVOLT);
  float v = (float)raw * 1.25e-3f; // 1.25 mV per bit
  return v;
}

float ina226ReadCurrent_A() {
  uint16_t raw = ina226ReadRegister(INA226_REG_CURRENT);
  int16_t sraw = (int16_t)raw;
  float current = (float)sraw * ina_current_lsb;
  return current;
}

// ---------------- Other helpers ----------------
float readPackVoltage() {
  int adc = analogRead(PIN_VPACK_ADC);
  float vOut = (adc * ADC_REF) / ADC_MAX;
  float vPack = vOut * DIVIDER_RATIO;
  return vPack;
}

float readMaxTemperature() {
  sensors.requestTemperatures();
  float maxT = -1000.0f;
  for (int i=0;i<tempCount;i++){
    float t = sensors.getTempC(tempAddresses[i]);
    if (t > maxT) maxT = t;
  }
  if (tempCount == 0) return -1000.0f;
  return maxT;
}

void handleButtonPress() {
  Serial.println(F("Button pressed"));
  if (state == STATE_FAULT_OVERCURRENT) {
    if (currentFilt < OVERCURRENT_THRESHOLD_A*0.9) {
      Serial.println(F("Manual reset of overcurrent fault"));
      state = STATE_PRECHARGE;
      digitalWrite(PIN_PRECHARGE, HIGH);
      prechargeStart = millis();
      digitalWrite(PIN_BUZZER, LOW);
    } else {
      Serial.println(F("Cannot reset: current still high"));
    }
  } else if (state == STATE_FAULT_OVERTEMP) {
    if (readMaxTemperature() < OVERTEMP_WARN_C) {
      Serial.println(F("Manual reset of overtemp fault"));
      state = STATE_PRECHARGE;
      digitalWrite(PIN_PRECHARGE, HIGH);
      prechargeStart = millis();
      digitalWrite(PIN_BUZZER, LOW);
    } else {
      Serial.println(F("Cannot reset: temperature still high"));
    }
  } else if (state == STATE_OFF) {
    Serial.println(F("Start requested"));
    state = STATE_PRECHARGE;
    digitalWrite(PIN_PRECHARGE, HIGH);
    prechargeStart = millis();
  } else if (state == STATE_RUNNING) {
    Serial.println(F("Manual stop requested"));
    digitalWrite(PIN_CONTACTOR, LOW);
    digitalWrite(PIN_PRECHARGE, LOW);
    state = STATE_OFF;
  }
}

void tripToFault(State f) {
  state = f;
  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_BUZZER, HIGH);
  Serial.print(F("TRIP -> "));
  Serial.println(stateToString(f));
}

const char* stateToString(State s) {
  switch(s) {
    case STATE_OFF: return "OFF";
    case STATE_PRECHARGE: return "PRECHARGE";
    case STATE_RUNNING: return "RUNNING";
    case STATE_FAULT_UNDERVOLT: return "FAULT_UNDERVOLT";
    case STATE_FAULT_OVERCURRENT: return "FAULT_OVERCURRENT";
    case STATE_FAULT_OVERTEMP: return "FAULT_OVERTEMP";
    case STATE_MANUAL_LOCK: return "MANUAL_LOCK";
    default: return "UNKNOWN";
  }
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
