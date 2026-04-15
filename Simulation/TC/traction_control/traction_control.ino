/******************************************************
  Traction Control Arduino Sketch
  - Hall sensors on D2, D3 (interrupt pins)
  - Throttle analog on A0
  - Brake on D7 (active LOW)
  - Kill switch on D8 (active LOW)
  - PWM -> controllers on D5, D6 (use RC filters)
  - Contactor MOSFET gate on D10
  - TC enable on D11 (HIGH=ON)
  - Mode switch on D12 (HIGH=Manual, LOW=Auto)
  - Serial debug at 115200 baud
  - Author: ChatGPT (adapted for Aaditya's EV)
******************************************************/

// -------------------- CONFIG -------------------------
const int PIN_HALL1      = 2;    // INT0
const int PIN_HALL2      = 3;    // INT1
const int PIN_THROTTLE   = A0;   // analog throttle (0..1023)
const int PIN_PWM1       = 5;    // PWM -> controller 1
const int PIN_PWM2       = 6;    // PWM -> controller 2
const int PIN_BRAKE      = 7;    // brake switch (active LOW)
const int PIN_KILL       = 8;    // kill switch (active LOW)
const int PIN_CONTACTOR  = 10;   // contactor MOSFET gate (HIGH = coil energized)
const int PIN_TC_ENABLE  = 11;   // Traction Control ON/OFF (HIGH = ON)
const int PIN_MODE       = 12;   // Throttle Mode (HIGH = Manual, LOW = Auto)
const int PIN_LED        = 13;   // on-board LED as heartbeat / status

// Optional sensor pins (not used by default)
const int PIN_ANA_CURRENT = A1;  // optional ACS758 analog current sensor (if used)
// #define USE_INA226 true    // if using INA226 - requires Wire and INA226 library setup
// #define USE_DS18B20 true   // if using DS18B20 - requires OneWire & DallasTemperature libs

// Hardware / physical constants (tuning)
const float HALLS_PER_REV = 3.0;    // number of hall pulses per motor mechanical revolution (change to your motor)
const unsigned long RPM_WINDOW_MS = 200; // compute RPM every 200 ms
const int PWM_MAX = 255;            // 8-bit PWM range

// Traction control tuning params
const float SLIP_THRESHOLD = 0.15;  // 15% above reference considered slip
const float MAX_REDUCTION = 0.6;    // reduce throttle by up to 60% (i.e. to 40%)
const unsigned long SLIP_DEBOUNCE_MS = 120;   // slip must persist this long to be considered real
const unsigned long RECOVER_MS = 250;         // time to confirm recovery
const float PWM_RAMP_STEP = 0.02;  // step per control cycle (0..1) for ramping outputs

// Safety limits
const float PACK_VOLTAGE_NOMINAL = 36.0; // for reference (not measured unless sensor present)
const int PWM_MIN_SAFE = 0;

// -------------------- GLOBALS ------------------------
volatile unsigned long hallCount1 = 0;  // incremented in ISR
volatile unsigned long hallCount2 = 0;

unsigned long lastRPMcheck = 0;
float rpm1 = 0.0, rpm2 = 0.0;

unsigned long lastHallReadTime = 0;

// TC state
bool slipActive1 = false;
bool slipActive2 = false;
unsigned long slipStart1 = 0, slipStart2 = 0;
unsigned long recoverStart1 = 0, recoverStart2 = 0;

// PWM outputs in fractional (0..1)
float pwmOut1 = 0.0;
float pwmOut2 = 0.0;

// timing
unsigned long lastLoopMicros = 0;
unsigned long loopIntervalMicros = 20000; // target loop 50 Hz => 20 ms

// For auto mode simple profile
unsigned long startMillis = 0;
float autoTarget = 0.8; // default auto target throttle

// -------------------- SETUP --------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_HALL1, INPUT_PULLUP);
  pinMode(PIN_HALL2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL1), hall1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL2), hall2_isr, RISING);

  pinMode(PIN_BRAKE, INPUT_PULLUP);   // active LOW
  pinMode(PIN_KILL, INPUT_PULLUP);    // active LOW
  pinMode(PIN_CONTACTOR, OUTPUT);
  digitalWrite(PIN_CONTACTOR, LOW);   // ensure contactor off at boot
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);
  analogWrite(PIN_PWM1, 0);
  analogWrite(PIN_PWM2, 0);

  pinMode(PIN_TC_ENABLE, INPUT_PULLUP);  // if switch to ground to enable, invert logic; here HIGH=ON expected
  pinMode(PIN_MODE, INPUT_PULLUP);       // HIGH = Manual, LOW = Auto
  pinMode(PIN_LED, OUTPUT);

  lastRPMcheck = millis();
  startMillis = millis();
  lastLoopMicros = micros();

  Serial.println("Traction Control unit starting...");
  Serial.print("RPM window (ms): "); Serial.println(RPM_WINDOW_MS);
  Serial.print("PWM ramp step: "); Serial.println(PWM_RAMP_STEP);
}

// -------------------- ISRs ----------------------------
void hall1_isr() {
  hallCount1++;
}
void hall2_isr() {
  hallCount2++;
}

// -------------------- MAIN LOOP -----------------------
void loop() {
  // pacing loop to approx 50 Hz
  unsigned long nowMic = micros();
  if (nowMic - lastLoopMicros < loopIntervalMicros) {
    // allow background housekeeping
    delay(0);
    return;
  }
  lastLoopMicros = nowMic;

  // read inputs
  bool killPressed = (digitalRead(PIN_KILL) == LOW);   // active low
  bool brakePressed = (digitalRead(PIN_BRAKE) == LOW);
  bool tcEnabled = (digitalRead(PIN_TC_ENABLE) == HIGH);
  bool manualMode = (digitalRead(PIN_MODE) == HIGH);

  // immediate kill: cut outputs and open contactor
  if (killPressed) {
    doKill();
    return; // remain in loop but outputs are off
  }

  // Manage contactor: simple auto-close on start; user can extend logic to precharge
  if (millis() - startMillis > 200 && digitalRead(PIN_CONTACTOR) == LOW) {
    // energize contactor
    digitalWrite(PIN_CONTACTOR, HIGH);
  }

  // read throttle
  int rawThrottle = analogRead(PIN_THROTTLE); // 0..1023
  float throttle = rawThrottle / 1023.0;
  // apply deadband and linearization if wanted
  if (throttle < 0.02) throttle = 0.0;

  // if auto mode: create simple step profile: ramp to autoTarget after 2s
  if (!manualMode) {
    unsigned long t = millis() - startMillis;
    unsigned long stepTime = 2000;
    if (t >= stepTime) throttle = autoTarget;
    else throttle = ( (float)t / (float)stepTime ) * autoTarget;
  }

  // apply brake: simple approach -> reduce throttle
  float brakeVal = brakePressed ? 1.0 : 0.0;
  float thrReq = throttle * (1.0f - brakeVal);

  // compute RPMs periodically
  unsigned long now = millis();
  if (now - lastRPMcheck >= RPM_WINDOW_MS) {
    noInterrupts();
    unsigned long c1 = hallCount1;
    unsigned long c2 = hallCount2;
    hallCount1 = 0;
    hallCount2 = 0;
    interrupts();
    float windowSec = (now - lastRPMcheck) / 1000.0;
    // pulses -> revs
    float revs1 = (c1 / HALLS_PER_REV);
    float revs2 = (c2 / HALLS_PER_REV);
    rpm1 = (windowSec > 0) ? (revs1 / windowSec * 60.0) : rpm1;
    rpm2 = (windowSec > 0) ? (revs2 / windowSec * 60.0) : rpm2;
    lastRPMcheck = now;
  }

  // reference RPM: average or map from throttle; we use average of both if >0 else throttle->rpm mapping
  float rpmRef;
  if (rpm1 + rpm2 > 50.0) {
    rpmRef = (rpm1 + rpm2) / 2.0;
  } else {
    // approximate mapping throttle->rpmRef, needs tuning for your vehicle
    float noload_rpm = 4500.0;
    rpmRef = thrReq * noload_rpm;
  }

  // Traction control logic
  if (tcEnabled) {
    // motor1 slip
    float slip1 = (rpmRef > 1.0) ? (rpm1 - rpmRef) / rpmRef : 0.0;
    if (slip1 > SLIP_THRESHOLD) {
      if (!slipActive1) {
        if (slipStart1 == 0) slipStart1 = millis();
        if (millis() - slipStart1 >= SLIP_DEBOUNCE_MS) slipActive1 = true;
      }
    } else {
      slipStart1 = 0;
      if (slipActive1) {
        if (slip1 < SLIP_THRESHOLD * 0.3) {
          if (recoverStart1 == 0) recoverStart1 = millis();
          if (millis() - recoverStart1 >= RECOVER_MS) { slipActive1 = false; recoverStart1 = 0; }
        } else {
          recoverStart1 = 0;
        }
      }
    }
    // motor2 slip
    float slip2 = (rpmRef > 1.0) ? (rpm2 - rpmRef) / rpmRef : 0.0;
    if (slip2 > SLIP_THRESHOLD) {
      if (!slipActive2) {
        if (slipStart2 == 0) slipStart2 = millis();
        if (millis() - slipStart2 >= SLIP_DEBOUNCE_MS) slipActive2 = true;
      }
    } else {
      slipStart2 = 0;
      if (slipActive2) {
        if (slip2 < SLIP_THRESHOLD * 0.3) {
          if (recoverStart2 == 0) recoverStart2 = millis();
          if (millis() - recoverStart2 >= RECOVER_MS) { slipActive2 = false; recoverStart2 = 0; }
        } else {
          recoverStart2 = 0;
        }
      }
    }
  } else {
    // if TC disabled, clear all flags
    slipActive1 = slipActive2 = false;
    slipStart1 = slipStart2 = recoverStart1 = recoverStart2 = 0;
  }

  // compute per-motor targets
  float target1 = thrReq;
  float target2 = thrReq;
  if (slipActive1) target1 = thrReq * (1.0 - MAX_REDUCTION);
  if (slipActive2) target2 = thrReq * (1.0 - MAX_REDUCTION);

  // smooth ramping: move pwmOut toward target with step
  pwmOut1 = rampToward(pwmOut1, target1, PWM_RAMP_STEP);
  pwmOut2 = rampToward(pwmOut2, target2, PWM_RAMP_STEP);

  // Convert to PWM values 0..255
  int pwmVal1 = (int)(pwmOut1 * PWM_MAX + 0.5);
  int pwmVal2 = (int)(pwmOut2 * PWM_MAX + 0.5);
  pwmVal1 = constrain(pwmVal1, PWM_MIN_SAFE, PWM_MAX);
  pwmVal2 = constrain(pwmVal2, PWM_MIN_SAFE, PWM_MAX);

  analogWrite(PIN_PWM1, pwmVal1);
  analogWrite(PIN_PWM2, pwmVal2);

  // Heartbeat LED blink
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 300) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    lastBlink = millis();
  }

  // Serial telemetry (periodic)
  static unsigned long lastSerial = 0;
  if (millis() - lastSerial > 250) {
    Serial.print("T=");
    Serial.print(thrReq,3);
    Serial.print(" |rpm1=");
    Serial.print(rpm1,1);
    Serial.print(" rpm2=");
    Serial.print(rpm2,1);
    Serial.print(" |pwm1=");
    Serial.print(pwmVal1);
    Serial.print(" pwm2=");
    Serial.print(pwmVal2);
    Serial.print(" |slip1=");
    Serial.print(slipActive1);
    Serial.print(" slip2=");
    Serial.print(slipActive2);
    Serial.println();
    lastSerial = millis();
  }

  // small safety: if brake is pressed -> force pwm outputs to zero immediately
  if (brakePressed) {
    pwmOut1 = pwmOut2 = 0.0;
    analogWrite(PIN_PWM1, 0);
    analogWrite(PIN_PWM2, 0);
  }
}

// -------------------- helper functions --------------------
float rampToward(float current, float target, float step) {
  if (abs(target - current) <= step) return target;
  if (target > current) return current + step;
  else return current - step;
}

void doKill() {
  // immediate shutdown
  analogWrite(PIN_PWM1, 0);
  analogWrite(PIN_PWM2, 0);
  digitalWrite(PIN_CONTACTOR, LOW); // open contactor
  pwmOut1 = pwmOut2 = 0.0;
  Serial.println("KILL SWITCH PRESSED - SYSTEM OFF");
  // block here until kill is released to prevent accidental restart
  while (digitalRead(PIN_KILL) == LOW) {
    delay(50);
  }
  Serial.println("Kill released - manual restart required.");
}

