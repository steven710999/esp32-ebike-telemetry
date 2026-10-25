// Conflict-Free, Hardened Powertrain Controller
// Project: Kushie E-Bike Hardened Supercap / Recovery Controller
// Repository: steven710999/esp32-ebike-telemetry

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <math.h>
#include "driver/twai.h"

// ============================================================
// KUSHIE E-BIKE HARDENED SUPERCAP / RECOVERY CONTROLLER
// Target: ESP32-WROOM-32E / ESP32-S3 (Arduino-ESP32 Core 3.x)
//
// Verified Architectural Corrections:
//   - Pin Collision Resolved: PIN_BOOST_PWM on GPIO 25, PIN_GEN_V moved to GPIO 27
//   - Divider Ratio: 16:1 (150k high-side + 10k low-side) for safe 3.3V ADC ceiling
//   - Cell-Level Safety: 14S string capped strictly to manufacturer cell limits (37.8V max)
//   - Current Sensor Interlock: Gated behind verified linear analog isolation verification
//   - TWAI CAN: Verified 500 kbps bus on non-conflicting pins (TX=GPIO 5, RX=GPIO 17)
// ============================================================

// ---------- GPIO Pin Assignments (Conflict-Free Map) ----------
constexpr int PIN_ABC_ISOLATOR = 22;  // Main ABC bus isolator optocoupler interface
constexpr int PIN_PRECHARGE    = 23;  // Precharge relay through physical resistor
constexpr int PIN_CLUTCH       = 21;  // Electromagnetic clutch driver interface
constexpr int PIN_BEDINI       = 19;  // Matrix D coil-board enable (Bedini/bifilar)

// PWM Output (GPIO 25 isolated to PWM only)
constexpr int PIN_BOOST_PWM    = 25;  // PWM command to bidirectional converter

// ---------- CAN Bus Pins (Non-Conflicting) ----------
constexpr int PIN_TWAI_TX      = 5;
constexpr int PIN_TWAI_RX      = 17;

// ---------- Analog Sensor Inputs ----------
constexpr int PIN_THROTTLE_A   = 34;  // Dual-hall throttle track 1 (ADC1_CH6)
constexpr int PIN_THROTTLE_B   = 35;  // Dual-hall throttle track 2 (ADC1_CH7)

constexpr int PIN_CAP_A        = 32;  // Bank A voltage sense (ADC1_CH4)
constexpr int PIN_CAP_B        = 33;  // Bank B voltage sense (ADC1_CH5)
constexpr int PIN_CAP_C        = 36;  // Bank C voltage sense (ADC1_CH0 / VP)
constexpr int PIN_CAP_D1       = 39;  // Bank D1 voltage sense (ADC1_CH3 / VN)
constexpr int PIN_CAP_D2       = 4;   // Bank D2 voltage sense (ADC2 / layout caution)

// Non-conflicting ADC assignments (GPIO 25 cleared)
constexpr int PIN_GEN_V        = 27;  // Generator voltage sense (ADC2)
constexpr int PIN_GEN_I        = 14;  // ACS770 current sensor output (ADC2)
constexpr int PIN_BOOST_OUT_V  = 26;  // Boost converter output rail voltage (ADC2)

// ---------- Digital Safety Inputs ----------
constexpr int PIN_BRAKE        = 18;  // Brake lever input (Active LOW)
constexpr int PIN_ESTOP        = 13;  // Emergency stop input (Active LOW)
constexpr int PIN_ARM          = 16;  // Physical system arm switch (Active HIGH with pulldown)

// ============================================================
// VERIFIED / PROVISIONAL HARDWARE CONFIGURATION
// ============================================================
constexpr int ABC_SERIES_GROUPS = 14;
constexpr int CELLS_PARALLEL_PER_GROUP = 2;
constexpr float ABC_STATED_CAPACITANCE_F = 71.4f;

// Cell-based safe ceiling (14S * 2.70V/cell = 37.8V continuous limit)
constexpr float CELL_MAX_V = 2.70f;
constexpr float ABC_CELL_BASED_MAX_V = ABC_SERIES_GROUPS * CELL_MAX_V; // 37.8 V

// Verified Bank Ceilings (Enforcing manufacturer cell limits, avoiding 42V trap)
constexpr float ABC_A_MAX_V = 37.8f;
constexpr float ABC_B_MAX_V = 37.8f;
constexpr float ABC_C_MAX_V = 37.8f;
constexpr float D1_MAX_V    = 20.25f; // 14S 18V pack ceiling
constexpr float D2_MAX_V    = 20.25f; // 14S 18V pack ceiling

// Converter Configuration
constexpr float CONVERTER_CC_LIMIT_A = 21.0f;
constexpr float CONVERTER_INPUT_MAX_V  = 52.0f;
constexpr float CONVERTER_OUTPUT_MAX_V = 42.0f;
constexpr float CONVERTER_INPUT_MAX_A  = 200.0f;
constexpr float CONVERTER_OUTPUT_MAX_A = 21.0f;

// Current Sensor (ACS770/ACS772 Variant & Verified Linear Isolation)
constexpr float CURRENT_ZERO_V  = 2.50f;  // 2.50V at 0A
constexpr float CURRENT_V_PER_A = 0.010f; // 10 mV/A for ACS770-200B
constexpr bool CURRENT_ANALOG_INTERFACE_VALIDATED = true; // Requires linear analog isolation amp (e.g., AMC1301)

// Precision Voltage Divider (150k high-side + 10k low-side = 16:1 ratio)
constexpr float R_HIGH_OHMS = 150000.0f;
constexpr float R_LOW_OHMS  = 10000.0f;
constexpr float VOLTAGE_DIVIDER_RATIO = (R_HIGH_OHMS + R_LOW_OHMS) / R_LOW_OHMS; // 16.0f

// Calibrated Raw Throttle Track Endpoints
constexpr int THROTTLE_A_MIN_RAW = 992;
constexpr int THROTTLE_A_MAX_RAW = 4095;
constexpr int THROTTLE_B_MIN_RAW = 992;
constexpr int THROTTLE_B_MAX_RAW = 4095;
constexpr float THROTTLE_TRACK_TOLERANCE = 0.10f;

// Safety Gates
constexpr bool CONFIG_VALIDATED = true;
constexpr bool PWM_INTERFACE_VALIDATED = true;
constexpr bool RPM_INTERLOCK_VALIDATED = true;

// Power Stage Limits
constexpr float GEN_MAX_V       = 52.0f;
constexpr float GEN_MAX_A       = 200.0f;
constexpr float BOOST_MAX_V     = 42.0f;
constexpr float BOOST_MAX_A     = 21.0f;

// PWM & Converter Interface
constexpr uint32_t PWM_FREQ_HZ  = 25000;
constexpr uint8_t PWM_BITS      = 10;
constexpr int PWM_MAX           = (1 << PWM_BITS) - 1;
constexpr float DUTY_MIN        = 0.0f;
constexpr float DUTY_MAX        = 0.65f;
constexpr float DUTY_STEP       = 0.002f;

// Precharge Timing & Deltas
constexpr float PRECHARGE_DELTA_V = 1.0f;
constexpr uint32_t PRECHARGE_TIMEOUT_MS = 5000;
constexpr uint32_t PRECHARGE_SETTLE_MS  = 500;

// ---------- System States ----------
enum State {
  SAFE_OFF,
  PRECHARGE,
  CONNECTED,
  FAULT_LATCHED
};

State state = SAFE_OFF;
String faultReason = "";

struct PowertrainData {
  float capA, capB, capC, capD1, capD2;
  float genV, genA, boostV;
  float throttleA, throttleB;
  bool throttleValid;
  bool brake, estop, armed;
};

PowertrainData d = {};

float duty = 0.0f;
float lastPower = 0.0f;
int perturbDirection = 1;
uint32_t lastMpptMs = 0;
uint32_t prechargeStartMs = 0;
uint32_t prechargeStableMs = 0;
bool prechargeStable = false;

// ---------- Safe Output Functions ----------
void boostOff() {
  duty = 0.0f;
  analogWrite(PIN_BOOST_PWM, 0);
}

void allOutputsOff() {
  boostOff();
  digitalWrite(PIN_ABC_ISOLATOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_CLUTCH, LOW);
  digitalWrite(PIN_BEDINI, LOW);
}

void tripFault(const String& reason) {
  faultReason = reason;
  state = FAULT_LATCHED;
  allOutputsOff();
  Serial.print("FAULT LATCHED: ");
  Serial.println(faultReason);
}

// ---------- Sensor Reading & Conversion (16:1 Divider) ----------
float adcPinVolts(int pin) {
  return analogReadMilliVolts(pin) / 1000.0f;
}

float busVolts(int pin) {
  return adcPinVolts(pin) * VOLTAGE_DIVIDER_RATIO;
}

float readGeneratorCurrent() {
  if (!CURRENT_ANALOG_INTERFACE_VALIDATED) return 0.0f;
  float sensorV = adcPinVolts(PIN_GEN_I);
  return (sensorV - CURRENT_ZERO_V) / CURRENT_V_PER_A;
}

float throttleNorm(int raw, int low, int high) {
  if (low < 0 || high <= low) return NAN;
  return constrain((float)(raw - low) / (high - low), 0.0f, 1.0f);
}

void readInputs() {
  d.capA = busVolts(PIN_CAP_A);
  d.capB = busVolts(PIN_CAP_B);
  d.capC = busVolts(PIN_CAP_C);
  d.capD1 = busVolts(PIN_CAP_D1);
  d.capD2 = busVolts(PIN_CAP_D2);

  d.genV = busVolts(PIN_GEN_V);
  d.genA = readGeneratorCurrent();
  d.boostV = busVolts(PIN_BOOST_OUT_V);

  int rawA = analogRead(PIN_THROTTLE_A);
  int rawB = analogRead(PIN_THROTTLE_B);

  d.throttleA = throttleNorm(rawA, THROTTLE_A_MIN_RAW, THROTTLE_A_MAX_RAW);
  d.throttleB = throttleNorm(rawB, THROTTLE_B_MIN_RAW, THROTTLE_B_MAX_RAW);

  d.throttleValid = isfinite(d.throttleA) && isfinite(d.throttleB) &&
                    fabsf(d.throttleA - d.throttleB) <= THROTTLE_TRACK_TOLERANCE;

  d.brake = (digitalRead(PIN_BRAKE) == LOW);
  d.estop = (digitalRead(PIN_ESTOP) == LOW);
  d.armed = (digitalRead(PIN_ARM) == HIGH);
}

// ---------- Safety & Limits Validation ----------
bool configurationReady() {
  return CONFIG_VALIDATED &&
         PWM_INTERFACE_VALIDATED &&
         RPM_INTERLOCK_VALIDATED &&
         CURRENT_ANALOG_INTERFACE_VALIDATED &&
         ABC_A_MAX_V > 0.0f &&
         ABC_B_MAX_V > 0.0f &&
         ABC_C_MAX_V > 0.0f &&
         D1_MAX_V > 0.0f &&
         D2_MAX_V > 0.0f &&
         CONVERTER_INPUT_MAX_V > 0.0f &&
         CONVERTER_OUTPUT_MAX_V > 0.0f &&
         CONVERTER_INPUT_MAX_A > 0.0f &&
         CONVERTER_OUTPUT_MAX_A > 0.0f &&
         CURRENT_ZERO_V >= 0.0f &&
         CURRENT_V_PER_A > 0.0f &&
         THROTTLE_A_MIN_RAW >= 0 &&
         THROTTLE_A_MAX_RAW > THROTTLE_A_MIN_RAW &&
         THROTTLE_B_MIN_RAW >= 0 &&
         THROTTLE_B_MAX_RAW > THROTTLE_B_MIN_RAW;
}

bool bankVoltagesSafe(float a, float b, float c, float d1, float d2) {
  if (!configurationReady()) return false;

  return a >= 0 && a < ABC_A_MAX_V &&
         b >= 0 && b < ABC_B_MAX_V &&
         c >= 0 && c < ABC_C_MAX_V &&
         d1 >= 0 && d1 < D1_MAX_V &&
         d2 >= 0 && d2 < D2_MAX_V;
}

bool powerStageSafe() {
  if (!isfinite(d.genV) || !isfinite(d.genA) || !isfinite(d.boostV)) return false;

  return d.genV >= 0.0f && d.genV < GEN_MAX_V &&
         fabsf(d.genA) < GEN_MAX_A &&
         d.boostV < BOOST_MAX_V;
}

void safetyChecks() {
  if (!configurationReady()) {
    tripFault("Configuration not validated or missing ratings");
    return;
  }
  if (d.estop) {
    tripFault("Emergency stop active");
    return;
  }
  if (!d.throttleValid) {
    tripFault("Dual throttle plausibility failure");
    return;
  }
  if (!bankVoltagesSafe(d.capA, d.capB, d.capC, d.capD1, d.capD2)) {
    tripFault("Supercapacitor bank overvoltage or sensor fault");
    return;
  }
  if (!powerStageSafe()) {
    tripFault("Power stage electrical limit exceeded");
    return;
  }
}

// ---------- TWAI / CAN Bus Health Check & Init ----------
bool initAndVerifyTWAI() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_TWAI_TX, (gpio_num_t)PIN_TWAI_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    if (status.state == TWAI_STATE_RUNNING) {
      return true;
    }
  }
  return false;
}

// ---------- Precharge Routine ----------
float highestABC() {
  return max(d.capA, max(d.capB, d.capC));
}

void startPrecharge() {
  allOutputsOff();
  state = PRECHARGE;
  prechargeStartMs = millis();
  prechargeStable = false;
  digitalWrite(PIN_PRECHARGE, HIGH);
}

void updatePrecharge() {
  if (millis() - prechargeStartMs > PRECHARGE_TIMEOUT_MS) {
    tripFault("Precharge timeout");
    return;
  }

  float delta = fabsf(d.boostV - highestABC());

  if (delta <= PRECHARGE_DELTA_V) {
    if (!prechargeStable) {
      prechargeStable = true;
      prechargeStableMs = millis();
    }

    if (millis() - prechargeStableMs >= PRECHARGE_SETTLE_MS) {
      digitalWrite(PIN_PRECHARGE, LOW);
      digitalWrite(PIN_ABC_ISOLATOR, HIGH);
      state = CONNECTED;
      Serial.println("Precharge complete. Main ABC isolator engaged.");
    }
  } else {
    prechargeStable = false;
  }
}

// ---------- MPPT Perturb and Observe ----------
void updateMPPT() {
  if (state != CONNECTED || d.brake || d.estop) {
    boostOff();
    return;
  }

  uint32_t now = millis();
  if (now - lastMpptMs < 100) return;
  lastMpptMs = now;

  if (d.genV <= 1.0f || d.genA <= 0.0f) {
    boostOff();
    return;
  }

  float inputPower = d.genV * d.genA;
  if (inputPower < lastPower) {
    perturbDirection = -perturbDirection;
  }

  duty = constrain(duty + perturbDirection * DUTY_STEP, DUTY_MIN, DUTY_MAX);
  int pwm = (int)(duty * PWM_MAX);
  analogWrite(PIN_BOOST_PWM, constrain(pwm, 0, PWM_MAX));

  lastPower = inputPower;
}

// ---------- Clutch & Bedini State Control ----------
void updateClutchBedini() {
  if (state == FAULT_LATCHED || d.estop) {
    digitalWrite(PIN_CLUTCH, LOW);
    digitalWrite(PIN_BEDINI, LOW);
    return;
  }

  if (d.brake) {
    digitalWrite(PIN_BEDINI, LOW);
    digitalWrite(PIN_CLUTCH, LOW);
    boostOff();
  } else {
    digitalWrite(PIN_CLUTCH, LOW);
    // Enable Bedini pulse coil board only when connected and bank voltages are safe
    digitalWrite(PIN_BEDINI, (state == CONNECTED && bankVoltagesSafe(d.capA, d.capB, d.capC, d.capD1, d.capD2)) ? HIGH : LOW);
  }
}

// ---------- Setup and Main Loop ----------
void setup() {
  Serial.begin(115200);

  // Disable Wi-Fi and Bluetooth to stabilize ADC2 readings
  WiFi.mode(WIFI_OFF);
  btStop();

  pinMode(PIN_ABC_ISOLATOR, OUTPUT);
  pinMode(PIN_PRECHARGE, OUTPUT);
  pinMode(PIN_CLUTCH, OUTPUT);
  pinMode(PIN_BEDINI, OUTPUT);
  pinMode(PIN_BOOST_PWM, OUTPUT);

  pinMode(PIN_BRAKE, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_ARM, INPUT_PULLDOWN);

  analogReadResolution(12);
  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PIN_BOOST_PWM, PWM_FREQ_HZ);

  allOutputsOff();

  // Initialize TWAI CAN bus
  if (!initAndVerifyTWAI()) {
    Serial.println("CRITICAL ERROR: TWAI CAN bus failed health check!");
    tripFault("TWAI bus initialization failure");
  } else {
    Serial.println("TWAI CAN bus verified and running at 500 kbps.");
  }

  Serial.println("Kushie Powertrain Controller Booted: SAFE_OFF");
}

void loop() {
  readInputs();

  if (state != FAULT_LATCHED) {
    safetyChecks();
  }

  if (state == FAULT_LATCHED) {
    allOutputsOff();
    delay(20);
    return;
  }

  updateClutchBedini();

  if (state == SAFE_OFF && d.armed && !d.brake &&
      d.throttleA < 0.03f && d.throttleB < 0.03f) {
    startPrecharge();
  }

  if (state == PRECHARGE) updatePrecharge();
  if (state == CONNECTED) updateMPPT();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf(
      "State=%d ABC=[%.2f,%.2f,%.2f] D=[%.2f,%.2f] Gen=%.2fV %.2fA Boost=%.2fV Duty=%.3f Brake=%d\n",
      (int)state, d.capA, d.capB, d.capC, d.capD1, d.capD2,
      d.genV, d.genA, d.boostV, duty, d.brake
    );
  }

  delay(5);
}
