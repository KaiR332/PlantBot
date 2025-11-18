#include <Wire.h>
#include <math.h>
#include "Adafruit_VEML7700.h"
#include <Adafruit_INA219.h>

#define LNR_UP    10   // RPWM
#define LNR_DOWN   9   // LPWM

#define STP1_STEP  2   // Base (lazy susan)
#define STP1_DIR   3
#define STP1_EN    4

#define STP2_STEP  5   // Vertical arm rotation
#define STP2_DIR   6
#define STP2_EN    7

#define PCA_ADDR   0x70

// Multiplexer channels for each sensor
const uint8_t LIGHT_CHANNELS[4] = {0, 1, 2, 3};  // N, E, S, W
const uint8_t INA_CHANNEL = 4;

Adafruit_VEML7700 veml[4];
Adafruit_INA219 ina219;

// Control parameters
const float LIGHT_TOLERANCE = 25.0;    // lux difference tolerance for alignment
const int BASE_STEP_DELAY_US = 700;
const int BASE_STEP_SIZE = 10;         // steps per correction tick
const int ARM_STEP_DELAY_US = 800;
const int ARM_STEP_SIZE = 8;
const unsigned long EXTEND_TIME_MS = 2000;
const unsigned long RETRACT_TIME_MS = 2000;

bool armExtended = false;

void setup() {
  Serial.begin(115200);

  pinMode(LNR_UP, OUTPUT);
  pinMode(LNR_DOWN, OUTPUT);
  digitalWrite(LNR_UP, LOW);
  digitalWrite(LNR_DOWN, LOW);

  pinMode(STP1_STEP, OUTPUT);
  pinMode(STP1_DIR, OUTPUT);
  pinMode(STP1_EN, OUTPUT);
  digitalWrite(STP1_EN, HIGH);

  pinMode(STP2_STEP, OUTPUT);
  pinMode(STP2_DIR, OUTPUT);
  pinMode(STP2_EN, OUTPUT);
  digitalWrite(STP2_EN, HIGH);

  Wire.begin();
  initMultiplexedSensors();
}

void initMultiplexedSensors() {
  for (int i = 0; i < 4; i++) {
    selectMuxChannel(LIGHT_CHANNELS[i]);
    if (!veml[i].begin()) {
      Serial.print("VEML init failed on channel ");
      Serial.println(LIGHT_CHANNELS[i]);
    } else {
      veml[i].setGain(VEML7700_GAIN_1);
      veml[i].setIntegrationTime(VEML7700_IT_200MS);
    }
  }

  selectMuxChannel(INA_CHANNEL);
  if (!ina219.begin()) {
    Serial.println("INA219 init failed");
  }
}

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(2);
}

// Linear actuator up (milliseconds)
void LNRmoveUp(unsigned long ms) {
  analogWrite(LNR_UP, 255);   // full speed up
  analogWrite(LNR_DOWN, 0);
  delay(ms);
  LNRstop();
}

// Linear actuator down (milliseconds)
void LNRmoveDown(unsigned long ms) {
  analogWrite(LNR_UP, 0);
  analogWrite(LNR_DOWN, 255); // full speed down
  delay(ms); 
  LNRstop();
}

// Stop linear actuator
void LNRstop() {
  analogWrite(LNR_UP, 0);
  analogWrite(LNR_DOWN, 0);
  delay(1000);
}

void enableStepper(uint8_t pin) {
  digitalWrite(pin, LOW);   // LOW = enabled on most TMC2209 boards
}

void disableStepper(uint8_t pin) {
  digitalWrite(pin, HIGH);  // HIGH = disable
}

void stepperMove(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, bool direction, long steps, int delayUs) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
  enableStepper(enPin);

  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
  }

  disableStepper(enPin);
}

void rotateBase(bool clockwise, long steps) {
  stepperMove(STP1_STEP, STP1_DIR, STP1_EN, clockwise, steps, BASE_STEP_DELAY_US);
}

void tiltArm(bool up, long steps) {
  stepperMove(STP2_STEP, STP2_DIR, STP2_EN, up, steps, ARM_STEP_DELAY_US);
}

void readLightSensors(float readings[4]) {
  for (int i = 0; i < 4; i++) {
    selectMuxChannel(LIGHT_CHANNELS[i]);
    readings[i] = veml[i].readLux();
  }
}

void logSolarTelemetry() {
  selectMuxChannel(INA_CHANNEL);
  float voltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  Serial.print("Panel V: ");
  Serial.print(voltage);
  Serial.print(" V, I: ");
  Serial.print(current);
  Serial.println(" mA");
}

bool alignWithLight(const float readings[4]) {
  float north = readings[0];
  float east = readings[1];
  float south = readings[2];
  float west = readings[3];

  float verticalError = north - south;   // positive means tilt up
  float horizontalError = east - west;   // positive means rotate east

  bool verticalAligned = fabs(verticalError) < LIGHT_TOLERANCE;
  bool horizontalAligned = fabs(horizontalError) < LIGHT_TOLERANCE;

  if (!horizontalAligned) {
    rotateBase(horizontalError < 0, BASE_STEP_SIZE);
  }

  if (!verticalAligned) {
    tiltArm(verticalError < 0, ARM_STEP_SIZE);
  }

  return verticalAligned && horizontalAligned;
}

void loop() {
  float lightValues[4];
  readLightSensors(lightValues);

  Serial.print("Light sensors N/E/S/W: ");
  Serial.print(lightValues[0]); Serial.print(", ");
  Serial.print(lightValues[1]); Serial.print(", ");
  Serial.print(lightValues[2]); Serial.print(", ");
  Serial.println(lightValues[3]);

  bool aligned = alignWithLight(lightValues);

  if (aligned && !armExtended) {
    LNRmoveUp(EXTEND_TIME_MS);
    armExtended = true;
  } else if (!aligned && armExtended) {
    LNRmoveDown(RETRACT_TIME_MS);
    armExtended = false;
  }

  logSolarTelemetry();
  delay(500);
}
