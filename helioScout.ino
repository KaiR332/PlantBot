#include <Wire.h>
#include "Adafruit_VEML7700.h"
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Adafruit_VEML7700 veml;

#define LNR_IN3 11
#define LNR_IN4 10

#define STP1_STEP  2
#define STP1_DIR   3
#define STP1_EN    4

#define STP2_STEP  5
#define STP2_DIR   6
#define STP2_EN    7

#define PCA_ADDR   0x70
#define INA_ADDR   0x40

// Configuration constants
#define BALANCE_TOL 0.05 // tolerance
#define BALANCE_TOL 0.10 // extended tolerance
#define MIN_STEPS 50 // Minimum movement steps
#define MAX_STEPS 500 // Maximum movement steps
#define BASE_SPEED 2000 // Microseconds per step (slower = more torque)
#define ARM_SPEED 800

#define LNR_EXTEND_TIME 10000   // ms
#define LNR_RETRACT_TIME 10000  // ms
#define LNR_SPEED 255

// Sensor port mapping
enum SensorPort { NORTH = 2, EAST = 3, SOUTH = 1, WEST = 0 };
uint8_t ports[4] = {NORTH, EAST, SOUTH, WEST};

// Actuator states
enum ActuatorState { FULLY_RETRACTED, EXTENDING, FULLY_EXTENDED, RETRACTING };
ActuatorState actuatorState = FULLY_RETRACTED;
unsigned long actuatorStartTime = 0;

// System state
bool systemBalanced = false;

void setup() {
  Serial.begin(115200);
  
  // Linear Actuator
  pinMode(LNR_IN3, OUTPUT);
  pinMode(LNR_IN4, OUTPUT);
  LNRstop();

  // Stepper motor 1 (base)
  pinMode(STP1_STEP, OUTPUT);
  pinMode(STP1_DIR, OUTPUT);
  pinMode(STP1_EN, OUTPUT);
  digitalWrite(STP1_EN, HIGH);

  // Stepper motor 2 (arm)
  pinMode(STP2_STEP, OUTPUT);
  pinMode(STP2_DIR, OUTPUT);
  pinMode(STP2_EN, OUTPUT);
  digitalWrite(STP2_EN, HIGH);

  // Initialize VEML7700
  Wire.begin();
  for (int p = 0; p < 4; p++) {
    selectPCAChannel(p);
    delay(5);
    if (!veml.begin()) {
      Serial.print("VEML INIT FAIL on port ");
      Serial.println(p);
    } else {
      Serial.print("VEML INIT OK on port ");
      Serial.println(p);
    }
    delay(5);
    disablePCA();
  }

  // Initialize INA219
  selectPCAChannel(7);
  delay(5);
  ina219.begin();
  disablePCA();

  Serial.println("System initialized. Starting in RETRACTED state.");
}

//=====================================================================
// VEML7700

float readLux(uint8_t port) {
  selectPCAChannel(port);
  delay(4);
  float lux = veml.readLux();
  disablePCA();
  delayMicroseconds(200);
  return lux;
}

void readAllSensors(float *values) {
  for (uint8_t i = 0; i < 4; i++) {
    values[i] = readLux(ports[i]);
  }
}

//=====================================================================
// BALANCE CALCULATION (ratio for scale-invariance)

float balanceRatio(float positive, float negative) {
  // Returns error from -1.0 to +1.0
  // sign determines direction
  
  float minLux = 1.0;  // don't divide by zero
  if (positive < minLux) positive = minLux;
  if (negative < minLux) negative = minLux;
  
  float ratio = positive / negative;
  
  if (ratio > 1.0) {
    // Normalize: ratio of 2.0 → error ~0.5
    float error = (ratio - 1.0) / (ratio + 1.0);
    return error;
  } else {
    // Normalize: ratio of 0.5 → error ~-0.5
    float error = (ratio - 1.0) / (ratio + 1.0);
    return error;
  }
}

bool isBalanced(float error, float threshold) {
  return abs(error) < threshold;
}

//=====================================================================
// MOTOR CONTROL (dynamic steps)

void moveProportionalEW(float error) {
  // Map error magnitude to step count
  int steps = map(abs(error) * 1000, 0, 1000, MIN_STEPS, MAX_STEPS);
  steps = constrain(steps, MIN_STEPS, MAX_STEPS);
  
  Serial.print("EW Error: "); Serial.print(error, 4);
  Serial.print(" | Steps: "); Serial.println(steps);
  
  if (error > 0) {
    STP1East(steps, BASE_SPEED);
  } else {
    STP1West(steps, BASE_SPEED);
  }
}

void moveProportionalNS(float error) {
  int steps = map(abs(error) * 1000, 0, 1000, MIN_STEPS, MAX_STEPS);
  steps = constrain(steps, MIN_STEPS, MAX_STEPS);
  
  Serial.print("NS Error: "); Serial.print(error, 4);
  Serial.print(" | Steps: "); Serial.println(steps);
  
  if (error > 0) {
    STP2North(steps, ARM_SPEED);
  } else {
    STP2South(steps, ARM_SPEED);
  }
}

//=====================================================================
// BALANCE CHECK

bool checkAndBalance() {
  float values[4];
  readAllSensors(values);
  
  Serial.println("\n--- Sensor Readings ---");
  Serial.print("N: "); Serial.print(values[0]);
  Serial.print(" | E: "); Serial.print(values[1]);
  Serial.print(" | S: "); Serial.print(values[2]);
  Serial.print(" | W: "); Serial.println(values[3]);
  
  // Calculate errors for both axes
  float ewError = balanceRatio(values[1], values[3]); // East - West
  float nsError = balanceRatio(values[0], values[2]); // North - South
  
  bool ewBalanced = isBalanced(ewError, BALANCE_TOL);
  bool nsBalanced = isBalanced(nsError, BALANCE_TOL);
  
  // Move if not balanced
  if (!ewBalanced) {
    moveProportionalEW(ewError);
  }
  
  if (!nsBalanced) {
    moveProportionalNS(nsError);
  }
  
  bool fullyBalanced = ewBalanced && nsBalanced;
  
  if (fullyBalanced) {
    Serial.println("✓ SYSTEM BALANCED");
  }
  
  return fullyBalanced;
}

//=====================================================================
// ACTUATOR FUNCTIONS

void LNRstop() {
  analogWrite(LNR_IN3, 0);
  analogWrite(LNR_IN4, 0);
}

void startExtending() {
  Serial.println(">>> Starting EXTENSION");
  
  analogWrite(LNR_IN3, LNR_SPEED);
  analogWrite(LNR_IN4, 0);
  actuatorState = EXTENDING;
  actuatorStartTime = millis();
}

void startRetracting() {
  Serial.println("<<< Starting RETRACTION");
  analogWrite(LNR_IN3, 0);
  analogWrite(LNR_IN4, LNR_SPEED);
  actuatorState = RETRACTING;
  actuatorStartTime = millis();
}

void updateActuatorState() {
  unsigned long elapsed = millis() - actuatorStartTime;
  
  if (actuatorState == EXTENDING) {
    if (elapsed >= LNR_EXTEND_TIME) {
      LNRstop();
      actuatorState = FULLY_EXTENDED;
      Serial.println(">>> FULLY EXTENDED");
    }
  } 
  else if (actuatorState == RETRACTING) {
    if (elapsed >= LNR_RETRACT_TIME) {
      LNRstop();
      actuatorState = FULLY_RETRACTED;
      Serial.println("<<< FULLY RETRACTED");
    }
  }
}

//=====================================================================
// STEPPER MOTOR FUNCTIONS

void STP1Enable() {
  digitalWrite(STP1_EN, LOW);
}

void STP1Disable() {
  digitalWrite(STP1_EN, HIGH);
}

void STP2Enable() {
  digitalWrite(STP2_EN, LOW);
}

void STP2Disable() {
  digitalWrite(STP2_EN, HIGH);
}

void STP1West(long steps, int speed_delay_us) {
  digitalWrite(STP1_DIR, HIGH); 
  STP1Enable();
  for (long i = 0; i < steps; i++) {
    digitalWrite(STP1_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP1_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }
  STP1Disable();
}

void STP1East(long steps, int speed_delay_us) {
  digitalWrite(STP1_DIR, LOW); 
  STP1Enable();
  for (long i = 0; i < steps; i++) {
    digitalWrite(STP1_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP1_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }
  STP1Disable();
}

void STP2South(long steps, int speed_delay_us) {
  digitalWrite(STP2_DIR, LOW); 
  STP2Enable();
  for (long i = 0; i < steps; i++) {
    digitalWrite(STP2_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP2_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }
  STP2Hold();
}

void STP2North(long steps, int speed_delay_us) {
  digitalWrite(STP2_DIR, HIGH); 
  STP2Enable();
  for (long i = 0; i < steps; i++) {
    digitalWrite(STP2_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP2_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }
  STP2Hold();
}

void STP2Hold() {
  digitalWrite(STP2_EN, LOW);
  digitalWrite(STP2_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(STP2_STEP, LOW);
  delayMicroseconds(5);
  digitalWrite(STP2_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(STP2_STEP, LOW);
}

// ===== I2C MULTIPLEXER =====

void selectPCAChannel(uint8_t channel) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void disablePCA() {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

//=====================================================================
// SOLAR TRACKING (needs work)

float readINAVmW() {
  selectPCAChannel(7);
  delay(4);
  
  Wire.beginTransmission(INA_ADDR);
  uint8_t err = Wire.endTransmission();
  
  if (err != 0) {
    disablePCA();
    delayMicroseconds(200);
    return -1;
  }
  
  // Option 1: Use library's built-in function (recommended)
  // float power = ina219.getPower_mW();
  
  // Option 2: Manual calculation (if library function doesn't work)
  float busV = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float power = busV * current / 1000.0;  // Convert V*mA to mW
  
  disablePCA();
  delayMicroseconds(200);
  
  return power;
}

//=====================================================================
// LOOP

void loop() {
  // Update actuator state if moving
  updateActuatorState();
  
  // State machine logic
  switch (actuatorState) {
    case FULLY_RETRACTED:
      // Try to balance system
      systemBalanced = checkAndBalance();
      
      if (systemBalanced) {
        // Start extending once balanced
        startExtending();
      }
      delay(500);  // Small delay between balance attempts
      break;
      
    case EXTENDING:
      // Wait for extension to complete
      delay(100);
      break;
      
    case FULLY_EXTENDED:
      // Check if we've lost balance
      {
        float values[4];
        readAllSensors(values);
        
        Serial.println("\n--- Sensor Readings (Extended) ---");
        Serial.print("N: "); Serial.print(values[0]);
        Serial.print(" | E: "); Serial.print(values[1]);
        Serial.print(" | S: "); Serial.print(values[2]);
        Serial.print(" | W: "); Serial.println(values[3]);
        
        float ewError = balanceRatio(values[1], values[3]);
        float nsError = balanceRatio(values[0], values[2]);
        
        // Use larger threshold when extended to avoid oscillation
        bool ewBalanced = isBalanced(ewError, BALANCE_TOL);
        bool nsBalanced = isBalanced(nsError, BALANCE_TOL);
        
        systemBalanced = ewBalanced && nsBalanced;
        
        if (!systemBalanced) {
          Serial.println("✗ IMBALANCE DETECTED - Retracting immediately");
          Serial.print("EW Error: "); Serial.print(ewError, 4);
          Serial.print(" | NS Error: "); Serial.println(nsError, 4);
          startRetracting();
        } else {
          Serial.println("✓ Still balanced");
          // Monitor power while balanced and extended
          float power = readINAVmW();
          Serial.print("Power (mW): ");
          Serial.println(power);
        }
      }
      delay(1000);  // Check balance every second when extended
      break;
      
    case RETRACTING:
      // Wait for retraction to complete
      delay(100);
      break;
  }
}