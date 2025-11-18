#include <Wire.h>
#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml;

#define LNR_UP    10   // RPWM
#define LNR_DOWN   9   // LPWM

#define STP1_STEP  2
#define STP1_DIR   3
#define STP1_EN    4

#define STP2_STEP  5
#define STP2_DIR   6
#define STP2_EN    7

#define PCA_ADDR   0x70

void setup() {
  Serial.begin(115200);

  // Linear Actuator
  pinMode(LNR_UP, OUTPUT);
  pinMode(LNR_DOWN, OUTPUT);

  digitalWrite(LNR_UP, LOW);
  digitalWrite(LNR_DOWN, LOW);

  // Stepper motor 1 (base)
  pinMode(STP1_STEP, OUTPUT);
  pinMode(STP1_DIR, OUTPUT);
  pinMode(STP1_EN, OUTPUT);

  digitalWrite(STP1_EN, HIGH);

  // Stepper motor 2 (arm)
  pinMode(STP1_STEP, OUTPUT);
  pinMode(STP1_DIR, OUTPUT);
  pinMode(STP1_EN, OUTPUT);

  digitalWrite(STP1_EN, HIGH);

  //Sets up PCA (SDA & SCL)
  Wire.begin();
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

void STP1Enable() {
  digitalWrite(STP1_EN, LOW);   // LOW = enabled on most TMC2209 boards
}

void STP1Disable() {
  digitalWrite(STP1_EN, HIGH);  // HIGH = disable
}

void STP1Down(long steps, int speed_delay_us) {
  digitalWrite(STP1_DIR, HIGH); // relaxes pulley
  STP1Enable();

  for (long i = 0; i < steps; i++) {
    digitalWrite(STP1_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP1_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }

  STP1Disable();
}

void STP1Up(long steps, int speed_delay_us) {
  digitalWrite(STP1_DIR, LOW); // tightens pulley
  STP1Enable();

  for (long i = 0; i < steps; i++) {
    digitalWrite(STP1_STEP, HIGH);
    delayMicroseconds(speed_delay_us);
    digitalWrite(STP1_STEP, LOW);
    delayMicroseconds(speed_delay_us);
  }

  STP1Disable();
}

void loop() {
  STP1Up(2000, 700);

  delay(1000);

}


