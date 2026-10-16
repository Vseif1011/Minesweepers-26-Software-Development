/* Minesweepers A: standalone Arduino Mega serial firmware for the ROS 2 Pi bridge.
 * No rosserial, micro-ROS, or third-party encoder library required.
 * Host -> Mega: C,linear_m_per_s,angular_rad_per_s,controller\n
 * Mega -> host: T,left_int32,right_int32,metal_0_or_1\n at 20 Hz.
 * AVR Mega 2560 only. Encoder counts use all four quadrature transitions.
 */
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !defined(__AVR_ATmega2560__)
#error "Select Arduino Mega 2560"
#endif

const uint8_t PWM_PINS[] = {13, 12, 7, 5};
const uint8_t DIR_PINS[] = {22, 24, 26, 28};
const uint8_t METAL = 4, ALARM = 53, MAGNET = 6, BUZZER = 52;
const uint8_t ARM_DIR = 8, ARM_PWM = 9;
// Must match both workspace YAML files; signs must be calibrated on the robot.
const float TRACK_WIDTH = 0.55f, MAX_SPEED = 1.0f;
const int8_t LEFT_ENCODER_SIGN = 1, RIGHT_ENCODER_SIGN = 1;
const uint32_t COMMAND_TIMEOUT_MS = 500;
volatile int32_t leftTicks = 0, rightTicks = 0;
volatile uint8_t previousLeft = 0, previousRight = 0;
const int8_t QUADRATURE[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
uint32_t lastCommand = 0, lastTelemetry = 0, metalChanged = 0;
bool haveCommand = false, rawMetal = false, stableMetal = false;
char inputLine[96];
uint8_t inputLength = 0;
bool droppingLine = false;

uint8_t leftState() { return (digitalRead(10) << 1) | digitalRead(11); }
uint8_t rightState() { return (digitalRead(2) << 1) | digitalRead(3); }
ISR(PCINT0_vect) {
  uint8_t current = leftState();
  // Unsigned arithmetic defines wrap at 32 bits without signed overflow.
  leftTicks = (uint32_t)leftTicks + LEFT_ENCODER_SIGN * QUADRATURE[(previousLeft << 2) | current];
  previousLeft = current;
}
void rightEncoderChanged() {
  uint8_t current = rightState();
  rightTicks = (uint32_t)rightTicks + RIGHT_ENCODER_SIGN * QUADRATURE[(previousRight << 2) | current];
  previousRight = current;
}
void setMotor(uint8_t index, float speed) {
  digitalWrite(DIR_PINS[index], speed >= 0 ? HIGH : LOW);
  analogWrite(PWM_PINS[index], (int)roundf(constrain(fabsf(speed) / MAX_SPEED, 0.0f, 1.0f) * 255));
}
void drive(float linear, float angular) {
  float left = linear - angular * TRACK_WIDTH / 2;
  float right = linear + angular * TRACK_WIDTH / 2;
  float scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)) / MAX_SPEED);
  left /= scale; right /= scale;
  setMotor(0, -right); setMotor(1, left);
  setMotor(2, left); setMotor(3, -right);
}
void stopActuators() {
  drive(0, 0);
  analogWrite(ARM_PWM, 0);
  // Keep the magnet state to avoid dropping a held object on communications loss.
}
void controller(int value) {
  if (value == 11) digitalWrite(MAGNET, LOW);
  if (value == 10) digitalWrite(MAGNET, HIGH);
  if (value == 7 || value == 8) {
    digitalWrite(ARM_DIR, value == 8 ? HIGH : LOW);
    analogWrite(ARM_PWM, 80);
  } else {
    analogWrite(ARM_PWM, 0);
  }
  // Commands 5/6 remain accepted for compatibility; scaling is done by teleop.
}
bool parseFloatToken(char *token, float &value) {
  if (!token || !*token) return false;
  char *end;
  value = strtod(token, &end);
  return *end == '\0' && isfinite(value);
}
void processCommand(char *line) {
  char *save;
  char *tag = strtok_r(line, ",", &save);
  if (!tag || strcmp(tag, "C") != 0) return;
  float linear, angular;
  if (!parseFloatToken(strtok_r(NULL, ",", &save), linear) ||
      !parseFloatToken(strtok_r(NULL, ",", &save), angular)) return;
  char *controlText = strtok_r(NULL, ",", &save);
  if (!controlText) return;
  char *end;
  long value = strtol(controlText, &end, 10);
  if (end == controlText || *end || strtok_r(NULL, ",", &save)) return;
  if (!(value == 0 || (value >= 5 && value <= 11))) return;
  drive(linear, angular);
  controller(value);
  lastCommand = millis();
  haveCommand = true;
}
void setup() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(PWM_PINS[i], OUTPUT); pinMode(DIR_PINS[i], OUTPUT);
  }
  pinMode(METAL, INPUT); pinMode(ALARM, OUTPUT); pinMode(MAGNET, OUTPUT);
  pinMode(BUZZER, OUTPUT); pinMode(ARM_DIR, OUTPUT); pinMode(ARM_PWM, OUTPUT);
  digitalWrite(ALARM, HIGH); digitalWrite(MAGNET, HIGH); digitalWrite(BUZZER, LOW);
  stopActuators();
  pinMode(10, INPUT_PULLUP); pinMode(11, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
  previousLeft = leftState(); previousRight = rightState();
  // Mega pins 10/11 are PB4/PB5, supporting pin-change interrupt group 0.
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5);
  PCIFR |= _BV(PCIF0); PCICR |= _BV(PCIE0);
  attachInterrupt(digitalPinToInterrupt(2), rightEncoderChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderChanged, CHANGE);
  Serial.begin(115200);
}
void loop() {
  uint32_t now = millis();
  // A bounded number of bytes keeps watchdog and telemetry responsive to noise.
  for (uint8_t i = 0; i < 96 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c == '\n') {
      if (!droppingLine) { inputLine[inputLength] = '\0'; processCommand(inputLine); }
      inputLength = 0; droppingLine = false;
    } else if (c != '\r' && !droppingLine) {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
      else { droppingLine = true; inputLength = 0; }
    }
  }
  if (!haveCommand || (uint32_t)(now - lastCommand) > COMMAND_TIMEOUT_MS) stopActuators();
  bool metal = digitalRead(METAL) == HIGH;
  if (metal != rawMetal) { rawMetal = metal; metalChanged = now; }
  if ((uint32_t)(now - metalChanged) >= 40) stableMetal = rawMetal;
  digitalWrite(ALARM, stableMetal ? LOW : HIGH);
  digitalWrite(BUZZER, stableMetal ? HIGH : LOW);
  if ((uint32_t)(now - lastTelemetry) >= 50) {
    int32_t left, right;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { left = leftTicks; right = rightTicks; }
    Serial.print("T,"); Serial.print(left); Serial.print(',');
    Serial.print(right); Serial.print(','); Serial.println(stableMetal ? 1 : 0);
    lastTelemetry = now;
  }
}
