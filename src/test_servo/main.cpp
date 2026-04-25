// =============================================================================
// Test: Gimbal Servo + EDF ESC PWM
//
// Sweeps the gimbal servo between ±15° around center (99°),
// while holding the EDF ESC at a constant throttle.  Use this to verify
// servo range, direction, and EDF arm sequence.
//
// Uses writeMicroseconds() with attach(pin, 500, 2500) for precise control.
//
// Wiring:
//   Servo signal → Pin 9
//   ESC   signal → Pin 10
// =============================================================================

#include <Arduino.h>
#include <Servo.h>

// ── Pin Definitions ──────────────────────────────────────────────────────────
#define SERVO_PIN       9
#define EDF_PIN         10

// ── Final Calibrated Actuator Constants ──────────────────────────────────────
#define SERVO_MIN_US      600       // Prevents overshoot at 0°
#define SERVO_MAX_US      2500      // Confirmed travel at 180°
#define US_PER_DEG        13.5333f  // Physical System ID: 13.5333 us/deg
#define SERVO_CENTER_US   1600      // Identified mechanical center
#define DELTA_MAX_DEG     20.0f
#define GIMBAL_MIN_US     (int)(SERVO_CENTER_US - DELTA_MAX_DEG * US_PER_DEG)
#define GIMBAL_MAX_US     (int)(SERVO_CENTER_US + DELTA_MAX_DEG * US_PER_DEG)

// ── EDF Throttle ─────────────────────────────────────────────────────────────
#define EDF_ARM_US          1000
#define EDF_THROTTLE_US     1000

// ── Sweep Parameters ─────────────────────────────────────────────────────────
#define SWEEP_STEP_US       (int)(1.0f * US_PER_DEG)    // ~14 µs ≈ 1°
#define SWEEP_DELAY_MS      10

static Servo gimbalServo;
static Servo edfESC;

// Helper: convert µs to degrees for display
static float usToDeg(int us) {
  return (us - SERVO_MIN_US) / US_PER_DEG;
}

// =============================================================================
void setup() {
  Serial.begin(115200);

  // Attach with extended pulse range
  gimbalServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gimbalServo.writeMicroseconds(SERVO_CENTER_US);

  edfESC.attach(EDF_PIN);
  edfESC.writeMicroseconds(EDF_ARM_US);
  Serial.println(F("Arming ESC..."));
  delay(2000);

  edfESC.writeMicroseconds(EDF_THROTTLE_US);
  Serial.println(F("ESC armed.  EDF running."));
  Serial.print(F("Center: ")); Serial.print(SERVO_CENTER_US);
  Serial.println(F(" us"));
  Serial.print(F("Sweep: ")); Serial.print(GIMBAL_MIN_US);
  Serial.print(F(" - ")); Serial.print(GIMBAL_MAX_US);
  Serial.println(F(" us"));
  Serial.println(F("=== Servo Sweep Test ==="));
}

void loop() {
  // Sweep from center → max
  for (int us = SERVO_CENTER_US; us <= GIMBAL_MAX_US; us += SWEEP_STEP_US) {
    gimbalServo.writeMicroseconds(us);
    Serial.print(F("Servo: ")); Serial.print(us);
    Serial.print(F(" us (")); Serial.print(usToDeg(us), 1);
    Serial.println(F(" deg)"));
    delay(SWEEP_DELAY_MS);
  }

  // Sweep from max → min
  for (int us = GIMBAL_MAX_US; us >= GIMBAL_MIN_US; us -= SWEEP_STEP_US) {
    gimbalServo.writeMicroseconds(us);
    Serial.print(F("Servo: ")); Serial.print(us);
    Serial.print(F(" us (")); Serial.print(usToDeg(us), 1);
    Serial.println(F(" deg)"));
    delay(SWEEP_DELAY_MS);
  }

  // Sweep from min → center
  for (int us = GIMBAL_MIN_US; us <= SERVO_CENTER_US; us += SWEEP_STEP_US) {
    gimbalServo.writeMicroseconds(us);
    Serial.print(F("Servo: ")); Serial.print(us);
    Serial.print(F(" us (")); Serial.print(usToDeg(us), 1);
    Serial.println(F(" deg)"));
    delay(SWEEP_DELAY_MS);
  }

  delay(1000);
}
