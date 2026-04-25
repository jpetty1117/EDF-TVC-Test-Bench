// =============================================================================
// Test: Servo Step Response — characterize loaded servo tau
//
// Uses Paul Stoffregen's Encoder library for reliable quadrature decode.
// Steps gimbal +15° from center, records encoder at 200 Hz for 1 s.
//
// CSV columns: time_ms, ticks, angle_rad, angle_deg
//
// MATLAB:
//   data = readmatrix('step_response.csv');
//   t = data(:,1) / 1000;
//   theta = data(:,4);          % degrees
//   plot(t, theta); xlabel('Time (s)'); ylabel('Angle (deg)');
// =============================================================================

#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define SERVO_PIN       9
#define ENC_A_PIN       2
#define ENC_B_PIN       3

// ── Final Calibrated Actuator Constants ──────────────────────────────────────
#define SERVO_MIN_US      600       // Prevents overshoot at 0°
#define SERVO_MAX_US      2500      // Confirmed travel at 180°
#define US_PER_DEG        13.5333f  // Physical System ID: 13.5333 us/deg
#define SERVO_CENTER_US   1570      // Identified mechanical center
#define STEP_DEG          20        // Operating window for TVC characterization
#define STEP_US           (int)(STEP_DEG * US_PER_DEG)  // ~270 us

// ── Encoder ──────────────────────────────────────────────────────────────────
// Taiss 38S6 "600 P/R" = 600 lines × 4 = 2400 counts/rev (verified empirically).
#define ENCODER_COUNTS_PER_REV  2400
#define RAD_PER_COUNT           (2.0f * PI / ENCODER_COUNTS_PER_REV)
#define DEG_PER_COUNT           (360.0f / ENCODER_COUNTS_PER_REV)

// ── Capture ──────────────────────────────────────────────────────────────────
#define SAMPLE_PERIOD_US    5000UL    // 5 ms → 200 Hz
#define BASELINE_SAMPLES    20        // 100 ms at rest
#define RESPONSE_SAMPLES    200       // 1 s after step

// ── Hardware Objects ─────────────────────────────────────────────────────────
Encoder enc(ENC_A_PIN, ENC_B_PIN);
static Servo gimbalServo;

// =============================================================================
void setup() {
  Serial.begin(115200);

  gimbalServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gimbalServo.writeMicroseconds(SERVO_CENTER_US);
  delay(1000);

  Serial.println(F("=== Servo Step Response ==="));
  Serial.print(F("Center: ")); Serial.print(SERVO_CENTER_US);
  Serial.print(F(" us -> Step to: ")); Serial.print(SERVO_CENTER_US + STEP_US);
  Serial.print(F(" us  (+"));  Serial.print(STEP_DEG);
  Serial.println(F(" deg)"));
  Serial.println(F("Send any key to fire step..."));

  while (!Serial.available()) { ; }
  while (Serial.available()) Serial.read();

  enc.write(0);   // zero the encoder

  Serial.println(F("time_ms,ticks,angle_rad,angle_deg"));

  // ── Baseline ──────────────────────────────────────────────────────────────
  unsigned long t0 = micros();
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    long ticks = enc.read();
    unsigned long ms = (micros() - t0) / 1000UL;
    Serial.print(ms);    Serial.print(',');
    Serial.print(ticks); Serial.print(',');
    Serial.print(ticks * RAD_PER_COUNT, 6); Serial.print(',');
    Serial.print(ticks * DEG_PER_COUNT, 4);
    Serial.print('\n');
    while ((micros() - t0) < (unsigned long)(i + 1) * SAMPLE_PERIOD_US) { ; }
  }

  // ── STEP ──────────────────────────────────────────────────────────────────
  gimbalServo.writeMicroseconds(SERVO_CENTER_US + STEP_US);
  unsigned long stepTime = micros();

  // ── Response ──────────────────────────────────────────────────────────────
  for (int i = 0; i < RESPONSE_SAMPLES; i++) {
    long ticks = enc.read();
    unsigned long ms = (micros() - t0) / 1000UL;
    Serial.print(ms);    Serial.print(',');
    Serial.print(ticks); Serial.print(',');
    Serial.print(ticks * RAD_PER_COUNT, 6); Serial.print(',');
    Serial.print(ticks * DEG_PER_COUNT, 4);
    Serial.print('\n');
    unsigned long target = stepTime + (unsigned long)(i + 1) * SAMPLE_PERIOD_US;
    while (micros() < target) { ; }
  }

  // ── Return ────────────────────────────────────────────────────────────────
  gimbalServo.writeMicroseconds(SERVO_CENTER_US);

  Serial.println();
  Serial.println(F("DONE"));
  Serial.print(F("Step at sample ")); Serial.println(BASELINE_SAMPLES);
  Serial.print(F("Expected: ")); Serial.print(STEP_DEG);
  Serial.print(F(" deg = ")); Serial.print((int)(STEP_DEG * (long)ENCODER_COUNTS_PER_REV / 360.0f));
  Serial.println(F(" ticks"));
}

void loop() { }