// =============================================================================
// Test: FreeRTOS Task Scheduling Verification
//
// Creates three tasks at the same priorities used in the real TVC code to
// verify that FreeRTOS is running, tasks execute at the correct rates, and
// higher-priority tasks preempt lower ones.
//
// NOTE: feilipu/FreeRTOS on AVR uses Timer0 at ~62 Hz  →  1 tick ≈ 16 ms.
//       pdMS_TO_TICKS() rounds down, so anything < 16 ms becomes 0 ticks.
//       This test uses rates compatible with that resolution and prints
//       only every Nth cycle to avoid flooding Serial.
//
// Expected output pattern (repeating):
//   [  xxx] CTRL  #N
//   [  xxx] SENS  #N
//   [  xxx] ACT   #N
// =============================================================================

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// ── Timing ───────────────────────────────────────────────────────────────────
// Use periods that are multiples of the ~16 ms tick to get real blocking.
// Control at ~62 Hz (1 tick), Sensor at ~62 Hz (1 tick), Actuation at ~31 Hz (2 ticks).
// In the real main_code, the sensor ISR runs outside the scheduler so tick
// resolution doesn't limit it — this test just validates task scheduling.
#define CONTROL_PERIOD_TICKS   1     // ~16 ms  (highest priority)
#define SENSOR_PERIOD_TICKS    1     // ~16 ms  (medium priority)
#define ACTUATION_PERIOD_TICKS 2     // ~32 ms  (lowest priority)

// Print every Nth cycle to avoid Serial flooding
#define CTRL_PRINT_EVERY    30       // print every ~480 ms
#define SENS_PRINT_EVERY    30
#define ACT_PRINT_EVERY     15       // runs at half rate, so 15 × 32 ms ≈ 480 ms

// ── LED ──────────────────────────────────────────────────────────────────────
#define LED_PIN   LED_BUILTIN

// ── Mutex for Serial ─────────────────────────────────────────────────────────
static SemaphoreHandle_t xSerialMutex;

// ── Helper: thread-safe timestamped print ────────────────────────────────────
static void safePrint(const __FlashStringHelper *label, unsigned long count) {
  if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.print(F("["));
    Serial.print(millis());
    Serial.print(F("] "));
    Serial.print(label);
    Serial.print(F(" #"));
    Serial.println(count);
    xSemaphoreGive(xSerialMutex);
  }
}

// ── Control Task (Priority 3 — highest) ─────────────────────────────────────
void TaskControl(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWake = xTaskGetTickCount();
  unsigned long n = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWake, CONTROL_PERIOD_TICKS);
    n++;
    if (n % CTRL_PRINT_EVERY == 0) {
      safePrint(F("CTRL "), n);
    }
  }
}

// ── Sensor Task (Priority 2) ────────────────────────────────────────────────
void TaskSensor(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWake = xTaskGetTickCount();
  unsigned long n = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWake, SENSOR_PERIOD_TICKS);
    n++;
    if (n % SENS_PRINT_EVERY == 0) {
      safePrint(F("SENS "), n);
    }
  }
}

// ── Actuation Task (Priority 1 — lowest) ────────────────────────────────────
void TaskActuation(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWake = xTaskGetTickCount();
  unsigned long n = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWake, ACTUATION_PERIOD_TICKS);
    n++;

    // Toggle LED for visual heartbeat
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    if (n % ACT_PRINT_EVERY == 0) {
      safePrint(F("ACT  "), n);
    }
  }
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  xSerialMutex = xSemaphoreCreateMutex();

  Serial.println(F("=== FreeRTOS Scheduling Test ==="));
  Serial.print(F("Tick period: "));
  Serial.print(portTICK_PERIOD_MS);
  Serial.println(F(" ms"));
  Serial.println(F("Expecting CTRL, SENS, and ACT lines ~every 480 ms."));
  Serial.println();

  xTaskCreate(TaskControl,   "Ctrl", 128, NULL, 3, NULL);
  xTaskCreate(TaskSensor,    "Sens", 128, NULL, 2, NULL);
  xTaskCreate(TaskActuation, "Act",  128, NULL, 1, NULL);

  // Scheduler starts automatically after setup() on AVR
}

void loop() {
  // Intentionally empty — all work is in FreeRTOS tasks
}
