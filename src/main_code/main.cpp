#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Servo.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// pin definitions
#define SERVO_PIN      9       
#define EDF_PIN        10      
#define ENC_A_PIN      2       // (pd2)
#define ENC_B_PIN      3       // (pd3)

// pipeline timing
#define SENSOR_FREQ_HZ      200
#define CTRL_DECIMATION     4                                   // run control 1/4th as often (50 hz) to give cpu breathing room
#define CTRL_FREQ_HZ        (SENSOR_FREQ_HZ / CTRL_DECIMATION)  // 50 hz
#define T_SAMP              (1.0f / CTRL_FREQ_HZ)               // 0.02 s

// actuator constants
#define SERVO_MIN_US        600
#define SERVO_MAX_US        2500
#define US_PER_DEG          13.5333f
#define SERVO_CENTER_US     1600
#define DELTA_MAX_DEG       15.0f
#define DELTA_MAX_RAD       (DELTA_MAX_DEG * PI / 180.0f)
#define US_PER_RAD          (US_PER_DEG * 180.0f / PI)
#define DELTA_MAX_US        (int)(DELTA_MAX_DEG * US_PER_DEG)

#define EDF_ARM_US          1000
#define EDF_THROTTLE_US     1300 //1300 for run

// encoder constants
volatile long encoderCount = 0; 
volatile uint8_t old_AB = 0;    
#define ENCODER_COUNTS_PER_REV  2400
#define RAD_PER_COUNT           (2.0f * PI / ENCODER_COUNTS_PER_REV)

// PD gains
// static const float Kp = 1.0801*1.01f;
// static const float Kd = 0.276f;
static const float Kp = 2.5586f;
static const float Kd = 0.5012f;

// tustin and lowpass controller
// tau_f acts as a low-pass filter to smooth out noisy derivative spikes
static const float TAU_F = 0.05f;                   
static const float N_TUSTIN = 2.0f / T_SAMP;        
static const float TAU_N = TAU_F * N_TUSTIN;        
static const float DENOM = 1.0f + TAU_N;            
static const float TUSTIN_A1 = (1.0f - TAU_N) / DENOM; 
static const float TUSTIN_B0 = Kp + (Kd * N_TUSTIN) / DENOM;
static const float TUSTIN_B1 = (Kp * TUSTIN_A1) - ((Kd * N_TUSTIN) / DENOM);

// shared states
// volatile ensures memory isn't cached, so rtos tasks always reads fresh data
static volatile float g_pitchRad    = 0.0f;
static volatile float g_gimbalRad   = 0.0f;
static volatile float g_errorRad    = 0.0f;
static volatile int g_edfThrottleUs = EDF_THROTTLE_US; 

static TaskHandle_t xSensingHandle   = NULL;
static TaskHandle_t xControlHandle   = NULL;
static TaskHandle_t xActuationHandle = NULL;
static TaskHandle_t xTelemetryHandle = NULL;

static Servo gimbalServo;
static Servo edfESC;

void TaskSensing   (void *pvParameters);
void TaskControl   (void *pvParameters);
void TaskActuation (void *pvParameters);
void TaskTelemetry (void *pvParameters);
void handleEncoderInterrupt();


// timer2: 200hz int for sensing task
static void timer2Init() {
  noInterrupts();
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // /1024 prescaler
  OCR2A  = 79;  
  TIMSK2 = (1 << OCIE2A);
  interrupts();
}

ISR(TIMER2_COMPA_vect) {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(xSensingHandle, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// ISR for encoder
void handleEncoderInterrupt() {
    static const int8_t lookup[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    
    // direct port read is faster than digitalread() and encoder library
    uint8_t current_AB = (PIND >> 2) & 0x03; 
    
    old_AB <<= 2;
    old_AB |= current_AB;
    encoderCount += lookup[old_AB & 0x0F];
}

void setup() {
  Serial.begin(115200);
  _delay_ms(500); 

  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  
  old_AB = (PIND >> 2) & 0x03;
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), handleEncoderInterrupt, CHANGE);

  // init actuators
  gimbalServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  gimbalServo.writeMicroseconds(SERVO_CENTER_US);
  edfESC.attach(EDF_PIN);
  edfESC.writeMicroseconds(EDF_ARM_US); 

  // arm edf
  Serial.println(F("\n=== SYSTEM STANDBY ==="));
  Serial.println(F("Type 'arm' to unlock ESC and start TVC Pipeline."));

  bool isArmed = false;
  while (!isArmed) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.equalsIgnoreCase("arm")) {
        edfESC.writeMicroseconds(EDF_ARM_US); // confirm arming signal
        isArmed = true;
        Serial.println(F("ESC Armed, Spooling up in 2 seconds"));
      } else {
        Serial.println(F("Command ignored. Type 'arm' to start"));
      }
    }
  }
  
  // wait 2 seconds before 1500us
  _delay_ms(2000); 

  taskENTER_CRITICAL();
  encoderCount = 0;
  taskEXIT_CRITICAL();

  // control(4) -> actuation(3) -> sensing(2) -> telemetry(1)
  // control must compute before actuation can move
  xTaskCreate(TaskControl,   "Ctrl", 150, NULL, 4, &xControlHandle);
  xTaskCreate(TaskActuation, "Act",  100, NULL, 3, &xActuationHandle);
  xTaskCreate(TaskSensing,   "Sens", 128, NULL, 2, &xSensingHandle);
  xTaskCreate(TaskTelemetry, "Tele", 150, NULL, 1, &xTelemetryHandle);
}

void loop() {}

// sensing task (p2, 200 hz)
void TaskSensing(void *pvParameters) {
  timer2Init();
  uint8_t decimCounter = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

    // critical sections disable interrupts briefly so data isn't corrupted mid-read
    long ticks;
    taskENTER_CRITICAL();
    ticks = encoderCount;
    taskEXIT_CRITICAL();

    float current_pitch = (float)ticks * RAD_PER_COUNT;
    
    taskENTER_CRITICAL();
    g_pitchRad = current_pitch;
    taskEXIT_CRITICAL();

    // trigger control loop only when decimation counter fills up
    if (++decimCounter >= CTRL_DECIMATION) {
      decimCounter = 0;
      xTaskNotifyGive(xControlHandle); 
    }
  }
}

// control task (p4, 50 hz)
void TaskControl(void *pvParameters) {
  float c_prev = 0.0f;
  float e_prev = 0.0f;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

    float pitch;
    taskENTER_CRITICAL();
    pitch = g_pitchRad;
    taskEXIT_CRITICAL();

    float e_k = -pitch; 
    float c_k = -TUSTIN_A1 * c_prev + TUSTIN_B0 * e_k + TUSTIN_B1 * e_prev;

    float gim = -c_k;
    if (gim >  DELTA_MAX_RAD) gim =  DELTA_MAX_RAD;
    if (gim < -DELTA_MAX_RAD) gim = -DELTA_MAX_RAD;

    taskENTER_CRITICAL();
    g_gimbalRad = gim;
    g_errorRad  = e_k;
    taskEXIT_CRITICAL();

    e_prev = e_k;
    c_prev = c_k;

    // trigger actuation immediately after math finishes
    xTaskNotifyGive(xActuationHandle); 
  }
}

// actuation task (p3, 50 hz)
void TaskActuation(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float gim;
    int currentThrottle;
    
    taskENTER_CRITICAL();
    gim = g_gimbalRad;
    currentThrottle = g_edfThrottleUs; 
    taskEXIT_CRITICAL();

    int servoUs = SERVO_CENTER_US + (int)(gim * US_PER_RAD);
    
    if (servoUs > SERVO_CENTER_US + DELTA_MAX_US) servoUs = SERVO_CENTER_US + DELTA_MAX_US;
    if (servoUs < SERVO_CENTER_US - DELTA_MAX_US) servoUs = SERVO_CENTER_US - DELTA_MAX_US;

    gimbalServo.writeMicroseconds(servoUs);
    edfESC.writeMicroseconds(currentThrottle); 

    // trigger telemetry last, as it's lowest priority
    xTaskNotifyGive(xTelemetryHandle);
  }
}

// telemetry task (p1, 50 hz chained)
void TaskTelemetry(void *pvParameters) {
  Serial.println(F("Time_ms,Pitch_deg,Error_deg,Gimbal_deg"));
  const float r2d = 180.0f / PI;
  
  static char inputBuffer[16];
  static uint8_t bufIndex = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
    
    // non-blocking serial read for commands
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (bufIndex > 0) {
          inputBuffer[bufIndex] = '\0'; // null-terminate
          
          if (strcasecmp(inputBuffer, "off") == 0) {
            taskENTER_CRITICAL();
            g_edfThrottleUs = EDF_ARM_US; // turn off edf
            taskEXIT_CRITICAL();
          } else if (strcasecmp(inputBuffer, "arm") == 0) {
            taskENTER_CRITICAL();
            g_edfThrottleUs = EDF_THROTTLE_US; // turn on edf
            taskEXIT_CRITICAL();
          }
          bufIndex = 0; // reset buffer
        }
      } else {
        // prevent buffer overflow
        if (bufIndex < sizeof(inputBuffer) - 1) {
          inputBuffer[bufIndex++] = c;
        }
      }
    }

    unsigned long t_now = millis();

    float pitch, err, gim;
    taskENTER_CRITICAL(); 
    pitch = g_pitchRad;
    err   = g_errorRad;
    gim   = g_gimbalRad;
    taskEXIT_CRITICAL();

    Serial.print(t_now);
    Serial.print(F(","));
    Serial.print(pitch * r2d, 2);   
    Serial.print(F(","));
    Serial.print(err * r2d, 2);    
    Serial.print(F(","));
    Serial.println(gim * r2d, 2);
  }
}