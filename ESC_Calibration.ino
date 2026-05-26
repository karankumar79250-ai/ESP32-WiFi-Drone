// ============================================================
// ESC AUTOMATIC CALIBRATION — ESP32 Drone
// ============================================================
// PINS: Motor 1 = 25, Motor 2 = 14, Motor 3 = 27, Motor 4 = 26
//
// HOW TO USE:
//   1. REMOVE all propellers first — THIS IS CRITICAL
//   2. Upload this sketch 
//   3. Open Serial Monitor (115200 baud)
//   4. Connect the battery — When instructions for connecting the battery comes on Serial Monitor (you can connect the battery from initial also the code will  work)
//   5. ESC will beep after a few seconds (high signal received)
//   6. Then LOW signal is sent — ESC will calibrate
//   7. After calibration is complete, upload the main drone code
// ============================================================

#include <ESP32Servo.h>

#define MOTOR_1_PIN  25
#define MOTOR_2_PIN  14
#define MOTOR_3_PIN  27
#define MOTOR_4_PIN  26
#define LED_PIN       2

#define PWM_MAX      2000   // Full throttle signal
#define PWM_MIN      1000   // Zero throttle signal
#define PWM_MID      1500   // Mid signal (for some ESCs)

Servo motor_1, motor_2, motor_3, motor_4;

// ============================================================
// LED BLINK HELPER
// ============================================================
void blinkLED(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(ms);
    digitalWrite(LED_PIN, LOW);
    delay(ms);
  }
}

// ============================================================
// SEND SAME SIGNAL TO ALL MOTORS AT ONCE
// ============================================================
void setAll(int us) {
  motor_1.writeMicroseconds(us);
  motor_2.writeMicroseconds(us);
  motor_3.writeMicroseconds(us);
  motor_4.writeMicroseconds(us);
}

// ============================================================
// SETUP — RUNS ONCE
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("============================================");
  Serial.println("  ESC AUTOMATIC CALIBRATION — STARTING");
  Serial.println("============================================");
  Serial.println(">> REMOVE PROPELLERS NOW IF ATTACHED!");
  Serial.println(">> Calibration will start in 3 seconds...");
  Serial.println();

  // Warning blink — 3 times rapidly
  blinkLED(6, 250);

  // Attach motors — specify min/max range
  motor_1.attach(MOTOR_1_PIN, 1000, 2000);
  motor_2.attach(MOTOR_2_PIN, 1000, 2000);
  motor_3.attach(MOTOR_3_PIN, 1000, 2000);
  motor_4.attach(MOTOR_4_PIN, 1000, 2000);

  // --------------------------------------------------------
  // STEP 1: SEND HIGH SIGNAL
  // Let the ESC know this is the maximum throttle signal
  // --------------------------------------------------------
  Serial.println("[STEP 1] Sending HIGH signal (2000us)...");
  Serial.println("         Connect battery now (if not connected)");
  setAll(PWM_MAX);
  digitalWrite(LED_PIN, HIGH);

  // Wait 4 seconds — give ESC time to recognise HIGH signal
  // ESC will beep once or twice during this time
  delay(4000);

  Serial.println("         ESC should have received HIGH signal (heard a beep?)");
  Serial.println();

  // --------------------------------------------------------
  // STEP 2: SEND LOW SIGNAL
  // ESC sets its range: 1000 = min, 2000 = max
  // --------------------------------------------------------
  Serial.println("[STEP 2] Sending LOW signal (1000us)...");
  setAll(PWM_MIN);
  digitalWrite(LED_PIN, LOW);

  // Wait 3 seconds — ESC calibrates during this time
  // ESC will beep 2-3 times as confirmation
  delay(3000);

  Serial.println("         ESC calibrated! (heard confirmation beeps?)");
  Serial.println();

  // --------------------------------------------------------
  // STEP 3: VERIFICATION — APPLY A LITTLE THROTTLE
  // Check all motors run smoothly together
  // --------------------------------------------------------
  Serial.println("[STEP 3] VERIFICATION — Checking all motors run smoothly together...");
  Serial.println("         WARNING: Make sure NO propellers are attached!");
  Serial.println("         Running motors at 20% throttle for 3 seconds...");
  blinkLED(3, 200);

  // 20% throttle = 1200us (1000 min + 200 extra)
  setAll(1200);
  delay(3000);

  // Stop motors
  setAll(PWM_MIN);
  Serial.println("         Motors stopped.");
  Serial.println();

  // --------------------------------------------------------
  // COMPLETE
  // --------------------------------------------------------
  Serial.println("============================================");
  Serial.println("  CALIBRATION COMPLETE!");
  Serial.println("============================================");
  Serial.println(">> Now upload your main drone code");
  Serial.println(">> All 4 ESCs have been calibrated");
  Serial.println(">> If any motor feels weak, run this sketch again");
  Serial.println();

  // Slow blink — success
  blinkLED(10, 500);
}

// ============================================================
// LOOP — NOTHING TO DO, CALIBRATION IS COMPLETE
// ============================================================
void loop() {
  // Just blink LED — indicates calibration is done
  static bool s = false;
  s = !s;
  digitalWrite(LED_PIN, s);
  delay(1000);
}
