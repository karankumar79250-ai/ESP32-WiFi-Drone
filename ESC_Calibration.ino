// ============================================================
// ESC AUTOMATIC CALIBRATION — ESP32 Drone
// ============================================================
// PINS: Motor 1 = 25, Motor 2 = 26, Motor 3 = 27, Motor 4 = 14
//
// ISTEMAL KA TARIKA:
//   1. Propellers HATA do pehle — ZARURI HAI
//   2. Yeh sketch upload karo (drone_v6 wala nahi)
//   3. Serial Monitor kholo (115200 baud)
//   4. Battery LAGAO — LED teji se blink karega
//   5. Kuch seconds mein ESC beep karega (high signal mila)
//   6. Phir LOW signal jayega — ESC calibrate ho jayega
//   7. Calibration complete hone ke baad yeh sketch flash karo:
//      drone ka asli code wapas upload kar do
// ============================================================

#include <ESP32Servo.h>

#define MOTOR_1_PIN  25
#define MOTOR_2_PIN  26
#define MOTOR_3_PIN  27
#define MOTOR_4_PIN  14
#define LED_PIN       2

#define PWM_MAX      2000   // Full throttle signal
#define PWM_MIN      1000   // Zero throttle signal
#define PWM_MID      1500   // Mid signal (kuch ESC ke liye)

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
// SABHI MOTORS KO EK SAATH SIGNAL DENA
// ============================================================
void setAll(int us) {
  motor_1.writeMicroseconds(us);
  motor_2.writeMicroseconds(us);
  motor_3.writeMicroseconds(us);
  motor_4.writeMicroseconds(us);
}

// ============================================================
// SETUP — SIRF BAAR CHALTA HAI
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("============================================");
  Serial.println("  ESC AUTOMATIC CALIBRATION — STARTING");
  Serial.println("============================================");
  Serial.println(">> PROPELLER LAGA HAI TOH ABHI HATA DO!");
  Serial.println(">> 3 second mein calibration shuru hogi...");
  Serial.println();

  // Warning blink — 3 baar teji se
  blinkLED(6, 250);

  // Motors attach karo — min/max specify karo
  motor_1.attach(MOTOR_1_PIN, 1000, 2000);
  motor_2.attach(MOTOR_2_PIN, 1000, 2000);
  motor_3.attach(MOTOR_3_PIN, 1000, 2000);
  motor_4.attach(MOTOR_4_PIN, 1000, 2000);

  // --------------------------------------------------------
  // STEP 1: HIGH SIGNAL BHEJO
  // ESC ko pata chale ki yeh maximum signal hai
  // --------------------------------------------------------
  Serial.println("[STEP 1] HIGH signal (2000us) bhej raha hoon...");
  Serial.println("         Ab battery lagao (agar nahi lagi)");
  setAll(PWM_MAX);
  digitalWrite(LED_PIN, HIGH);

  // 4 second ruko — ESC ko time do HIGH signal samajhne ka
  // Is dauran ESC ek ya do beep karta hai
  delay(4000);

  Serial.println("         ESC ne HIGH signal pakad liya hoga (beep suni?)");
  Serial.println();

  // --------------------------------------------------------
  // STEP 2: LOW SIGNAL BHEJO
  // ESC range set kar leta hai: 1000 = min, 2000 = max
  // --------------------------------------------------------
  Serial.println("[STEP 2] LOW signal (1000us) bhej raha hoon...");
  setAll(PWM_MIN);
  digitalWrite(LED_PIN, LOW);

  // 3 second ruko — ESC calibrate ho jata hai
  // Is dauran ESC 2-3 confirmation beep karta hai
  delay(3000);

  Serial.println("         ESC calibrate ho gaya! (confirmation beep suni?)");
  Serial.println();

  // --------------------------------------------------------
  // STEP 3: VERIFICATION — THODA THROTTLE DO
  // Check karo ki sabhi motors ek saath smoothly chalti hain
  // --------------------------------------------------------
  Serial.println("[STEP 3] VERIFICATION — Motors test kar raha hoon...");
  Serial.println("         WARNING: Propeller NAHI laga hona chahiye!");
  Serial.println("         20% throttle pe 3 seconds chalaunga...");
  blinkLED(3, 200);

  // 20% throttle = 1200us (1000 min + 200 extra)
  setAll(1200);
  delay(3000);

  // Band karo
  setAll(PWM_MIN);
  Serial.println("         Motors band kar diye.");
  Serial.println();

  // --------------------------------------------------------
  // COMPLETE
  // --------------------------------------------------------
  Serial.println("============================================");
  Serial.println("  CALIBRATION COMPLETE!");
  Serial.println("============================================");
  Serial.println(">> Ab apna asli drone code wapas upload karo");
  Serial.println(">> Sabhi 4 ESC calibrate ho gaye hain");
  Serial.println(">> Agar koi motor kamzor lage toh dobara run karo");
  Serial.println();

  // Slow blink — success
  blinkLED(10, 500);
}

// ============================================================
// LOOP — KUCH NAHI KARNA, CALIBRATION COMPLETE HO GAYI
// ============================================================
void loop() {
  // Sirf LED blink karo — signal ki calibration done hai
  static bool s = false;
  s = !s;
  digitalWrite(LED_PIN, s);
  delay(1000);
}
