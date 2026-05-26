// ============================================================
// ESP32 WiFi Drone Flight Controller
// Acro Mode (Rate Mode) + Angle Mode (Stabilize Mode)
// Hardware: ESP32 + MPU6050 + 4x BLDC Motors + ESC
// WiFi Control via Web Browser
// ============================================================
// Motor Pins: M1=25, M2=14, M3=27, M4=26
// MPU6050:    SCL=22, SDA=21
// WiFi AP:    SSID="DroneController", PASS="drone1234"
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ============================================================
// WiFi Access Point Settings
// ============================================================
const char* ssid     = "DroneController";
const char* password = "drone1234";

WebServer server(80);

// ============================================================
// Motor Pins (ESP32 GPIO)
// ============================================================
#define MOTOR1_PIN 25   // Front-Right
#define MOTOR2_PIN 14   // Rear-Right
#define MOTOR3_PIN 27   // Rear-Left
#define MOTOR4_PIN 26   // Front-Left

Servo motor1, motor2, motor3, motor4;

// ============================================================
// MPU6050 Variables (Projects 4, 5, 14, 15)
// ============================================================
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int   RateCalibrationNumber;

// Accelerometer (Angle Mode only)
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Kalman Filter (Angle Mode only)
float KalmanAngleRoll       = 0, KalmanUncertaintyAngleRoll  = 2*2;
float KalmanAnglePitch      = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[]      = {0, 0};

// ============================================================
// PID Variables — Inner Loop (Rate) — Both Modes
// ============================================================
float DesiredRateRoll,  DesiredRatePitch,  DesiredRateYaw;
float ErrorRateRoll,    ErrorRatePitch,    ErrorRateYaw;
float InputRoll,        InputThrottle,     InputPitch, InputYaw;
float PrevErrorRateRoll,  PrevErrorRatePitch,  PrevErrorRateYaw;
float PrevItermRateRoll,  PrevItermRatePitch,  PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

// Rate PID Gains (Project 12 — same as original)
float PRateRoll  = 0.6;  float PRatePitch = PRateRoll;   float PRateYaw  = 2;
float IRateRoll  = 0;  float IRatePitch = IRateRoll;   float IRateYaw  = 0;
float DRateRoll  = 0; float DRatePitch = DRateRoll;   float DRateYaw  = 0;

// ============================================================
// PID Variables — Outer Loop (Angle) — Angle Mode only
// ============================================================
float DesiredAngleRoll,  DesiredAnglePitch;
float ErrorAngleRoll,    ErrorAnglePitch;
float PrevErrorAngleRoll,  PrevErrorAnglePitch;
float PrevItermAngleRoll,  PrevItermAnglePitch;

// Angle PID Gains (Project 16 — same as original)
float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0; float DAnglePitch = DAngleRoll;

// ============================================================
// Motor Output Variables
// ============================================================
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// ============================================================
// Control Loop Timer
// ============================================================
uint32_t LoopTimer;

// ============================================================
// WiFi Control Inputs (replaces ReceiverValue[] from radio)
// These are updated from HTTP requests in real-time
// Range: 1000-2000 microseconds (same as original PWM logic)
// ============================================================
volatile float ReceiverValue[8] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
// [0]=Roll, [1]=Pitch, [2]=Throttle, [3]=Yaw

// ============================================================
// Flight Mode: 0 = Acro (Rate), 1 = Angle (Stabilize)
// ============================================================
volatile int FlightMode = 0;

// ============================================================
// Safety: Motors armed or not
// ============================================================
volatile bool Armed = false;

// ============================================================
// MUTEX — Race Condition Fix
// Core 0 (WiFi) aur Core 1 (Gyro/PID) dono ReceiverValue[]
// aur shared variables ko ek saath access karte the — ab nahi karenge
// ============================================================
SemaphoreHandle_t dataMutex;

// ============================================================
// FUNCTION: gyro_signals() — Acro Mode
// Source: Project 13 (gyroscope only, no accelerometer)
// MPU6050 SCL=22, SDA=21 (set in Wire.begin())
// ============================================================
void gyro_signals_acro(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);   // Low-pass filter
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);   // Gyro ±500 deg/s
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;
}

// ============================================================
// FUNCTION: gyro_signals() — Angle Mode
// Source: Project 16 (gyroscope + accelerometer)
// Accelerometer sensitivity ±8g, Gyro ±500 deg/s
// ============================================================
void gyro_signals_angle(void) {
  // Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);   // Accel ±8g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  // Accelerometer calibration offsets (Project 16 same values)
  AccX = (float)AccXLSB / 4096 + 0.03;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096 + 0.18;

  AngleRoll  =  atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * (1.0/(3.142/180.0));
  AnglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * (1.0/(3.142/180.0));
}

// ============================================================
// FUNCTION: kalman_1d()
// Source: Project 15 (same as original, not changed)
// ============================================================
void kalman_1d(float KalmanState, float KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement) {
  KalmanState       = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain  = KalmanUncertainty * 1.0 / (1.0 * KalmanUncertainty + 3*3);
  KalmanState       = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ============================================================
// FUNCTION: pid_equation()
// Source: Projects 12 & 16 (same logic as original, not changed)
// ============================================================
void pid_equation(float Error, float P, float I, float D,
                  float PrevError, float PrevIterm) {
  float Pterm     = P * Error;
  float Iterm     = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm >  400) Iterm =  400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm     = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput >  400) PIDOutput =  400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// ============================================================
// FUNCTION: reset_pid()
// Source: Projects 12 & 16 (same as original)
// ============================================================
void reset_pid(void) {
  PrevErrorRateRoll  = 0; PrevErrorRatePitch  = 0; PrevErrorRateYaw  = 0;
  PrevItermRateRoll  = 0; PrevItermRatePitch  = 0; PrevItermRateYaw  = 0;
  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}

// ============================================================
// FUNCTION: write_motors()
// Maps 1000-2000us range to ESP32Servo microseconds
// ============================================================
void write_motors(float m1, float m2, float m3, float m4) {
  motor1.writeMicroseconds((int)m1);
  motor2.writeMicroseconds((int)m2);
  motor3.writeMicroseconds((int)m3);
  motor4.writeMicroseconds((int)m4);
}

// ============================================================
// WiFi HTML Page (served to browser)
// ============================================================
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="hi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Drone Controller</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;600;700&family=Share+Tech+Mono&display=swap');

  :root {
    --bg: #0a0c10;
    --panel: #111520;
    --border: #1e2a3a;
    --accent: #00e5ff;
    --accent2: #ff6b35;
    --danger: #ff2d55;
    --safe: #39ff14;
    --text: #c8d8e8;
    --dim: #4a5a6a;
    --glow: 0 0 12px rgba(0,229,255,0.4);
  }

  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Rajdhani', sans-serif;
    font-size: 15px;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Header */
  .header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 16px;
    background: var(--panel);
    border-bottom: 1px solid var(--border);
    position: sticky;
    top: 0;
    z-index: 100;
  }
  .header h1 {
    font-size: 20px;
    font-weight: 700;
    letter-spacing: 3px;
    color: var(--accent);
    text-shadow: var(--glow);
  }
  .status-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--danger);
    box-shadow: 0 0 8px var(--danger);
    display: inline-block;
    margin-right: 6px;
    transition: all 0.3s;
  }
  .status-dot.armed { background: var(--safe); box-shadow: 0 0 8px var(--safe); }
  .status-label { font-family: 'Share Tech Mono', monospace; font-size: 12px; }

  /* Tabs */
  .tabs {
    display: flex;
    background: var(--panel);
    border-bottom: 1px solid var(--border);
  }
  .tab-btn {
    flex: 1;
    padding: 12px 8px;
    background: none;
    border: none;
    color: var(--dim);
    font-family: 'Rajdhani', sans-serif;
    font-size: 14px;
    font-weight: 600;
    letter-spacing: 2px;
    cursor: pointer;
    border-bottom: 3px solid transparent;
    transition: all 0.25s;
  }
  .tab-btn.active {
    color: var(--accent);
    border-bottom-color: var(--accent);
    text-shadow: var(--glow);
  }

  /* Tab content */
  .tab-content { display: none; padding: 14px 12px; }
  .tab-content.active { display: block; }

  /* Value display row */
  .val-row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr 1fr;
    gap: 8px;
    margin-bottom: 14px;
  }
  .val-box {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 8px 6px;
    text-align: center;
  }
  .val-label { font-size: 10px; letter-spacing: 1px; color: var(--dim); display: block; }
  .val-num {
    font-family: 'Share Tech Mono', monospace;
    font-size: 16px;
    color: var(--accent);
    display: block;
    margin-top: 2px;
  }

  /* Throttle slider */
  .throttle-wrap {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    margin-bottom: 14px;
  }
  .slider-label {
    display: flex;
    justify-content: space-between;
    font-size: 12px;
    letter-spacing: 1px;
    color: var(--dim);
    margin-bottom: 8px;
  }
  input[type=range] {
    -webkit-appearance: none;
    width: 100%;
    height: 8px;
    border-radius: 4px;
    background: var(--border);
    outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 22px; height: 22px;
    border-radius: 50%;
    background: var(--accent2);
    box-shadow: 0 0 10px rgba(255,107,53,0.7);
    cursor: pointer;
  }

  /* Joystick */
  .joystick-row {
    display: flex;
    gap: 12px;
    justify-content: center;
    margin-bottom: 14px;
  }
  .joystick-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
  }
  .joystick-label {
    font-size: 11px;
    letter-spacing: 1px;
    color: var(--dim);
    text-align: center;
  }
  .joystick-canvas {
    border-radius: 50%;
    background: var(--panel);
    border: 2px solid var(--border);
    touch-action: none;
    cursor: pointer;
  }

  /* Buttons */
  .btn-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin-bottom: 14px;
  }
  .btn {
    padding: 14px 8px;
    border-radius: 8px;
    border: 2px solid;
    font-family: 'Rajdhani', sans-serif;
    font-size: 14px;
    font-weight: 700;
    letter-spacing: 2px;
    cursor: pointer;
    background: none;
    transition: all 0.2s;
    user-select: none;
    -webkit-user-select: none;
  }
  .btn-arm {
    border-color: var(--safe);
    color: var(--safe);
  }
  .btn-arm:active, .btn-arm.on {
    background: var(--safe);
    color: var(--bg);
    box-shadow: 0 0 16px rgba(57,255,20,0.5);
  }
  .btn-disarm {
    border-color: var(--danger);
    color: var(--danger);
  }
  .btn-disarm:active {
    background: var(--danger);
    color: #fff;
    box-shadow: 0 0 16px rgba(255,45,85,0.5);
  }
  .btn-full {
    grid-column: 1 / -1;
    border-color: var(--accent);
    color: var(--accent);
    padding: 12px;
  }
  .btn-full:active {
    background: var(--accent);
    color: var(--bg);
  }

  /* Direction pad */
  .dpad {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr 1fr;
    gap: 6px;
    max-width: 200px;
    margin: 0 auto 14px;
  }
  .dpad-btn {
    padding: 16px 8px;
    border-radius: 6px;
    border: 1px solid var(--border);
    background: var(--panel);
    color: var(--text);
    font-size: 20px;
    cursor: pointer;
    text-align: center;
    user-select: none;
    -webkit-user-select: none;
    transition: all 0.15s;
    touch-action: none;
  }
  .dpad-btn:active, .dpad-btn.pressed {
    background: var(--accent);
    color: var(--bg);
    box-shadow: var(--glow);
  }
  .dpad-center { background: var(--bg); border: none; pointer-events: none; }

  /* Info panel */
  .info-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 12px;
    font-family: 'Share Tech Mono', monospace;
    font-size: 11px;
    color: var(--dim);
    line-height: 1.8;
    margin-bottom: 10px;
  }
  .info-panel span { color: var(--accent); }

  /* Divider */
  .section-title {
    font-size: 11px;
    letter-spacing: 2px;
    color: var(--dim);
    margin-bottom: 8px;
    padding-bottom: 4px;
    border-bottom: 1px solid var(--border);
  }

  /* PID Tuning Tab styles */
  .pid-group {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 12px;
    margin-bottom: 12px;
  }
  .pid-group-title {
    font-size: 11px;
    letter-spacing: 2px;
    color: var(--accent);
    margin-bottom: 10px;
    display: block;
  }
  .pid-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 8px;
    gap: 6px;
  }
  .pid-label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    color: var(--text);
    min-width: 110px;
  }
  .pid-controls {
    display: flex;
    align-items: center;
    gap: 4px;
  }
  .pid-btn {
    width: 32px; height: 32px;
    border-radius: 6px;
    border: 1px solid var(--border);
    background: var(--bg);
    color: var(--accent);
    font-size: 18px;
    font-weight: 700;
    cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    user-select: none; -webkit-user-select: none;
    touch-action: manipulation;
    transition: all 0.15s;
    flex-shrink: 0;
  }
  .pid-btn:active {
    background: var(--accent);
    color: var(--bg);
  }
  .pid-input {
    width: 72px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--accent);
    font-family: 'Share Tech Mono', monospace;
    font-size: 14px;
    text-align: center;
    padding: 5px 2px;
    -webkit-appearance: none;
  }
  .pid-input:focus { outline: none; border-color: var(--accent); }
  .pid-step-row {
    display: flex;
    align-items: center;
    gap: 6px;
    margin-bottom: 10px;
    font-size: 11px;
    color: var(--dim);
  }
  .step-btn {
    padding: 4px 10px;
    border-radius: 5px;
    border: 1px solid var(--border);
    background: var(--bg);
    color: var(--text);
    font-family: 'Share Tech Mono', monospace;
    font-size: 11px;
    cursor: pointer;
    user-select: none;
    transition: all 0.15s;
  }
  .step-btn.active-step { border-color: var(--accent); color: var(--accent); background: rgba(0,229,255,0.08); }
  .step-btn:active { background: var(--accent); color: var(--bg); }
  .btn-reset-pid {
    width: 100%;
    padding: 13px;
    border-radius: 8px;
    border: 2px solid var(--accent2);
    background: none;
    color: var(--accent2);
    font-family: 'Rajdhani', sans-serif;
    font-size: 14px;
    font-weight: 700;
    letter-spacing: 2px;
    cursor: pointer;
    margin-bottom: 12px;
    transition: all 0.2s;
    user-select: none;
  }
  .btn-reset-pid:active { background: var(--accent2); color: var(--bg); }
  .pid-saved-msg {
    text-align: center;
    font-family: 'Share Tech Mono', monospace;
    font-size: 11px;
    color: var(--safe);
    height: 16px;
    margin-bottom: 6px;
    opacity: 0;
    transition: opacity 0.4s;
  }
  .pid-saved-msg.show { opacity: 1; }
</style>
</head>
<body>

<div class="header">
  <h1>⬡ DRONE</h1>
  <div style="display:flex;align-items:center">
    <span class="status-dot" id="armDot"></span>
    <span class="status-label" id="armLabel">DISARMED</span>
  </div>
</div>

<div class="tabs">
  <button class="tab-btn active" onclick="switchTab('acro')" id="tabAcro">ACRO MODE</button>
  <button class="tab-btn" onclick="switchTab('angle')" id="tabAngle">ANGLE MODE</button>
  <button class="tab-btn" onclick="switchTab('pid')" id="tabPid">PID TUNE</button>
</div>

<!-- ===================== ACRO TAB ===================== -->
<div class="tab-content active" id="tab-acro">
  <div class="section-title">LIVE VALUES</div>
  <div class="val-row">
    <div class="val-box"><span class="val-label">ROLL</span><span class="val-num" id="aRoll">0</span></div>
    <div class="val-box"><span class="val-label">PITCH</span><span class="val-num" id="aPitch">0</span></div>
    <div class="val-box"><span class="val-label">YAW</span><span class="val-num" id="aYaw">0</span></div>
    <div class="val-box"><span class="val-label">THROT</span><span class="val-num" id="aThrot">1000</span></div>
  </div>

  <div class="section-title">THROTTLE</div>
  <div class="throttle-wrap">
    <div class="slider-label"><span>MIN 1000</span><span id="thrVal">1000</span><span>MAX 1800</span></div>
    <input type="range" id="throttleSlider" min="1000" max="1800" value="1000"
      oninput="onThrottleChange(this.value)">
  </div>

  <div class="section-title">ROLL / PITCH / YAW (Joystick)</div>
  <div class="joystick-row">
    <div class="joystick-wrap">
      <canvas class="joystick-canvas" id="joyLeft" width="140" height="140"></canvas>
      <div class="joystick-label">YAW ← → / (unused)</div>
    </div>
    <div class="joystick-wrap">
      <canvas class="joystick-canvas" id="joyRight" width="140" height="140"></canvas>
      <div class="joystick-label">ROLL ← → / PITCH ↑ ↓</div>
    </div>
  </div>

  <div class="section-title">ARM / DISARM</div>
  <div class="btn-row">
    <button class="btn btn-arm" id="armBtn" onclick="sendArm()">ARM</button>
    <button class="btn btn-disarm" onclick="sendDisarm()">DISARM</button>
  </div>

  <div class="info-panel">
    MODE: <span>ACRO (Rate)</span> &nbsp;|&nbsp;
    LOOP: <span>250 Hz</span><br>
    PID P: <span>R=0.6 P=0.6 Y=2</span><br>
    PID I: <span>R=3.5 P=3.5 Y=12</span><br>
    PID D: <span>R=0.03 P=0.03 Y=0</span>
  </div>
</div>

<!-- ===================== ANGLE TAB ===================== -->
<div class="tab-content" id="tab-angle">
  <div class="section-title">LIVE VALUES</div>
  <div class="val-row">
    <div class="val-box"><span class="val-label">ROLL°</span><span class="val-num" id="bRoll">0</span></div>
    <div class="val-box"><span class="val-label">PITCH°</span><span class="val-num" id="bPitch">0</span></div>
    <div class="val-box"><span class="val-label">YAW</span><span class="val-num" id="bYaw">0</span></div>
    <div class="val-box"><span class="val-label">THROT</span><span class="val-num" id="bThrot">1000</span></div>
  </div>

  <div class="section-title">THROTTLE</div>
  <div class="throttle-wrap">
    <div class="slider-label"><span>MIN 1000</span><span id="thrVal2">1000</span><span>MAX 1800</span></div>
    <input type="range" id="throttleSlider2" min="1000" max="1800" value="1000"
      oninput="onThrottleChange2(this.value)">
  </div>

  <div class="section-title">ROLL / PITCH / YAW (Joystick)</div>
  <div class="joystick-row">
    <div class="joystick-wrap">
      <canvas class="joystick-canvas" id="joyLeft2" width="140" height="140"></canvas>
      <div class="joystick-label">YAW ← →</div>
    </div>
    <div class="joystick-wrap">
      <canvas class="joystick-canvas" id="joyRight2" width="140" height="140"></canvas>
      <div class="joystick-label">ROLL ← → / PITCH ↑ ↓</div>
    </div>
  </div>

  <div class="section-title">ARM / DISARM</div>
  <div class="btn-row">
    <button class="btn btn-arm" id="armBtn2" onclick="sendArm()">ARM</button>
    <button class="btn btn-disarm" onclick="sendDisarm()">DISARM</button>
  </div>

  <div class="info-panel">
    MODE: <span>ANGLE (Stabilize)</span> &nbsp;|&nbsp;
    LOOP: <span>250 Hz</span><br>
    OUTER PID: <span>P=2 I=0 D=0</span><br>
    INNER PID P: <span>R=0.6 P=0.6 Y=2</span><br>
    KALMAN FILTER: <span>ACTIVE</span>
  </div>
</div>

<!-- ===================== PID TUNE TAB ===================== -->
<div class="tab-content" id="tab-pid">

  <div class="section-title">STEP SIZE (har + / - click mein kitna badlega)</div>
  <div class="pid-step-row">
    STEP:
    <button class="step-btn" onclick="setStep(0.001)">0.001</button>
    <button class="step-btn" onclick="setStep(0.01)">0.01</button>
    <button class="step-btn active-step" onclick="setStep(0.1)">0.1</button>
    <button class="step-btn" onclick="setStep(0.5)">0.5</button>
    <button class="step-btn" onclick="setStep(1)">1.0</button>
    <span id="stepDisplay" style="margin-left:4px;color:var(--accent);font-family:'Share Tech Mono',monospace;font-size:11px">= 0.1</span>
  </div>

  <!-- RATE PID (Roll/Pitch) -->
  <div class="pid-group">
    <span class="pid-group-title">RATE PID — ROLL &amp; PITCH (Inner Loop)</span>
    <div class="pid-row">
      <span class="pid-label">P Roll/Pitch</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('prr',-1)" onclick="pidAdj('prr',-1)">−</button>
        <input class="pid-input" id="inp_prr" type="number" step="0.001" value="0.6" onchange="pidSet('prr',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('prr',+1)" onclick="pidAdj('prr',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Roll/Pitch</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('irr',-1)" onclick="pidAdj('irr',-1)">−</button>
        <input class="pid-input" id="inp_irr" type="number" step="0.001" value="3.5" onchange="pidSet('irr',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('irr',+1)" onclick="pidAdj('irr',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Roll/Pitch</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('drr',-1)" onclick="pidAdj('drr',-1)">−</button>
        <input class="pid-input" id="inp_drr" type="number" step="0.001" value="0.03" onchange="pidSet('drr',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('drr',+1)" onclick="pidAdj('drr',+1)">+</button>
      </div>
    </div>
  </div>

  <!-- RATE PID (Yaw) -->
  <div class="pid-group">
    <span class="pid-group-title">RATE PID — YAW (Inner Loop)</span>
    <div class="pid-row">
      <span class="pid-label">P Yaw</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('pry',-1)" onclick="pidAdj('pry',-1)">−</button>
        <input class="pid-input" id="inp_pry" type="number" step="0.001" value="2" onchange="pidSet('pry',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('pry',+1)" onclick="pidAdj('pry',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Yaw</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('iry',-1)" onclick="pidAdj('iry',-1)">−</button>
        <input class="pid-input" id="inp_iry" type="number" step="0.001" value="12" onchange="pidSet('iry',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('iry',+1)" onclick="pidAdj('iry',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Yaw</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('dry',-1)" onclick="pidAdj('dry',-1)">−</button>
        <input class="pid-input" id="inp_dry" type="number" step="0.001" value="0" onchange="pidSet('dry',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('dry',+1)" onclick="pidAdj('dry',+1)">+</button>
      </div>
    </div>
  </div>

  <!-- ANGLE PID (Roll/Pitch outer loop) -->
  <div class="pid-group">
    <span class="pid-group-title">ANGLE PID — ROLL &amp; PITCH (Outer Loop)</span>
    <div class="pid-row">
      <span class="pid-label">P Angle</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('par',-1)" onclick="pidAdj('par',-1)">−</button>
        <input class="pid-input" id="inp_par" type="number" step="0.001" value="2" onchange="pidSet('par',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('par',+1)" onclick="pidAdj('par',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Angle</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('iar',-1)" onclick="pidAdj('iar',-1)">−</button>
        <input class="pid-input" id="inp_iar" type="number" step="0.001" value="0" onchange="pidSet('iar',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('iar',+1)" onclick="pidAdj('iar',+1)">+</button>
      </div>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Angle</span>
      <div class="pid-controls">
        <button class="pid-btn" ontouchstart="pidAdj('dar',-1)" onclick="pidAdj('dar',-1)">−</button>
        <input class="pid-input" id="inp_dar" type="number" step="0.001" value="0" onchange="pidSet('dar',this.value)">
        <button class="pid-btn" ontouchstart="pidAdj('dar',+1)" onclick="pidAdj('dar',+1)">+</button>
      </div>
    </div>
  </div>

  <div class="pid-saved-msg" id="pidSavedMsg">✓ ESP32 PE APPLY HO GAYA</div>
  <button class="btn-reset-pid" onclick="pidReset()">↺ DEFAULT VALUES PE RESET KARO</button>

  <div class="info-panel">
    DEFAULT RATE: <span>P=0.6 I=3.5 D=0.03 (Roll/Pitch)</span><br>
    DEFAULT RATE: <span>P=2 I=12 D=0 (Yaw)</span><br>
    DEFAULT ANGLE: <span>P=2 I=0 D=0 (Roll/Pitch Outer)</span><br>
    NOTE: <span>Roll aur Pitch hamesha same rehte hain</span>
  </div>
</div>

<script>
// ============================================================
// State
// ============================================================
let roll=1500, pitch=1500, throttle=1000, yaw=1500;
let armed=false;
let currentMode=0; // 0=acro, 1=angle
let sending=false;

// ============================================================
// Tab Switch
// ============================================================
function switchTab(tab) {
  document.querySelectorAll('.tab-content').forEach(el=>el.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(el=>el.classList.remove('active'));
  document.getElementById('tab-'+tab).classList.add('active');
  document.getElementById('tab'+tab.charAt(0).toUpperCase()+tab.slice(1)).classList.add('active');
  if(tab==='acro') currentMode=0;
  else if(tab==='angle') currentMode=1;
  // PID tab mein mode nahi badlta, flight mode same rehta hai
  if(tab!=='pid') sendCommand();
  if(tab==='pid') fetchPid(); // PID tab open hone par current values load karo
}

// ============================================================
// PID Tuning Logic
// ============================================================
let pidStep = 0.1;
// Current PID values (browser side mirror)
const pidVals = {prr:0.6, pry:2, irr:3.5, iry:12, drr:0.03, dry:0, par:2, iar:0, dar:0};

function setStep(s) {
  pidStep = s;
  document.getElementById('stepDisplay').textContent = '= '+s;
  document.querySelectorAll('.step-btn').forEach(b=>{
    b.classList.toggle('active-step', parseFloat(b.textContent)==s);
  });
}

function round4(v) { return Math.round(v*10000)/10000; }

function pidAdj(key, dir) {
  pidVals[key] = round4(pidVals[key] + dir * pidStep);
  if(pidVals[key] < 0) pidVals[key] = 0; // PID values negative nahi ho sakti
  document.getElementById('inp_'+key).value = pidVals[key];
  sendPid(key);
}

function pidSet(key, val) {
  pidVals[key] = round4(Math.max(0, parseFloat(val)||0));
  document.getElementById('inp_'+key).value = pidVals[key];
  sendPid(key);
}

function sendPid(changedKey) {
  const url = '/pid?'+changedKey+'='+pidVals[changedKey];
  fetch(url,{method:'GET',keepalive:true})
    .then(res=>res.json())
    .then(data=>{ updatePidUI(data); showSaved(); })
    .catch(()=>{});
}

function pidReset() {
  fetch('/pid?reset=1',{method:'GET',keepalive:true})
    .then(res=>res.json())
    .then(data=>{ updatePidUI(data); showSaved(); })
    .catch(()=>{});
}

function fetchPid() {
  fetch('/pid',{method:'GET',keepalive:true})
    .then(res=>res.json())
    .then(data=>updatePidUI(data))
    .catch(()=>{});
}

function updatePidUI(data) {
  const keys=['prr','pry','irr','iry','drr','dry','par','iar','dar'];
  keys.forEach(k=>{
    if(data[k]!==undefined){
      pidVals[k]=data[k];
      document.getElementById('inp_'+k).value=data[k];
    }
  });
}

function showSaved() {
  const el = document.getElementById('pidSavedMsg');
  el.classList.add('show');
  setTimeout(()=>el.classList.remove('show'), 1800);
}

// ============================================================
// Throttle
// ============================================================
function onThrottleChange(v) {
  throttle=parseInt(v);
  document.getElementById('thrVal').textContent=v;
  document.getElementById('aThrot').textContent=v;
  // keep both sliders in sync
  document.getElementById('throttleSlider2').value=v;
  document.getElementById('thrVal2').textContent=v;
  document.getElementById('bThrot').textContent=v;
  sendCommand();
}
function onThrottleChange2(v) {
  throttle=parseInt(v);
  document.getElementById('thrVal2').textContent=v;
  document.getElementById('bThrot').textContent=v;
  document.getElementById('throttleSlider').value=v;
  document.getElementById('thrVal').textContent=v;
  document.getElementById('aThrot').textContent=v;
  sendCommand();
}

// ============================================================
// Arm / Disarm
// ============================================================
function sendArm() {
  armed=true;
  updateArmUI();
  // Set throttle to 1030 to satisfy ESP32 safety check
  throttle=1030;
  document.getElementById('throttleSlider').value=1030;
  document.getElementById('throttleSlider2').value=1030;
  sendCommand();
}
function sendDisarm() {
  armed=false;
  throttle=1000;
  roll=1500; pitch=1500; yaw=1500;
  document.getElementById('throttleSlider').value=1000;
  document.getElementById('throttleSlider2').value=1000;
  document.getElementById('thrVal').textContent=1000;
  document.getElementById('thrVal2').textContent=1000;
  updateArmUI();
  sendCommand();
}
function updateArmUI() {
  document.getElementById('armDot').className='status-dot'+(armed?' armed':'');
  document.getElementById('armLabel').textContent=armed?'ARMED':'DISARMED';
  ['armBtn','armBtn2'].forEach(id=>{
    const el=document.getElementById(id);
    if(el) el.classList.toggle('on',armed);
  });
}

// ============================================================
// Send Command to ESP32 (single HTTP request, no repeat tap)
// Uses fetch with keepalive; batches rapid changes via rAF
// ============================================================
let pendingSend=false;
function sendCommand() {
  if(pendingSend) return;
  pendingSend=true;
  requestAnimationFrame(()=>{
    pendingSend=false;
    const r=Math.round(roll), p=Math.round(pitch),
          t=Math.round(throttle), y=Math.round(yaw);
    const url=`/cmd?r=${r}&p=${p}&t=${t}&y=${y}&arm=${armed?1:0}&mode=${currentMode}`;
    fetch(url,{method:'GET',keepalive:true})
      .then(res=>res.json())
      .then(data=>{
        // Update live values
        document.getElementById('aRoll').textContent=data.roll.toFixed(1);
        document.getElementById('aPitch').textContent=data.pitch.toFixed(1);
        document.getElementById('aYaw').textContent=data.yaw.toFixed(1);
        document.getElementById('bRoll').textContent=data.roll.toFixed(1);
        document.getElementById('bPitch').textContent=data.pitch.toFixed(1);
        document.getElementById('bYaw').textContent=data.yaw.toFixed(1);
      })
      .catch(()=>{});
  });
}

// ============================================================
// Joystick Implementation (canvas-based, touch & mouse)
// Single touch = continuous hold, auto-centers on release
// ============================================================
function makeJoystick(canvasId, onMove, opts) {
  const canvas = document.getElementById(canvasId);
  const ctx    = canvas.getContext('2d');
  const W=canvas.width, H=canvas.height, R=W/2;
  const stickR=22, deadR=8;
  let active=false, sx=R, sy=R;

  function draw() {
    ctx.clearRect(0,0,W,H);
    // Outer ring
    ctx.beginPath(); ctx.arc(R,R,R-4,0,Math.PI*2);
    ctx.strokeStyle='#1e2a3a'; ctx.lineWidth=2; ctx.stroke();
    // Cross hairs
    ctx.strokeStyle='#1e2a3a'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(R,4); ctx.lineTo(R,H-4); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(4,R); ctx.lineTo(W-4,R); ctx.stroke();
    // Stick
    const grd=ctx.createRadialGradient(sx,sy,2,sx,sy,stickR);
    grd.addColorStop(0,'rgba(0,229,255,0.9)');
    grd.addColorStop(1,'rgba(0,229,255,0.1)');
    ctx.beginPath(); ctx.arc(sx,sy,stickR,0,Math.PI*2);
    ctx.fillStyle=grd; ctx.fill();
    ctx.beginPath(); ctx.arc(sx,sy,stickR,0,Math.PI*2);
    ctx.strokeStyle='rgba(0,229,255,0.6)'; ctx.lineWidth=2; ctx.stroke();
  }

  function getPos(e, el) {
    const rect=el.getBoundingClientRect();
    const src=e.touches ? e.touches[0] : e;
    return {x:src.clientX-rect.left, y:src.clientY-rect.top};
  }

  function update(x,y) {
    let dx=x-R, dy=y-R;
    const dist=Math.sqrt(dx*dx+dy*dy);
    const maxR=R-stickR-4;
    if(dist>maxR){dx=dx/dist*maxR; dy=dy/dist*maxR;}
    sx=R+dx; sy=R+dy;
    // Normalize -1..+1, apply deadzone
    let nx=dx/maxR, ny=dy/maxR;
    if(Math.abs(nx)<deadR/maxR) nx=0;
    if(Math.abs(ny)<deadR/maxR) ny=0;
    onMove(nx, ny);
    draw();
    sendCommand();
  }

  function onStart(e){e.preventDefault();active=true;update(...Object.values(getPos(e,canvas)));}
  function onMove2(e){e.preventDefault();if(active)update(...Object.values(getPos(e,canvas)));}
  function onEnd(e){e.preventDefault();active=false;sx=R;sy=R;onMove(0,0);draw();sendCommand();}

  canvas.addEventListener('touchstart',onStart,{passive:false});
  canvas.addEventListener('touchmove',onMove2,{passive:false});
  canvas.addEventListener('touchend',onEnd,{passive:false});
  canvas.addEventListener('mousedown',onStart);
  canvas.addEventListener('mousemove',onMove2);
  canvas.addEventListener('mouseup',onEnd);
  canvas.addEventListener('mouseleave',onEnd);
  draw();
}

// Acro Left Joystick: YAW only (x-axis)
makeJoystick('joyLeft', (nx,ny)=>{
  yaw = 1500 + Math.round(nx*500);
  document.getElementById('aYaw').textContent=yaw;
  document.getElementById('bYaw').textContent=yaw;
});

// Acro Right Joystick: ROLL (x), PITCH (y)
makeJoystick('joyRight', (nx,ny)=>{
  roll  = 1500 + Math.round(nx*500);
  pitch = 1500 - Math.round(ny*500);
  document.getElementById('aRoll').textContent=roll;
  document.getElementById('aPitch').textContent=pitch;
  document.getElementById('bRoll').textContent=roll;
  document.getElementById('bPitch').textContent=pitch;
});

// Angle Left Joystick: YAW
makeJoystick('joyLeft2', (nx,ny)=>{
  yaw = 1500 + Math.round(nx*500);
  document.getElementById('aYaw').textContent=yaw;
  document.getElementById('bYaw').textContent=yaw;
});

// Angle Right Joystick: ROLL, PITCH
makeJoystick('joyRight2', (nx,ny)=>{
  roll  = 1500 + Math.round(nx*500);
  pitch = 1500 - Math.round(ny*500);
  document.getElementById('aRoll').textContent=roll;
  document.getElementById('aPitch').textContent=pitch;
  document.getElementById('bRoll').textContent=roll;
  document.getElementById('bPitch').textContent=pitch;
});

// Poll sensor data every 200ms
setInterval(()=>{ if(!pendingSend) sendCommand(); }, 200);
</script>
</body>
</html>
)rawhtml";

// ============================================================
// HTTP Handlers
// ============================================================
void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

// PID Tuning endpoint — browser se PID values receive karta hai
// aur current values wapas bhejta hai JSON mein
void handlePid() {
  // Rate PID update (agar browser ne bheja to)
  if (server.hasArg("prr")) { PRateRoll   = server.arg("prr").toFloat(); PRatePitch = PRateRoll; }
  if (server.hasArg("pry")) { PRateYaw    = server.arg("pry").toFloat(); }
  if (server.hasArg("irr")) { IRateRoll   = server.arg("irr").toFloat(); IRatePitch = IRateRoll; }
  if (server.hasArg("iry")) { IRateYaw    = server.arg("iry").toFloat(); }
  if (server.hasArg("drr")) { DRateRoll   = server.arg("drr").toFloat(); DRatePitch = DRateRoll; }
  if (server.hasArg("dry")) { DRateYaw    = server.arg("dry").toFloat(); }
  // Angle PID update
  if (server.hasArg("par")) { PAngleRoll  = server.arg("par").toFloat(); PAnglePitch = PAngleRoll; }
  if (server.hasArg("iar")) { IAngleRoll  = server.arg("iar").toFloat(); IAnglePitch = IAngleRoll; }
  if (server.hasArg("dar")) { DAngleRoll  = server.arg("dar").toFloat(); DAnglePitch = DAngleRoll; }
  // Reset to defaults
  if (server.hasArg("reset") && server.arg("reset") == "1") {
    PRateRoll=0.6; PRatePitch=0.6; PRateYaw=2;
    IRateRoll=0; IRatePitch=0; IRateYaw=0;
    DRateRoll=0; DRatePitch=0; DRateYaw=0;
    PAngleRoll=2; PAnglePitch=2;
    IAngleRoll=0; IAnglePitch=0;
    DAngleRoll=0; DAnglePitch=0;
  }
  // Return current PID values as JSON
  String json = "{";
  json += "\"prr\":"  + String(PRateRoll,  3) + ",";
  json += "\"pry\":"  + String(PRateYaw,   3) + ",";
  json += "\"irr\":"  + String(IRateRoll,  3) + ",";
  json += "\"iry\":"  + String(IRateYaw,   3) + ",";
  json += "\"drr\":"  + String(DRateRoll,  4) + ",";
  json += "\"dry\":"  + String(DRateYaw,   3) + ",";
  json += "\"par\":"  + String(PAngleRoll, 3) + ",";
  json += "\"iar\":"  + String(IAngleRoll, 3) + ",";
  json += "\"dar\":"  + String(DAngleRoll, 3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCmd() {
  // FIX: mutex lो — Core 1 (loop) is waqt ReceiverValue[] nahi padhega
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  // Parse parameters sent from browser
  if (server.hasArg("r"))    ReceiverValue[0] = server.arg("r").toFloat();   // Roll
  if (server.hasArg("p"))    ReceiverValue[1] = server.arg("p").toFloat();   // Pitch
  if (server.hasArg("t"))    ReceiverValue[2] = server.arg("t").toFloat();   // Throttle
  if (server.hasArg("y"))    ReceiverValue[3] = server.arg("y").toFloat();   // Yaw
  if (server.hasArg("arm"))  Armed            = (server.arg("arm") == "1");
  if (server.hasArg("mode")) FlightMode       = server.arg("mode").toInt();
  xSemaphoreGive(dataMutex);
  // FIX: mutex do — ab Core 1 padh sakta hai

  // Return current gyro data as JSON
  String json = "{";
  json += "\"roll\":"  + String(RateRoll,  2) + ",";
  json += "\"pitch\":" + String(RatePitch, 2) + ",";
  json += "\"yaw\":"   + String(RateYaw,   2) + ",";
  json += "\"armed\":"   + String(Armed ? "true" : "false") + ",";
  json += "\"mode\":"    + String(FlightMode);
  json += "}";
  server.send(200, "application/json", json);
}

// ============================================================
// WiFi TASK — Core 0 pe chalta hai (Gyro/PID se alag)
// FIX: server.handleClient() loop() se hataya — ab yeh
//      alag core pe chalta hai taaki gyro read block na ho
// ============================================================
void wifiTask(void *pvParameters) {
  for (;;) {
    server.handleClient();
    vTaskDelay(1);   // 1ms — doosre tasks ko bhi time milega
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // FIX: Mutex banao — Race Condition rokne ke liye
  dataMutex = xSemaphoreCreateMutex();

  // ----- I2C: MPU6050 (SCL=22, SDA=21) -----
  Wire.begin(21, 22);           // SDA=21, SCL=22
  Wire.setClock(400000);
  delay(250);
  // Wake up MPU6050 (register 0x6B = 0x00 = sleep off)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Gyro Calibration (auto on power-on, 2000 samples) -----
  // Same logic as original Projects 5 & 16 — no change
  Serial.println("Calibrating gyro...");
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals_acro();   // Use acro signals for calibration (gyro only)
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }
  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;
  Serial.println("Gyro calibration done.");

  // ----- ESC / Motors (ESP32Servo) -----
  // Motor pins: M1=25, M2=14, M3=27, M4=26
  motor1.setPeriodHertz(250);   // 250 Hz same as original
  motor2.setPeriodHertz(250);
  motor3.setPeriodHertz(250);
  motor4.setPeriodHertz(250);
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);
  motor3.attach(MOTOR3_PIN, 1000, 2000);
  motor4.attach(MOTOR4_PIN, 1000, 2000);

  // Send 1000us to all ESCs (armed low signal)
  write_motors(1000, 1000, 1000, 1000);
  delay(2000);   // ESC initialization beep wait

  // ----- WiFi Access Point -----
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  // ----- Web Server Routes -----
  server.on("/",    handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/pid", handlePid);
  server.begin();
  Serial.println("Web server started.");

  // FIX: WiFi task Core 0 pe start karo
  //      loop() mein handleClient() nahi chalega ab
  //      Gyro + PID Core 1 pe chalega — dono alag alag
  xTaskCreatePinnedToCore(
    wifiTask,       // function
    "WiFiTask",     // naam
    10000,          // stack size
    NULL,           // parameter
    1,              // priority
    NULL,           // task handle
    0               // Core 0
  );

  // ----- Start Loop Timer -----
  LoopTimer = micros();
}

// ============================================================
// MAIN LOOP (250 Hz target)
// ============================================================
void loop() {
  // FIX: server.handleClient() yahan se hataya — WiFiTask (Core 0) handle karta hai

  // ---- Read Gyro & Subtract Calibration ----
  if (FlightMode == 0) {
    gyro_signals_acro();
  } else {
    gyro_signals_angle();
  }
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // ---- If NOT Armed, keep motors off ----
  if (!Armed) {
    write_motors(1000, 1000, 1000, 1000);
    reset_pid();
    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();
    return;
  }

  // FIX: mutex lo — WiFi task is waqt ReceiverValue[] nahi likhega
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  float localRoll     = ReceiverValue[0];
  float localPitch    = ReceiverValue[1];
  float localThrottle = ReceiverValue[2];
  float localYaw      = ReceiverValue[3];
  bool  localArmed    = Armed;
  int   localMode     = FlightMode;
  xSemaphoreGive(dataMutex);
  // FIX: mutex diya — ab safe local copies use hongi

  InputThrottle = localThrottle;

  // ============================================================
  // ACRO MODE (Rate Mode) — Project 13 logic, unchanged
  // ============================================================
  if (localMode == 0) {

    DesiredRateRoll  = 0.15 * (localRoll  - 1500);
    DesiredRatePitch = 0.15 * (localPitch - 1500);
    DesiredRateYaw   = 0.15 * (localYaw   - 1500);

    ErrorRateRoll  = DesiredRateRoll  - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw   = DesiredRateYaw   - RateYaw;

    pid_equation(ErrorRateRoll,  PRateRoll,  IRateRoll,  DRateRoll,
                 PrevErrorRateRoll,  PrevItermRateRoll);
    InputRoll         = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch          = PIDReturn[0];
    PrevErrorRatePitch  = PIDReturn[1];
    PrevItermRatePitch  = PIDReturn[2];

    pid_equation(ErrorRateYaw,   PRateYaw,   IRateYaw,   DRateYaw,
                 PrevErrorRateYaw,   PrevItermRateYaw);
    InputYaw          = PIDReturn[0];
    PrevErrorRateYaw  = PIDReturn[1];
    PrevItermRateYaw  = PIDReturn[2];

  }
  // ============================================================
  // ANGLE MODE (Stabilize Mode) — Project 16 logic, unchanged
  // ============================================================
  else {

    // Kalman filter
    kalman_1d(KalmanAngleRoll,  KalmanUncertaintyAngleRoll,  RateRoll,  AngleRoll);
    KalmanAngleRoll             = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll  = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    // Desired angles from receiver
    DesiredAngleRoll  = 0.10 * (localRoll  - 1500);
    DesiredAnglePitch = 0.10 * (localPitch - 1500);
    DesiredRateYaw    = 0.15 * (localYaw   - 1500);

    // Angle errors
    ErrorAngleRoll  = DesiredAngleRoll  - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    // Outer loop: Angle PID -> Desired Rate
    pid_equation(ErrorAngleRoll,  PAngleRoll,  IAngleRoll,  DAngleRoll,
                 PrevErrorAngleRoll,  PrevItermAngleRoll);
    DesiredRateRoll       = PIDReturn[0];
    PrevErrorAngleRoll    = PIDReturn[1];
    PrevItermAngleRoll    = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch,
                 PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch      = PIDReturn[0];
    PrevErrorAnglePitch   = PIDReturn[1];
    PrevItermAnglePitch   = PIDReturn[2];

    // Inner loop: Rate errors
    ErrorRateRoll  = DesiredRateRoll  - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw   = DesiredRateYaw   - RateYaw;

    pid_equation(ErrorRateRoll,  PRateRoll,  IRateRoll,  DRateRoll,
                 PrevErrorRateRoll,  PrevItermRateRoll);
    InputRoll         = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch          = PIDReturn[0];
    PrevErrorRatePitch  = PIDReturn[1];
    PrevItermRatePitch  = PIDReturn[2];

    pid_equation(ErrorRateYaw,   PRateYaw,   IRateYaw,   DRateYaw,
                 PrevErrorRateYaw,   PrevItermRateYaw);
    InputYaw          = PIDReturn[0];
    PrevErrorRateYaw  = PIDReturn[1];
    PrevItermRateYaw  = PIDReturn[2];
  }

  // ---- Throttle Limit (same as original: max 1800) ----
  if (InputThrottle > 1800) InputThrottle = 1800;

  // ---- Motor Mix (Project 11 — same equations, same 1.024 factor) ----
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

  // ---- Motor Limits Max ----
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  // ---- Motor Idle Min (18% = 1180us) ----
  int ThrottleIdle = 1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // ---- Safety CutOff: Throttle too low = motors off ----
  int ThrottleCutOff = 1000;
  if (localThrottle < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  // ---- Write to Motors ----
  write_motors(MotorInput1, MotorInput2, MotorInput3, MotorInput4);
  // ===== DEBUG: Serial Monitor Motor Speeds =====
  // Tuning khatam hone par in lines ko // se comment kar dena
  static uint8_t dbgCount = 0;
  if (++dbgCount >= 25) {          // 250Hz / 25 = 10 baar per second
    dbgCount = 0;
    Serial.print("M1:"); Serial.print((int)MotorInput1);
    Serial.print("\tM2:"); Serial.print((int)MotorInput2);
    Serial.print("\tM3:"); Serial.print((int)MotorInput3);
    Serial.print("\tM4:"); Serial.print((int)MotorInput4);
    Serial.print("\tTHR:"); Serial.print((int)InputThrottle);
    Serial.print("\tIr:"); Serial.print(PrevItermRateRoll, 1);
    Serial.print("\tIp:"); Serial.println(PrevItermRatePitch, 1);
  }

  // ---- 250 Hz Loop Timing ----
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
