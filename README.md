# ⬡ ESP32 WiFi Drone — Flight Controller

> **No app. No remote. Just open a browser and fly.**
> A fully WiFi-controlled quadcopter built on ESP32 + MPU6050.
> Control from any phone or PC browser — no installation needed.

---

## ✨ Features

- 🌐 **Browser-Based Controller** — open `192.168.4.1` on any device, no app install
- 🎮 **Dual Virtual Joysticks** — touch and mouse support, works on phone and PC
- ✈️ **Acro Mode (Rate Mode)** — manual flying, flips and freestyle
- 🧘 **Angle Mode (Stabilize Mode)** — self-leveling, great for beginners
- 📐 **Kalman Filter** — smooth and accurate angle estimation (gyro + accelerometer fused)
- 🔧 **Live PID Tuning** — change P, I, D values from the browser in real time, no restart needed
- 🔒 **Safety Arm / Disarm** — motors will never spin unless you manually ARM
- ⚡ **250 Hz Control Loop** — fast, responsive flight
- 🧵 **Dual Core Architecture** — WiFi on Core 0, Gyro + PID on Core 1 (mutex protected, no race conditions)
- 📊 **Serial Monitor Debug** — motor speeds and PID values printed 10 times/second for tuning

---

## 📁 Repository Structure

```
ESP32-WiFi-Drone/
│
├── ESP32_WiFi_Controlled_Drone_Code.ino   ← Main flight controller
│
├── ESC_Calibration.ino                   ← Run this ONCE before first flight
│
├── Accelerometer_Calibration.ino         ← Run this for finding AccX, AccY and AccZ for your MPU6050
│
└── README.md
```

---

## 🔧 Hardware Required

| Component | Specification |
|-----------|--------------|
| Microcontroller | ESP32 Dev Module (30-pin or 38-pin) |
| IMU Sensor | MPU6050 (I2C, 3.3V) |
| Motors | 4x BLDC Brushless Motor (1000KV – 2300KV) |
| ESC | 4x ESC (20A or 30A, PWM input) |
| Frame | 210mm or 250mm or 450mm quadcopter frame |
| Battery | 3S or 4S LiPo (1300mAh – 2200mAh) |
| Power Distribution | PDB or 4-in-1 ESC board |
| Propellers | Matched to your motor KV rating |

---

## 📌 Wiring / Pin Connections

```
ESP32 GPIO      →    Component
──────────────────────────────────────
GPIO 25         →    Motor 1 ESC Signal  (Front-Right)
GPIO 14         →    Motor 2 ESC Signal  (Rear-Right)
GPIO 27         →    Motor 3 ESC Signal  (Rear-Left)
GPIO 26         →    Motor 4 ESC Signal  (Front-Left)

GPIO 21 (SDA)   →    MPU6050 SDA
GPIO 22 (SCL)   →    MPU6050 SCL
3.3V            →    MPU6050 VCC
GND             →    MPU6050 GND
GPIO 2          →    Onboard LED (built-in, no wiring needed)
```

### Motor Layout — Top View

```
           FRONT
    M4 (GPIO26)     M1 (GPIO25)
         ↖               ↗
               [ ESP32 ]
         ↙               ↘
    M3 (GPIO27)     M2 (GPIO14)
           BACK

  M1 & M3 → spin Clockwise (CW)
  M2 & M4 → spin Counter-Clockwise (CCW)
```

---

## 📶 WiFi Settings

```
Network Name (SSID) : DroneController
Password            : drone1234
Drone IP Address    : 192.168.4.1
Web Server Port     : 80
```

The ESP32 creates its own WiFi hotspot. Connect your phone or PC to `DroneController`, then open `192.168.4.1` in any browser.

---

## 🚀 Setup Guide — Step by Step

### Step 1 — Install Arduino IDE

Download and install Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)

### Step 2 — Add ESP32 Board Support

1. Open **File → Preferences**
2. In "Additional Board Manager URLs", paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open **Tools → Board → Boards Manager**
4. Search for `esp32` and click **Install**

### Step 3 — Install Required Libraries

Open **Tools → Manage Libraries** and install:

| Library | Author |
|---------|--------|
| `ESP32Servo` | Kevin Harrington |

`Wire`, `WiFi`, and `WebServer` are built-in — no installation needed.

### Step 4 — Select Your Board

Go to **Tools → Board** and select:
```
ESP32 Dev Module
```

---

## 🔌 ESC Calibration — Do This Once

> ⚠️ **REMOVE ALL PROPELLERS before ESC calibration. This is critical.**

ESC calibration teaches each ESC the throttle range (1000µs = min, 2000µs = max). You only need to do this once, or after replacing an ESC.

**How to run:**

1. Open `tools/ESC_Calibration.ino` in Arduino IDE
2. Upload it to your ESP32 (do NOT connect battery yet)
3. Open **Serial Monitor** at `115200 baud`
4. Read the instructions on screen — connect battery when told( you can also connect the battery from begining)
5. The program will automatically send HIGH signal, wait, then LOW signal
6. You will hear ESC beep sequences confirming calibration
7. After calibration completes, the LED blinks slowly (success)
8. **Upload the main drone code** (`firmware/ESP32_WiFi_Controlled_Drone_Code.ino`) — this step is required

**What the ESC Calibration sketch does:**

```
Step 1 → Sends 2000µs (full throttle signal) to all 4 ESCs
         You connect battery here. ESC hears high signal → beeps once or twice.

Step 2 → Sends 1000µs (zero throttle signal) to all 4 ESCs
         ESC sets range: 1000=min, 2000=max → beeps 2-3 times (confirmation).

Step 3 → Runs motors at 20% throttle (1200µs) for 3 seconds
         Verifies all 4 motors respond equally.
         (No propellers! This is just a test.)

Done  → LED blinks slowly. 
```

> If one motor spins slower than others after calibration, run the sketch again.

---

## 🧪 Accelerometer Calibration (AccX, AccY, AccZ)

The MPU6050 accelerometer has a small factory offset — it won't read exactly 0g on X/Y and 1g on Z even when perfectly flat. You need to find your sensor's specific offset values and paste them into the main drone code.

> The main code already has example values:
> ```cpp
> AccX = (float)AccXLSB / 4096 + 0.03;
> AccY = (float)AccYLSB / 4096 + 0.02;
> AccZ = (float)AccZLSB / 4096 + 0.18;
> ```
> **These are values for the developer's specific sensor. Your sensor will be different.**
> If you skip this step, Angle Mode may not work correctly.

**How to find your calibration values:**

Use the accelerometer calibration tool 

1. Upload the accelerometer calibartion code and open Serial Monitor at `115200 baud`
2. Place MPU6050 flat on a level surface
3. Look for AccX, AccY, AccZ readings 
4. Note the values. When flat: AccX and AccY should be near 0, AccZ near 1.0
5. If AccX, AccY and AccZ are found -0.03, -0.02 and 0.82 instead of 0, 0 and 1 respectively.Then you have to calculate your found value - ideal value( -0.03-0 = -0.03, -0.02-0=-0.02 , 0.82-1= -0.18). your correction is: `AccX = (float)AccXLSB / 4096 - your calculated value (  (float)AccXLSB / 4096 + 0.03 ), similarily AccY = (float)AccXLSB / 4096 + 0.02 , AccZ = (float)AccXLSB / 4096 + 0.18.
6. Update these 3 lines in `gyro_signals_angle()` inside the main code:

```cpp
// Find these lines (around line 181-183):
AccX = (float)AccXLSB / 4096 + 0.03;   // replace 0.03 with your value
AccY = (float)AccYLSB / 4096 + 0.02;   // replace 0.02 with your value
AccZ = (float)AccZLSB / 4096 + 0.18;   // replace 0.18 with your value
```

> Acro Mode does NOT use the accelerometer — you can fly Acro Mode without doing this step.

---

## 🎮 Flying — Browser Controller

### Connect and Open

```
1. Remove USB cable (optional — battery powered now)
2. Power on the drone (connect LiPo battery)
3. Wait ~3 seconds for gyro calibration to complete
4. Open WiFi settings on your phone or PC
5. Connect to:  DroneController  (password: drone1234)
6. Open browser and go to:  192.168.4.1
7. The controller page loads instantly
```

### Controller Layout

The page has three tabs:

**ACRO MODE tab**
- Throttle slider (left side, vertical) — controls altitude
- Left joystick — Yaw (rotate left/right)
- Right joystick — Roll (tilt left/right) and Pitch (tilt forward/back)
- ARM / DISARM buttons

**ANGLE MODE tab**
- Same layout as Acro
- Drone self-levels — joystick controls the angle, not rotation rate
- Recommended for beginners and stable hovering

**PID TUNE tab**
- Adjust P, I, D values for Roll/Pitch/Yaw live
- Changes apply instantly to ESP32 — no restart needed
- Step size selector: choose how much each +/- button changes the value

### Arming Procedure

> **Motors will NOT spin until you ARM. This is a safety feature.**

```
1. Make sure throttle slider is at MINIMUM (bottom)
2. Press ARM button
3. Status dot turns GREEN → ARMED
4. Slowly raise throttle — motors begin spinning
5. Press DISARM at any time to cut motors immediately
```

---

## 🔧 PID Tuning Guide

PID tuning makes the drone fly smoothly and respond correctly. Tune in Acro Mode first.

### Default Values

```
Rate PID — Roll / Pitch:   P = 0.6    I = 3.5    D = 0.03
Rate PID — Yaw:            P = 2.0    I = 12.0   D = 0.0
Angle PID — Roll / Pitch:  P = 2.0    I = 0.0    D = 0.0
```

### Tuning Tips

| Symptom | What to do |
|---------|-----------|
| Drone oscillates / shakes fast | Lower P value |
| Drone responds too slowly | Raise P value |
| Drone drifts slowly in one direction | Raise I value slightly |
| High frequency vibration / buzz | Lower D value |
| Overcorrects and bounces back | Lower P, lower D |
| Feels sluggish in Angle Mode | Raise Angle P value |

**Recommended order:** Tune Rate P first → then Rate D → then Rate I → finally Angle P
                    ** If any motor run faster than other and starts heating , then make all I value 0.

---

## 🛡️ Safety Features

| Feature | How it works |
|---------|-------------|
| **Manual Arm required** | Motors never spin on boot — you must press ARM |
| **Throttle cutoff** | If throttle drops below 1050µs, all motors cut immediately |
| **Motor max limit** | Motors are capped at 1999µs — can never exceed safe range |
| **Motor idle** | When armed at zero throttle, motors run at 1180µs (idle, not full stop) |
| **Gyro calibration on boot** | ESP32 calibrates gyroscope every time it starts — keep drone still for 3 seconds after power-on |
| **Dual core mutex** | WiFi commands and flight loop are separated — no data corruption at high speeds |

---

## 🔍 Serial Monitor Debug

While the drone is connected via USB and flying, Serial Monitor (115200 baud) shows:

```
M1:1320  M2:1285  M3:1310  M4:1295  THR:1400  Ir:0.2  Ip:-0.1
```

| Field | Meaning |
|-------|---------|
| M1–M4 | Motor speeds in microseconds (1000=off, 2000=full) |
| THR | Current throttle input |
| Ir | I-term accumulation for Roll |
| Ip | I-term accumulation for Pitch |

These print 10 times per second. Comment out the debug lines in `loop()` for final builds.

---

## ⚙️ Code Architecture

```
setup()
├── Gyro calibration (2000 samples, ~2 seconds)
├── WiFi Access Point start
├── Web server routes registered (/  /cmd  /pid)
└── WiFiTask started on Core 0

Core 0 — WiFiTask (runs forever)
└── server.handleClient() — receives browser commands

Core 1 — loop() at 250 Hz (every 4000µs)
├── Read gyro (Acro) or gyro + accelerometer (Angle)
├── Apply gyro calibration offsets
├── If not Armed → motors off, wait, return
├── Take mutex → copy ReceiverValues → give mutex
├── Acro Mode: calculate desired rates from joystick
│   └── Run Rate PID (Roll, Pitch, Yaw)
├── Angle Mode: run Kalman filter
│   ├── Outer loop: Angle PID → desired rate
│   └── Inner loop: Rate PID (Roll, Pitch, Yaw)
├── Motor mixing (throttle ± roll ± pitch ± yaw)
├── Apply motor limits (1180 min, 1999 max)
├── Throttle cutoff check
└── Write to motors
```

---

## ❓ Troubleshooting

**Browser page does not load**
- Make sure you are connected to `DroneController` WiFi, not your home WiFi
- Type `192.168.4.1` exactly — do not add `https://`
- Try a different browser

**Drone drifts after arming**
- Gyro was not calibrated properly — drone was moved during the first 3 seconds after power-on
- Power cycle and keep drone completely still on startup

**One motor slower than others**
- Re-run `ESC_Calibration.ino`
- Check ESC signal wire connections

**Angle Mode flies unstable**
- AccX/AccY/AccZ calibration values in the code are for a different sensor
- Find your sensor's values and update lines 181–183 in `gyro_signals_angle()`

**Motors spin immediately after power-on (no ARM)**
- This should never happen with this firmware — ARM is required
- Check that you uploaded the correct code

**WiFi drops during flight**
- ESP32 dual-core architecture prevents this
- If it still happens, check power supply to ESP32 (voltage sag from motors)

---

## ⚠️ Disclaimer

This project is for **educational and hobby purposes only.**

- Always fly in open areas away from people and property
- Remove propellers whenever uploading code or testing on the bench
- Follow your local drone regulations and laws
- The author is not responsible for any damage or injury caused by this project
- **Always test indoors with propellers removed first**

---

## 🤝 Contributing

Pull requests are welcome. If you find a bug or want to add a feature:

1. Fork this repository
2. Create a new branch: `git checkout -b feature/your-feature-name`
3. Make your changes and commit
4. Open a Pull Request

---

## 📄 License

MIT License — free to use, modify, and share with attribution.

---

## 👤 Author

**Karan Kumar** — Firmware & Flight Controller

**Rakesh Chaurasiya** — Hardware & Assembly

**Ankit Kumar** — Testing & Documentation

If this project helped you build your drone, please give it a ⭐ **Star** on GitHub!

*Built with ❤️ in India 🇮🇳*
