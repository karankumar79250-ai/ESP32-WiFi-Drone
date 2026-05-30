// ================================================================
//  Measuring Angles (Accelerometer only)
// ================================================================
// Purpose: Calculate Roll and Pitch angles from MPU-6050
//          accelerometer using trigonometry
//
// Use this code for calibration:
//   1. Upload the code
//   2. Place sensor flat on a table
//   3. Open Serial Monitor (115200 baud)
//   4. Note down AccX, AccY, AccZ values
//   5. Calculate calibration values (difference from 0 and 1)
// ================================================================

#include <Wire.h>

// Accelerometer raw values (in g)
float AccX, AccY, AccZ;

// Calculated angles
float AngleRoll, AnglePitch;

// ================================================================
// gyro_signals() — Read accelerometer data + calculate angles
// ================================================================
void gyro_signals(void) {

  // Turn ON low-pass filter (10 Hz cutoff) — filters vibrations
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer: +-8g range → 4096 LSB/g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);  // AFS_SEL = 2
  Wire.endTransmission();

  // Read accelerometer registers 0x3B to 0x40 (6 bytes)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Convert from LSB to g (4096 LSB/g)
  // WARNING: BEFORE CALIBRATION — do not apply any correction
  // First observe these raw values in Serial Monitor
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Calculate Roll and Pitch angle using trigonometry
  // θ_roll  = atan( AccY / sqrt(AccX² + AccZ²) ) × (180/π)
  // θ_pitch = atan( -AccX / sqrt(AccY² + AccZ²) ) × (180/π)
  AngleRoll  =  atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);  // Baud rate
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Wake up MPU-6050 (bring out of sleep mode)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  gyro_signals();

  
  Serial.print("AccX [g]: ");
  Serial.print(AccX);
  Serial.print("  AccY [g]: ");
  Serial.print(AccY);
  Serial.print("  AccZ [g]: ");
  Serial.println(AccZ);

  

  delay(50);
}
