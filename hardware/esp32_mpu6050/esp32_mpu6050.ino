// ESP32 + MPU6050 vibration rig: streams accel (m/s^2) and gyro (deg/s) at 500 Hz over USB.
// Wiring: MPU6050 VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22.
// Output line: t_us,ax,ay,az,gx,gy,gz   (921600 baud)
#include <Wire.h>

#define MPU 0x68
#define FS_HZ 500

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(921600);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  writeReg(0x6B, 0x00);   // wake up
  writeReg(0x19, 0x01);   // sample rate divider: 1 kHz / (1+1) = 500 Hz
  writeReg(0x1A, 0x00);   // internal DLPF off (260 Hz) so our own filters do the work
  writeReg(0x1B, 0x08);   // gyro +-500 deg/s
  writeReg(0x1C, 0x08);   // accel +-4 g
  delay(100);
  Serial.println("t_us,ax,ay,az,gx,gy,gz");
}

void loop() {
  static uint32_t next = micros();
  while ((int32_t)(micros() - next) < 0) {}
  next += 1000000UL / FS_HZ;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14);
  int16_t v[7];
  for (int i = 0; i < 7; i++) v[i] = (Wire.read() << 8) | Wire.read();

  const float aS = 9.81f / 8192.0f;   // +-4 g
  const float gS = 1.0f / 65.5f;      // +-500 deg/s
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", micros(),
                v[0] * aS, v[1] * aS, v[2] * aS, v[4] * gS, v[5] * gS, v[6] * gS);
}
