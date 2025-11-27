#include <Wire.h>

const int MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Uyku modundan çıkar
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // WHO_AM_I oku
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);

  if (Wire.available()) {
    uint8_t id = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    Serial.println(id, HEX);

    if (id == 0x68) {
      Serial.println("Cihaz: MPU6050 (accel+gyro, 6 eksen)");
    } else if (id == 0x70) {
      Serial.println("Cihaz: MPU6500 (accel+gyro, 6 eksen)");
    } else {
      Serial.println("Beklenmeyen ID, baglantilari veya sensor tipini kontrol et.");
    }
  } else {
    Serial.println("WHO_AM_I okunamadi.");
  }
}

void loop() {}
