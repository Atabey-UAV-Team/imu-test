#include <Wire.h>
#include <math.h>

float RateRoll, RatePitch, RateYaw; // Açısal hız değerleri (deg/s)
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0; // Kalibre sırasında kullanılan değişkenler

// Filtre sonrası çıkış değerleri
float AngleRoll = 0.0;
float AnglePitch = 0.0;
float AngleYaw = 0.0;

unsigned long prevMicros = 0; // Sabit örneklenme frekansı sağlamak için değişken

// MPU6050'den okunan 16-bit integer değerleri
int16_t AccX_raw, AccY_raw, AccZ_raw;
int16_t GyroX_raw, GyroY_raw, GyroZ_raw;

// Filtre katsayısı
const float alpha = 0.98;

// IMU sensöründen (MPU6050) Accel 3-axis ve Gyro 3-axis değerlerini okuma fonksiyonu
void readIMU() {
  Wire.beginTransmission(0x68); // I2C protokolünü başlat
  Wire.write(0x3B); // Okumaya başlanacak ilk byte (ACCEL_XOUT_H)
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 14, true); // Önceki okunan bitin sonrasındaki 14 biti okumak

  // Accelerometer (X, Y, Z)
  AccX_raw = (Wire.read() << 8) | Wire.read(); // 0x3B + 0x3C
  AccY_raw = (Wire.read() << 8) | Wire.read(); // 0x3D + 0x3E
  AccZ_raw = (Wire.read() << 8) | Wire.read(); // 0x3F + 0x40

  Wire.read(); Wire.read(); // Accelerometer ve Gyroscope çıkış interruptları arasındaki registerleri atlamak için kullanılan blok (Optimize edilmeli)

  // Gyroscope (X, Y, Z)
  GyroX_raw = (Wire.read() << 8) | Wire.read(); // 0x43 + 0x44
  GyroY_raw = (Wire.read() << 8) | Wire.read(); // 0x45 + 0x46
  GyroZ_raw = (Wire.read() << 8) | Wire.read(); // 0x47 + 0x48

  // +-500deg/s aralığında çalıştığı için 
  RateRoll  = (float)GyroX_raw / 65.5;  
  RatePitch = (float)GyroY_raw / 65.5;
  RateYaw   = (float)GyroZ_raw / 65.5;
}

void setup() {
  Serial.begin(57600); // Seri port başlatma bloğu
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // Sensör ön ayarları
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Wire.write(0x00); // Sensörü uyku modundan çıkart
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // CONFIG Register
  Wire.write(0x05); // 10 Hz çalışma (DLPF_CFG = 5 / 101 (Binary) )
  Wire.endTransmission();

 // Gyroscope aralığının belirlenmesi
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Gyroscope Config
  Wire.write(0x08); // FS_SEL = 1 (±500 deg/s)
  Wire.endTransmission();

  Serial.println("Gyro kalibrasyonu basliyor, sensörü masaya birak, dokunma...");
  delay(1000);

  const int samples = 2000; // Kalibrasyon için örnek sayısı
  for (int i = 0; i < samples; i++) {
    readIMU();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }

  // Sıfır kalibrasyonu için 3 eksen veri ortalaması
  RateCalibrationRoll  /= samples;
  RateCalibrationPitch /= samples;
  RateCalibrationYaw   /= samples;

  // Kalibrasyon bilgileri
  Serial.println("Kalibrasyon bitti.");
  Serial.print("Roll offset: ");  Serial.println(RateCalibrationRoll);
  Serial.print("Pitch offset: "); Serial.println(RateCalibrationPitch);
  Serial.print("Yaw offset: ");   Serial.println(RateCalibrationYaw);
  Serial.println("-------------------------");

  prevMicros = micros();
}

void loop() {
  // Sabit Örnekleme
  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevMicros) / 1000000.0;
  prevMicros = nowMicros;

  readIMU(); // IMU'dan 6 Eksen verisini almak için fonksiyon

  // Hız ölçümü için 0 noktasına resetleme
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // Açı işlemlerinde kullanmak için kesirli sayıya çevirdik
  float ax = (float)AccX_raw;
  float ay = (float)AccY_raw;
  float az = (float)AccZ_raw;

  float RollAcc = atan2((float)AccY_raw, (float)AccZ_raw); // Roll açısının referansını Accelerometre verileri ile hesaplanması
  float PitchAcc = atan2(-(float)AccX_raw, sqrt((float)AccY_raw * (float)AccY_raw + (float)AccZ_raw * (float)AccZ_raw) * 180.0 / PI; // Pitch açısının Accelerometre verileri ile hesaplanması

  // Complementary Filter
  AngleRoll  = alpha * (AngleRoll  + RateRoll  * dt) + (1.0 - alpha) * RollAcc;
  AnglePitch = alpha * (AnglePitch + RatePitch * dt) + (1.0 - alpha) * PitchAcc;
  
  AngleYaw += RateYaw * dt;

  // Verilerin Seri porta yazdırılması
  Serial.print("Roll[deg]= ");   Serial.print(AngleRoll, 2);
  Serial.print("  Pitch[deg]= "); Serial.print(AnglePitch, 2);
  Serial.print("  Yaw[deg]= ");   Serial.print(AngleYaw, 2);
  Serial.print("  |  RateRoll= ");  Serial.print(RateRoll, 1);
  Serial.print("  RatePitch= ");    Serial.print(RatePitch, 1);
  Serial.print("  RateYaw= ");      Serial.println(RateYaw, 1);

  delay(20);  // 50 Hz
}
