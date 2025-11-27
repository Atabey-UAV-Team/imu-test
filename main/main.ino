#include <Wire.h>
#include <math.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float AngleRoll = 0.0;
float AnglePitch = 0.0;
float AngleYaw = 0.0;

unsigned long prevMicros = 0;

int16_t AccX_raw, AccY_raw, AccZ_raw;
int16_t GyroX_raw, GyroY_raw, GyroZ_raw;

const float alpha = 0.98;

void readIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 14, true);

  AccX_raw = (Wire.read() << 8) | Wire.read();
  AccY_raw = (Wire.read() << 8) | Wire.read();
  AccZ_raw = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read();

  GyroX_raw = (Wire.read() << 8) | Wire.read();
  GyroY_raw = (Wire.read() << 8) | Wire.read();
  GyroZ_raw = (Wire.read() << 8) | Wire.read();

  RateRoll  = (float)GyroX_raw / 65.5;
  RatePitch = (float)GyroY_raw / 65.5;
  RateYaw   = (float)GyroZ_raw / 65.5;
}

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  RateCalibrationRoll  = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw   = 0;

  Serial.println("Gyro kalibrasyonu basliyor, sensörü masaya birak, dokunma...");
  delay(1000);

  const int samples = 2000;
  for (int i = 0; i < samples; i++) {
    readIMU();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }

  RateCalibrationRoll  /= samples;
  RateCalibrationPitch /= samples;
  RateCalibrationYaw   /= samples;

  Serial.println("Kalibrasyon bitti.");
  Serial.print("Roll offset: ");  Serial.println(RateCalibrationRoll);
  Serial.print("Pitch offset: "); Serial.println(RateCalibrationPitch);
  Serial.print("Yaw offset: ");   Serial.println(RateCalibrationYaw);
  Serial.println("-------------------------");

  prevMicros = micros();
}

void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevMicros) / 1000000.0;
  prevMicros = nowMicros;

  readIMU();

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  float ax = (float)AccX_raw;
  float ay = (float)AccY_raw;
  float az = (float)AccZ_raw;

  float RollAcc  = atan2(ay, az) * 180.0 / PI;
  float PitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

  
  AngleRoll  = alpha * (AngleRoll  + RateRoll  * dt) + (1.0 - alpha) * RollAcc;
  AnglePitch = alpha * (AnglePitch + RatePitch * dt) + (1.0 - alpha) * PitchAcc;
  
  AngleYaw += RateYaw * dt;

  Serial.print("Roll[deg]= ");   Serial.print(AngleRoll, 2);
  Serial.print("  Pitch[deg]= "); Serial.print(AnglePitch, 2);
  Serial.print("  Yaw[deg]= ");   Serial.print(AngleYaw, 2);
  Serial.print("  |  RateRoll= ");  Serial.print(RateRoll, 1);
  Serial.print("  RatePitch= ");    Serial.print(RatePitch, 1);
  Serial.print("  RateYaw= ");      Serial.println(RateYaw, 1);

  delay(20);  // 50 Hz
}
