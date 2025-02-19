#include <Wire.h>

// MPU6050のI2Cアドレス
#define MPU6050_ADDR 0x68

// MPU6050のレジスタアドレス
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40
#define ACCEL_CONFIG 0x1C
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define GYRO_CONFIG  0x1B

// 感度設定用の定数
#define ACCEL_FS_2G  0x00  // ±2g  (デフォルト)
#define ACCEL_FS_4G  0x08  // ±4g
#define ACCEL_FS_8G  0x10  // ±8g
#define ACCEL_FS_16G 0x18  // ±16g

// ジャイロの感度設定用の定数
#define GYRO_FS_250  0x00  // ±250°/s
#define GYRO_FS_500  0x08  // ±500°/s
#define GYRO_FS_1000 0x10  // ±1000°/s
#define GYRO_FS_2000 0x18  // ±2000°/s

// スケーリング係数
// float accel_scale = 16384.0;  // デフォルト：±2gの場合
// float gyro_scale = 131.0;   // デフォルト：250dpsの場合
float accel_scale = 1671.8;  // 32767.0/2/9.81    ±2gの場合
float gyro_scale = 131.1;   // 32767.0/250        250dpsの場合

float calibration_x = 0;
float calibration_y = 0;
float calibration_z = 0;
float calibration_gx = 0;
float calibration_gy = 0;
float calibration_gz = 0;
const float threshold = 0.1;

// レジスタに値を書き込む
void writeRegister(byte reg_addr, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}

// 加速度データを読み取る（2バイト）
int16_t readAxis(byte reg_addr) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)2, (bool)true);
  byte high = Wire.read();
  byte low = Wire.read();
  
  return (high << 8) | low;
}

void calibration() {
  float x_g = 0;
  float y_g = 0;
  float z_g = 0;
  float gx_g = 0;
  float gy_g = 0;
  float gz_g = 0;
  for (int i = 0; i < 100; i++) {
    x_g += readAxis(ACCEL_XOUT_H) / accel_scale;
    y_g += readAxis(ACCEL_YOUT_H) / accel_scale;
    z_g += readAxis(ACCEL_ZOUT_H) / accel_scale;
    gx_g += readAxis(GYRO_XOUT_H) / gyro_scale;
    gy_g += readAxis(GYRO_YOUT_H) / gyro_scale;
    gz_g += readAxis(GYRO_ZOUT_H) / gyro_scale;
  }
  x_g /= 100;
  y_g /= 100;
  z_g /= 100;
  gx_g /= 100;
  gy_g /= 100;
  gz_g /= 100;
  calibration_x = x_g;
  calibration_y = y_g;
  calibration_z = z_g;
  calibration_gx = gx_g;
  calibration_gy = gy_g;
  calibration_gz = gz_g;
  Serial.print(calibration_x);
  Serial.print(", ");
  Serial.print(calibration_y);
  Serial.print(", ");
  Serial.print(calibration_z);
  Serial.print(", ");
  Serial.print(calibration_gx);
  Serial.print(", ");
  Serial.print(calibration_gy);
  Serial.print(", ");
  Serial.println(calibration_gz);
}

float getAccelData() {
  float x_g = (readAxis(ACCEL_XOUT_H) / accel_scale) - calibration_x;
  float y_g = (readAxis(ACCEL_YOUT_H) / accel_scale) - calibration_y;
  float z_g = (readAxis(ACCEL_ZOUT_H) / accel_scale) - calibration_z;
  Serial.print(x_g);
  Serial.print(", ");
  Serial.print(y_g);
  Serial.print(", ");
  Serial.println(z_g);

  return x_g, y_g, z_g;
}

float getGyroData() {
  float gx_g = (readAxis(GYRO_XOUT_H) / gyro_scale) - calibration_gx;
  float gy_g = (readAxis(GYRO_YOUT_H) / gyro_scale) - calibration_gy;
  float gz_g = (readAxis(GYRO_ZOUT_H) / gyro_scale) - calibration_gz;
  Serial.print(gx_g);
  Serial.print(", ");
  Serial.print(gy_g);
  Serial.print(", ");
  Serial.println(gz_g);

  return gx_g, gy_g, gz_g;
}

void initMPU6050() {
  // MPU6050の初期化
  // スリープモードを解除
  writeRegister(PWR_MGMT_1, 0x00);

  // 加速度センサーの感度設定
  writeRegister(ACCEL_CONFIG, ACCEL_FS_2G);  // ±2g  (16384 LSB/g)
  // writeRegister(ACCEL_CONFIG, ACCEL_FS_4G);  // ±4g  (8192 LSB/g)
  // writeRegister(ACCEL_CONFIG, ACCEL_FS_8G);     // ±8g  (4096 LSB/g)
  // writeRegister(ACCEL_CONFIG, ACCEL_FS_16G); // ±16g (2048 LSB/g)

  // ジャイロセンサーの感度設定
  writeRegister(GYRO_CONFIG, GYRO_FS_250); // 250dps (131 LSB/dps)

  // スケーリング係数の設定
  // accel_scale = 8192.0;  // ±4gの場合
  // accel_scale = 4096.0;  // ±8gの場合
  // accel_scale = 2048.0;  // ±16gの場合

  calibration();
}

