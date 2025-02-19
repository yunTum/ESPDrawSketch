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

// DLPFの設定用の定数
#define CONFIG_REG   0x1A
#define DLPF_CFG_0   0x00    // 帯域幅 ACC:260Hz GYRO:256Hz
#define DLPF_CFG_1   0x01    // 帯域幅 ACC:184Hz GYRO:188Hz
#define DLPF_CFG_2   0x02    // 帯域幅 ACC:94Hz  GYRO:98Hz
#define DLPF_CFG_3   0x03    // 帯域幅 ACC:44Hz  GYRO:42Hz
#define DLPF_CFG_4   0x04    // 帯域幅 ACC:21Hz  GYRO:20Hz
#define DLPF_CFG_5   0x05    // 帯域幅 ACC:10Hz  GYRO:10Hz
#define DLPF_CFG_6   0x06    // 帯域幅 ACC:5Hz   GYRO:5Hz

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
// float accel_scale = 1671.8;  // 32767.0/2/9.81  1g:16384 → 1m/s^2:1671.8  ±2gの場合
// float accel_scale = 16384.0;  // 32767.0/2/9.81  1g:16384 → 1m/s^2:16384  ±2gの場合
float accel_scale = 4096.0;  // 32767.0/8/9.81  1g:4096 → 1m/s^2:4096  ±8gの場合
float gyro_scale = 131.1;   // 32767.0/250  1dps:131.1 → 1°/s:131.1  250dpsの場合

float calibration_x = 0;
float calibration_y = 0;
float calibration_z = 0;
float calibration_gx = 0;
float calibration_gy = 0;
float calibration_gz = 0;

const int calibration_count = 255;

AccelData getAccelData();
GyroData getGyroData();

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
  float x_g = 0, y_g = 0, z_g = 0;
  float gx_g = 0, gy_g = 0, gz_g = 0;

    // キャリブレーションデータの収集
    for (int i = 0; i < calibration_count; i++) {
        // 加速度のキャリブレーション
        // 理想的には、Z軸は1g（重力加速度）を示し、X軸とY軸は0gを示すはずです
        x_g += readAxis(ACCEL_XOUT_H) / accel_scale;
        y_g += readAxis(ACCEL_YOUT_H) / accel_scale;
        z_g += (readAxis(ACCEL_ZOUT_H) / accel_scale) - 1.0;  // 重力加速度を考慮

        // ジャイロのキャリブレーション（静止時は0になるはず）
        gx_g += readAxis(GYRO_XOUT_H) / gyro_scale;
        gy_g += readAxis(GYRO_YOUT_H) / gyro_scale;
        gz_g += readAxis(GYRO_ZOUT_H) / gyro_scale;

        // サンプリング間隔を設定
        delay(5);
    }
    // 平均値の計算
    calibration_x = x_g / calibration_count;
    calibration_y = y_g / calibration_count;
    calibration_z = z_g / calibration_count;
    calibration_gx = gx_g / calibration_count;
    calibration_gy = gy_g / calibration_count;
    calibration_gz = gz_g / calibration_count;
  #if DEBUG
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
  #endif

  delay(500);
}

// グローバル変数として追加
float prev_x = 0, prev_y = 0, prev_z = 0;
float filtered_x = 0, filtered_y = 0, filtered_z = 0;
AccelData getAccelData() {
    float x_g = (readAxis(ACCEL_XOUT_H) / accel_scale) - calibration_x;
    float y_g = (readAxis(ACCEL_YOUT_H) / accel_scale) - calibration_y;
    float z_g = (readAxis(ACCEL_ZOUT_H) / accel_scale) - calibration_z;

    // ローパスフィルター係数（重力成分の抽出）
    const float alpha = 0.8;
    
    // ローパスフィルターで重力成分を抽出
    filtered_x = alpha * filtered_x + (1 - alpha) * x_g;
    filtered_y = alpha * filtered_y + (1 - alpha) * y_g;
    filtered_z = alpha * filtered_z + (1 - alpha) * z_g;

    // 重力成分を除去した実際の加速度を計算
    float real_x = x_g - filtered_x;
    float real_y = y_g - filtered_y;
    float real_z = z_g - filtered_z;
    #if DEBUG
        Serial.print(x_g);
        Serial.print(", ");
        Serial.print(y_g);
        Serial.print(", ");
        Serial.println(z_g);
    #endif

    return {real_x, real_y, real_z};
}

GyroData getGyroData() {
    float gx_g = (readAxis(GYRO_XOUT_H) / gyro_scale) - calibration_gx;
    float gy_g = (readAxis(GYRO_YOUT_H) / gyro_scale) - calibration_gy;
    float gz_g = (readAxis(GYRO_ZOUT_H) / gyro_scale) - calibration_gz;
    #if DEBUG
        Serial.print(gx_g);
        Serial.print(", ");
        Serial.print(gy_g);
        Serial.print(", ");
        Serial.println(gz_g);
    #endif

    return {gx_g, gy_g, -gz_g};
}

void initMPU6050() {
  // MPU6050の初期化
  // スリープモードを解除
  writeRegister(PWR_MGMT_1, 0x00);
  delay(50);
  // デジタルローパスフィルターの設定
  writeRegister(CONFIG_REG, DLPF_CFG_4);

  // 加速度センサーの感度設定
  writeRegister(ACCEL_CONFIG, ACCEL_FS_8G);  // ±2g  (16384 LSB/g)
  delay(50);
  // ジャイロセンサーの感度設定
  writeRegister(GYRO_CONFIG, GYRO_FS_250); // 250dps (131 LSB/dps)
  delay(50);
  calibration();
}

