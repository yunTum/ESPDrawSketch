#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  initWifi();
  initMPU6050();
  delay(100);
}

void loop() {
  // X, Y軸の加速度データを読み取り
  float x_g, y_g, z_g = getAccelData();

  // ジャイロセンサーのデータを読み取り
  float gx_g, gy_g, gz_g = getGyroData();

  // シリアルプロッターに送信
  Serial.print(x_g);
  Serial.print(", ");
  Serial.print(y_g);
  Serial.print(", ");
  Serial.print(z_g);
  Serial.print(", ");
  Serial.print(gx_g);
  Serial.print(", ");
  Serial.print(gy_g);
  Serial.print(", ");
  Serial.println(gz_g);

  sendAccData(x_g, y_g, z_g, gx_g, gy_g, gz_g);
  delay(50);
}