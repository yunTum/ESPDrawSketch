#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 各ビーコンで異なる名前を設定
#define BEACON_NAME "ESP32-BEACON-3"  // ビーコン2では "ESP32-BEACON-2" に変更
// #define BEACON_NAME "ESP32-BEACON-2"  // ビーコン2では "ESP32-BEACON-2" に変更
// #define BEACON_NAME "ESP32-BEACON-1"  // ビーコン2では "ESP32-BEACON-2" に変更
#define BEACON_INTERVAL_MS 20  // アドバタイジング間隔（ミリ秒）

BLEAdvertising *pAdvertising;

void setup() {
  Serial.begin(115200);
  
  // ビーコンの初期化
  BLEDevice::init(BEACON_NAME);

  // BLEサーバーの作成
  BLEServer *pServer = BLEDevice::createServer();
  
  // アドバタイジングの設定
  pAdvertising = BLEDevice::getAdvertising();
  
  // アドバタイジングデータの設定
  BLEAdvertisementData advData;
  advData.setName(BEACON_NAME);
  advData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode
  pAdvertising->setAdvertisementData(advData);
  
  // アドバタイジング間隔の設定
  uint16_t interval = (BEACON_INTERVAL_MS / 0.625);
  pAdvertising->setMinInterval(interval);
  pAdvertising->setMaxInterval(interval);
  
  // 送信電力の設定
  esp_power_level_t power_level = ESP_PWR_LVL_P9;  // 最大送信電力
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, power_level);
  
  // アドバタイジング開始
  pAdvertising->start();
  
  Serial.println("ビーコン起動完了");
  Serial.printf("ビーコン名: %s\n", BEACON_NAME);
  Serial.printf("送信間隔: %dms\n", BEACON_INTERVAL_MS);
  Serial.printf("送信電力: %d\n", power_level);
}

void loop() {
  // 定期的にアドバタイジングを再開
  static unsigned long lastRestart = 0;
  if (millis() - lastRestart >= 5000) {  // 5秒ごと
    Serial.println("アドバタイジング再開");
    pAdvertising->stop();
    delay(10);
    pAdvertising->start();
    lastRestart = millis();
  }
  delay(10);
}