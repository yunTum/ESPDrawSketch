#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

Madgwick MadgwickFilter;
// WebSocket設定
const char* wsHost = "XXX.XXX.XXX.XXX";  // PCのIPアドレス
const int wsPort = 8080;

WebSocketsClient webSocket;
BLEScan* pBLEScan;
static bool isScanning = false;

// データ構造体
struct SensorData {
  float acc[3];
  float gyro[3];
  int rssi_beacon1;
  int rssi_beacon2;
  int rssi_beacon3;
  float roll;
  float pitch;
  float yaw;
  unsigned long timestamp;
};

struct AccelData {
    float x, y, z;
};

struct GyroData {
    float x, y, z;
};

float ROLL, PITCH, YAW;

AccelData getAccelData();
GyroData getGyroData();

SensorData sensorData;
// メモリ使用量を削減するためのJSONドキュメントサイズ
StaticJsonDocument<128> doc;

// BLEスキャンのコールバック
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (!advertisedDevice.haveName()) return;  // 名前がない場合はスキップ
    String name = advertisedDevice.getName().c_str();
    int rssi = advertisedDevice.getRSSI();
    // ビーコンからの信号のみを処理
    if (name == "ESP32-BEACON-1" || name == "ESP32-BEACON-2" || name == "ESP32-BEACON-3") {
      Serial.printf("ビーコン検出: %s (RSSI: %d)\n", name.c_str(), rssi);
      
      if (name == "ESP32-BEACON-1") {
        sensorData.rssi_beacon1 = rssi;
      } else if (name == "ESP32-BEACON-2") {
        sensorData.rssi_beacon2 = rssi;
      } else if (name == "ESP32-BEACON-3") {
        sensorData.rssi_beacon3 = rssi;
      }
    }

    // スキャン完了を示すフラグを設定
    isScanning = false;
    advertisedDevice.getScan()->stop();
  }
};

void setupWebSocket() {
  Serial.println("WebSocket Setup Start!");
  webSocket.begin(wsHost, wsPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  // プロトコルバージョンを13に設定（最新のWebSocket仕様）
  webSocket.enableHeartbeat(15000, 3000, 2);
  Serial.println("WebSocket Setup Complete!");
}

void setupBLE() {
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(49);
  Serial.println("BLE Setup Complete!");
}

void sendSensorData() {
  doc.clear();
  doc["acc_x"] = sensorData.acc[0];
  doc["acc_y"] = sensorData.acc[1];
  doc["acc_z"] = sensorData.acc[2];
  doc["gyro_x"] = sensorData.gyro[0];
  doc["gyro_y"] = sensorData.gyro[1];
  doc["gyro_z"] = sensorData.gyro[2];
  // doc["rssi1"] = sensorData.rssi_beacon1;
  // doc["rssi2"] = sensorData.rssi_beacon2;
  // doc["rssi3"] = sensorData.rssi_beacon3;
  doc["roll"] = sensorData.roll;
  doc["pitch"] = sensorData.pitch;
  doc["yaw"] = sensorData.yaw;
  doc["timestamp"] = sensorData.timestamp;
  
  String jsonString;
  serializeJson(doc, jsonString);
  // Serial.println("Sending: " + jsonString);
  webSocket.sendTXT(jsonString);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected!");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected!");
      break;
    case WStype_TEXT:
      // Serial.println("WebSocket Received!");
      break;
    case WStype_ERROR:
      Serial.println("WebSocket Error!");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  initWifi();
  initMPU6050();
  setupWebSocket();
  // setupBLE();
  MadgwickFilter.begin(20);
  MadgwickFilter.setGain(0.4);
}

void loop() {
  static unsigned long lastCheck = 0;
  static unsigned long lastScan = 0;
  unsigned long currentMillis = millis();
  static unsigned long lastDataUpdate = 0;

  // WiFi接続チェック
  if (currentMillis - lastCheck >= 5000) {
    if (!checkWiFiConnection()) {
      Serial.println("WiFi接続エラー。再起動します...");
      ESP.restart();
      return;
    }
    lastCheck = currentMillis;
  }

  // WebSocket処理
  webSocket.loop();

  // // BLEスキャン（100msごと）
  // if (currentMillis - lastScan >= 100) {
  //   // 前回のスキャン結果をクリア
  //   pBLEScan->clearResults();
  //   // スキャン開始（0.1秒間）
  //   pBLEScan->start(0.5, false);
  //   isScanning = true;
  //   lastScan = currentMillis;
  // }

  // データ更新と送信（50msごと）
  if (currentMillis - lastDataUpdate >= 50) {
    updateAndSendData(currentMillis);
    lastDataUpdate = currentMillis;
  }
  // 短い遅延
  // delay(10);
}

void updateAndSendData(unsigned long currentMillis) {
  // IMUデータの読み取り
  AccelData accel = getAccelData();
  GyroData gyro = getGyroData();
  // Madgwickフィルタを使用して姿勢を更新
  MadgwickFilter.updateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);
  ROLL = MadgwickFilter.getRoll(); // ロール角 degree
  PITCH = MadgwickFilter.getPitch(); // ピッチ角 degree
  YAW  = MadgwickFilter.getYaw(); // ヨー角 degree
  // データの更新
  sensorData.acc[0] = accel.x;
  sensorData.acc[1] = accel.y;
  sensorData.acc[2] = accel.z;
  sensorData.gyro[0] = gyro.x;
  sensorData.gyro[1] = gyro.y;
  sensorData.gyro[2] = gyro.z;
  sensorData.roll = ROLL;
  sensorData.pitch = PITCH;
  sensorData.yaw = YAW;
  sensorData.timestamp = currentMillis;
  Serial.printf("Accel: %f, %f, %f\n", accel.x, accel.y, accel.z);
  // Serial.printf("%f, %f, %f\n", accel.x, accel.y, accel.z);
  Serial.printf("Gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);
  Serial.printf("Roll: %f, Pitch: %f, Yaw: %f\n", ROLL, PITCH, YAW);

  // WebSocket接続中ならデータを送信
  if (webSocket.isConnected()) {
    sendSensorData();
  }
}


