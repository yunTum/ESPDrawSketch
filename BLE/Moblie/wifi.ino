#include <WiFi.h>

// ネットワーク設定
const char* ssid = "JCOM_XBUW";
const char* password = "681230514672";
String act_ip;

void initWifi(){
  // Wi-Fi connection
  WiFi.mode(WIFI_STA);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);

  // 接続試行回数を設定
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(ssid, password);
  // 接続待機
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi接続成功!");
    act_ip = WiFi.localIP().toString();
    Serial.print("IPアドレス: ");
    Serial.println(act_ip);
    Serial.print("信号強度(RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\nWiFi接続失敗");
    ESP.restart();  // 接続失敗時は再起動
  }

  // 接続後の安定化待ち
  delay(1000);
}

// WiFi接続状態を確認する関数
bool checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi接続が切断されました。再接続を試みます...");
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi再接続成功!");
      return true;
    } else {
      Serial.println("\nWiFi再接続失敗");
      return false;
    }
  }
  return true;
}