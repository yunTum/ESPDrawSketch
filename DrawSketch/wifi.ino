#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP Udp;

// ネットワーク設定
const char* ssid = "XXXXXXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXXXXXX";
String act_ip;
uint16_t seqno = 0;

unsigned int localUdpPort = 4214;  //  port to listen on
unsigned int rcUdpPort = 4215;  //  port to listen on
const char* send_ip = "XXX.XXX.XXX.XXX";
const int sendPacketSize = 21;

uint8_t sendBuffer[sendPacketSize];
bool success = false;

// 符号付きfloat値用
void storeRCFloatSigned(float in, uint8_t * out)
{
  uint16_t converted = abs((int16_t)(in * 100.0));
  uint8_t sign = (in < 0) ? 1 : 0;  // 符号情報
  out[0] = sign;  // 符号を別バイトで送信
  out[1] = (converted >> 8) & 0xFF;  // 上位8ビット
  out[2] = converted & 0xFF;         // 下位8ビット
}

// int16_t型用
void storeRC(int16_t in, uint8_t * out)
{
  out[0] = in>>8;
  out[1] = in&0xFF;
}

void initWifi(){
  // Wi-Fi connection
  WiFi.mode(WIFI_STA);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  act_ip = WiFi.localIP().toString().c_str();
  Serial.println(act_ip);
  // UDP通信開始
  Udp.begin(localUdpPort);
  Serial.printf("UDP port %d\n", localUdpPort);
}

void sendAccData(float AccX, float AccY, float AccZ, float Gx, float Gy, float Gz) 
{
  bool success = Udp.beginPacket(send_ip, rcUdpPort);
  if (!success)
  {
    return;
  }
  // バッファサイズを12バイトに設定
  // 0：0x55
  sendBuffer[0] = 0x55;
  // 1-2：シーケンス番号
  storeRC(seqno++, &sendBuffer[1]);
  // 3-11：加速度値 
  storeRCFloatSigned(AccX, &sendBuffer[3]);
  storeRCFloatSigned(AccY, &sendBuffer[6]);
  storeRCFloatSigned(AccZ, &sendBuffer[9]);
  // 12-19：ジャイロ値
  storeRCFloatSigned(Gx, &sendBuffer[12]);
  storeRCFloatSigned(Gy, &sendBuffer[15]);
  storeRCFloatSigned(Gz, &sendBuffer[18]);

  Udp.write(sendBuffer, sendPacketSize);
  Udp.endPacket();

  // bufferをクリア
  memset(sendBuffer, 0, sizeof(sendBuffer));
  success = false;
}