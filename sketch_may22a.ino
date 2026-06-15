/**
 * 智能物联网家居系统 - 完整版
 * 平台: FireBeetle Board-ESP32 + 华为云IoT
 * 传感器: DHT11温湿度、土壤湿度、红外热释电(人体)、黄色按键
 * 执行器: 红色LED、继电器、电磁锁、功放喇叭
 *
 * 联动逻辑:
 *   PIR+布防ON → 喇叭响+LED闪+继电器1锁死+推送"入侵"
 *   PIR+布防OFF → 仅推送"有人经过"
 *   温度>30+布防OFF → 喇叭蜂鸣+推送
 *   温度>30+布防ON → 仅推送
 *   土壤<30%(不限) → LED常亮+继电器2吸合+推送
 *   按键+布防OFF → 切换继电器1
 *   按键+布防ON → 无效(锁死)
 */

#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

WiFiServer httpServer(80);  // HTTP 服务，浏览器直接连 ESP32

// ==================== WiFi 配置 ====================
const char* ssid = "wuhu";
const char* password = "asdfghjkl";

// ==================== 华为云 IoT MQTT 配置 ====================
const char* mqttHost = "b6498afe8e.st1.iotda-device.cn-north-4.myhuaweicloud.com";
const int   mqttPort = 1883;

const char* deviceId  = "6a100c027f2e6c302f735aa0_esp32_002";
const char* clientId  = "6a100c027f2e6c302f735aa0_esp32_002_0_0_2026061317";
const char* username  = "6a100c027f2e6c302f735aa0_esp32_002";
const char* mqttPassword = "6a716ba215a2cd66c40bf9ecc09987a445ae91da96561bcec973cc4eb83b23ce";

// MQTT 主题
String topicReport;     // 属性上报
String topicCmd;        // 命令下发（订阅）
String topicSet;        // 属性设置（订阅）
String topicGet;        // 属性获取（订阅）
String topicMsgDown;    // 消息下发（订阅）
String topicSetResp;    // 属性设置响应
String topicGetResp;    // 属性获取响应
String topicMsgUp;      // 消息上报

// ==================== 引脚定义（已按扩展板背面标注修正） ====================
// 安全端口: D2(25) D3(26) D4(27) D7(13) D8(5) A0(36)
// ⚠️ D5(9) D6(10) 是ESP32 Flash引脚，禁用！
#define DHTPIN        26   // DHT11 → D3 (GPIO26，无启动干扰)
#define SOIL_PIN      36   // 土壤湿度 → A0 (GPIO36)
#define PIR_PIN       2    // 红外热释电 → D9 (GPIO2)
#define BUTTON_PIN    5    // 黄色按键 → D8 (GPIO5)
#define LED_PIN       13   // 红色LED → D7 (GPIO13)
#define RELAY_PIN     25   // 继电器1 → D2 (GPIO25, PIR/按键控制)
// 电磁锁通过继电器1螺丝端子(COM/NO)连接
#define RELAY2_PIN    0    // 继电器2 → D0 (GPIO0, 土壤自动)
#define BUZZER_PIN    27   // 功放喇叭 → D4 (GPIO27)

// ==================== 阈值配置 ====================
#define TEMP_THRESHOLD      30.0   // 温度阈值(°C)，超过则打开继电器
#define SOIL_THRESHOLD      30.0   // 土壤湿度阈值(%)，低于则LED警告
#define ALARM_DURATION     5000    // 报警持续时间(ms)
#define REPORT_INTERVAL    5000    // 属性上报间隔(ms)
#define PING_INTERVAL      30000   // MQTT心跳间隔(ms)

// WxPusher 推送配置（免费无限制）
const char* wxAppToken = "AT_PalYfm2J6QpIh07KyJ70PF0WtpKro6Bf";
const char* wxUid = "UID_Z5g0tVn2awK3ARQoHwTVU6NqkBjT";

// ==================== 全局状态变量 ====================
WiFiClient tcpClient;        // 华为云 MQTT
WiFiClient pubClient;        // 公网 MQTT (broker.emqx.io)
WiFiClient bemfaWifi;        // 巴法云 MQTT
PubSubClient bemfaMqtt(bemfaWifi);

// 公网 MQTT 配置（无需密码，全球可访问）
const char* pubHost = "broker.emqx.io";
const int   pubPort = 1883;
const char* pubTopicStatus  = "smarthome/esp32_002/status";
const char* pubTopicControl = "smarthome/esp32_002/control";
bool pubConnected = false;
uint16_t pubPid = 1;

// 巴法云 MQTT 配置（小爱同学语音控制）
const char* bmUid = "bca72863b8b1f2811641a64085c45fa7";
const char* bmServer = "bemfa.com";
const int   bmPort = 9501;
// 巴法云5个主题
const char* T_LED    = "YzxiyyBvk002";
const char* T_BUZZER = "nG9QPACuQ002";
const char* T_TEMP   = "UNp3tB8R0002";
const char* T_HUM    = "Gnl7EopOB002";
const char* T_SOIL   = "jfxLyHahw002";
bool bmConnected = false;
// 前置声明
void setLED(bool); void setBuzzer(bool); void setRelay(bool); void setRelay2(bool); void setLock(bool);

// 传感器数据
float temperature   = 0.0;
float humidity      = 0.0;
int   soilMoisture  = 0;
bool  motionDetected = false;

// 执行器状态
bool ledState    = false;   // LED 状态
bool relayState  = false;   // 继电器1 状态(PIR/按键)
bool relay2State = false;   // 继电器2 状态(土壤自动)
bool lockState   = false;   // 电磁锁状态
bool buzzerState = false;   // 喇叭状态
bool alarmActive = false;   // 报警系统激活/布防
bool autoMode    = true;    // 自动联动模式

// 报警计时
unsigned long alarmStartTime = 0;
bool alarmTriggered = false;

// 按钮去抖
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
#define DEBOUNCE_DELAY 50

// MQTT 状态
bool mqttConnected = false;
uint16_t packetId = 1;          // MQTT 包ID（递增）
unsigned long lastReportTime = 0;
unsigned long lastPingTime = 0;
unsigned long lastMqttActivity = 0;

// 订阅状态
bool subAcked = false;

// ==================== MQTT 底层协议实现 ====================

/**
 * 读取 MQTT 剩余长度（可变长度编码）
 * 返回: 剩余长度值，-1 表示错误
 */
int readRemainingLength() {
  int value = 0;
  int multiplier = 1;
  uint8_t byte;
  for (int i = 0; i < 4; i++) {
    if (!tcpClient.available()) return -1;
    byte = tcpClient.read();
    value += (byte & 0x7F) * multiplier;
    if ((byte & 0x80) == 0) break;
    multiplier *= 128;
    if (multiplier > 128 * 128 * 128) return -1;
  }
  return value;
}

/**
 * 编码剩余长度为可变长度格式
 * 返回: 编码后的字节数
 */
int encodeRemainingLength(uint8_t* buf, int length) {
  int idx = 0;
  do {
    uint8_t byte = length % 128;
    length /= 128;
    if (length > 0) byte |= 0x80;
    buf[idx++] = byte;
  } while (length > 0);
  return idx;
}

/**
 * MQTT CONNECT - 连接到华为云 IoT（已验证可用）
 */
bool mqttConnect() {
  if (!tcpClient.connect(mqttHost, mqttPort)) {
    Serial.println("[MQTT] TCP 连接失败");
    return false;
  }
  Serial.println("[MQTT] TCP 已连接，发送 CONNECT...");

  uint16_t clientIdLen = strlen(clientId);
  uint16_t usernameLen = strlen(username);
  uint16_t passwordLen = strlen(mqttPassword);

  // 可变头: 协议名(6) + 协议级别(1) + 连接标志(1) + 保活(2) = 10
  uint16_t varHeaderSize = 10;
  uint16_t payloadSize = 2 + clientIdLen + 2 + usernameLen + 2 + passwordLen;
  uint16_t remainingLen = varHeaderSize + payloadSize;

  uint8_t packet[256];
  int idx = 0;

  // 固定头: CONNECT = 0x10
  packet[idx++] = 0x10;

  // 剩余长度
  idx += encodeRemainingLength(packet + idx, remainingLen);

  // 可变头: 协议名 "MQTT"
  packet[idx++] = 0x00; packet[idx++] = 0x04;
  packet[idx++] = 'M';  packet[idx++] = 'Q';
  packet[idx++] = 'T';  packet[idx++] = 'T';

  // 协议级别 (MQTT 3.1.1)
  packet[idx++] = 0x04;

  // 连接标志: 用户名 + 密码 + 清除会话 (0xC2 = 1100 0010)
  packet[idx++] = 0xC2;

  // 保活时间 60 秒
  packet[idx++] = 0x00; packet[idx++] = 0x3C;

  // Payload: Client ID
  packet[idx++] = (clientIdLen >> 8) & 0xFF;
  packet[idx++] = clientIdLen & 0xFF;
  memcpy(packet + idx, clientId, clientIdLen);
  idx += clientIdLen;

  // Username
  packet[idx++] = (usernameLen >> 8) & 0xFF;
  packet[idx++] = usernameLen & 0xFF;
  memcpy(packet + idx, username, usernameLen);
  idx += usernameLen;

  // Password
  packet[idx++] = (passwordLen >> 8) & 0xFF;
  packet[idx++] = passwordLen & 0xFF;
  memcpy(packet + idx, mqttPassword, passwordLen);
  idx += passwordLen;

  tcpClient.write(packet, idx);
  Serial.printf("[MQTT] 已发送 %d 字节, remainingLen=%d\n", idx, remainingLen);

  // 等待 CONNACK
  unsigned long t = millis();
  while (!tcpClient.available() && millis() - t < 5000) { delay(10); }
  if (!tcpClient.available()) {
    Serial.println("[MQTT] 未收到 CONNACK");
    return false;
  }

  uint8_t resp = tcpClient.read();
  if (resp != 0x20) {
    Serial.printf("[MQTT] 异常响应: 0x%02X\n", resp);
    return false;
  }

  int rl = readRemainingLength();
  if (rl < 2) {
    Serial.println("[MQTT] CONNACK 长度异常");
    return false;
  }

  tcpClient.read(); // 确认标志
  uint8_t rc = tcpClient.read();

  if (rc == 0) {
    Serial.println("[MQTT] ✅ CONNECT 成功!");
    lastMqttActivity = millis();
    return true;
  } else {
    Serial.printf("[MQTT] ❌ CONNACK 拒绝, rc=%d\n", rc);
    return false;
  }
}

/**
 * MQTT SUBSCRIBE - 订阅主题
 */
uint16_t mqttSubscribe(const char* topic, uint8_t qos = 1) {
  uint16_t pid = packetId++;
  uint16_t topicLen = strlen(topic);

  uint8_t packet[256];
  int idx = 0;

  // 固定头: SUBSCRIBE = 0x82 (QoS 1)
  packet[idx++] = 0x82;

  // 剩余长度 = Packet ID(2) + TopicLen(2) + Topic + QoS(1)
  uint16_t rl = 2 + 2 + topicLen + 1;
  idx += encodeRemainingLength(packet + idx, rl);

  // Packet ID
  packet[idx++] = (pid >> 8) & 0xFF;
  packet[idx++] = pid & 0xFF;

  // Topic Filter
  packet[idx++] = (topicLen >> 8) & 0xFF;
  packet[idx++] = topicLen & 0xFF;
  memcpy(packet + idx, topic, topicLen);
  idx += topicLen;

  // QoS
  packet[idx++] = qos;

  tcpClient.write(packet, idx);
  Serial.printf("[MQTT] SUBSCRIBE → %s (pid=%d)\n", topic, pid);
  return pid;
}

/**
 * MQTT PUBLISH (QoS 1) - 发布消息
 */
uint16_t mqttPublish(const char* topic, const char* payload, bool retained = false) {
  uint16_t pid = packetId++;
  uint16_t topicLen = strlen(topic);
  uint16_t payloadLen = strlen(payload);

  uint8_t packet[512];
  int idx = 0;

  // 固定头: PUBLISH QoS1 = 0x32 (retain 加 0x01)
  uint8_t flags = 0x32;
  if (retained) flags |= 0x01;
  packet[idx++] = flags;

  // 剩余长度
  uint16_t rl = 2 + topicLen + 2 + payloadLen;
  idx += encodeRemainingLength(packet + idx, rl);

  // Topic
  packet[idx++] = (topicLen >> 8) & 0xFF;
  packet[idx++] = topicLen & 0xFF;
  memcpy(packet + idx, topic, topicLen);
  idx += topicLen;

  // Packet ID
  packet[idx++] = (pid >> 8) & 0xFF;
  packet[idx++] = pid & 0xFF;

  // Payload
  memcpy(packet + idx, payload, payloadLen);
  idx += payloadLen;

  tcpClient.write(packet, idx);
  Serial.printf("[MQTT] PUBLISH → %s (pid=%d, %d bytes)\n", topic, pid, payloadLen);
  return pid;
}

/**
 * MQTT PUBACK - 确认收到 QoS1 消息
 */
void mqttPuback(uint16_t pid) {
  uint8_t packet[4];
  packet[0] = 0x40;       // PUBACK
  packet[1] = 0x02;       // 剩余长度
  packet[2] = (pid >> 8) & 0xFF;
  packet[3] = pid & 0xFF;
  tcpClient.write(packet, 4);
}

/**
 * MQTT PINGREQ - 心跳请求
 */
void mqttPing() {
  uint8_t packet[2] = { 0xC0, 0x00 };
  tcpClient.write(packet, 2);
  Serial.println("[MQTT] PINGREQ");
}

// ==================== 公网 MQTT (broker.emqx.io) ====================
bool pubMqttConnect() {
  if (!pubClient.connect(pubHost, pubPort)) {
    Serial.println("[公网MQTT] TCP连接失败");
    return false;
  }
  // 简单 CONNECT 包（无认证）
  const char* cid = "esp32_002_smarthome";
  uint16_t cidLen = strlen(cid);
  uint16_t rl = 10 + 2 + cidLen; // 可变头10 + clientID
  uint8_t pkt[128]; int i = 0;
  pkt[i++] = 0x10;
  i += encodeRemainingLength(pkt + i, rl);
  // 协议名 MQTT
  pkt[i++]=0x00; pkt[i++]=0x04; pkt[i++]='M'; pkt[i++]='Q'; pkt[i++]='T'; pkt[i++]='T';
  pkt[i++]=0x04; // 协议级别
  pkt[i++]=0x02; // 清除会话
  pkt[i++]=0x00; pkt[i++]=0x3C; // 保活60s
  pkt[i++]=(cidLen>>8)&0xFF; pkt[i++]=cidLen&0xFF;
  memcpy(pkt+i, cid, cidLen); i+=cidLen;
  pubClient.write(pkt, i);
  // 等 CONNACK
  unsigned long t=millis();
  while(!pubClient.available()&&millis()-t<5000) delay(10);
  if(!pubClient.available()) return false;
  pubClient.read(); readRemainingLengthPub(); pubClient.read();
  uint8_t rc=pubClient.read();
  if(rc==0){Serial.println("[公网MQTT] ✅ 已连接");return true;}
  Serial.printf("[公网MQTT] ❌ rc=%d\n",rc);return false;
}

int readRemainingLengthPub() {
  int v=0,m=1; uint8_t b;
  for(int i=0;i<4;i++){if(!pubClient.available())return-1;b=pubClient.read();v+=(b&0x7F)*m;if(!(b&0x80))break;m*=128;}
  return v;
}

void pubMqttSubscribe(const char* topic) {
  uint16_t pid=pubPid++, tlen=strlen(topic);
  uint8_t pkt[128]; int i=0;
  uint16_t rl=2+2+tlen+1;
  pkt[i++]=0x82;
  i+=encodeRemainingLength(pkt+i,rl);
  pkt[i++]=(pid>>8)&0xFF; pkt[i++]=pid&0xFF;
  pkt[i++]=(tlen>>8)&0xFF; pkt[i++]=tlen&0xFF;
  memcpy(pkt+i,topic,tlen); i+=tlen;
  pkt[i++]=1; // QoS1
  pubClient.write(pkt,i);
  Serial.printf("[公网MQTT] 订阅 %s\n",topic);
}

void pubMqttPublish(const char* topic, const char* payload) {
  uint16_t tlen=strlen(topic), plen=strlen(payload);
  uint8_t pkt[512]; int i=0;
  pkt[i++]=0x30; // QoS0
  uint16_t rl=2+tlen+plen;
  i+=encodeRemainingLength(pkt+i,rl);
  pkt[i++]=(tlen>>8)&0xFF; pkt[i++]=tlen&0xFF;
  memcpy(pkt+i,topic,tlen); i+=tlen;
  memcpy(pkt+i,payload,plen); i+=plen;
  pubClient.write(pkt,i);
}

void pubMqttLoop() {
  if(!pubClient.connected()) return;
  while(pubClient.available()) {
    uint8_t fb=pubClient.read();
    uint8_t pt=fb&0xF0;
    int rl=readRemainingLengthPub();
    if(rl<0) return;
    uint8_t* data=(uint8_t*)malloc(rl);
    int rb=0; unsigned long t=millis();
    while(rb<rl&&millis()-t<1000){if(pubClient.available()){data[rb++]=pubClient.read();t=millis();}delay(1);}
    if(rb<rl){free(data);return;}

    if(pt==0x30||pt==0x32){
      int pos=0;
      uint16_t tlen=(data[pos]<<8)|data[pos+1]; pos+=2;
      char topic[128]={0};
      memcpy(topic,data+pos,min((int)tlen,127)); pos+=tlen;
      if(pt==0x32) pos+=2; // skip pid
      int plen=rl-pos;
      // 处理控制命令
      char* js=(char*)malloc(plen+1);
      memcpy(js,data+pos,plen); js[plen]=0;
      Serial.printf("[公网MQTT] 收到: %s → %s\n",topic,js);
      JsonDocument doc;
      if(!deserializeJson(doc,js)){
        if(doc["led"].is<bool>()) setLED(doc["led"].as<bool>());
        if(doc["relay"].is<bool>()) setRelay(doc["relay"].as<bool>());
        if(doc["lock"].is<bool>()) setLock(doc["lock"].as<bool>());
        if(doc["buzzer"].is<bool>()) setBuzzer(doc["buzzer"].as<bool>());
        if(doc["autoMode"].is<bool>()) autoMode=doc["autoMode"].as<bool>();
        if(doc["alarmActive"].is<bool>()) alarmActive=doc["alarmActive"].as<bool>();
        reportProperties();
      }
      free(js);
    }
    free(data);
  }
}

// ==================== 巴法云 MQTT（小爱同学语音控制，PubSubClient）====================

// MQTT 回调：小爱下发的命令
void bemfaCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.toLowerCase();
  Serial.printf("[巴法MQTT] ← %s → %s\n", topic, msg.c_str());

  bool state = (msg == "on" || msg == "1");
  String t(topic);
  if (t == T_LED)    setLED(state);
  else if (t == T_BUZZER) setBuzzer(state);
  else { Serial.printf("[巴法MQTT] 未知主题: %s\n", topic); return; }
  Serial.printf("[语音] 小爱控制 %s → %s\n", topic, state ? "ON" : "OFF");
  reportProperties();
}

void connectBemfa() {
  bemfaMqtt.setServer(bmServer, bmPort);
  bemfaMqtt.setCallback(bemfaCallback);
  if (bemfaMqtt.connect(bmUid)) {
    Serial.println("[巴法MQTT] ✅ 已连接");
    bemfaMqtt.subscribe(T_LED);
    bemfaMqtt.subscribe(T_BUZZER);
    bemfaMqtt.subscribe(T_TEMP);
    bemfaMqtt.subscribe(T_HUM);
    bemfaMqtt.subscribe(T_SOIL);
    Serial.println("[巴法MQTT] 已订阅5个主题");
    bmConnected = true;
  } else {
    Serial.println("[巴法MQTT] ❌ 连接失败");
    bmConnected = false;
  }
}

void bemfaReport(const char* topic, const char* msg) {
  if (!bmConnected) return;
  bemfaMqtt.publish(topic, msg, true);  // retained 小爱可查询
  Serial.printf("[巴法MQTT] 上报 %s=%s\n", topic, msg);
}

/**
 * 发送属性设置响应
 */
void sendSetResponse(int code, const char* desc) {
  char json[128];
  snprintf(json, sizeof(json), "{\"result_code\":%d,\"result_desc\":\"%s\"}", code, desc);
  mqttPublish(topicSetResp.c_str(), json);
}

/**
 * 处理收到的 MQTT 消息
 */
void handleIncomingMessage(const char* topic, uint8_t* payload, int payloadLen) {
  // 确保 payload 以 null 结尾
  // 华为云命令下发：payload 前 2 字节是二进制头，跳过找到 JSON 起始
  int offset = 0;
  if (strstr(topic, "/sys/commands/") != NULL && payloadLen > 2) {
    for (int i = 0; i < payloadLen; i++) {
      if (payload[i] == '{') { offset = i; break; }
    }
  }
  char* jsonStr = (char*)malloc(payloadLen - offset + 1);
  memcpy(jsonStr, payload + offset, payloadLen - offset);
  jsonStr[payloadLen - offset] = '\0';

  Serial.printf("[MQTT] 📩 收到: %s\n", topic);
  Serial.printf("[MQTT] Payload: %s\n", jsonStr);

  // 解析 JSON
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    Serial.printf("[MQTT] JSON 解析失败: %s\n", err.c_str());
    sendSetResponse(1, "JSON parse error");
    free(jsonStr);
    return;
  }

  // ========== 方式1: 属性设置 ==========
  if (strstr(topic, "/sys/properties/set/") != NULL) {
    JsonObject services = doc["services"][0];
    JsonObject props = services["properties"];
    if (!props.isNull()) {

      if (!props["led"].isNull()) {
        ledState = props["led"].as<bool>();
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        Serial.printf("[执行] LED → %s\n", ledState ? "ON" : "OFF");
      }
      if (!props["relay"].isNull()) {
        relayState = props["relay"].as<bool>();
        digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
        Serial.printf("[执行] 继电器1 → %s\n", relayState ? "ON" : "OFF");
      }
      if (!props["relay2"].isNull()) {
        relay2State = props["relay2"].as<bool>();
        digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
        Serial.printf("[执行] 继电器2 → %s\n", relay2State ? "ON" : "OFF");
      }
      if (!props["lock"].isNull()) {
        lockState = props["lock"].as<bool>();
        relayState = lockState;
        digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
        Serial.printf("[执行] 电磁锁 → %s\n", lockState ? "OPEN" : "LOCKED");
      }
      if (!props["buzzer"].isNull()) {
        buzzerState = props["buzzer"].as<bool>();
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
        Serial.printf("[执行] 喇叭 → %s\n", buzzerState ? "ON" : "OFF");
      }
      if (!props["autoMode"].isNull()) {
        autoMode = props["autoMode"].as<bool>();
        Serial.printf("[执行] 自动模式 → %s\n", autoMode ? "ON" : "OFF");
      }
      if (!props["alarmActive"].isNull()) {
        alarmActive = props["alarmActive"].as<bool>();
        Serial.printf("[执行] 报警系统 → %s\n", alarmActive ? "ARMED" : "DISARMED");
      }

      sendSetResponse(0, "success");
      reportProperties();
    }
  }

  // ========== 方式2: 命令下发 ==========
  if (strstr(topic, "/sys/commands/") != NULL) {
    const char* cmdName = doc["command_name"];
    JsonObject paras = doc["paras"];

    if (cmdName && !paras.isNull()) {
      Serial.printf("[命令] 执行: %s\n", cmdName);

      if (strcmp(cmdName, "controlLED") == 0) {
        setLED(paras["led"].as<bool>() || paras["Led"].as<bool>());
      }
      else if (strcmp(cmdName, "controlRelay") == 0) {
        setRelay(paras["relay"].as<bool>() || paras["Relay"].as<bool>());
      }
      else if (strcmp(cmdName, "controlLock") == 0) {
        setLock(paras["lock"].as<bool>() || paras["Lock"].as<bool>());
      }
      else if (strcmp(cmdName, "controlBuzzer") == 0) {
        setBuzzer(paras["buzzer"].as<bool>() || paras["Buzzer"].as<bool>());
      }
      else if (strcmp(cmdName, "setAutoMode") == 0) {
        autoMode = paras["autoMode"].as<bool>() || paras["AutoMode"].as<bool>();
        Serial.printf("[执行] 自动模式 → %s\n", autoMode ? "ON" : "OFF");
      }
      else if (strcmp(cmdName, "setAlarm") == 0) {
        alarmActive = paras["alarmActive"].as<bool>() || paras["AlarmActive"].as<bool>();
        Serial.printf("[执行] 报警系统 → %s\n", alarmActive ? "ARMED" : "DISARMED");
      }
      else {
        Serial.printf("[命令] ⚠️ 未知命令: %s\n", cmdName);
      }
      reportProperties();

      // 回复命令响应（华为云要求，否则超时报错）
      const char* reqId = strstr(topic, "request_id=");
      if (reqId) {
        String respTopic = "$oc/devices/" + String(deviceId) + "/sys/commands/response/" + String(reqId);
        char respJson[128];
        snprintf(respJson, sizeof(respJson),
          "{\"result_code\":0,\"response_name\":\"%s\",\"paras\":{}}", cmdName);
        mqttPublish(respTopic.c_str(), respJson);
        Serial.printf("[命令] 响应已发送 → %s\n", respTopic.c_str());
      }
    }
  }

  // ========== 方式3: 属性获取 ==========
  if (strstr(topic, "/sys/properties/get/") != NULL) {
    reportProperties();
  }

  free(jsonStr);
}

/**
 * 检查并处理 MQTT 入站数据
 */
void mqttLoop() {
  if (!tcpClient.connected()) return;

  // 处理所有可用数据
  while (tcpClient.available()) {
    lastMqttActivity = millis();

    uint8_t firstByte = tcpClient.read();
    uint8_t pktType = firstByte & 0xF0;

    int rl = readRemainingLength();
    if (rl < 0) {
      Serial.println("[MQTT] ⚠️ 剩余长度读取错误");
      return;
    }

    // 读取剩余数据
    uint8_t* data = (uint8_t*)malloc(rl);
    int readBytes = 0;
    unsigned long timeout = millis();
    while (readBytes < rl && millis() - timeout < 2000) {
      if (tcpClient.available()) {
        data[readBytes++] = tcpClient.read();
        timeout = millis();
      }
      delay(1);
    }

    if (readBytes < rl) {
      Serial.printf("[MQTT] ⚠️ 数据不完整: got %d / %d\n", readBytes, rl);
      free(data);
      return;
    }

    switch (pktType) {
      case 0xD0: // PINGRESP
        Serial.println("[MQTT] PINGRESP ✓");
        break;

      case 0x90: { // SUBACK
        uint16_t ackPid = (data[0] << 8) | data[1];
        Serial.printf("[MQTT] SUBACK pid=%d, granted QoS=%d\n", ackPid, data[2]);
        subAcked = true;
        break;
      }

      case 0x40: { // PUBACK
        uint16_t ackPid = (data[0] << 8) | data[1];
        Serial.printf("[MQTT] PUBACK pid=%d\n", ackPid);
        break;
      }

      case 0x30:  // PUBLISH QoS 0
      case 0x32: { // PUBLISH QoS 1
        int pos = 0;
        uint16_t topicLen = (data[pos] << 8) | data[pos + 1];
        pos += 2;

        char topic[128] = {0};
        memcpy(topic, data + pos, min((int)topicLen, 127));
        pos += topicLen;

        // QoS1 有 Packet ID
        uint16_t msgPid = 0;
        if (pktType == 0x32) {
          msgPid = (data[pos] << 8) | data[pos + 1];
          pos += 2;
          mqttPuback(msgPid);
        }

        int payloadLen = rl - pos;
        Serial.printf("[MQTT] DEBUG rl=%d topicLen=%d pos=%d payloadLen=%d\n", rl, topicLen, pos, payloadLen);
        // 打印 payload 前 20 字节的十六进制
        Serial.printf("[MQTT] HEX: ");
        for (int i = 0; i < payloadLen && i < 20; i++) {
          Serial.printf("%02X ", data[pos + i]);
        }
        Serial.println();
        handleIncomingMessage(topic, data + pos, payloadLen);
        break;
      }

      default:
        Serial.printf("[MQTT] 未知包类型: 0x%02X (len=%d)\n", pktType, rl);
        break;
    }

    free(data);
  }
}

// ==================== 传感器读取 ====================

/**
 * 读取 DHT11 温湿度传感器
 * 返回值: true=成功, false=失败
 */
bool readDHT11() {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  // 发送起始信号
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  delay(18);
  digitalWrite(DHTPIN, HIGH);
  delayMicroseconds(30);
  pinMode(DHTPIN, INPUT);  // V2模块自带10K上拉，不用INPUT_PULLUP

  // 等待 DHT11 响应
  unsigned long t0 = micros();
  while (digitalRead(DHTPIN) == LOW) {
    if (micros() - t0 > 120) { Serial.println("[DHT11] ❌ 无响应"); return false; }
  }
  t0 = micros();
  while (digitalRead(DHTPIN) == HIGH) {
    if (micros() - t0 > 120) { Serial.println("[DHT11] ❌ 总线异常"); return false; }
  }

  // 读取 40 位
  for (int i = 0; i < 40; i++) {
    t0 = micros();
    while (digitalRead(DHTPIN) == LOW) {
      if (micros() - t0 > 80) return false;
    }
    t0 = micros();
    while (digitalRead(DHTPIN) == HIGH) {
      if (micros() - t0 > 80) return false;
    }
    if (micros() - t0 > 35) {
      data[i / 8] |= (1 << (7 - (i % 8)));
    }
  }

  // 校验和
  if ((data[0] + data[1] + data[2] + data[3]) != data[4]) {
    Serial.printf("[DHT11] 校验错误: %d+%d+%d+%d != %d\n",
                  data[0], data[1], data[2], data[3], data[4]);
    return false;
  }

  if (data[0] + data[2] > 0) {
    humidity = data[0];
    temperature = data[2];
    Serial.printf("[DHT11] 温度: %.1f°C, 湿度: %.1f%%\n", temperature, humidity);
    return true;
  }
  return false;
}

/**
 * 读取土壤湿度传感器
 * 返回: 0-100 百分比（0=干燥, 100=湿润）
 */
int readSoilMoisture() {
  int raw = analogRead(SOIL_PIN);
  // ADC 12位: 0-4095, 映射到 0-100%
  // 注意: 土壤湿度传感器通常是反向的（水越多，电压越低）
  int moisture = map(raw, 0, 2800, 0, 100);  // V2: 湿度高→电压高
  moisture = constrain(moisture, 0, 100);
  Serial.printf("[土壤] 湿度: %d%% (raw=%d)\n", moisture, raw);
  return moisture;
}

/**
 * 读取红外热释电传感器（人体检测）
 */
bool readPIR() {
  bool motion = digitalRead(PIR_PIN) == HIGH;
  if (motion) {
    Serial.println("[PIR] 👤 检测到人体移动!");
  }
  return motion;
}

/**
 * 读取按键（带去抖）
 * 返回: true=检测到按下
 */
bool readButton() {
  static unsigned long lastPress = 0;
  if (digitalRead(BUTTON_PIN) == HIGH && millis() - lastPress > 500) {
    lastPress = millis();
    return true;
  }
  return false;
}

// ==================== 执行器控制 ====================

void setLED(bool state) {
  if (ledState == state) return;  // 状态没变，跳过
  ledState = state;
  digitalWrite(LED_PIN, state ? HIGH : LOW);
  Serial.printf("[执行] LED → %s\n", state ? "ON" : "OFF");
}

void setRelay(bool state) {
  if (relayState == state) return;  // 状态没变，跳过
  relayState = state;
  lockState = state;  // 继电器和电磁锁是同一个物理设备
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  Serial.printf("[执行] 继电器/电磁锁 → %s\n", state ? "吸合/开锁" : "断开/锁定");
}

void setRelay2(bool state) {
  if (relay2State == state) return;
  relay2State = state;
  digitalWrite(RELAY2_PIN, state ? HIGH : LOW);
  Serial.printf("[执行] 继电器2(土壤) → %s\n", state ? "ON" : "OFF");
}

void setLock(bool state) {
  setRelay(state);
}

void setBuzzer(bool state) {
  if (buzzerState == state) return;
  buzzerState = state;
  if (state) {
    ledcSetup(0, 1000, 8);
    ledcAttachPin(BUZZER_PIN, 0);
    ledcWrite(0, 128);
  } else {
    ledcDetachPin(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
  Serial.printf("[执行] 喇叭 → %s\n", state ? "ON" : "OFF");
}

// ==================== 联动逻辑 ====================

/**
 * 智能家居联动规则处理
 * 传感器和执行器之间的自动联动
 */
void processAutomation() {
  if (!autoMode) return;

  // ===== 规则1: PIR 人体检测 =====
  static bool pirPushed = false;  // 是否已推送过"有人"
  if (motionDetected) {
    if (alarmActive) {
      // 布防ON: 喇叭响 + LED闪 + 继电器锁死 + 推送入侵
      if (!alarmTriggered) {
        Serial.println("[联动] 🚨 入侵警报! 布防模式触发!");
        alarmTriggered = true;
        alarmStartTime = millis();
        setBuzzer(true);
        setLock(true);  // 继电器1锁死
        sendPush("🚨 入侵警报", "检测到人体移动！继电器已锁定，喇叭已触发");
      }
    } else {
      // 布防OFF: 仅推送"有人经过"
      if (!pirPushed) {
        sendPush("👤 有人经过", "PIR检测到人体移动（未布防，不执行动作）");
        Serial.println("[联动] 👤 有人经过（未布防）");
        pirPushed = true;
      }
    }
  } else {
    if (pirPushed) pirPushed = false;
  }

  // 报警持续5秒后自动解除
  if (alarmTriggered && (millis() - alarmStartTime > ALARM_DURATION)) {
    Serial.println("[联动] 🔕 警报自动解除");
    alarmTriggered = false;
    setBuzzer(false);
  }

  // ===== 规则2: 土壤 < 30% → LED常亮 + 继电器2吸合 + 推送 =====
  static bool soilWarned = false;
  if (soilMoisture < SOIL_THRESHOLD) {
    if (!soilWarned) {
      Serial.printf("[联动] ⚠️ 土壤干燥(%d%%)  LED+继电器2开启\n", soilMoisture);
      soilWarned = true;
      char msg[64];
      snprintf(msg, sizeof(msg), "当前湿度: %d%% (阈值: %d%%)", soilMoisture, (int)SOIL_THRESHOLD);
      sendPush("🌱 土壤干燥警告", msg);
      setLED(true);
      setRelay2(true);
    }
  } else {
    if (soilWarned) {
      Serial.println("[联动] ✅ 土壤湿度恢复正常");
      soilWarned = false;
      if (!alarmTriggered) setLED(false);
      setRelay2(false);
    }
  }

  // ===== 规则3: 温度 > 30°C =====
  static bool tempHigh = false;
  if (temperature > TEMP_THRESHOLD) {
    if (!tempHigh) {
      tempHigh = true;
      char msg[64];
      snprintf(msg, sizeof(msg), "当前温度: %.1f°C (阈值: %.0f°C)", temperature, TEMP_THRESHOLD);
      if (alarmActive) {
        sendPush("🔥 温度过高", msg);
        Serial.printf("[联动] 🔥 温度过高(%.1f°C) 布防模式,仅推送\n", temperature);
      } else {
        sendPush("🔥 温度过高警告", msg);
        setBuzzer(true);  // 仅触发一次，后续手动可覆盖
        Serial.printf("[联动] 🔥 温度过高(%.1f°C) 未布防,蜂鸣告警\n", temperature);
      }
    }
  } else {
    if (tempHigh) {
      Serial.printf("[联动] ✅ 温度恢复正常(%.1f°C)\n", temperature);
      tempHigh = false;
      if (!alarmTriggered) setBuzzer(false);
    }
  }

  // ===== 报警时 LED 闪烁 =====
  if (alarmTriggered) {
    bool blink = (millis() / 200) % 2 == 0;
    digitalWrite(LED_PIN, blink ? HIGH : LOW);
  }
}

// ==================== 云端通信 ====================

/**
 * 发送推送通知到手机（WxPusher，免费）
 */
void sendPush(String title, String content) {
  WiFiClient pushClient;
  if (!pushClient.connect("wxpusher.zjiecode.com", 80)) {
    Serial.println("[推送] ❌ 连接失败");
    return;
  }
  String body = "{\"appToken\":\"" + String(wxAppToken) +
    "\",\"content\":\"" + content +
    "\",\"summary\":\"" + title +
    "\",\"contentType\":1" +
    ",\"uids\":[\"" + String(wxUid) + "\"]}";
  String req = "POST /api/send/message HTTP/1.1\r\n";
  req += "Host: wxpusher.zjiecode.com\r\n";
  req += "Content-Type: application/json\r\n";
  req += "Content-Length: " + String(body.length()) + "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;
  pushClient.print(req);
  Serial.println("[推送] ✅ " + title);
  unsigned long t = millis();
  while (!pushClient.available() && millis() - t < 2000) delay(10);
  while (pushClient.available() || pushClient.connected()) {
    if (pushClient.available()) pushClient.read();
    if (millis() - t > 3000) break;
  }
  pushClient.stop();
}

/**
 * 构造并上报设备属性到华为云 IoT
 */
void reportProperties() {
  JsonDocument doc;
  char eventTime[30];
  time_t now;
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(eventTime, sizeof(eventTime), "%Y%m%dT%H%M%SZ", &timeinfo);
  }

  JsonArray services = doc["services"].to<JsonArray>();

  // --- 服务1: SensorService（传感器数据） ---
  JsonObject svcSensor = services.add<JsonObject>();
  svcSensor["service_id"] = "SensorService";
  JsonObject sp = svcSensor["properties"].to<JsonObject>();
  sp["temperature"]   = round(temperature * 10) / 10.0;
  sp["humidity"]      = round(humidity * 10) / 10.0;
  sp["soil_moisture"] = soilMoisture;
  sp["motion"]        = motionDetected;
  svcSensor["event_time"] = eventTime;

  // --- 服务2: ControlService（执行器状态） ---
  JsonObject svcCtrl = services.add<JsonObject>();
  svcCtrl["service_id"] = "ControlService";
  JsonObject cp = svcCtrl["properties"].to<JsonObject>();
  cp["led"]          = ledState;
  cp["relay"]        = relayState;
  cp["relay2"]       = relay2State;
  cp["lock"]         = lockState;
  cp["buzzer"]       = buzzerState;
  cp["autoMode"]     = autoMode;
  cp["alarmActive"]  = alarmActive;
  svcCtrl["event_time"] = eventTime;

  String jsonStr;
  serializeJson(doc, jsonStr);

  mqttPublish(topicReport.c_str(), jsonStr.c_str());
  Serial.println("[上报] 属性已上报(华为云)");
  delay(50);
  // 同时上报到公网 MQTT（供网页仪表盘）
  if (pubConnected) {
    pubMqttPublish(pubTopicStatus, jsonStr.c_str());
    Serial.println("[上报] 属性已上报(公网)");
    delay(50);
  }
  // 同时上报到巴法云 MQTT（小爱查询/控制）
  if (bmConnected) {
    bemfaReport(T_LED,    ledState    ? "on" : "off");
    bemfaReport(T_BUZZER, buzzerState ? "on" : "off");
    bemfaReport(T_TEMP,   temperature > 30  ? "on" : "off");
    bemfaReport(T_HUM,    humidity    < 40  ? "on" : "off");
    bemfaReport(T_SOIL,   soilMoisture < 30 ? "on" : "off");
  }
}

// ==================== Arduino 生命周期 ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║   智能物联网家居系统 - Smart Home v2.0   ║");
  Serial.println("║   ESP32 + 华为云IoT + 多传感器联动      ║");
  Serial.println("╚══════════════════════════════════════════╝\n");

  // 初始化引脚
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);

  // 初始状态: 全部关闭
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // --- Wi-Fi 连接（带超时保护，防止看门狗复位）---
  Serial.print("[WiFi] 连接中");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 最大发射功率
  WiFi.begin(ssid, password);

  int wifiRetry = 0;
  const int WIFI_TIMEOUT = 40;  // 最多重试 40 次 × 500ms = 20 秒
  while (WiFi.status() != WL_CONNECTED && wifiRetry < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
    wifiRetry++;
    yield();  // 喂看门狗
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] ✅ 已连接");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    httpServer.begin();
    Serial.println("[HTTP] API就绪 → http://" + WiFi.localIP().toString() + "/api/status");
  } else {
    Serial.printf("\n[WiFi] ❌ 连接超时 (状态码: %d)\n", WiFi.status());
    Serial.println("[WiFi] 请检查: 1)WiFi名称密码 2)路由器是否正常工作");
    Serial.println("[WiFi] 5秒后重启...");
    delay(5000);
    ESP.restart();
  }

  // 同步时间（用于事件时间戳）
  configTime(8 * 3600, 0, "ntp.aliyun.com", "ntp.ntsc.ac.cn");
  Serial.print("[NTP] 同步时间");
  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 10) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (retry < 10) {
    Serial.println("\n[NTP] ✅ 时间已同步");
  } else {
    Serial.println("\n[NTP] ⚠️ 时间同步超时，继续运行");
  }

  // --- 初始化 MQTT 主题 ---
  topicReport  = "$oc/devices/" + String(deviceId) + "/sys/properties/report";
  topicCmd     = "$oc/devices/" + String(deviceId) + "/sys/commands/#";
  topicSet     = "$oc/devices/" + String(deviceId) + "/sys/properties/set/#";
  topicGet     = "$oc/devices/" + String(deviceId) + "/sys/properties/get/#";
  topicMsgDown = "$oc/devices/" + String(deviceId) + "/sys/messages/down";
  topicSetResp = "$oc/devices/" + String(deviceId) + "/sys/properties/set/response";
  topicGetResp = "$oc/devices/" + String(deviceId) + "/sys/properties/get/response";
  topicMsgUp   = "$oc/devices/" + String(deviceId) + "/sys/messages/up";

  // --- MQTT 连接 ---
  mqttConnected = mqttConnect();

  if (mqttConnected) {
    // 订阅命令主题
    Serial.println("[MQTT] 订阅主题...");
    mqttSubscribe(topicCmd.c_str(), 1);
    delay(100);
    mqttSubscribe(topicSet.c_str(), 1);
    delay(100);
    mqttSubscribe(topicGet.c_str(), 1);
    delay(100);
    mqttSubscribe(topicMsgDown.c_str(), 1);
    delay(100);

    // 初始上报
    reportProperties();
    lastReportTime = millis();
    lastPingTime = millis();

    // --- 公网 MQTT 连接 ---
    pubConnected = pubMqttConnect();
    if (pubConnected) {
      pubMqttSubscribe(pubTopicControl);
    }

    // --- 巴法云连接（小爱同学） ---
    connectBemfa();
    if (bmConnected) {
      // 读取传感器（失败重试一次）
      if (!readDHT11()) { delay(100); readDHT11(); }
      soilMoisture = readSoilMoisture();
      motionDetected = readPIR();
      delay(100);  // 等传感器数据稳定
      reportProperties();
    }
  }

  // 默认关闭报警系统，需要时手动开启
  alarmActive = false;

  Serial.println("\n[系统] ✅ 初始化完成，进入主循环\n");
  Serial.println("═══════════════════════════════════════════\n");
}

void loop() {
  // ==================== 0. HTTP 请求处理 ====================
  WiFiClient httpClient = httpServer.accept();
  if (httpClient) {
    // 等数据到达
    unsigned long t = millis();
    while (!httpClient.available() && millis() - t < 1000) { delay(5); }

    String req = "";
    t = millis();
    while (millis() - t < 500) {
      while (httpClient.available()) { req += (char)httpClient.read(); t = millis(); }
      delay(10);
    }
    Serial.println("[HTTP] 收到请求: " + req.substring(0, 80));

    // CORS + JSON 头
    const char* headers = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";

    if (req.indexOf("GET /api/status") >= 0) {
      httpClient.print(headers);
      JsonDocument d;
      d["temperature"]    = round(temperature * 10) / 10.0;
      d["humidity"]       = round(humidity * 10) / 10.0;
      d["soil_moisture"]  = soilMoisture;
      d["motion"]         = motionDetected;
      d["led"]            = ledState;
      d["relay"]          = relayState;
      d["lock"]           = lockState;
      d["buzzer"]         = buzzerState;
      d["autoMode"]       = autoMode;
      d["alarmActive"]    = alarmActive;
      String json;
      serializeJson(d, json);
      httpClient.print(json);
    }
    else if (req.indexOf("GET /api/control") >= 0) {
      // 解析: /api/control?device=led&value=1
      bool val = (req.indexOf("value=1") >= 0 || req.indexOf("value=true") >= 0);
      if (req.indexOf("device=led") >= 0) setLED(val);
      else if (req.indexOf("device=relay") >= 0) setRelay(val);
      else if (req.indexOf("device=lock") >= 0) setLock(val);
      else if (req.indexOf("device=buzzer") >= 0) setBuzzer(val);
      else if (req.indexOf("device=autoMode") >= 0) autoMode = val;
      else if (req.indexOf("device=alarmActive") >= 0) alarmActive = val;
      reportProperties();
      httpClient.print(headers);
      httpClient.print("{\"ok\":true}");
    }
    else {
      httpClient.print(headers);
      httpClient.print("{\"name\":\"SmartHome\",\"ip\":\"" + WiFi.localIP().toString() + "\"}");
    }
    httpClient.stop();
  }

  // ==================== 1. 检查 WiFi 连接 ====================
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] ⚠️ WiFi 断开，尝试重连...");
    WiFi.reconnect();
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
      delay(500);
      Serial.print(".");
      retry++;
      yield();
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[WiFi] ✅ 已重连");
    } else {
      Serial.println("\n[WiFi] ❌ 重连失败，重启...");
      delay(2000);
      ESP.restart();
    }
  }
  if (mqttConnected) {
    mqttLoop();
  }

  // ==================== 1.5 公网 MQTT ====================
  if (pubConnected) {
    pubMqttLoop();
    if (!pubClient.connected()) {
      pubConnected = pubMqttConnect();
      if (pubConnected) pubMqttSubscribe(pubTopicControl);
    }
  }

  // ==================== 1.6 巴法云 MQTT（小爱同学） ====================
  if (!bemfaMqtt.connected()) {
    bmConnected = false;
    connectBemfa();
  }
  if (bmConnected) bemfaMqtt.loop();

  if (!tcpClient.connected() || !mqttConnected) {
    Serial.println("[MQTT] ⚠️ 连接断开，5秒后重连...");
    tcpClient.stop();  // 清理旧连接
    delay(5000);
    mqttConnected = mqttConnect();
    if (mqttConnected) {
      mqttSubscribe(topicCmd.c_str(), 1);
      delay(50);
      mqttSubscribe(topicSet.c_str(), 1);
      delay(50);
      mqttSubscribe(topicGet.c_str(), 1);
      delay(50);
      mqttSubscribe(topicMsgDown.c_str(), 1);
      reportProperties();
      lastPingTime = millis();
      lastReportTime = millis();
    }
  }

  // ==================== 3. MQTT 心跳 ====================
  if (mqttConnected && (millis() - lastPingTime > PING_INTERVAL)) {
    mqttPing();
    lastPingTime = millis();
  }

  // ==================== 4. 读取传感器 ====================
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > 2000) {  // 每2秒读取一次
    lastSensorRead = millis();

    // DHT11 温湿度
    readDHT11();

    // 土壤湿度
    soilMoisture = readSoilMoisture();

    // 人体红外
    motionDetected = readPIR();
  }

  // ==================== 5. 按键处理 ====================
  if (readButton()) {
    if (alarmActive) {
      Serial.println("[按键] 🟡 布防中，按键锁定无效");
    } else {
      Serial.println("[按键] 🟡 切换继电器1");
      setLock(!lockState);  // 切换继电器1状态
      reportProperties();
    }
  }

  // ==================== 6. 自动化联动 ====================
  processAutomation();

  // ==================== 7. 定时上报属性 ====================
  if (mqttConnected && (millis() - lastReportTime > REPORT_INTERVAL)) {
    reportProperties();
    lastReportTime = millis();
  }

  // ==================== 8. 看门狗（可选） ====================
  // yield() 给 ESP32 后台任务（WiFi等）运行时间
  delay(10);
}
