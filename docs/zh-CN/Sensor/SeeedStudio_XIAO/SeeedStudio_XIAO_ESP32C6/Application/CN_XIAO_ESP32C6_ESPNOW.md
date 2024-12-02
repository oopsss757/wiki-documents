---
description: 使用 XIAO ESP32S3/XIAO ESP32C3/XIAO ESP32C6 ESP-NOW 协议进行通信
title: XIAO ESP32 系列上的 ESP-NOW 协议
keywords:
- ESPNOW
image: https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/126.png
slug: /cn/xiao_esp32c6_espnow
last_update:
  date: 11/18/2024
  author: Agnes
---

# 在 XIAO 系列上使用 ESP-NOW 协议开始运行

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/126.png" style={{width:1100, height:'auto'}}/></div>
<br />

本 Wiki 将介绍什么是 ESP-NOW 协议，并教你如何使用 XIAO ESP32 系列通过该协议进行通信，整个过程非常简单。为了让大家能够在 XIAO ESP32 系列上使用 ESP-NOW 协议，我们准备了三款 XIAO ESP32 类型：C6、C3 和 S3 来进行通信，让我们开始这个旅程吧！

顺便说一下，如果你刚刚拿到这个开发板，请点击以下链接，它会告诉你如何开始：
- [Seeed Studio XIAO ESP32S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Seeed Studio XIAO ESP32C3](https://wiki.seeedstudio.com/xiao_esp32c3_getting_started/)
- [Seeed Studio XIAO ESP32C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)

## 什么是 ESP-NOW 协议？

官方定义：ESP-NOW 是由 Espressif 提供的一种无线通信协议，允许直接、快速且低功耗地控制智能设备，无需路由器。它可以与 Wi-Fi 和蓝牙 LE 共存，支持多种系列的 SoC，例如 Lexin ESP8266、ESP32、ESP32-S 和 ESP32-C。ESP-NOW 广泛应用于智能家居、电器、远程控制和传感器等领域。

特点如下：
- 通过 MAC 地址连接方式，配对可以快速完成，无需网络条件，设备可以实现单对多、单对单、多对单以及多对多的连接方式。
- ESP-NOW 是一种基于数据链路层的无线通信协议，它将五层 OSI 上层协议简化为一层，无需添加数据包头并逐层解包。它大大减轻了网络拥堵时由于数据包丢失导致的延迟和滞后，响应速度更高。

与 Wi-Fi 和蓝牙的对比：
- Wi-Fi：ESP-NOW 支持设备之间的点对点通信，因此功耗较低，传输速度更高，且通信距离更远。
- 蓝牙：ESP-NOW 不需要配对过程，使其使用更加简单和方便，因此功耗较低，传输速度更高。

但 ESP-NOW 更适合用于需要快速、可靠、低功耗以及点对点通信的应用场景，而蓝牙和 Wi-Fi 更适合用于复杂的网络环境和设备较多的场景。

## 硬件准备

在本项目中，为了考虑到有些人可能只拥有 XIAO ESP32S3、XIAO ESP32C3 或 XIAO ESP32C6，为了帮助大家更好地学习 ESPNOW 通信，我们使用三款 XIAO ESP32 型号：XIAO ESP32S3、XIAO ESP32C3 和 XIAO ESP32C6 来相互通信。你只需要稍微调整代码，就可以使用这三款型号中的任意两款或三款进行实际操作。不再赘述，接下来我们来看看以下代码是如何实现的，来吧，开始吧！

如果你还没有这两款 XIAO ESP32 系列的开发板，以下是购买链接。

<div class="table-center">
	<table align="center">
		<tr>
			<th>XIAO ESP32C3</th>
			<th>XIAO ESP32S3</th>
            <th>XIAO ESP32C6</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:110, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:250, height:'auto'}}/></div></td>
            <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
            <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

## 功能实现

首先，我们了解一下代码的一般框架。本实例使用了三款 XIAO ESP32 板：XIAO ESP32S3、XIAO ESP32C3 和 XIAO ESP32C6，其中 XIAO ESP32S3 作为发送端，XIAO ESP32C6 和 XIAO ESP32C3 作为接收端。当然，这只是代码中的角色分配。通过我下面的解释，如果你想更改、添加或删除接收端和发送端的角色，都会变得非常简单，让我们开始吧！

### 第 1 部分：XIAO ESP32S3 发送端代码

```c
#include <Arduino.h>
#include "WiFi.h"
#include "esp_now.h" 

#define ESPNOW_WIFI_CHANNEL 0
#define MAX_ESP_NOW_MAC_LEN 6
#define BAUD 115200
#define MAX_CHARACTERS_NUMBER 20
#define NO_PMK_KEY false

typedef uint8_t XIAO;
typedef int XIAO_status;

// 你需要输入你的 XIAO ESP32 系列的 MAC 地址，不能直接复制！！！
static uint8_t Receiver_XIAOC3_MAC_Address[MAX_ESP_NOW_MAC_LEN] = {0x64, 0xe8, 0x33, 0x89, 0x80, 0xb8};
static uint8_t Receiver_XIAOC6_MAC_Address[MAX_ESP_NOW_MAC_LEN] = {0xf0, 0xf5, 0xbd, 0x1a, 0x97, 0x20};

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo1;

typedef struct receiver_meesage_types{
  char Reveiver_device[MAX_CHARACTERS_NUMBER];
  char Reveiver_Trag[MAX_CHARACTERS_NUMBER];
}receiver_meesage_types;

receiver_meesage_types XIAOC3_RECEIVER_INFORATION;
receiver_meesage_types XIAOC6_RECEIVER_INFORATION;

typedef struct message_types{
  char device[MAX_CHARACTERS_NUMBER];
  char Trag[MAX_CHARACTERS_NUMBER];
}message_types;

message_types Personal_XIAOC3_Information;
message_types Personal_XIAOC6_Information;

void espnow_init();
void espnow_deinit();
void SenderXIAOS3_MACAddress_Requir();
void SenderXIAOS3_Send_Data();
void SenderXIAOS3_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status);
void Association_ReceiverXIAOC3_peer();
void Association_ReceiverXIAOC6_peer();
void ReceiverXIAOC3_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void ReceiverXIAOC6_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len);

void setup(){
    Serial.begin(BAUD);
    while(!Serial);
    SenderXIAOS3_MACAddress_Requir();
    SenderXIAOS3_MACAddress_Requir();
    espnow_init();

    esp_now_register_send_cb(SenderXIAOS3_Send_Data_cb);

    Association_ReceiverXIAOC6_peer();
    Association_ReceiverXIAOC3_peer();

    esp_now_register_recv_cb(ReceiverXIAOC3_Recive_Data_cb);
    esp_now_register_recv_cb(ReceiverXIAOC6_Recive_Data_cb);
}

void loop(){
  SenderXIAOS3_Send_Data();
  delay(100);
}

void SenderXIAOS3_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status){
  char macStr[18];
  Serial.print("发送数据包到: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" 发送状态:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "发送成功" : "发送失败");
  Serial.println("");
}

void Association_ReceiverXIAOC3_peer(){
  Serial.println("尝试关联 XIAOC3 的对等设备...");
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = NO_PMK_KEY;

  memcpy(peerInfo.peer_addr, Receiver_XIAOC3_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("添加对等设备失败");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("成功添加对等设备");
  }
}

void Association_ReceiverXIAOC6_peer(){
  Serial.println("尝试关联 XIAOC6 的对等设备...");
  peerInfo1.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo1.encrypt = NO_PMK_KEY;

  memcpy(peerInfo1.peer_addr, Receiver_XIAOC6_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo1);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("添加对等设备失败");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("成功添加对等设备");
  }
}

void SenderXIAOS3_Send_Data(){
  
  strcpy(Personal_XIAOC3_Information.device, "XIAOS3"); 
  strcpy(Personal_XIAOC3_Information.Trag, "你好，我是发送端"); 

  strcpy(Personal_XIAOC6_Information.device, "XIAOS3"); 
  strcpy(Personal_XIAOC6_Information.Trag, "你好，我是发送端"); 

  esp_err_t XIAOS3_RECEIVER_INFORATION_data1 = esp_now_send(Receiver_XIAOC3_MAC_Address, (uint8_t *)&Personal_XIAOC3_Information, sizeof(message_types));
  esp_err_t XIAOS3_RECEIVER_INFORATION_data2 = esp_now_send(Receiver_XIAOC6_MAC_Address, (uint8_t *)&Personal_XIAOC6_Information, sizeof(message_types));

  if (XIAOS3_RECEIVER_INFORATION_data1 == ESP_OK || XIAOS3_RECEIVER_INFORATION_data2 == ESP_OK)
  {
    Serial.println("成功发送：XIAOS3_RECEIVER_INFORATION_data1 和 XIAOS3_RECEIVER_INFORATION_data2");
  }
  delay(4000);
}

void ReceiverXIAOC3_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOC3_RECEIVER_INFORATION, incomingData, sizeof(XIAOC3_RECEIVER_INFORATION));
  Serial.print("接收字节数: ");
  Serial.println(len);
  Serial.print("接收设备: ");
  Serial.println(XIAOC3_RECEIVER_INFORATION.Reveiver_device);
  Serial.print("接收目标: ");
  Serial.println(XIAOC3_RECEIVER_INFORATION.Reveiver_Trag);
  Serial.println();
}

void ReceiverXIAOC6_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOC6_RECEIVER_INFORATION, incomingData, sizeof(XIAOC6_RECEIVER_INFORATION));
  Serial.print("接收字节数: ");
  Serial.println(len);
  Serial.print("接收设备: ");
  Serial.println(XIAOC6_RECEIVER_INFORATION.Reveiver_device);
  Serial.print("接收目标: ");
  Serial.println(XIAOC6_RECEIVER_INFORATION.Reveiver_Trag);
  Serial.println();
}

void SenderXIAOS3_MACAddress_Requir(){
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    XIAO mac[MAX_ESP_NOW_MAC_LEN];
    while(!WiFi.STA.started()){
      Serial.print(".");
      delay(100);
    }
      WiFi.macAddress(mac);
      Serial.println();
      Serial.printf("const uint8_t mac_self[6] = {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x};", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println();
}

void espnow_init(){
  XIAO_status espnow_sign = esp_now_init();
  if(espnow_sign == ESP_OK)
  {
    Serial.println("ESP-NOW 初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 初始化失败");
  }
}

void espnow_deinit(){
  XIAO_status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("ESP-NOW 反初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 反初始化失败！");
  }
}
```

#### 解决方案 第 1 部分 代码

包含的库
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心功能
- `espnow_init()`  
  - 作用：初始化 ESPNOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 作用：反初始化 ESPNOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `SenderXIAOS3_MACAddress_Requir()`  
  - 作用：设置 WiFi 模式为 STA，并获取 MAC 地址，打印到串口
- `SenderXIAOS3_Send_Data()`  
  - 作用：发送特定消息
- `SenderXIAOS3_Send_Data_cb()`  
  - 作用：这是一个回调函数，执行时会打印消息是否成功发送及目标 MAC 地址
- `Association_ReceiverXIAOC3_peer() 和 Association_ReceiverXIAOC6_peer`  
  - 作用：添加对等节点，如果需要更多接收设备，可以创建节点并编写与发送方和接收方匹配的消息
- `esp_now_register_send_cb()`  
  - 作用：注册回调函数，验证消息是否已发送到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]
- `ReceiverXIAOC3_Recive_Data_cb()`  
  - 作用：接收回调函数，处理接收到的数据
- `ReceiverXIAOC6_Recive_Data_cb()`  
  - 作用：接收回调函数，处理接收到的数据
- `esp_now_register_recv_cb()`  
  - 作用：注册回调函数，验证数据是否已成功接收到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]

默认变量
- `#define ESPNOW_WIFI_CHANNEL`  
  - 作用：设置发送方和接收方的 WiFi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 作用：定义 MAC 地址长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 作用：接受或发送的最大字符数
- `#define BAUD 115200`  
  - 作用：设置串口波特率
- `static uint8_t Receiver_XIAOC3_MAC_Address[MAX_ESP_NOW_MAC_LEN] 和 static uint8_t Receiver_XIAOC6_MAC_Address`  
  - 作用：存储 XIAO ESP32C3 和 XIAO ESP32C6 的 MAC 地址，它们作为接收方
  - 补充：请注意，这些是我的 MAC 地址，不能直接写入
- `NO_PMK_KEY`  
  - 作用：选择不加密的设备配对

### 第 2 部分：XIAO ESP32C3 接收端代码

```c
#include<Arduino.h>
#include "WiFi.h"
#include "esp_now.h"

#define ESPNOW_WIFI_CHANNEL 0
#define MAX_ESP_NOW_MAC_LEN 6
#define BAUD 115200
#define MAX_CHARACTERS_NUMBER 20
#define NO_PMK_KEY false

typedef uint8_t XIAO;
typedef int status;

// 你需要输入你的 XIAO ESP32 系列 MAC 地址，不能直接复制！！！
static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN] = {0xcc, 0x8d, 0xa2, 0x0c, 0x57, 0x5c};

esp_now_peer_info_t peerInfo_sender;

typedef struct receiver_meesage_types{
  char Reveiver_device[MAX_CHARACTERS_NUMBER];
  char Reveiver_Trag[MAX_CHARACTERS_NUMBER];
}receiver_meesage_types;

receiver_meesage_types XIAOC3_RECEIVER_INFORATION;

typedef struct message_types{
  char Sender_device[MAX_CHARACTERS_NUMBER];
  char Sender_Trag[MAX_CHARACTERS_NUMBER];
}message_types;

message_types XIAOS3_SENDER_INFORATION;

void Receiver_MACAddress_requir();
void espnow_init();
void espnow_deinit();
void ReceiverXIAOC3_Recive_Data_cb(const uint8_t * mac, const uint8_t *incomingData, int len);
void ReceiverXIAOC3_Send_Data();
void ReceiverXIAOC3_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status);
void Association_SenderXIAOS3_peer();

void setup() {
  Serial.begin(BAUD);
  while(!Serial);
  Receiver_MACAddress_requir();
  espnow_init();

  esp_now_register_recv_cb(ReceiverXIAOC3_Recive_Data_cb);

  esp_now_register_send_cb(ReceiverXIAOC3_Send_Data_cb);
  Association_SenderXIAOS3_peer();  
}

void loop() {
  ReceiverXIAOC3_Send_Data();
  delay(1000);
}

void espnow_init(){
  status espnow_sign = esp_now_init();
  if(espnow_sign == ESP_OK)
  {
    Serial.println("ESP-NOW 初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 初始化失败");
  }
}

void espnow_deinit(){
  status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("ESP-NOW 反初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 反初始化失败！");
  }
}

void Receiver_MACAddress_requir(){
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    XIAO mac[MAX_ESP_NOW_MAC_LEN];
    while(!WiFi.STA.started()){
      Serial.print(".");
      delay(100);
    }
      WiFi.macAddress(mac);
      Serial.println();
      Serial.printf("const uint8_t mac_self[6] = {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x};", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println();
}

void ReceiverXIAOC3_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOS3_SENDER_INFORATION, incomingData, sizeof(XIAOS3_SENDER_INFORATION));
  Serial.print("接收到字节数: ");
  Serial.println(len);
  Serial.print("发送设备: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_device);
  Serial.print("发送目标: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_Trag);
  Serial.println();
}

void ReceiverXIAOC3_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status){
  char macStr[18];
  Serial.print("数据包发送到: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" 发送状态:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "发送成功" : "发送失败");
    Serial.println("");
}

void ReceiverXIAOC3_Send_Data(){
  
  strcpy(XIAOC3_RECEIVER_INFORATION.Reveiver_device, "XIAOC3"); 
  strcpy(XIAOC3_RECEIVER_INFORATION.Reveiver_Trag, "我已收到"); 

  esp_err_t XIAOC3_RECEIVER_INFORATION_data1 = esp_now_send(XIAOS3_Sender_MAC_Address, (uint8_t *)&XIAOC3_RECEIVER_INFORATION, sizeof(receiver_meesage_types));

  if (XIAOC3_RECEIVER_INFORATION_data1 == ESP_OK)
  {
    Serial.println("成功发送：XIAOC3_RECEIVER_INFORATION_data1");
  }
  delay(4000);
}

void Association_SenderXIAOS3_peer(){
  Serial.println("尝试关联 XIAOS3 的对等设备...");
  peerInfo_sender.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo_sender.encrypt = NO_PMK_KEY;

  memcpy(peerInfo_sender.peer_addr, XIAOS3_Sender_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo_sender);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("添加对等设备失败");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("成功添加对等设备");
  }
}
```

#### 解决方案 第 2 部分 代码

包含的库
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心功能
- `espnow_init()`  
  - 作用：初始化 ESPNOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 作用：反初始化 ESPNOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `Receiver_MACAddress_requir()`  
  - 作用：设置 WiFi 模式为 STA，并获取 MAC 地址，打印到串口
- `ReceiverXIAOC3_Send_Data()`  
  - 作用：发送特定消息
- `ReceiverXIAOC3_Recive_Data_cb()`  
  - 作用：这是一个回调函数，执行时会打印消息是否成功发送及目标 MAC 地址
- `Association_SenderXIAOS3_peer()`  
  - 作用：为 XIAO ESP32S3 添加一个通道节点，用于接收消息
- `esp_now_register_send_cb()`  
  - 作用：注册回调函数，验证消息是否已发送到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]
- `ReceiverXIAOC3_Send_Data_cb`  
  - 作用：这是一个回调函数，执行时会打印消息是否成功发送及目标 MAC 地址
- `esp_now_register_recv_cb()`  
  - 作用：注册回调函数，验证数据是否已成功接收到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]

默认变量
- `#define ESPNOW_WIFI_CHANNEL`  
  - 作用：设置发送方和接收方的 WiFi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 作用：定义 MAC 地址长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 作用：接受或发送的最大字符数
- `#define BAUD 115200`  
  - 作用：设置串口波特率
- `static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN]`  
  - 作用：存储 XIAO ESP32S3 的 MAC 地址
  - 补充：请注意，这些是我的 MAC 地址，不能直接写入！
- `NO_PMK_KEY`  
  - 作用：选择不加密的设备配对

### 第 3 部分：XIAO ESP32C6 接收端代码

```c
#include<Arduino.h>
#include "WiFi.h"
#include "esp_now.h"

#define ESPNOW_WIFI_CHANNEL 0
#define MAX_ESP_NOW_MAC_LEN 6
#define BAUD 115200
#define MAX_CHARACTERS_NUMBER 20
#define NO_PMK_KEY false

typedef uint8_t XIAO;
typedef int status;

// 你需要输入你的 XIAO ESP32 系列 MAC 地址，不能直接复制！！！
static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN] = {0xcc, 0x8d, 0xa2, 0x0c, 0x57, 0x5c};

esp_now_peer_info_t peerInfo_sender;

typedef struct receiver_meesage_types{
  char Reveiver_device[MAX_CHARACTERS_NUMBER];
  char Reveiver_Trag[MAX_CHARACTERS_NUMBER];
}receiver_meesage_types;

receiver_meesage_types XIAOC6_RECEIVER_INFORATION;

typedef struct message_types{
  char Sender_device[MAX_CHARACTERS_NUMBER];
  char Sender_Trag[MAX_CHARACTERS_NUMBER];
}message_types;

message_types XIAOS3_SENDER_INFORATION;

void Receiver_MACAddress_requir();
void espnow_init();
void espnow_deinit();
void ReceiverXIAOC6_Recive_Data_cb(const uint8_t * mac, const uint8_t *incomingData, int len);
void ReceiverXIAOC6_Send_Data();
void ReceiverXIAOC6_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status);
void Association_SenderXIAOS3_peer();

void setup() {
  Serial.begin(BAUD);
  while(!Serial);
  Receiver_MACAddress_requir();
  espnow_init();

  esp_now_register_recv_cb(ReceiverXIAOC6_Recive_Data_cb);

  esp_now_register_send_cb(ReceiverXIAOC6_Send_Data_cb);
  Association_SenderXIAOS3_peer();  
}

void loop() {
  ReceiverXIAOC6_Send_Data();
  delay(1000);
}

void espnow_init(){
  status espnow_sign = esp_now_init();
  if(espnow_sign == ESP_OK)
  {
    Serial.println("ESP-NOW 初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 初始化失败");
  }
}

void espnow_deinit(){
  status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("ESP-NOW 反初始化成功！");
  }else
  {
    Serial.println("ESP-NOW 反初始化失败！");
  }
}

void Receiver_MACAddress_requir(){
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    XIAO mac[MAX_ESP_NOW_MAC_LEN];
    while(!WiFi.STA.started()){
      Serial.print(".");
      delay(100);
    }
      WiFi.macAddress(mac);
      Serial.println();
      Serial.printf("const uint8_t mac_self[6] = {0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x};", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println();
}

void ReceiverXIAOC6_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOS3_SENDER_INFORATION, incomingData, sizeof(XIAOS3_SENDER_INFORATION));
  Serial.print("接收到字节数: ");
  Serial.println(len);
  Serial.print("发送设备: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_device);
  Serial.print("发送目标: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_Trag);
  Serial.println();
}

void ReceiverXIAOC6_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status){
  char macStr[18];
  Serial.print("数据包发送到: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" 发送状态:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "发送成功" : "发送失败");
  Serial.println("");
}

void ReceiverXIAOC6_Send_Data(){
  
  strcpy(XIAOC6_RECEIVER_INFORATION.Reveiver_device, "XIAOC6"); 
  strcpy(XIAOC6_RECEIVER_INFORATION.Reveiver_Trag, "我已收到"); 

  esp_err_t XIAOC6_RECEIVER_INFORATION_data1 = esp_now_send(XIAOS3_Sender_MAC_Address, (uint8_t *)&XIAOC6_RECEIVER_INFORATION, sizeof(receiver_meesage_types));

  if (XIAOC6_RECEIVER_INFORATION_data1 == ESP_OK)
  {
    Serial.println("成功发送：XIAOC6_RECEIVER_INFORATION_data1");
  }
  delay(4000);
}

void Association_SenderXIAOS3_peer(){
  Serial.println("尝试关联 XIAOC6 的对等设备...");
  peerInfo_sender.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo_sender.encrypt = NO_PMK_KEY;

  memcpy(peerInfo_sender.peer_addr, XIAOS3_Sender_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo_sender);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("添加对等设备失败");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("成功添加对等设备");
  }
}
```

#### 解决方案 第 3 部分 代码

包含的库
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心功能
- `espnow_init()`  
  - 作用：初始化 ESPNOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 作用：反初始化 ESPNOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `Receiver_MACAddress_requir()`  
  - 作用：设置 WiFi 模式为 STA，并获取 MAC 地址，打印到串口
- `ReceiverXIAOC6_Send_Data()`  
  - 作用：发送特定消息
- `ReceiverXIAOC6_Recive_Data_cb()`  
  - 作用：这是一个回调函数，执行时会打印消息是否成功发送及目标 MAC 地址
- `Association_SenderXIAOS3_peer()`  
  - 作用：为 XIAO ESP32S3 添加一个通道节点，用于接收消息
- `ReceiverXIAOC6_Send_Data_cb()`  
  - 作用：这是一个回调函数，执行时会打印消息是否成功发送及目标 MAC 地址
- `esp_now_register_send_cb()`  
  - 作用：注册回调函数，验证消息是否已发送到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]
- `esp_now_register_recv_cb()`  
  - 作用：注册回调函数，验证数据是否已成功接收到 MAC 层  
  - 返回值：MAC 层成功接收数据：[ESP_NOW_SEND_SUCCESS]；否则：[ESP_NOW_SEND_FAIL]
- `NO_PMK_KEY`  
  - 作用：选择不加密的设备配对

默认变量
- `#define ESPNOW_WIFI_CHANNEL`  
  - 作用：设置发送方和接收方的 WiFi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 作用：接收方 MAC 地址长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 作用：接受或发送的最大字符数
- `#define BAUD 115200`  
  - 作用：设置串口波特率
- `static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN]`  
  - 作用：存储 XIAO ESP32S3 的 MAC 地址
  - 补充：请注意，这些是我的 MAC 地址，不能直接写入！
- `NO_PMK_KEY`  
  - 作用：选择不加密的设备配对

## 演示效果

以下是使用 ESPNOW 进行 XIAO ESP32 通信的结果

#### 发送端 XIAO ESP32S3 结果 

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/121.png" style={{width:600, height:'auto'}}/></div>

#### 接收端 XIAO ESP32C3 结果

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/122.png" style={{width:600, height:'auto'}}/></div>

#### 接收端 XIAO ESP32C6 结果

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/123.png" style={{width:600, height:'auto'}}/></div>

## ESPNOW 总结

低功耗：
- 适用于电池供电的设备，无需连接 Wi-Fi 即可通信。

快速连接：
- 设备可以快速建立连接，无需复杂的配对过程。

多对多通信：
- 支持多设备之间的通信，允许一个设备向多个设备发送数据。

安全性：
- 支持加密功能，确保数据传输的安全性。

短距离通信：
- 通常用于短距离（几十米）的无线通信。

## 故障排除

### 问题 1：无法连接，程序没有报错

- 检查 XIAO ESP32 的 MAC 地址是否正确
- 检查你的 XIAO ESP32 的 Wi-Fi 通道是否相同
- 重置你的 XIAO ESP32，重新打开串口监视器

### 问题 2：接收到消息，但不完整

- 检测发送方和接收方时，确保它们的结构体成员一致

## 资源

- **[Espressif 官方文档]** [ESPRESSIF ESP-IDF ESP-NOW](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/network/esp_now.html?highlight=espnow#esp-now)

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供多种支持方式，确保您在使用我们的产品时拥有顺畅的体验。我们提供多个沟通渠道，以满足不同的需求和偏好。

<div class="table-center">
  <div class="button_tech_support_container">
  <a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
  <a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
  </div>

  <div class="button_tech_support_container">
  <a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
  </div>
</div>