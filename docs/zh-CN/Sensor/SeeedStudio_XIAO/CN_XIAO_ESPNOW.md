---
description: 使用 XIAO ESP32S3/XIAO ESP32C3/XIAO ESP32C6 ESP-NOW 协议进行通信
title: ESP-NOW 登陆 XIAO ESP32 系列
keywords:
- ESPNOW
image: https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/126.png
slug: /cn/xiao_espnow
last_update:
  date: 07/24/2024
  author: Agnes
---

# 使用 ESP-NOW 协议在 XIAO 系列板上运行

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/126.png" style={{width:1100, height:'auto'}}/></div>
<br />

本文将向您介绍 ESP-NOW 协议，并教您如何使用 XIAO ESP32 系列通过该协议进行通信。过程非常简单。为了让大家能够利用 XIAO ESP32 系列使用 ESP-NOW 协议进行通信，我们准备了三种 XIAO ESP32 型号：C6、C3 和 S3，它们将互相通信，赶快开始这段旅程吧！

顺便提一下，如果您刚刚获取了这些开发板，请点击以下链接，它将教您如何开始使用：
- [Seeed Studio XIAO ESP32S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Seeed Studio XIAO ESP32C3](https://wiki.seeedstudio.com/xiao_esp32c3_getting_started/)
- [Seeed Studio XIAO ESP32C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)

## 什么是 ESP-NOW 协议？

官方定义：ESP-NOW 是由 Espressif 定义的一种无线通信协议，能够实现智能设备的直接、快速和低功耗控制，无需路由器。它可以与 Wi-Fi 和蓝牙低能耗（Bluetooth LE）共存，支持多种 SoC 系列，如 Lexin ESP8266、ESP32、ESP32-S 和 ESP32-C。ESP-NOW 广泛应用于智能家电、遥控器和传感器等领域。

以下是 ESP-NOW 的特点：
- 根据 MAC 地址连接方式，可以在没有网络的情况下快速配对，设备可以通过单对多、单对单、多对单和多对多等方式进行连接。
- ESP-NOW 是基于数据链路层的无线通信协议，简化了五层 OSI 上层协议为一层，不需要添加数据包头，也不需要逐层解包。它极大地缓解了网络拥塞时数据包丢失所造成的延迟，并且具有更高的响应速度。

与 Wi-Fi 和蓝牙相比：
- Wi-Fi：ESP-NOW 支持设备间的点对点通信，因此具有更低的功耗和更高的传输速度，同时也拥有更长的通信距离。
- 蓝牙：ESP-NOW 不需要配对过程，使用起来更简单、更方便，功耗更低，传输速度更高。

不过，ESP-NOW 更适用于需要快速、可靠、低功耗和点对点通信的应用场景，而蓝牙和 Wi-Fi 更适用于复杂的网络环境和设备较多的场景。

## 硬件准备

在本项目中，为了考虑到某些用户可能只拥有 XIAO ESP32S3、XIAO ESP32C3 或 XIAO ESP32C6，我们使用三种 XIAO ESP32 型号（XIAO ESP32S3、XIAO ESP32C3 和 XIAO ESP32C6）来相互通信。您只需要稍微修改代码，就可以使用上述任意两种或三种型号进行实际操作。接下来我们来看一下代码是如何实现的，让我们开始吧！

如果您还没有两块 XIAO ESP32 系列的开发板，下面是购买链接：

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

首先，让我们了解一下代码的一般框架。这个实例使用了三块 XIAO ESP32 板，XIAO ESP32S3 作为发送端，XIAO ESP32C6 和 XIAO ESP32C3 作为接收端。当然，这只是本代码中的角色分配。接下来通过我的解释，如果你想修改、增加或删除接收端和发送端的角色，将会非常简单。让我们开始吧！

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

//您需要输入您的 XIAO ESP32 系列 MAC，无法直接复制!!!
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
  Serial.print("Packet to: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("");
}

void Association_ReceiverXIAOC3_peer(){
  Serial.println("Attempting to associate peer for XIAOC3...");
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = NO_PMK_KEY;

  memcpy(peerInfo.peer_addr, Receiver_XIAOC3_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("Failed to add peer");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("Successful to add peer");
  }
}

void Association_ReceiverXIAOC6_peer(){
  Serial.println("Attempting to associate peer for XIAOC6...");
  peerInfo1.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo1.encrypt = NO_PMK_KEY;

  memcpy(peerInfo1.peer_addr, Receiver_XIAOC6_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo1);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("Failed to add peer");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("Successful to add peer");
  }
}

void SenderXIAOS3_Send_Data(){
  
  strcpy(Personal_XIAOC3_Information.device, "XIAOS3"); 
  strcpy(Personal_XIAOC3_Information.Trag, "Hello,i'm sender"); 

  strcpy(Personal_XIAOC6_Information.device, "XIAOS3"); 
  strcpy(Personal_XIAOC6_Information.Trag, "Hello,i'm sender"); 

  esp_err_t XIAOS3_RECEIVER_INFORATION_data1 = esp_now_send(Receiver_XIAOC3_MAC_Address, (uint8_t *)&Personal_XIAOC3_Information, sizeof(message_types));
  esp_err_t XIAOS3_RECEIVER_INFORATION_data2 = esp_now_send(Receiver_XIAOC6_MAC_Address, (uint8_t *)&Personal_XIAOC6_Information, sizeof(message_types));

  if (XIAOS3_RECEIVER_INFORATION_data1 == ESP_OK || XIAOS3_RECEIVER_INFORATION_data2 == ESP_OK)
  {
    Serial.println("Sent with success: XIAOS3_RECEIVER_INFORATION_data1 and XIAOS3_RECEIVER_INFORATION_data2");
  }
  delay(4000);
}

void ReceiverXIAOC3_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOC3_RECEIVER_INFORATION, incomingData, sizeof(XIAOC3_RECEIVER_INFORATION));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Reveiver_device: ");
  Serial.println(XIAOC3_RECEIVER_INFORATION.Reveiver_device);
  Serial.print("Reveiver_Trag: ");
  Serial.println(XIAOC3_RECEIVER_INFORATION.Reveiver_Trag);
  Serial.println();
}

void ReceiverXIAOC6_Recive_Data_cb(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&XIAOC6_RECEIVER_INFORATION, incomingData, sizeof(XIAOC6_RECEIVER_INFORATION));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Reveiver_device: ");
  Serial.println(XIAOC6_RECEIVER_INFORATION.Reveiver_device);
  Serial.print("Reveiver_Trag: ");
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
    Serial.println("the esp now is successful init!");
  }else
  {
    Serial.println("the esp now is failed init");
  }
}

void espnow_deinit(){
  XIAO_status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("the esp now is successful deinit!");
  }else
  {
    Serial.println("the esp now is failed deinit!");
  }
}
```

#### 第 1 部分 解决方案代码

包含库：
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心函数：
- `espnow_init()`  
  - 角色：初始化 ESP-NOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 角色：反初始化 ESP-NOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `SenderXIAOS3_MACAddress_Requir()`  
  - 角色：将 Wi-Fi 模式设置为 STA，并获取 MAC 地址以便打印在串口上
- `SenderXIAOS3_Send_Data()`  
  - 角色：发送特定的消息
- `SenderXIAOS3_Send_Data_cb()`  
  - 角色：这是一个回调函数，当它执行时，会打印消息是否成功发送以及发送到哪个 MAC 地址
- `Association_ReceiverXIAOC3_peer()` 和 `Association_ReceiverXIAOC6_peer`  
  - 角色：添加对等节点，如果需要更多接收器，可以创建节点，并编写与发送端和接收端匹配的消息
- `esp_now_register_send_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]
- `ReceiverXIAOC3_Recive_Data_cb()`  
  - 角色：接受来自发送端的数据的回调函数
- `ReceiverXIAOC6_Recive_Data_cb()`  
  - 角色：接受来自发送端的数据的回调函数
- `esp_now_register_recv_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]

默认变量：
- `#define ESPNOW_WIFI_CHANNE`  
  - 角色：发送端和接收端所在的 Wi-Fi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 角色：MAC 地址长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 角色：接收或发送的最大字符数
- `#define BAUD 115200`  
  - 角色：设置串口的波特率
- `static uint8_t Receiver_XIAOC3_MAC_Address[MAX_ESP_NOW_MAC_LEN]` 和 `static uint8_t Receiver_XIAOC6_MAC_Address`  
  - 角色：存储我的 XIAO ESP32C3 和 XIAO ESP32C6 的 MAC 地址，它们作为接收端。  
  - 补充：请注意，这些是我的 MAC 地址，不能写入其他地址。
- `NO_PMK_KEY`  
  - 角色：选择不加密的配对方式

### 第 2 部分 XIAO ESP32C3 接收端代码

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

//您需要输入您的 XIAO ESP32 系列 MAC，无法直接复制!!!
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
    Serial.println("the esp now is successful init!");
  }else
  {
    Serial.println("the esp now is failed init");
  }
}

void espnow_deinit(){
  status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("the esp now is successful deinit!");
  }else
  {
    Serial.println("the esp now is failed deinit!");
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
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Sender_device: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_device);
  Serial.print("Sender_Trag: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_Trag);
  Serial.println();
}

void ReceiverXIAOC3_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status){
  char macStr[18];
  Serial.print("Packet to: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    Serial.println("");
}

void ReceiverXIAOC3_Send_Data(){
  
  strcpy(XIAOC3_RECEIVER_INFORATION.Reveiver_device, "XIAOC3"); 
  strcpy(XIAOC3_RECEIVER_INFORATION.Reveiver_Trag, "I'm get it"); 

  esp_err_t XIAOC3_RECEIVER_INFORATION_data1 = esp_now_send(XIAOS3_Sender_MAC_Address, (uint8_t *)&XIAOC3_RECEIVER_INFORATION, sizeof(receiver_meesage_types));

  if (XIAOC3_RECEIVER_INFORATION_data1 == ESP_OK)
  {
    Serial.println("Sent with success: XIAOC3_RECEIVER_INFORATION_data1");
  }
  delay(4000);
}

void Association_SenderXIAOS3_peer(){
  Serial.println("Attempting to associate peer for XIAOC6...");
  peerInfo_sender.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo_sender.encrypt = NO_PMK_KEY;

  memcpy(peerInfo_sender.peer_addr, XIAOS3_Sender_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo_sender);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("Failed to add peer");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("Successful to add peer");
  }
}
```

#### 第 2 部分 解决方案代码

包含库：
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心函数：
- `espnow_init()`  
  - 角色：初始化 ESP-NOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 角色：反初始化 ESP-NOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `Receiver_MACAddress_requir()`  
  - 角色：将 Wi-Fi 模式设置为 STA，并获取 MAC 地址以便打印在串口上
- `ReceiverXIAOC3_Send_Data()`  
  - 角色：发送特定的消息
- `ReceiverXIAOC3_Recive_Data_cb()`  
  - 角色：这是一个回调函数，当它执行时，会打印消息是否成功发送以及发送到哪个 MAC 地址
- `Association_SenderXIAOS3_peer()`  
  - 角色：为 XIAO ESP32S3 添加一个通道节点，以便向其发送消息
- `esp_now_register_send_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]
- `ReceiverXIAOC3_Send_Data_cb()`  
  - 角色：这是一个回调函数，当它执行时，会打印消息是否成功发送以及发送到哪个 MAC 地址
- `esp_now_register_recv_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]

默认变量：
- `#define ESPNOW_WIFI_CHANNE`  
  - 角色：发送端和接收端所在的 Wi-Fi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 角色：MAC 地址长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 角色：接收或发送的最大字符数
- `#define BAUD 115200`  
  - 角色：设置串口的波特率
- `static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN]`  
  - 角色：存储我的 XIAO ESP32S3 的 MAC 地址
  - 补充：请注意，这些是我的 MAC 地址，不能写入其他地址！
- `NO_PMK_KEY`  
  - 角色：选择不加密的配对方式

### 第 3 部分 XIAO ESP32C6 接收端代码

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

//您需要输入您的 XIAO ESP32 系列 MAC，无法直接复制!!!
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
    Serial.println("the esp now is successful init!");
  }else
  {
    Serial.println("the esp now is failed init");
  }
}

void espnow_deinit(){
  status espnow_sign = esp_now_deinit();
  if(espnow_sign == ESP_OK){
    Serial.println("the esp now is successful deinit!");
  }else
  {
    Serial.println("the esp now is failed deinit!");
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
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Sender_device: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_device);
  Serial.print("Sender_Trag: ");
  Serial.println(XIAOS3_SENDER_INFORATION.Sender_Trag);
  Serial.println();
}
void ReceiverXIAOC6_Send_Data_cb(const XIAO *mac_addr,esp_now_send_status_t status){
  char macStr[18];
  Serial.print("Packet to: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  delay(500);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("");
}

void ReceiverXIAOC6_Send_Data(){
  
  strcpy(XIAOC6_RECEIVER_INFORATION.Reveiver_device, "XIAOC6"); 
  strcpy(XIAOC6_RECEIVER_INFORATION.Reveiver_Trag, "I'm get it"); 

  esp_err_t XIAOC6_RECEIVER_INFORATION_data1 = esp_now_send(XIAOS3_Sender_MAC_Address, (uint8_t *)&XIAOC6_RECEIVER_INFORATION, sizeof(receiver_meesage_types));

  if (XIAOC6_RECEIVER_INFORATION_data1 == ESP_OK)
  {
    Serial.println("Sent with success: XIAOC6_RECEIVER_INFORATION_data1");
  }
  delay(4000);
}

void Association_SenderXIAOS3_peer(){
  Serial.println("Attempting to associate peer for XIAOC6...");
  peerInfo_sender.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo_sender.encrypt = NO_PMK_KEY;

  memcpy(peerInfo_sender.peer_addr, XIAOS3_Sender_MAC_Address, 6);
  esp_err_t addPressStatus = esp_now_add_peer(&peerInfo_sender);
  if (addPressStatus != ESP_OK)
  {
    Serial.print("Failed to add peer");
    Serial.println(addPressStatus);
  }else
  {
    Serial.println("Successful to add peer");
  }
}
```

#### 第 3 部分 解决方案代码

包含库：
- `#include "WiFi.h"`
- `#include "esp_now.h"`

核心函数：
- `espnow_init()`  
  - 角色：初始化 ESP-NOW 功能  
  - 返回值：初始化成功：[ESP_OK]；失败：[ESP_FAIL]
- `espnow_deinit()`  
  - 角色：反初始化 ESP-NOW 功能，所有与配对设备相关的信息将被删除  
  - 返回值：初始化成功：[ESP_OK]
- `Receiver_MACAddress_requir()`  
  - 角色：将 Wi-Fi 模式设置为 STA，并获取 MAC 地址以便打印在串口上
- `ReceiverXIAOC6_Send_Data()`  
  - 角色：发送特定的消息
- `ReceiverXIAOC6_Recive_Data_cb()`  
  - 角色：这是一个回调函数，当它执行时，会打印消息是否成功发送以及发送到哪个 MAC 地址
- `Association_SenderXIAOS3_peer()`  
  - 角色：为 XIAO ESP32S3 添加一个通道节点，以便向其发送消息
- `ReceiverXIAOC6_Send_Data_cb()`  
  - 角色：这是一个回调函数，当它执行时，会打印消息是否成功发送以及发送到哪个 MAC 地址
- `esp_now_register_send_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]
- `esp_now_register_recv_cb()`  
  - 角色：注册回调函数，以验证数据是否已经发送到 MAC 层  
  - 返回值：MAC 层成功接收到数据：[ESP_NOW_SEND_SUCCESS]，否则：[ESP_NOW_SEND_FAIL]
- `NO_PMK_KEY`  
  - 角色：选择不加密的配对方式

默认变量：
- `#define ESPNOW_WIFI_CHANNE`  
  - 角色：发送端和接收端所在的 Wi-Fi 通道
- `#define MAX_ESP_NOW_MAC_LEN`  
  - 角色：接收端 MAC 地址的长度
- `#define MAX_CHARACTERS_NUMBER`  
  - 角色：接收或发送的最大字符数
- `#define BAUD 115200`  
  - 角色：设置串口的波特率
- `static uint8_t XIAOS3_Sender_MAC_Address[MAX_ESP_NOW_MAC_LEN]`  
  - 角色：存储我的 XIAO ESP32S3 的 MAC 地址
  - 补充：请注意，这些是我的 MAC 地址，不能写入其他地址！
- `NO_PMK_KEY`  
  - 角色：选择不加密的配对方式

## 演示效果

以下是使用 ESPNOW 协议进行 XIAO ESP32 通信的结果：

#### 发送端 XIAO ESP32S3 结果

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/121.png" style={{width:600, height:'auto'}}/></div>

#### 接收端 XIAO ESP32C3 结果

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/122.png" style={{width:600, height:'auto'}}/></div>

#### 接收端 XIAO ESP32C6 结果

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/123.png" style={{width:600, height:'auto'}}/></div>

## ESPNOW 总结

低功耗：
- 适用于电池供电的设备，可以在不连接 Wi-Fi 的情况下进行通信。

快速连接：
- 设备可以快速建立连接，无需复杂的配对过程。

多对多通信：
- 支持多设备之间的通信，允许一个设备向多个设备发送数据。

安全性：
- 支持加密功能，确保数据传输的安全。

短距离通信：
- 通常用于短距离（几十米）的无线通信。

## 故障排除

### 问题 1：无法连接，程序没有报告任何错误

- 检查您的 XIAO ESP32 的 MAC 地址是否正确
- 确认您的 XIAO ESP32 所连接的 Wi-Fi 通道是否一致
- 重置您的 XIAO ESP32，并重新打开串口监视器

### 问题 2：接收到消息，但不完整

- 在检测发送端和接收端时，可能存在结构体成员相似的问题

## 资源

- **[Espressif 官方文档]** [ESPRESSIF ESP-IDF ESP-NOW](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/network/esp_now.html?highlight=espnow#esp-now)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时获得最佳体验。我们提供多种沟通渠道，以满足不同的需求和偏好。

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