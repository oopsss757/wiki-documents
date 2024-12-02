---
description: Seeed Studio XIAO ESP32C3上的WiFi使用。
title: WiFi 使用
keywords:
- Xiao
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/XIAO_ESP32C3_WiFi_Usage
last_update:
  date: 11/12/2024
  author: Agnes
---

# 使用 XIAO ESP32C3 的 Wi-Fi 功能

Seeed Studio XIAO ESP32C3支持IEEE 802.11b/g/n的WiFi连接。本wiki将介绍该板子上WiFi使用的基本知识。

:::cautionATTENTION
⚠️ 当您尝试使用该板作为热点(接入点)时，请小心。可能会有过热的问题，导致烧伤。
:::

## 硬件设置

- **步骤 1.** 将附带的**WiFi/蓝牙天线**连接到电路板上的**IPEX连接器**
<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-6.png" alt="pir" width={130} height="auto" /></div>

- **步骤  2.**通过USB Type-C数据线将XIAO ESP32C3连接到您的计算机

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/cable-connect.png" alt="pir" width={120} height="auto" /></div>

## 扫描WiFi网络(站模式)

在本例中，我们将使用XIAO ESP32C3扫描其周围可用的WiFi网络。在这里，单板将配置为STA (Station)模式。

- **步骤 1.** 将下面的代码复制粘贴到Arduino IDE中

<Tabs>
  <TabItem value="basic wifi scan" label="基础 Wi-Fi 扫描" default>

```cpp
#include "WiFi.h"

void setup() {
  Serial.begin(115200);

  // 设置 Wi-Fi 为站点模式，并在之前连接的 AP 中断开连接
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("设置完成");
}

void loop() {
  Serial.println("扫描开始");

  // WiFi.scanNetworks 将返回找到的网络数量
  int n = WiFi.scanNetworks();
  Serial.println("扫描完成");
  if (n == 0) {
    Serial.println("未找到网络");
  } else {
    Serial.print(n);
    Serial.println(" 个网络被发现");
    for (int i = 0; i < n; ++i) {
      // 打印每个网络的 SSID 和 RSSI
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");

  // 等待一段时间再进行下一次扫描
  delay(5000);
}
```

  </TabItem>
  <TabItem value="advan-wifi-scan" label="高级 Wi-Fi 扫描">

```cpp title="https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiScan/WiFiScan.ino"
/*
 *  此草图演示如何扫描 Wi-Fi 网络。
 *  该 API 基于 Arduino WiFi Shield 库，但进行了重大更改，因为支持了更新的 Wi-Fi 功能。
 *  例如，`encryptionType()` 的返回值有所不同，因为支持了更现代的加密方式。
 */
#include "WiFi.h"

void setup() {
  Serial.begin(115200);

  // 设置 Wi-Fi 为站点模式，并在之前连接的 AP 中断开连接。
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("设置完成");
}

void loop() {
  Serial.println("扫描开始");

  // WiFi.scanNetworks 将返回找到的网络数量。
  int n = WiFi.scanNetworks();
  Serial.println("扫描完成");
  if (n == 0) {
    Serial.println("未找到网络");
  } else {
    Serial.print(n);
    Serial.println(" 个网络被发现");
    Serial.println("编号 | SSID                             | RSSI | 通道 | 加密方式");
    for (int i = 0; i < n; ++i) {
      // 打印每个网络的 SSID 和 RSSI
      Serial.printf("%2d", i + 1);
      Serial.print(" | ");
      Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
      Serial.print(" | ");
      Serial.printf("%4ld", WiFi.RSSI(i));
      Serial.print(" | ");
      Serial.printf("%2ld", WiFi.channel(i));
      Serial.print(" | ");
      switch (WiFi.encryptionType(i)) {
        case WIFI_AUTH_OPEN:            Serial.print("开放"); break;
        case WIFI_AUTH_WEP:             Serial.print("WEP"); break;
        case WIFI_AUTH_WPA_PSK:         Serial.print("WPA"); break;
        case WIFI_AUTH_WPA2_PSK:        Serial.print("WPA2"); break;
        case WIFI_AUTH_WPA_WPA2_PSK:    Serial.print("WPA+WPA2"); break;
        case WIFI_AUTH_WPA2_ENTERPRISE: Serial.print("WPA2-EAP"); break;
        case WIFI_AUTH_WPA3_PSK:        Serial.print("WPA3"); break;
        case WIFI_AUTH_WPA2_WPA3_PSK:   Serial.print("WPA2+WPA3"); break;
        case WIFI_AUTH_WAPI_PSK:        Serial.print("WAPI"); break;
        default:                        Serial.print("未知");
      }
      Serial.println();
      delay(10);
    }
  }
  Serial.println("");

  // 删除扫描结果以释放内存
  WiFi.scanDelete();

  // 等待一段时间再进行下一次扫描。
  delay(5000);
}
```
  </TabItem>
</Tabs>

**步骤 2.** 上传代码并打开串行监视器开始扫描WiFi网络

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-1.jpg" alt="pir" width={500} height="auto" /></div>

## 连接WiFi网络

在本例中，我们将使用XIAO ESP32C3连接WiFI网络。

- **步骤 1.** 将下面的代码复制粘贴到Arduino IDE中

<Tabs>
  <TabItem value="basic wifi connect" label="基础 Wi-Fi 连接" default>

```cpp
#include <WiFi.h>

const char* ssid = "你的-SSID";
const char* password = "你的-密码";

void setup() {
  Serial.begin(115200);
  delay(10);

  // 我们首先连接到一个 Wi-Fi 网络
  Serial.println();
  Serial.println();
  Serial.print("连接到 ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Wi-Fi 连接成功");
  Serial.println("IP 地址: ");
  Serial.println(WiFi.localIP());
}
void loop() {}
```

  </TabItem>
  <TabItem value="advan-wifi-connect" label="高级 Wi-Fi 连接">

```cpp title="https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiClientConnect/WiFiClientConnect.ino"
#include <WiFi.h>

const char *ssid = "你的-SSID";
const char *password = "你的-密码";

void setup() {
  Serial.begin(115200);
  delay(10);

  // 我们首先连接到一个 Wi-Fi 网络
  Serial.println();
  Serial.println();
  Serial.print("连接到 ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // 尝试约 10 秒钟（20 次 500 毫秒）
  int tryDelay = 500;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  while (true) {
      switch (WiFi.status()) {
        case WL_NO_SSID_AVAIL: Serial.println("[WiFi] 未找到 SSID"); break;
        case WL_CONNECT_FAILED:
          Serial.print("[WiFi] 连接失败 - Wi-Fi 未连接！原因：");
          return;
          break;
        case WL_CONNECTION_LOST: Serial.println("[WiFi] 连接丢失"); break;
        case WL_SCAN_COMPLETED:  Serial.println("[WiFi] 扫描完成"); break;
        case WL_DISCONNECTED:    Serial.println("[WiFi] Wi-Fi 已断开连接"); break;
        case WL_CONNECTED:
          Serial.println("[WiFi] Wi-Fi 已连接！");
          Serial.print("[WiFi] IP 地址: ");
          Serial.println(WiFi.localIP());
          return;
          break;
        default:
          Serial.print("[WiFi] Wi-Fi 状态: ");
          Serial.println(WiFi.status());
          break;
      }
          delay(tryDelay);

    if (numberOfTries <= 0) {
      Serial.print("[WiFi] 无法连接到 Wi-Fi！");
      // 使用断开连接功能强制停止连接尝试
      WiFi.disconnect();
      return;
    } else {
      numberOfTries--;
    }
  }

  Serial.println("");
  Serial.println("Wi-Fi 连接成功");
  Serial.println("IP 地址: ");
  Serial.println(WiFi.localIP());
}
void loop() {}
```

  </TabItem>
</Tabs>

**步骤 2.** 上传代码，打开串口监视器，检查单板是否连接WiFI网络


<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-2.jpg" alt="pir" width={500} height="auto" /></div>

## 模式 2：Soft-AP 模式（作为 STA）- 用作接入点

在本例中，我们将使用XIAO ESP32C3作为WiFi接入点，其他设备可以连接到它。这类似于手机的WiFi热点功能。

- **步骤 1.** 将下面的代码复制粘贴到Arduino IDE中

```cpp
#include "WiFi.h"
void setup()
{
  Serial.begin(115200);
  WiFi.softAP("ESP_AP", "123456789");
}

void loop()
{
  Serial.print("Host Name:");
  Serial.println(WiFi.softAPgetHostname());
  Serial.print("Host IP:");
  Serial.println(WiFi.softAPIP());
  Serial.print("Host IPV6:");
  Serial.println(WiFi.softAPIPv6());
  Serial.print("Host SSID:");
  Serial.println(WiFi.SSID());
  Serial.print("Host Broadcast IP:");
  Serial.println(WiFi.softAPBroadcastIP());
  Serial.print("Host mac Address:");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("Number of Host Connections:");
  Serial.println(WiFi.softAPgetStationNum());
  Serial.print("Host Network ID:");
  Serial.println(WiFi.softAPNetworkID());
  Serial.print("Host Status:");
  Serial.println(WiFi.status());
  delay(1000);
}
```

:::caution note
如果您的 ESP32 开发板版本已经更新到 3.0.0，则需要将代码从 ```softAPIPv6()``` 更改为 ```softAPlinkLocalIPv6()```。
:::

**步骤 2.** 上传代码并打开Serial Monitor查看有关WiFI接入点的更多细节

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-3.png" alt="pir" width={700} height="auto" /></div>

**步骤  3.** 扫描个人电脑或移动电话上的WiFi网络，您将能够使用我们在代码中指定的密码连接到这个新创建的网络

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-4.png" alt="pir" width="{300}" height="auto" /></div>

现在您将看到串行监视器上的主机连接数**已更新为1**

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-5.png" alt="pir" width={700} height="auto" /></div>

## XIAO ESP32C3 & 家庭助手

我们很高兴地宣布，我们已经支持XIAO ESP32C3访问ESPHome和家庭助手!
有关本节的更多信息，请参考相关教程。

- [使用ESPHome将Grove模块连接到家庭助理](https://wiki.seeedstudio.com/Connect-Grove-to-Home-Assistant-ESPHome/)
- [LinkStar家庭助手](https://wiki.seeedstudio.com/h68k-ha-esphome/)

## 参考

- [Wi-Fi API - esp-arduino](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html)

## 技术支持和产品讨论
感谢您选择我们的产品！我们在这里为您提供不同的支持，以确保您在使用我们产品的过程中尽可能顺利。我们提供多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
