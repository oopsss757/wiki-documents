---
description: Seeed Studio XIAO ESP32C6 的 WiFi 使用情况。
title: WiFi 使用情况
keywords:
  - esp32c6
  - xiao
  - arduino
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_wifi_usage_esp32c6
sidebar_position: 3
last_update:
  date: 11/16/2024
  author: Agnes
---

# Seeed Studio XIAO ESP32C6 的 WiFi 使用情况

<div class="table-center">
  <table align="center">
    <tr>
        <th>Seeed Studio XIAO ESP32C6</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
    </tr>
      <tr>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
          </a>
      </div></td>
    </tr>
  </table>
</div>

Seeed Studio XIAO ESP32C6 是一款嵌入式开发板，凭借其对 2.4GHz Wifi - 802.11b/g/n 和蓝牙低功耗（BLE）5.0 双无线通信的支持，提供了卓越的 RF 性能。该功能使得 XIAO ESP32C6 能够为各种物联网（IoT）应用提供可靠且高速的无线连接。该开发板还配备了内置陶瓷天线，免去了外接天线的需求，简化了设计过程。ESP32C6 芯片还具有低功耗特性，非常适合用于电池供电的物联网设备。在本教程中，我们将探讨如何利用 XIAO ESP32C6 的 Wi-Fi 功能连接到 Wi-Fi 网络，并执行基本的网络任务。

:::tip
GPIO14 用于选择使用内置天线还是外部天线。在此之前，您需要将 GPIO3 设置为低电平以启用此功能。如果 GPIO14 设置为低电平，则使用内置天线；如果设置为高电平，则使用外部天线。默认设置为低电平。如果要将其设置为高电平，请参考以下代码：
```cpp
void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); // 启用此功能
  delay(100);
  pinMode(14, OUTPUT); 
  digitalWrite(14, HIGH); // 使用外部天线
}
```
:::

## WiFi 库的常用接口

ESP32-C6 提供了丰富的 WiFi 网络功能。通常，我们可以在 ESP32 的内置包中看到 WiFi 库的函数，并选择相应的功能以实现所需的功能。接下来，我们将列出一些常用接口，并介绍它们的使用。

### 通用 WiFi 功能

- `WiFiGenericClass::getHostname()` -- 这是 WiFi 库中的一个函数，用于返回设备的主机名（hostname），作为字符串。主机名是一个唯一的名称，用于标识网络上的设备。此函数会检索先前通过 `WiFiGenericClass::setHostname()` 设置的主机名。如果没有设置主机名，则返回默认的主机名。

- `WiFiGenericClass::persistent(bool persistent)` -- 这是一个方法，用于启用或禁用 ESP32 WiFi 库的持久模式。当启用持久模式时，Wi-Fi 配置信息会存储在非易失性内存（NVM）中，即使在断电或重启后也会保留。当禁用持久模式时，配置信息存储在 RAM 中，断电或重启后会丢失。

	- **输入参数**
		- **persistent**: 如果参数为 true，则启用持久模式；如果为 false，则禁用持久模式。

- `WiFiGenericClass::enableLongRange(bool enable)` -- 该函数用于启用或禁用 WiFi 模块的长距离（LR）功能。当启用时，LR 功能允许模块连接到比平常更远的 WiFi 网络，但数据传输速度较低。

	- **输入参数**
		- **enable**: 将参数设置为 true 以启用该功能，false 以禁用它。

- `WiFiGenericClass::mode(wifi_mode_t m)` -- 该函数用于设置设备的 WiFi 模式。

	- **输入参数**
		- **m**: 指定要设置的模式，可以是以下常量之一，这些常量定义在 wifi_mode_t 枚举中：
			- **WIFI_MODE_NULL**: 禁用 WiFi 站点和接入点模式。
			- **WIFI_MODE_STA**: 启用 WiFi 站点模式，连接到现有的 WiFi 网络。
			- **WIFI_MODE_AP**: 启用接入点模式，创建一个新的 WiFi 网络。
			- **WIFI_MODE_APSTA**: 启用 WiFi 站点和接入点模式。

- `WiFiGenericClass::setSleep(wifi_ps_type_t sleepType)` -- 该函数用于设置 WiFi 模块的省电模式。

	- **输入参数**
		- **sleepType**: 指定要使用的省电模式类型，参数是一个枚举类型，共有三种模式：
			- **WIFI_PS_NONE**: 这是默认的省电模式，WiFi 模块不会进入省电模式。
			- **WIFI_PS_MIN_MODEM**: 在此模式下，WiFi 模块关闭其调制解调器，但仍保持与接入点（AP）的连接。
			- **WIFI_PS_MAX_MODEM**: 在此模式下，WiFi 模块关闭调制解调器和站点，导致与 AP 的连接断开。

### STA functions

- `WiFiSTAClass::status()` -- Return Connection status.

	- **输出**: 返回 wl_status_t 中定义的一个值。
		- **WL_NO_SHIELD**: 表示 Wi-Fi 模块不存在。
    	- **WL_IDLE_STATUS**: 表示 Wi-Fi 模块未执行任何操作。
    	- **WL_NO_SSID_AVAIL**: 表示扫描过程中未找到任何 Wi-Fi 网络。
    	- **WL_SCAN_COMPLETED**: 表示 Wi-Fi 扫描已成功完成。
    	- **WL_CONNECTED**: 表示 ESP32 已成功连接到 Wi-Fi 网络。
    	- **WL_CONNECT_FAILED**: 表示连接 Wi-Fi 网络失败。
    	- **WL_CONNECTION_LOST**: 表示与 Wi-Fi 网络的连接丢失。
    	- **WL_DISCONNECTED**: 表示 ESP32 曾连接过 Wi-Fi 网络，但当前未连接到任何网络。

- `WiFiSTAClass::begin(const char* wpa2_ssid, wpa2_auth_method_t method, const char* wpa2_identity, const char* wpa2_username, const char *wpa2_password, const char* ca_pem, const char* client_crt, const char* client_key, int32_t channel, const uint8_t* bssid, bool connect)` -- 启动与 WPA2 企业 AP 的 WiFi 连接。

	- **输入参数** （可选）
		- **ssid**: SSID 字符串的指针。
		- **method**: WPA2 认证方法（WPA2_AUTH_TLS、WPA2_AUTH_PEAP、WPA2_AUTH_TTLS）。
		- **wpa2_identity**: 实体的指针。
		- **wpa2_username**: 用户名的指针。
		- **wpa2_password**: 密码的指针。
		- **ca_pem**: 指向 CA 证书内容的 .pem 文件字符串。
		- **client_crt**: 指向客户端证书内容的 .crt 文件字符串。
		- **client_key**: 指向客户端密钥内容的 .key 文件字符串。
		- **channel**: 可选，AP 的频道。
		- **bssid**: 可选，AP 的 BSSID / MAC 地址。
		- **connect**: 可选，是否连接。

- `WiFiSTAClass::reconnect()` -- 强制断开连接，然后重新连接到 AP。

	- **输出**: True/False.

- `WiFiSTAClass::disconnect(bool wifioff, bool eraseap)` -- 断开与网络的连接。

	- **输入参数**
		- **wifioff**:  `true` 关闭 Wi-Fi 无线电。
		- **eraseap**:  `true` 从 NVS 内存中擦除 AP 配置信息。
	- **输出**: True/False.

- `WiFiSTAClass::config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1, IPAddress dns2)` -- 更改 IP 配置，禁用 DHCP 客户端。

	- **输入参数**
		- **local_ip**: 静态 IP 配置。
		- **gateway**: 静态网关配置。
		- **subnet**: 静态子网掩码。
		- **dns1**: 静态 DNS 服务器 1。
		- **dns2**: 静态 DNS 服务器 2。

- `WiFiSTAClass::setAutoConnect(bool autoConnect)` -- 已废弃。设置 ESP32 站点在开机时是否自动连接到已记录的 AP。默认启用自动连接。

	- **输入参数**
		- **autoConnect**: autoConnect 布尔值。

	- **输出**: False.

- `WiFiSTAClass::waitForConnectResult(unsigned long timeoutLength)` -- 等待 Wi-Fi 连接结果。

	- **输入参数**
		- **timeoutLength**: 指定等待连接建立的最大时间（以毫秒为单位）。

	- **输出**: wl_status_t 中定义的一个值。

- `WiFiSTAClass::localIP()` -- 获取站点接口的 IP 地址。

	- **输出**: IPAddress 站点的 IP 地址。

- `WiFiSTAClass::macAddress(uint8_t* mac)` -- 获取站点接口的 MAC 地址。

	- **输入参数**
		- **mac** （可选）：指向长度为 WL_MAC_ADDR_LENGTH 的 uint8_t 数组的指针。

	- **输出**: 指向 uint8_t* 的指针。

- `WiFiSTAClass::SSID()` -- 返回当前与网络关联的 SSID。

	- **输出**: SSID.

- `WiFiSTAClass::RSSI(void)` -- 返回当前网络的 RSSI。

	- **输出**: RSSI.

### AP 功能

- `WiFiAPClass::softAP(const char* ssid, const char* passphrase, int channel, int ssid_hidden, int max_connection, bool ftm_responder)` --  在 ESP32-C6 的 WiFi 库中，这是一个用于设置 SoftAP（软件接入点）的函数，允许其他设备连接到 ESP32-C6 并访问其资源。

	- **输入参数**
		-  **ssid**:              SSID 字符串的指针（最大 63 个字符）。
 		-  **passphrase**:        WPA2 密码（最少 8 个字符，若为开放式网络则为 NULL）。
 		-  **channel**:           Wi-Fi 频道号，范围 1 - 13。
 		-  **ssid_hidden**:       网络隐藏（0 = 广播 SSID，1 = 隐藏 SSID）。
 		-  **max_connection**:    最多同时连接的客户端数，1 - 4。

	- **输出**: True/False.

- `WiFiAPClass::softAPgetStationNum()` -- 获取连接到 SoftAP 接口的客户端数量。

	- **输出**: 连接的客户端数量。

- `WiFiAPClass::softAPConfig(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dhcp_lease_start)` -- 配置 SoftAP 的函数。

	- **输入参数**
		- **local_ip**:      接入点的 IP 地址。
		- **gateway**:       网关的 IP 地址。
		- **subnet**:        子网掩码。

	- **输出**: True/False.

- `WiFiAPClass::softAPIP()` -- 获取 SoftAP 接口的 IP 地址。

	- **输出**: IPAddress SoftAP 的 IP 地址。

- `WiFiAPClass::softAPmacAddress(uint8_t* mac)` -- 获取 SoftAP 接口的 MAC 地址。

	- **输入参数**
		- **mac** （可选）：指向长度为 WL_MAC_ADDR_LENGTH 的 uint8_t 数组的指针。

	- **输出**: 指向 uint8_t* 或 String 类型的 MAC 地址。

### WiFi 扫描功能

- `WiFiScanClass::scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan, uint8_t channel, const char * ssid, const uint8_t * bssid)` -- 启动 WiFi 网络扫描以查找可用网络。

	- **输入参数**
		- **async**: 一个布尔值，决定扫描是否异步进行。如果设置为 true，函数会立即返回，并可以稍后通过调用 getScanResults() 函数获取扫描结果。如果设置为 false，函数会阻塞直到扫描完成。
		- **show_hidden**: 一个布尔值，决定是否在扫描结果中包含隐藏的网络。
		- **passive**: 一个布尔值，决定是否进行被动扫描。如果设置为 true，函数在扫描过程中不会发送任何数据包，虽然扫描可能会更长，但在某些情况下可能会更有用。
		- **max_ms_per_chan**: 每个频道扫描的最大时间，单位为毫秒。
		- **channel**: 要扫描的 Wi-Fi 频道。如果设置为 0，函数将扫描所有可用的频道。
		- **ssid**: 一个指向以 null 结尾的字符串的指针，包含要扫描的网络的 SSID。如果设置为 nullptr，函数将扫描所有可用的网络。
		- **bssid**: 一个指向包含接入点 MAC 地址的 6 字节数组的指针。如果设置为 nullptr，函数将扫描所有接入点。

	- **输出**: 函数返回一个整数，表示扫描到的网络数量。

- `WiFiScanClass::getNetworkInfo(uint8_t i, String &ssid, uint8_t &encType, int32_t &rssi, uint8_t* &bssid, int32_t &channel)` -- 从扫描的 WiFi 网络中加载所有信息到指针参数中。

	- **输入参数**
		- **i**: 用于检索指定索引 i 上扫描到的网络信息。
		- **ssid**: ssid 参数是一个 String 类型的引用，用于存储网络的 SSID。
		- **encType**: encType 参数是一个 uint8_t 类型的引用，用于存储网络的加密类型（0 = 开放，1 = WEP，2 = WPA_PSK，3 = WPA2_PSK，4 = WPA_WPA2_PSK）。
		- **rssi**: rssi 参数是一个 int32_t 类型的引用，用于存储网络的接收信号强度指示（RSSI）。
		- **bssid**: bssid 参数是一个 uint8_t* 类型的引用，用于存储网络的 BSSID（MAC 地址）。
		- **channel**:  channel 参数是一个 int32_t 类型的引用，用于存储网络的频道号。

	- **输出**: True/False.

- `WiFiScanClass::SSID(uint8_t i)` -- 返回扫描过程中发现的 SSID。

	- **输入参数**
		- **i**: 指定要获取信息的网络项索引。

	- **输出**: 返回指定网络项的 SSID 字符串。

- `WiFiScanClass::RSSI(uint8_t i)` -- 返回扫描过程中发现的网络的 RSSI。

	- **输入参数**
		- **i**: 指定要获取信息的网络项索引。

	- **输出**: 返回指定网络项的 RSSI，返回值为有符号的整数。

### WiFi 客户端功能

- `WiFiClient::connect(IPAddress ip, uint16_t port, int32_t timeout)` -- 在 WiFiClient 库中，用于连接到远程 IP 地址和端口，并设置超时时间。

	- **输入参数**
		- **ip**:   要连接的服务器的 IP 地址。
		- **port**: 要连接的服务器的端口号。
		- **timeout** （可选）: 连接建立的最大等待时间，单位为毫秒。如果在该时间内无法建立连接，函数将返回错误。如果 timeout 设置为 0，函数将无限期等待连接建立。

- `WiFiClient::stop()` --  该函数用于断开客户端与服务器的连接，并释放客户端使用的套接字/端口。调用此函数后，客户端将无法再发送或接收数据。

- `WiFiClient::setTimeout(uint32_t seconds)` -- 该函数设置客户端等待连接或接收数据的最大秒数。如果连接或数据传输时间超过指定的超时，连接将被关闭。

	- **输入参数**
		- **seconds**:   超时时间，以秒为单位。

- `WiFiClient::write(uint8_t data)` -- 将一个字节的数据写入到通过 WiFiClient 实例连接的服务器。或者使用 `WiFiClient::write(const uint8_t *buf, size_t size)`。

	- **输入参数**
		- **data**:   要通过建立的网络连接发送的单个字节数据。

- `WiFiClient::read()` -- 该函数从连接的服务器读取一个字节的输入数据。它返回读取的字节作为整数值。如果没有数据可读取，返回 -1。或者使用 `read(uint8_t *buf, size_t size)`.

	- **输出**: 返回一个整数，表示接收到的字节数。如果返回值为 0，表示服务器已关闭连接。
- `WiFiClient::peek()` -- 该函数用于检查是否有可用数据可以从服务器读取，但并不真正读取它。

	- **输出**: 返回接收到的下一个字节的数据，但不会将其从接收缓冲区中移除。如果没有数据可用，返回 -1。

- `WiFiClient::available()` -- 该函数用于检查有多少字节的数据可从服务器读取。

	- **输出**: 返回一个整数值，表示可供读取的字节数。

### WiFi 服务器功能

- `WiFiServer::stopAll()` -- 该函数是 WiFiServer 类的一个方法，用于停止通过 WiFiServer 类创建的所有服务器实例。当你希望一次性停止所有服务器时，而不是逐个调用 `stop()` 方法时，使用此函数非常有用。

- `WiFiServer::begin(uint16_t port, int enable)` -- 该函数用于启动一个服务器，监听指定端口的客户端连接。

	- **输入参数**
		- **port**: 监听的端口号。
		- **enable** （可选）: 一个标志，指示服务器在启动后是否立即启用。默认情况下，该标志设置为 true。

- `WiFiServer::hasClient()` -- 该函数用于检查是否有客户端连接到服务器。可以在循环中使用该函数不断检查是否有新连接。

	- **输出**: 如果有客户端连接，将返回一个 WiFiClient 对象；如果没有客户端等待连接，则返回 NULL 指针。

- `WiFiServer::end()` -- 该函数用于停止服务器并释放相关资源。调用此函数后，服务器将无法再接受新的客户端连接。任何现有的客户端连接将保持打开状态，直到客户端或服务器关闭连接。 `WiFiServer::close()` 和 `WiFiServer::stop()` 功能相同。

### WiFi 多个功能

- `WiFiMulti::addAP(const char* ssid, const char *passphrase)` -- 用于将新的接入点（AP）添加到 WiFiMulti 对象的可用 AP 列表中，WiFiMulti 对象将尝试连接到这些 AP。

	- **输入参数**
		- **ssid**: SSID 的指针（最大 63 个字符）。
		- **passphrase**: （对于 WPA2 至少 8 个字符，开放网络使用 NULL）。

	- **输出**: True/False


- `WiFiMulti::run(uint32_t connectTimeout)` -- 该函数尝试按顺序连接到已保存的接入点，直到成功连接为止。

	- **输入参数**
		- **connectTimeout**: 指定等待连接的最大时间，单位为毫秒。如果 connectTimeout 设置为 0，函数将无限期地尝试连接。

	- **输出**: 返回连接状态。

## 扫描附近的 WiFi 网络

以下是一个使用 XIAO ESP32C6 扫描附近 WiFi 网络的示例程序。

:::info
XIAO C6 仅支持 `2.4GHz` 频段。
:::

在 Arduino IDE 中，转到 **File > Examples > WiFi > WiFiScan**。这将加载一个示例程序，扫描 XIAO ESP32C6 范围内的 WiFi 网络。

此功能有助于检查您尝试连接的 WiFi 网络是否在您的板子范围内，或者用于其他应用。您的 WiFi 项目可能无法正常工作，因为它可能由于 WiFi 信号强度不足而无法连接到路由器。

```cpp
#include <WiFi.h>

void setup() {
  Serial.begin(115200);

  // 设置 WiFi 为工作站模式，并断开之前连接的接入点
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Setup done");
}

void loop() {
  Serial.println("Starting Wi-Fi scan...");

  // WiFi.scanNetworks 返回找到的网络数量
  int numNetworks = WiFi.scanNetworks();
  Serial.println("Scan done");

  if (numNetworks == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print(numNetworks);
    Serial.println(" networks found");
    for (int i = 0; i < numNetworks; i++) {
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

  // 等待一会再重新扫描
  delay(5000);
}
```

上传并运行程序后，您应该会看到串口监视器输出 XIAO ESP32C6 可搜索到的附近 WiFi 网络。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/37.png" style={{width:600, height:'auto'}}/></div>

### 程序注释

首先，要使用 XIAO ESP32C6 的 WiFi 功能，您需要在代码中包含 **WiFi.h** 库，如下所示：

```cpp
#include <WiFi.h>
```

XIAO ESP32C6 可以作为 WiFi 工作站、接入点或两者兼有。要设置 WiFi 模式，使用 `WiFi.mode()` 并将所需模式作为参数传递。

```cpp
WiFi.mode(WIFI_STA);
```

当 ESP32 被设置为 Wi-Fi 工作站时，它可以连接到其他网络（例如您的路由器）。

`WiFi.scanNetworks()`返回找到的网络数量。扫描完成后，您可以访问每个网络的参数。 `WiFi.SSID()` 打印特定网络的 SSID。

`WiFi.RSSI()` 返回该网络的 RSSI 值。RSSI 是接收到的信号强度指示（Received Signal Strength Indicator）。它是 RF 客户端设备从接入点或路由器接收到的功率水平的估计值。

最后， `WiFi.encryptionType()` 返回网络的加密类型。该示例中，如果是开放网络会在后面加上 *。不过，该函数可能返回以下选项中的一种（不仅仅是开放网络）：
- WIFI_AUTH_OPEN
- WIFI_AUTH_WEP
- WIFI_AUTH_WPA_PSK
- WIFI_AUTH_WPA2_PSK
- WIFI_AUTH_WPA_WPA2_PSK
- WIFI_AUTH_WPA2_ENTERPRISE

## 连接到 WiFi 网络

要将 ESP32 连接到特定的 Wi-Fi 网络，您需要知道其 SSID 和密码。此外，该网络必须在 ESP32 WiFi 范围内（您可以使用前面的示例扫描 WiFi 网络来检查这一点）。

以下是使用 XIAO ESP32C6 连接到指定网络的示例。在程序中，`initWiFi()` 函数用于连接到网络。

> 要连接到 Wi-Fi 网络，您可以使用 `WiFi.begin()` 函数。该函数以网络的 SSID 和密码作为参数。

```cpp
#include "WiFi.h"

// 用您的网络凭证替换
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  // 设置 WiFi 为工作站模式，并断开之前连接的接入点
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  initWiFi();
}

void loop() {
  // 这里是您的代码
}
```

上传并运行程序，打开串口监视器。在连接到网络时，串口监视器将打印出一串点，直到连接成功，然后会打印出 XIAO 的 IP 地址。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/38.png" style={{width:600, height:'auto'}}/></div>

### 程序注释

让我们快速了解一下这个函数是如何工作的。

首先，设置 WiFi 模式。如果 XIAO ESP32C6 将连接到另一个网络（接入点/热点），它必须处于工作站模式。

```cpp
WiFi.mode(WIFI_STA);
```

然后，使用 `WiFi.begin()` 来连接到网络。您必须将网络的 SSID 和密码作为参数传递：

```cpp
WiFi.begin(ssid, password);
```

连接到 WiFi 网络可能需要一些时间，因此通常我们会添加一个 `while` 循环，不断检查连接是否已经建立，通过 `WiFi.status()` 来进行判断。当连接成功时，它会返回 `WL_CONNECTED`。

如果您想获取 WiFi 连接强度，可以在 WiFi 连接后直接调用 `WiFi.RSSI()`。

## softAP 使用

如果您将 XIAO ESP32C6 设置为接入点（热点），则可以通过任何具有 WiFi 功能的设备连接到 ESP32，而无需连接到路由器。

简单来说，当您将 XIAO ESP32C6 设置为接入点时，您创建了自己的 WiFi 网络，附近的 WiFi 设备（工作站）可以连接到它（比如您的智能手机或电脑）。

在您的 Arduino IDE 中，转到 **File > Examples > WiFi > WiFiAccessPoint**。这个示例将向您展示如何使用 XIAO ESP32C6 创建一个热点，并通过一个简单的网页控制连接到热点的灯的开关。

:::note
1. 我们对示例程序做了一些小的修改，注释掉了 LED_BUILTIN，因为 XIAO ESP32C6 有自己的用户指示灯，我们不需要外部 LED。
2. 当 XIAO ESP32C6 上的用户 LED 引脚设置为高电平时，LED 才会熄灭；当引脚设置为低电平时，LED 才会亮起。
3. 您还需要在程序中修改热点名称和密码为您想要的。
:::

```cpp
/*
  WiFiAccessPoint.ino 创建一个 WiFi 接入点，并提供一个 Web 服务器。

  步骤：
  1. 连接到接入点 "yourAp"
  2. 在浏览器中访问 http://192.168.4.1/H 来打开 LED，或访问 http://192.168.4.1/L 来关闭 LED
     或
     在 PuTTY 终端中运行原始 TCP "GET /H" 和 "GET /L"，IP 地址为 192.168.4.1，端口为 80

  为 arduino-esp32 创建，日期：2018年7月4日
  作者：Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

//#define LED_BUILTIN 2   // 设置连接测试 LED 的 GPIO 引脚，或者如果开发板上有内建 LED，则注释掉此行

// 设置为您想要的凭证。
const char *ssid = "XIAO_ESP32C6";
const char *password = "password";

WiFiServer server(80);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // 如果您希望 AP 为开放的，可以去掉密码参数。
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");
}

void loop() {
  WiFiClient client = server.available();   // 监听传入的客户端

  if (client) {                             // 如果有客户端连接
    Serial.println("New Client.");           // 在串口打印一条消息
    String currentLine = "";                // 创建一个字符串来保存来自客户端的数据
    while (client.connected()) {            // 当客户端连接时循环
      if (client.available()) {             // 如果客户端有数据可读
        char c = client.read();             // 读取一个字节，然后
        Serial.write(c);                    // 在串口显示
        if (c == '\n') {                    // 如果字节是换行符

          // 如果当前行为空，表示收到了两个连续的换行符。
          // 这是客户端 HTTP 请求的结束，因此发送响应：
          if (currentLine.length() == 0) {
            // HTTP 响应头总是以响应代码（例如 HTTP/1.1 200 OK）开始
            // 然后是内容类型，客户端知道将要接收的内容类型，接着是一个空行：
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // HTTP 响应内容跟随在头部之后：
            client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");

            // HTTP 响应以另一个空行结束：
            client.println();
            // 跳出循环：
            break;
          } else {    // 如果收到换行符，则清空 currentLine：
            currentLine = "";
          }
        } else if (c != '\r') {  // 如果收到的是回车符以外的任何字符
          currentLine += c;      // 将它添加到 currentLine 的末尾
        }

        // 检查客户端请求是否是 "GET /H" 或 "GET /L"：
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(LED_BUILTIN, LOW);                 // GET /H 打开 LED
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, HIGH);                // GET /L 关闭 LED
        }
      }
    }
    // 关闭连接：
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
```

上传并运行程序后，XIAO ESP32C6 将创建一个名为 "XIAO_ESP32C6" 的热点。您可以使用计算机或手机连接到此网络，密码为 "password"。然后，在浏览器中打开 "192.168.4.1" 访问控制 LED 开关的网页。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/39.png" style={{width:800, height:'auto'}}/></div>

### 程序注释

在 `setup()` 函数中，有一部分代码用来通过 `softAP()` 方法将 ESP32 设置为接入点（AP）：

```cpp
WiFi.softAP(ssid, password);
```

接下来，我们使用 softAPIP() 方法获取接入点的 IP 地址，并在串口监视器中打印出来。

```cpp
IPAddress myIP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(myIP);
server.begin();
```

这些是你需要在 Web 服务器代码中包含的片段，以将 XIAO ESP32C6 设置为接入点。

## WiFi 与 MQTT 使用

XIAO ESP32C6 是一款强大的主板，支持 MQTT 协议，是需要设备间可靠且高效通信的 IoT 项目的理想选择。

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

// 替换为你的网络凭证
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// MQTT 代理的 IP 地址
const char* mqtt_server = "test.mosquitto.org";

// 初始化 WiFi 和 MQTT 客户端对象
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);

  // 连接 WiFi 网络
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("正在连接 WiFi...");
  }

  Serial.println("已连接到 WiFi");

  // 设置 MQTT 代理的 IP 地址和端口
  client.setServer(mqtt_server, 1883);

  // 连接到 MQTT 代理
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("已连接到 MQTT 代理");
    } else {
      Serial.print("连接到 MQTT 代理失败，错误码=");
      Serial.print(client.state());
      Serial.println(" 5 秒后重试");
      delay(5000);
    }
  }

  // 订阅 MQTT 主题
  client.subscribe("test/topic");
}

void loop() {
  // 检查 MQTT 客户端是否连接
  if (!client.connected()) {
    // 重新连接到 MQTT 代理
    if (client.connect("ESP32Client")) {
      Serial.println("已连接到 MQTT 代理");
      // 重新连接后订阅主题
      client.subscribe("test/topic");
    }
  }

  // 处理 MQTT 消息
  client.loop();

  // 向 MQTT 代理发布消息
  client.publish("test/topic", "来自 XIAO ESP32C6 的问候");
  delay(5000);
}
```

在这个示例程序中，XIAO ESP32C6 通过 WiFi 连接到网络，并连接到指定的 MQTT 代理，订阅了主题  **test/topic**，并每 5 秒向该主题发布一条消息。

当 XIAO ESP32C6 从 MQTT 代理接收到消息时，可以通过 `client.onMessage` 回调函数进行处理。你需要将示例程序中的 `ssid`、`password`、`mqtt_server` 等变量替换为你自己的网络和 MQTT 服务器信息。

:::tip
示例程序中提供的 MQTT 服务器地址 `test.mosquitto.org` 仅供测试使用。请勿将任何个人信息发送到该地址，因为任何人都可以使用该链接获取你的信息。
:::

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/41.png" style={{width:800, height:'auto'}}/></div>


## WiFi 与 HTTP/HTTPS 使用

这部分可以参考我们为 XIAO ESP32C3 编写的示例，介绍了如何使用 WiFiClient 和 HTTPClient。

- [Learn to use WiFiClient and HTTPClient on XIAO ESP32C3 - XIAO ESP32C3 与 ChatGPT 实战](https://wiki.seeedstudio.com/xiaoesp32c3-chatgpt)

## WiFi Mesh

根据 [Espressif 文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/mesh.html):

"ESP-MESH 是一个基于 Wi-Fi 协议构建的网络协议。ESP-MESH 允许多个设备（称为节点）在一个大的物理区域（室内和室外）内通过一个单一的 WLAN（无线局域网）互联。ESP-MESH 是自组织和自愈的，意味着该网络可以自主构建和维护。"

在传统的 Wi-Fi 网络架构中，一个节点（接入点——通常是路由器）连接到所有其他节点（站点）。每个节点可以通过接入点互相通信，但这仅限于接入点的 Wi-Fi 覆盖范围，每个站点必须处于接入点的范围内才能直接连接。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/42.png" style={{width:800, height:'auto'}}/></div>

使用 ESP-MESH 后，节点不再需要连接到中心节点。节点负责转发彼此的传输，这使得多个设备可以在广泛的物理区域内分布。节点可以自组织并动态地相互通信，确保数据包能够到达最终节点。如果某个节点从网络中移除，网络将自组织确保数据包仍能到达目标。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/43.png" style={{width:800, height:'auto'}}/></div>

[painlessMesh 库](https://gitlab.com/painlessMesh/painlessMesh) 让我们可以轻松地用 ESP32 板创建一个 Mesh 网络。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/44.png" style={{width:800, height:'auto'}}/></div>

如果弹出窗口提示我们下载一些依赖包来使用该库，我们也需要一起下载它们。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/45.png" style={{width:500, height:'auto'}}/></div>

如果没有弹出窗口，你需要安装以下库依赖：

- [ArduinoJson](https://github.com/bblanchon/ArduinoJson) (by bblanchon)
- [TaskScheduler](https://github.com/arkhipenko/TaskScheduler)
- [AsyncTCP](https://github.com/me-no-dev/AsyncTCP) (ESP32)

要开始使用 ESP-MESH，首先我们将尝试该库的基本示例。这个示例创建了一个 Mesh 网络，其中所有的板子都向其他板子广播消息。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/46.png" style={{width:700, height:'auto'}}/></div>

在上传代码之前，你可以设置 `MESH_PREFIX`（它是 Mesh 网络的名称）和 `MESH_PASSWORD`（它是 Mesh 密码，设置为你喜欢的值）。

然后，建议你为每个板子更改以下行，以便轻松识别发送消息的节点。例如，对于节点 1，可以更改消息如下：

```cpp
String msg = "Hi from node 1 ";
```

接下来，我们以两块 XIAO ESP32C6 为例，网络连接后的概念图大致如下。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/47.png" style={{width:700, height:'auto'}}/></div>

分别将程序上传到两块 XIAO，打开串口监视器并将波特率设置为 115200。（如果有两块 XIAO，可能需要额外的串口软件），如果程序运行顺利，你将看到以下结果：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/48.png" style={{width:800, height:'auto'}}/></div>

### 程序注释

首先，包含 `painlessMesh` 库。接着，添加 Mesh 网络的详细信息。`MESH_PREFIX` 是 Mesh 网络的名称，`MESH_PASSWORD` 是 Mesh 密码。所有节点必须使用相同的 `MESH_PREFIX` 和 `MESH_PASSWORD`。`MESH_PORT` 指定了 Mesh 服务器运行的 TCP 端口，默认为 **5555**.

建议在 Mesh 网络代码中避免使用 `delay()`。为了维护 Mesh 网络，某些任务需要在后台执行。使用 `delay()` 会阻止这些任务的执行，并可能导致 Mesh 网络的稳定性下降或崩溃。相反，建议使用 `TaskScheduler` 来运行任务，`painlessMesh` 库本身就使用了这个方法。以下代码行创建了一个新的 `Scheduler`，命名为 `userScheduler`。

```cpp
Scheduler userScheduler; // 控制个人任务
```

创建一个名为 mesh 的 `painlessMesh` 对象来处理 Mesh 网络。

```cpp
painlessMesh  mesh;
```

创建一个名为 `taskSendMessage` 的任务，该任务负责每秒调用一次 `sendMessage()` 函数，只要程序在运行。

```cpp
Task taskSendMessage(TASK_SECOND * 1 , TASK_FOREVER, &sendMessage);
```

`sendMessage()` 函数将消息发送到 Mesh 网络中的所有节点（广播）。

```cpp
void sendMessage() {
  String msg = "Hello from node 1";
  msg += mesh.getNodeId();
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
}
```

该消息包含“来自节点 1 的问候”文本，后跟板卡芯片 ID。

要广播消息，只需使用 mesh 对象的 `sendBroadcast()` 方法，并将你想要发送的消息（msg）作为参数传递。

```cpp
mesh.sendBroadcast(msg);
```

每次发送新消息时，代码都会更改消息之间的间隔（1 到 5 秒）。

```cpp
taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
```

接下来，创建多个回调函数，这些函数将在 Mesh 网络上发生特定事件时被调用。`receivedCallback()` 函数打印消息发送者（from）和消息内容（`msg.c_str()`）。

```cpp
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}
```

`newConnectionCallback()` 函数每当一个新节点加入网络时运行。此函数简单地打印新节点的芯片 ID。你可以修改该函数以执行其他任务。

```cpp
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}
```

`changedConnectionCallback()` 函数每当网络中的连接发生变化时运行（例如，节点加入或离开网络）。

```cpp
void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}
```

`nodeTimeAdjustedCallback()` 函数在网络调整时间时运行，以确保所有节点同步。它打印时间偏移量。

```cpp
void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}
```

`setup()`, 函数中，初始化串口监视器。选择所需的调试信息类型：

```cpp
//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // 启用所有类型

mesh.setDebugMsgTypes( ERROR | STARTUP );  // 在 init() 之前设置，以便查看启动消息
```

使用之前定义的详细信息初始化 Mesh。

```cpp
mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
```

将所有回调函数分配给其对应的事件。

```cpp
mesh.onReceive(&receivedCallback);
mesh.onNewConnection(&newConnectionCallback);
mesh.onChangedConnections(&changedConnectionCallback);
mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
```

最后，将 taskSendMessage 函数添加到 userScheduler。调度器负责在正确的时间处理和运行任务。

```cpp
userScheduler.addTask(taskSendMessage);
```

最后，启用 taskSendMessage，这样程序就会开始向 Mesh 发送消息。

```cpp
taskSendMessage.enable();
```

为了保持 Mesh 网络的运行，将 `mesh.update()` 添加到 `loop()` 中。

```cpp
void loop() {
  // 它也会运行用户调度器
  mesh.update();
}
```

## 引用与参考

本文内容参考了 **[Random Nerd Tutorials](https://randomnerdtutorials.com/)**上的 ESP32 网络教程，并在 Seeed Studio 的 XIAO ESP32C6 上验证。

特别感谢 **Random Nerd Tutorials** 的作者们，他们的辛勤工作使得这些内容得以分享！

以下是原始文章的参考链接，欢迎通过以下链接了解更多关于 ESP32 网络的知识

- [ESP32 Useful Wi-Fi Library Functions (Arduino IDE)](https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/)
- [ESP32 MQTT – Publish and Subscribe with Arduino IDE](https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/)
- [ESP-MESH with ESP32 and ESP8266: Getting Started (painlessMesh library)](https://randomnerdtutorials.com/esp-mesh-esp32-esp8266-painlessmesh/)

欲了解更多关于使用 ESP32 开发板的信息，请阅读 Random Nerd Tutorials 的官方网站。

- [Random Nerd Tutorials](https://randomnerdtutorials.com/)

并参考以下文档获取更多 Arduino API 的详细信息：

- [WiFi - Arduino Reference](https://www.arduino.cc/en/Reference/WiFi)
- [Wi-Fi API - Arduino ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html)

## 技术支持与产品讨论

感谢您选择我们的产品！我们提供多种支持方式，确保您使用我们的产品时有流畅的体验。我们提供多个沟通渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>






