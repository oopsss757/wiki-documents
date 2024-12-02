---
title: Quectel L76K
description: 开始使用 XIAO 的 L76K GNSS 模块
keywords:
  - XIAO
  - Quectel L76K
  - GNSS
image: https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/1-L76K-GNSS-Module-for-Seeed-Studio-XIAO-45font.jpg
slug: /cn/get_start_l76k_gnss
sidebar_position: 0
last_update:
  date: 2024-11-20
  author: Agnes
---

# 用于 SeeedStudio XIAO 的 L76K GNSS 模块入门

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/1-L76K-GNSS-Module-for-Seeed-Studio-XIAO-45font.jpg" style={{width:600, height:'auto'}}/></div>

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://www.seeedstudio.com/L76K-GNSS-Module-for-Seeed-Studio-XIAO-p-5864.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    </a>
</div>

<!-- TODO Add bazzar link -->

## 介绍

SeeedStudio XIAO的L76K GNSS模块是一款多GNSS（全球导航卫星系统）模块，兼容所有XIAO开发板，支持GPS、北斗（BDS）、GLONASS和QZSS系统，支持多系统组合或单系统独立定位。它还支持AGNSS功能，内置低噪声放大器和表面声波滤波器，提供快速、精确、高性能的良好定位体验。

该模块配备了一款高性能的主动GNSS天线，旨在覆盖GPS L1 C/A、北斗B1和GLONASS L1频段。设计中还包含了一颗亮绿色LED，用于指示固定时的1PPS输出。

### 特点

- **增强接收性能：** 内置低噪声放大器和表面声波滤波器，提升灵敏度并减少噪声
- **高精度：** 32/72个通道，-162dBm跟踪，-160dBm重新获取灵敏度
- **能效：** 41mA跟踪/获取，360µA待机
- **多GNSS系统：** 采用Quectel L76K，支持GPS、北斗、GLONASS和QZSS
- **陶瓷天线：** 增强信号接收，优于传统天线。

### 规格

<div class="table-center">
<table align="center">
 <tr>
     <th>项目</th>
     <th>详细信息</th>
 </tr>
 <tr>
     <th>GNSS频段</th>
     <td>GPS L1 C/A: 1575.42MHz<br></br> GLONASS L1: 1602MHz<br></br> 北斗B1: 1561.098MHz</td>
 </tr>
 <tr>
     <th>通道数</th>
     <td>32个跟踪通道/72个获取通道</td>
 </tr>
  <tr>
     <th>TTFF（首次定位时间）</th>
     <td>冷启动：30秒（无AGNSS），5.5秒（有GNSS）<br></br> 热启动：5.5秒（无AGNSS），2秒（有AGNSS）</td>
 </tr>
  <tr>
     <th>灵敏度</th>
     <td>自动获取：-148dBm<br></br> 跟踪：-162dBm<br></br> 重新获取：-160dBm</td>
 </tr>
  <tr>
     <th>精度</th>
     <td>定位：2.0m CEP<br></br> 速度：0.1m/s<br></br> 加速度：0.1m/s²<br></br> 定时：30ns</td>
 </tr>
  <tr>
     <th>UART接口</th>
     <td>波特率：9600~115200bps（默认9600bps）<br></br> 更新率：1Hz（默认），5Hz（最大）<br></br> 协议：NMEA 0183，CASIC专有协议</td>
 </tr>
  <tr>
     <th>天线</th>
     <td>类型：主动天线<br></br> 工作频率：1559–1606MHz<br></br> 同轴电缆：RF1.13，长度=10cm<br></br> 电缆连接器：U.FL接头RA</td>
 </tr>
  <tr>
     <th>功耗（带主动天线）</th>
     <td>自动获取：41mA<br></br> 跟踪：41mA<br></br> 待机：360µA</td>
 </tr>
  <tr>
     <th>尺寸</th>
     <td>18mm x 21mm</td>
 </tr>
</table>
</div>

## 硬件概述

在开始之前，我们可以参考以下图片，了解SeeedStudio XIAO的L76K GNSS模块的引脚设计，帮助我们理解该模块的功能。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/gnss-xiao-pinout.png" style={{width:800, height:'auto'}}/></div>

## 入门指南

### 硬件准备

为了充分体验L76K GNSS模块的功能，我们建议将其与XIAO系列的主板配合使用。以下*任意型号的XIAO*都可以与L76K GNSS模块兼容：

<table align="center">
 <tr>
  <th>Seeed Studio XIAO SAMD21</th>
  <th>Seeed Studio XIAO RP2040</th>
  <th>Seeed Studio XIAO nRF52840 (Sense)</th>
  <th>Seeed Studio XIAO ESP32C3</th>
     <th>Seeed Studio XIAO ESP32S3 (Sense)</th>
 </tr>
 <tr>
  <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Seeeduino-XIAO-preview-1.jpg" style={{width:400, height:'auto'}}/></div></td>
  <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/102010428_Preview-07.jpg" style={{width:500, height:'auto'}}/></div></td>
     <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/round_display_for_xiao/xiaoblesense.jpg" style={{width:500, height:'auto'}}/></div></td>
  <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/round_display_for_xiao/xiaoesp32c3.jpg" style={{width:450, height:'auto'}}/></div></td>
     <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:500, height:'auto'}}/></div></td>
 </tr>
    <tr>
     <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
      <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
      </a>
  </div></td>
  <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
      <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
      </a>
  </div></td>
  <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
      <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
      </a>
  </div></td>
  <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
      <a class="get_one_now_item" href="https://www.seeedstudio.com/seeed-xiao-esp32c3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
      </a>
  </div></td>
     <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
      <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
      </a>
  </div></td>
 </tr>
</table>

在使用此模块与XIAO主板配合之前，您需要在模块上安装引脚座，并将主动GNSS天线连接到模块上。在连接到XIAO时，请特别注意模块的安装方向，请勿反向插入，否则可能会烧毁模块或XIAO。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/gnss-xiao-assembled.png" style={{width:500, height:'auto'}}/></div>

:::caution
请特别注意模块的安装方向，请勿反向插入，否则可能会烧毁模块或XIAO。
:::

### 软件准备

要使用SeeedStudio XIAO的L76K GNSS模块，我们需要对XIAO系列进行编程。推荐的编程工具是Arduino IDE，您需要为XIAO配置Arduino环境并添加板载软件包。

:::tip
如果这是您第一次使用Arduino，强烈建议您参考[Arduino入门指南](/Getting_Started_with_Arduino/)。
:::

#### 步骤1：根据您的操作系统下载并安装Arduino IDE的稳定版本

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载Arduino IDE</font></span></strong>
    </a>
</div>

#### 步骤2：启动Arduino应用程序

#### 步骤3：为您使用的XIAO配置Arduino IDE

- 如果您想使用**Seeed Studio XIAO SAMD21**进行后续程序，请参考**[此教程](/Seeeduino-XIAO/#software)**完成添加。

- 如果您想使用**Seeed Studio XIAO RP2040**进行后续程序，请参考**[此教程](/XIAO-RP2040-with-Arduino/#software-setup)**完成添加。

- 如果您想使用**Seeed Studio XIAO nRF52840**进行后续程序，请参考**[此教程](/XIAO_BLE/#software-setup)**完成添加。

- 如果您想使用**Seeed Studio XIAO ESP32C3**进行后续程序，请参考**[此教程](/XIAO_ESP32C3_Getting_Started#software-setup)**完成添加。

- 如果您想使用**Seeed Studio XIAO ESP32S3**进行后续程序，请参考**[此教程](/xiao_esp32s3_getting_started#software-preparation)**完成添加。

#### 步骤4：将TinyGPSPlus库添加到Arduino

首先，您需要在Arduino IDE中搜索并下载最新版本的**TinyGPSPlus**库。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/installing-tinygpsplus.png" style={{width:800, height:'auto'}}/></div>

## 应用示例

### 示例1：读取并显示GNSS数据

硬件和软件准备好之后，我们可以开始上传第一个示例程序。L76K GNSS模块在上电后每秒通过串口输出一次GNSS信息。在此示例中，我们将使用**TinyGPSPlus**库解析从模块接收到的NMEA句子，并将结果（包括纬度、经度和时间）打印到Arduino IDE的串口监视器。

以下是源代码：

```cpp
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   这个示例代码演示了如何在SeeedStudio XIAO上使用L76K GNSS模块。
*/
static const int RXPin = D7, TXPin = D6;
static const uint32_t GPSBaud = 9600;

// TinyGPSPlus对象
TinyGPSPlus gps;

// 与GNSS模块的串口连接
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
#ifdef ARDUINO_SEEED_XIAO_RP2040
  pinMode(D10,OUTPUT);
  digitalWrite(D10,1);
  pinMode(D0,OUTPUT);
  digitalWrite(D0,1);
#endif
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("一个使用TinyGPSPlus与L76K GNSS模块的简单示范"));
  Serial.print(F("测试TinyGPSPlus库版本："));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("由Mikal Hart提供"));
  Serial.println();
}

void loop() {
  // 每次正确编码一个新的句子时，显示信息
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("没有检测到GPS：请检查连接。"));
    while (true);
  }
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
```

只需选择您使用的XIAO板和XIAO所在的端口号，编译并上传代码。

确保L76K GNSS模块放置在可以接收到良好GNSS信号的户外环境中。上传代码到您的XIAO并等待几分钟，您应该会在串口监视器中看到显示的信息。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/gnss-output.png" style={{width:800, height:'auto'}}/></div>

此代码使用TinyGPSPlus库通过串口连接从L76K GNSS模块读取数据，并在串口监视器上显示有效的位置信息和日期/时间。

## 配置

### 示例1：改变LED的行为

本节演示了如何通过串口通信发送特定的十六进制命令来控制一个绿色LED。下面的示例展示了如何关闭LED，然后将其恢复到正常的闪烁状态。

```cpp
static const int RXPin = D7, TXPin = D6;
static const uint32_t GPSBaud = 9600;
SoftwareSerial SerialGNSS(RXPin, TXPin);

void setup() {
  SerialGNSS.begin(GPSBaud);

  // 定义关闭LED的字节数组
  byte OffState[] = {0xBA, 0xCE, 0x10, 0x00, 0x06, 0x03, 0x40, 
                     0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00, 
                     // highlight-start
                     0x00, 
                     // highlight-end
                     0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 
                     // highlight-start
                     0xF0, 
                    // highlight-end
                     0xC8, 0x17, 0x08};

  // 定义恢复LED闪烁状态的字节数组
  byte RecoverState[] = {0xBA, 0xCE, 0x10, 0x00, 0x06, 0x03, 0x40, 
                         0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00, 
                         // highlight-start
                         0x03, 
                         // highlight-end
                         0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00,
                         // highlight-start
                         0xF3, 
                         // highlight-end
                         0xC8, 0x17, 0x08};

  // 发送关闭LED的命令
  SerialGNSS.write(OffState, sizeof(OffState));
  // 等待5秒
  delay(5000);
  // 发送恢复LED闪烁状态的命令
  SerialGNSS.write(RecoverState, sizeof(RecoverState));
}

void loop() {}
```

:::info
有关详细信息，请参阅 Quectel_L76K_GNSS 的 CASIC 协议消息。

```c
struct CASIC_Messages {  
  uint16_t header; // 0xBA, 0xCE
  uint16_t len;    // 0x10, 0x00
  uint8_t class;   // 0x06
  uint8_t id;      // 0x03
  uint8_t* payload; // 0x40, 0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00, ->8
                   // 0x00, 0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, ->8
  uint8_t checksum; // 0xF0,0xC8, 0x17, 0x08
} L76KStruct;
```

:::

## 资源

- **PDF**: [L76K GNSS 模块 for Seeed Studio XIAO 原理图](https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/L76K/109100021-L76K-GNSS-Module-for-Seeed-Studio-XIAO-Schematic.pdf)
- **PDF**: [Quectel_L76K_GNSS_协议规范_V1.0](https://raw.githubusercontent.com/Seeed-Projects/Seeed_L76K-GNSS_for_XIAO/fb74b715224e0ac153c3884e578ee8e024ed8946/docs/Quectel_L76K_GNSS_协议规范_V1.0.pdf)
- **PDF**: [Quectel_L76K_GNSS_协议规范_V1.1](https://raw.githubusercontent.com/Seeed-Projects/Seeed_L76K-GNSS_for_XIAO/fb74b715224e0ac153c3884e578ee8e024ed8946/docs/Quectel_L76K_GNSS_Protocol_Specification_V1.1.pdf)
- **GitHub**: [Seeed_L76K-GNSS_for_XIAO](https://github.com/Seeed-Projects/Seeed_L76K-GNSS_for_XIAO)

## 故障排除

<details>
<summary>可充电电池能为 XIAO 供电吗？</summary>
不能，本文中的可充电电池仅用于实时时钟（RTC）和维持 L76K GNSS 模块的热启动状态。它不能作为 XIAO 或 GNSS 模块的一般操作的主要电源。
</details>

<details>
<summary>为什么 GNSS 信息不显示在串口监视器上？</summary>

确保将 L76K GNSS 模块放置在户外，确保可以接收到良好的 GNSS 信号。
</details>

<details>
<summary>为什么设备的绿色指示灯在插入 XIAO RP2040 后一直亮着？</summary>
为了解决此问题，您需要将 D0 和 D10 拉高。绿色指示灯持续亮起表示设备已进入异常工作状态。

```cpp
pinMode(D10,OUTPUT);
digitalWrite(D10,1);
pinMode(D0,OUTPUT);
digitalWrite(D0,1);
```

</details>

## 技术支持和产品讨论

感谢您选择我们的产品！我们致力于为您提供不同的支持，以确保您对我们产品的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
