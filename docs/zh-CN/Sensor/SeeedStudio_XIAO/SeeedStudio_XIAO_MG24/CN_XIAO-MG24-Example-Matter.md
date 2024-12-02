---
description: Seeed Studio XIAO MG24 示例 - Matter
title: Seeed Studio XIAO MG24 示例 - Matter
image: https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/top.jpg
slug: /cn/xiao_mg24_matter
sidebar_position: 1
last_update:
  date: 11/19/2024
  author: Agnes
---

## 介绍

Matter 是一个**开源的统一标准**，用于智能家居技术，促进不同设备和生态系统之间的互操作性。它由连接标准联盟（Connectivity Standards Alliance，CSA）开发，使不同制造商的设备能够无缝通信，无需互联网连接。Matter 原生支持与 Apple HomeKit、Google Home 和 Amazon Alexa 等平台的兼容，使得在智能家居系统中集成设备变得更加容易。要深入了解 Matter，请参考[官方 Matter 文档](https://project-chip.github.io/connectedhomeip-doc/index.html)。

> 在2024年[^1]，Silicon Labs 和 Arduino 联手合作，降低了 Matter 采用的门槛，提供了一条简化使用 Matter 与 Arduino 生态系统的开发路径。这一合作旨在使 Matter 开发更加易于访问，帮助 Arduino 开发者克服常见的挑战，并无缝地采用 Matter。

[^1]: [Silicon Labs 和 Arduino 合作推动 Matter 普及 - 2024年2月6日](https://news.silabs.com/2024-02-06-Silicon-Labs-and-Arduino-Partner-to-Democratize-Matter)

Matter 在局域网中高效运行，提供可靠、低延迟的通信，而无需互联网访问。这一特性显著提高了安全性和设备性能。

本文档将指导您如何在 XIAO MG24 上使用 Arduino 开发 Matter 应用程序。

## 前提条件

在开始在 XIAO MG24 上开发 Matter 应用程序之前，请确保以下硬件和软件组件已准备好。

### 硬件

- **Seeed Studio XIAO MG24** 开发板。
- **支持的 Matter Hub**（例如 Apple HomePod mini），用于连接到 Matter 网络。
- **Matter 控制器**（例如 Apple HomeKit 应用程序），用于管理和与 Matter 支持的设备进行交互。

以下表格[^2]提供了各个生态系统中兼容 Matter 的 Hub 示例：

| 制造商 / 生态系统          | 设备                       |
| ------------------------- | -------------------------- |
| Google Home               | Nest Hub Gen2              |
| Apple HomeKit             | HomePod Gen2, HomePod mini |
| Amazon Alexa              | Echo Gen4                  |
| Raspberry Pi OTBR         | Raspberry Pi               |

默认情况下，假设您已经准备好了至少一个[Matter Hub](https://en.wikipedia.org/wiki/Matter_(standard)#Supported_ecosystems_and_hubs)和一个 Matter 控制器（例如安装了 HomeKit 的 iPhone），以便进行测试。

[^2]: [README - Arduino Matter 库](https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/readme.md)

### 软件

所需的软件包括**带有 Silicon Labs Arduino 核心的 Arduino IDE**：

- 如果尚未安装，请下载并设置[Silicon Labs Arduino 核心](https://github.com/SiliconLabs/arduino)，以与 XIAO MG24 兼容。
- 有关详细的设置说明，请参阅[入门指南](/xiao_mg24_getting_started/#add-board)。

请确保选择 Matter 协议栈：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-arduino-tool-option.png" style={{width:480, height: 'auto', "border-radius": '12.8px'}}/></div>

## 快速入门 Matter 灯泡示例

[Matter 灯泡示例](https://github.com/Silabs/arduino-matter/tree/main/examples/MatterLightBulb)展示了一个简单的 Matter 应用程序，允许通过 Matter 网络控制 `内置 LED`。这个示例为刚接触 Matter 集成的 XIAO MG24 开发者提供了一个实用的起点。

要在 Arduino IDE 中访问该示例：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-bulb-example.png" style={{width:480, height:'auto', "border-radius": '12.8px'}}/></div>

为了方便，以下提供了对示例代码中设备名称的修改，以便进行个性化设置。

```cpp
/*
   Matter 灯泡示例

   本示例展示了如何使用 Arduino Matter API 创建一个简单的开关灯泡。

   用户可以通过 Matter 控制板载 LED。
   设备必须先与 Matter Hub 配对。

   作者：Tamas Jozsi (Silicon Labs)
   修改者：Spencer Y (Seeed Studio)
 */
#include <Matter.h>
#include <MatterLightbulb.h>

MatterLightbulb matter_bulb;

void setup()
{
  Serial.begin(115200);
  Matter.begin();
  matter_bulb.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);

  Serial.println("Matter 灯泡");

  matter_bulb.set_device_name("XIAO MG24 bulb");    // 自定义默认设备名称
  matter_bulb.set_vendor_name("Seeed Studio");      // 设置供应商名称
  matter_bulb.set_product_name("Seeed Matter 灯泡"); // 定义产品名称

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("Matter 设备尚未配对");
    Serial.println("使用手动配对码或二维码将其配对到您的 Matter Hub");
    Serial.printf("手动配对码: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("二维码 URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  }
  while (!Matter.isDeviceCommissioned()) {
    delay(200);
  }

  Serial.println("等待 Thread 网络...");
  while (!Matter.isDeviceThreadConnected()) {
    delay(200);
  }
  Serial.println("已连接到 Thread 网络");

  Serial.println("等待 Matter 设备发现...");
  while (!matter_bulb.is_online()) {
    delay(200);
  }
  Serial.println("Matter 设备已上线");
}

void loop()
{
  static bool matter_lightbulb_last_state = false;
  bool matter_lightbulb_current_state = matter_bulb.get_onoff();

  // 如果状态为开且上一个状态为关，则打开 LED
  if (matter_lightbulb_current_state && !matter_lightbulb_last_state) {
    matter_lightbulb_last_state = matter_lightbulb_current_state;
    digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
    Serial.println("灯泡开");
  }

  // 如果状态为关且上一个状态为开，则关闭 LED
  if (!matter_lightbulb_current_state && matter_lightbulb_last_state) {
    matter_lightbulb_last_state = matter_lightbulb_current_state;
    digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
    Serial.println("灯泡关");
  }
}
```

### 刷写固件

1. 将代码复制到 Arduino IDE 中，并上传到 XIAO MG24 开发板。

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-lightbulb-flash.png" style={{width:480, height: 'auto', "border-radius": '12.8px'}}/></div>
2. 刷写固件后，按下 `RESET` 按钮或重新连接 XIAO MG24 以重启开发板。
3. 打开串口监视器以确认设置。您应该看到类似以下的输出：

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-qr-url.png" style={{width:480, height: 'auto', "border-radius": '12.8px'}}/></div>

### 配对二维码

串口监视器将显示一个 URL，用于生成设备配对所需的二维码。复制该 URL，粘贴到浏览器中，并使用您的 Matter 控制器（例如，HomeKit）扫描生成的二维码。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-qr-scan.png" style={{width:480, height: 'auto', "border-radius": '12.8px'}}/></div>

### 测试设备

扫描二维码后，Matter 控制器（如 HomeKit）会提示您确认设备的身份。一旦确认，设备将显示在应用程序中。您现在可以直接从应用界面控制 XIAO MG24 的内置 LED 并测试其响应能力。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/matter-device-online.png" style={{width:480, height: 'auto', "border-radius": '12.8px'}}/></div>

<iframe class="youtube-video-r" src="https://youtube.com/embed/tmCpIWuRojQ" title="MG24 Matter 灯泡示例" width="560" height="315" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## 参考资料与资源

如果您是 Matter 新手，以下资源为您提供了在 Matter 生态系统中工作所需的基础知识和开发支持：

- **[快速入门指南](https://docs.silabs.com/matter/2.2.0/matter-fundamentals/)**：学习 Matter 基础知识的理想起点，涵盖了生态系统中的基本概念和组件。
- **[Matter 开发者之旅](https://www.silabs.com/wireless/matter/matter-developer-journey)**：全面的 Matter 开发流程指南，包括所需的工具、资源以及有效实施的最佳实践。
- **[Matter 规范](https://csa-iot.org/developer-resource/specifications-download-request/)**：Matter 协议及其组件的技术规范。这是理解协议功能和操作细节的主要资源。
- **[设备数据模型 - Google Home 开发者](https://developers.home.google.com/matter/primer/device-data-model)**：设备数据模型的深入解释，标准化了如何在 Matter 生态系统中表示设备功能和能力。
- **[Matter 开发框架概述](/matter_development_framework)**：专门为 XIAO ESP32C6 量身定制的 Matter 开发框架指南，提供了另一种 Matter 设备开发方法。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供多种支持，确保您的使用体验尽可能顺畅。我们提供了多种沟通渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
