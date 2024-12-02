---
description: Seeed Studio XIAO ESP32C6 与 PlatformIO
title: XIAO ESP32C6 与 PlatformIO
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/esp32c6_circuitpython/title.png
slug: /cn/xiao_esp32c6_with_platform_io
last_update:
  date: 11/16/2024
  author: Agnes
---

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_platformio/7.png" /></div>

## PlatformIO 介绍

PlatformIO 是一个集成了多种开发板的开发平台，具有良好的可扩展性。如果平台没有您需要的开发板类型，您可以手动添加开发板类型。您在 Arduino 上编写的代码也可以在 PlatformIO 上使用，只需添加相应的库即可。

在本 wiki 中，我们将介绍如何在 PlatformIO 中安装和运行示例代码。

## 使用 PlatformIO 配置 XIAO ESP32C6

### 设置 1 . 安装 [PlatformIO](https://platformio.org/platformio-ide) 官方网站

如果您尚未安装 PlatformIO 软件，可以点击上面的链接进行安装。

### 设置 2 . 在 PlatformIO 中创建项目

因为平台已经提供了 XIAO ESP32S3 和 XIAO ESP32C3 开发板选项，我们可以选择其中一个来创建文件。当然，其他选项也可以使用，没关系。项目名称也可以随意选择。

<table align="center">
  <tr>
      <th>操作一</th>
        <th>操作二</th>
  </tr>
  <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/esp32c6_platformio/4.png" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/esp32c6_platformio/3.png" style={{width:700, height:'auto'}}/></div></td>
  </tr>
</table>

:::tip
在此之前，我已经安装了 XIAO ESP32C6 安装包，因此在操作二中可以看到 XIAO ESP32C6 的选项，但如果您在执行时没有看到它，请检查安装包是否正确安装。
:::

### 设置 3 . 修改 platformio.ini 文件

当您成功创建 PlatformIO 文件时，左侧会出现许多文件。我们可以看到一个名为 platform.ini 的文件。接下来，我们需要替换文件中的内容。
<table align="center">
  <tr>
      <th>操作三</th>
  </tr>
  <tr>
<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/esp32c6_platformio/2.png" /></div>
  </tr>
</table>

您需要复制以下代码，并替换 platform.ini 文件中的内容：

```
platform = https://github.com/mnowak32/platform-espressif32.git#boards/seeed_xiao_esp32c6
platform_packages = 
	framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.2
	framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.2/esp32-arduino-libs-3.0.2.zip
framework = arduino
board = seeed_xiao_esp32c6
```
:::tip
记得保存文件，使用 ctrl+s，以确保文件正确加载。
:::

### 设置 4 . 编译和烧录

<table align="center">
  <tr>
      <th>操作四</th>
  </tr>
  <tr>
<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c6_platformio/setup3.png" /></div>
  </tr>
</table>

最后，如果您看到与下图相同的结果，那么说明您成功添加了 XIAO ESP32C6 开发板。当您再次创建项目时，XIAO ESP32C6 的操作会显示。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们产品的体验尽可能顺畅。我们提供了多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>