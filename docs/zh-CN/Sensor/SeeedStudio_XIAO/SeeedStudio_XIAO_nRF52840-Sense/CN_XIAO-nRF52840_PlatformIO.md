---
description: 带有 PlatformIO 的 Seeed Studio XIAO nRF52840
title: XIAO nRF52840-Sense 与 PlatformIO
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/nRF52840_PlatformIO/1.png
slug: /cn/xiao_nrf52840_with_platform_io
last_update:
  date: 11/13/2024
  author: Agnes
---

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/nRF52840_PlatformIO/1.png" /></div>

# **Seeed Studio XIAO nRF52840 与 PlatformIO**

PlatformIO 是一个集成了多种开发板类型并且具有良好扩展性的开发平台。如果平台中没有你需要的开发板类型，你可以手动添加。你在 Arduino 中编写的代码可以直接在 PlatformIO 中使用，只需要添加相应的库。

在本篇 Wiki 中，我们将介绍如何在 PlatformIO 中安装并运行示例代码。

## 在 PlatformIO 中使用 XIAO nRF52840

### 步骤 1 . 安装 [PlatformIO](https://platformio.org/platformio-ide) 官方网站中的软件

如果你还没有安装 PlatformIO，可以点击上述链接进行下载和安装。

### 步骤 2 . 在 PlatformIO 中创建项目

因为平台已经包含了 XIAO ESP32S3 和 XIAO ESP32C3 开发板选项，所以我们可以选择其中一个来创建项目。当然，其他开发板文件也可以创建，项目名称可以随意选择。

<table align="center">
  <tr>
      <th>操作 1</th>
        <th>操作 2</th>
  </tr>
  <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/esp32c6_platformio/4.png" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/esp32c6_platformio/3.png" style={{width:700, height:'auto'}}/></div></td>
  </tr>
</table>

:::tip
在此之前，我已经安装了 XIAO ESP32C6 和 XIAO nRF52840 安装包，因此你会看到操作二中的图像中有 XIAO ESP32C6 和 XIAO nRF52840 的选项。如果你在执行操作时没有这些选项，请参考后续步骤。
:::

### 步骤 3 . 修改 platformio.ini 文件

成功创建 PlatformIO 项目后，左侧栏会出现多个文件。我们可以看到一个名为 platform.ini 的文件。接下来，我们需要替换其中的内容。
<table align="center">
  <tr>
      <th>操作 3</th>
  </tr>
  <tr>
<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c6_platformio/2.png" /></div>
  </tr>
</table>

你需要复制以下代码，并替换 platform.ini 文件中的内容：

```
[env]
platform = https://github.com/maxgerhardt/platform-nordicnrf52
framework = arduino
 
[env:xiaoblesense_arduinocore_mbed]
board = xiaoblesense
 
[env:xiaoble_arduinocore_mbed]
board = xiaoble
```
:::tip
记得保存文件，按 ctrl+s，文件将会被加载。
:::

### 步骤 4 .  编译并烧录

<table align="center">
  <tr>
      <th>操作 4</th>
  </tr>
  <tr>
<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/nRF52840_PlatformIO/2.png" /></div>
  </tr>
</table>

最终，如果你看到与下图相同的结果，说明你已经成功添加了 XIAO nRF52840 开发板。当你再次创建项目时，就可以选择 XIAO nRF52840 进行操作。

## 技术支持与产品讨论

感谢你选择我们的产品！我们提供不同的支持方式，确保你在使用我们的产品时获得顺畅的体验。我们提供多个沟通渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>