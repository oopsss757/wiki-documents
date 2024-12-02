---
title: 开始使用 Seeed Studio XIAO RA4M1
description: |
  XIAO RA4M1 - 基于32位Renesas RA4M1的最小开发板，兼容Arduino IDE，拥有19个GPIO，RGB LED，CAN总线，USB 2.0
image: https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/1-102010551-Seeed-Studio-XIAO-RA4M1.jpg
slug: /cn/getting_started_xiao_ra4m1
keywords:
  - XIAO
  - RA4M1
last_update:
  date: 11/01/2024
  author: Agnes
sidebar_position: 0
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


<!-- Product Description -->

<div style={{ textAlign: 'center' }}>
  <img 
    src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" 
    style={{ width: 480, height: 'auto', "border-radius": '12.8px' }} 
  />
</div>

<div class="get_one_now_container" style={{textAlign: 'center'}}>
  <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1">
  <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
  </a>
</div><br></br>

## 简介

XIAO RA4M1 将瑞萨的 RA4M1 芯片（32 位 ARM® Cortex®-M4 MCU，最高可达 48 MHz）集成到经典的 XIAO 外形中。该开发板提供 256KB Flash、32KB SRAM、8KB EEPROM、USB 2.0 连接器、复位和引导按钮、3 个 LED、14 位 A/D 转换器、12 位 D/A 转换器以及 CAN 总线接口。配备了板载充电电路和低功耗模式（低至 45μA），非常适合电池供电的应用。与 Arduino Uno R4 共享相同的 32 位 R7FA4M1AB3CFM 微控制器，它与 Arduino IDE 及丰富的 XIAO 配件原生兼容，是电子项目的完美起点

### 特性

<!-- Key Features with bullet -->

- **流行的微控制器：**采用瑞萨 RA4M1，32 位 ARM® Cortex®-M4 R7FA4M1AB3CFM MCU，最高运行频率 48 MHz，256 KB Flash 内存和 32 KB SRAM。
- **突出板载资源：**配备 14 位 ADC、12 位 DAC、CAN 总线、USB 2.0 及板载 RGB LED。
- **扩展 8 个新 IO：**与之前的 XIAO 板相比，在背面新增 8 个 IO 引脚（总共 19 个 GPIO），支持更复杂的应用。
- **强大的安全特性：**内置硬件加密、安全启动、密钥存储等功能以确保应用安全。
- **软件兼容性：**与 Arduino IDE 完全兼容，实现无缝的项目开发和原型制作。
- **高效电源设计：**提供 4 种工作模式，深度睡眠下功耗低至 45μA，支持锂电池充电管理。
- **紧凑的拇指大小设计：**尺寸为 21 x 17.8mm，采用 Seeed Studio 经典的 XIAO 外形，适合对空间要求较高的应用。
- **适合生产：**表面贴装设备（SMD）设计，所有组件均在前面，两个侧面有冲孔，便于高效大规模生产。


### 规格

<!-- Technical Specifications | Table | or bullet -->

<table>
    <thead>
        <tr>
            <th>Product</th>
            <th>XIAO RA4M1</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <th rowspan="2">处理器</th>
            <td><a href="https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m1-32-bit-microcontrollers-48mhz-arm-cortex-m4-and-lcd-controller-and-cap-touch-hmi">瑞萨 RA4M1</a>
            </td>
        </tr>
        <tr>
            <td>48-MHz Arm® Cortex®-M4 核心</td>
        </tr>
        <tr>
            <th>RAM</th>
            <td>32 KB SRAM</td>
        </tr>
        <tr>
            <th>Flash</th>
            <td>256 KB</td>
        </tr>
        <tr>
            <th>LEDs</th>
            <td>1 个用户 LED，1 个电源 LED，1 个 RGB LED</td>
        </tr>
        <tr>
            <th>接口</th>
            <td>19 个引脚：14 个模拟，19 个数字，2 个 IIC，2 个 UART，2 个 SPI</td>
        </tr>
        <tr>
            <th>按钮</th>
            <td>1 个复位按钮，1 个引导按钮</td>
        </tr>
        <tr>
            <th>安全性</th>
            <td>AES128/256</td>
        </tr>
        <tr>
            <th>低功耗</th>
            <td>45μA</td>
        </tr>
        <tr>
            <th>软件兼容性</th>
            <td>Arduino IDE</td>
        </tr>
        <tr>
            <th>工作温度</th>
            <td>-20°C-70°C</td>
        </tr>
        <tr>
            <th>尺寸</th>
            <td>21x17.8 mm</td>
        </tr>
        <tr>
            <th>电源</th>
            <td>Type-C: 5V<br />BAT: 3.8V</td>
        </tr>
    </tbody>
</table>

### 硬件概述

在一切开始之前，了解产品的一些基本参数是非常重要的。下表提供了 Seeed Studio XIAO RA4M1 的特性信息。

<!-- 硬件相关 -->

<!-- 引脚等的定义 -->
<table>
	<tr>
	    <th>XIAO RA4M1 前面指示图</th>
	</tr>
    <tr>
        <br />
        <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/XIAO-RA4M1-components.png" style={{width:380, height:'auto'}}/></div>
        <br />        
    </tr>
    <tr>
        <th>XIAO RA4M1 引脚列表</th>
    </tr>
    <tr>
        <br />
        <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/XIAO-RA4M1-pinout.png" style={{width:880, height:'auto'}}/></div>
        <br />
    </tr>
</table>

## 开始使用

### 硬件准备

您需要准备以下设备：

- 1 x [Seeed Studio XIAO RA4M1](https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1)
- 1 x 计算器
- 1 x USB Type-C 数据线

<br></br>

:::tip
某些 USB 数据线只能供电，无法传输数据。如果您没有 USB 数据线或不确定您的 USB 数据线是否可以传输数据，可以查看  [Seeed USB Type-C 支持 USB 3.1](https://www.seeedstudio.com/USB-3-1-Type-C-to-A-Cable-1-Meter-3-1A-p-4085.html)。
:::

### 软件准备

推荐使用 Arduino IDE 作为 XIAO RA4M1 的编程工具，因此在软件准备过程中，您需要完成 Arduino 的安装。

:::tip
如果您是第一次使用 Arduino，我们强烈建议您参考[ Arduino 入门指南](https://wiki.seeedstudio.com/Getting_Started_with_Arduino/).
:::

- **第 1 步：** 根据您的操作系统下载并安装稳定版本的 Arduino IDE。

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载 Arduino IDE</font></span></strong>
    </a>
</div>

- **第 2 步：** 启动 Arduino 应用程序。

- **第 3 步：** 将 RA4M1 板包添加到您的 Arduino IDE。

    导航到 **文件 > 首选项**, 并在 **"附加板管理器 URLs"** 中填写以下 URL：
    *<https://files.seeedstudio.com/arduino/package_renesas_1.2.0_index.json>*
    
    <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/arduino_preference.png" style={{width:800, height:'auto'}}/></div>

   导航到 **工具 > 板 > 板管理器...**, 在搜索框中输入关键字 **RA4M1** ，选择最新版本的**Seeed Renesas Board**并安装。

    <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/board_manager.png" style={{width:600, height:'auto'}}/></div>
- **第 4 步：** 选择您的板和端口。

    在 Arduino IDE 顶部，您可以在左侧的开发板中搜索**xiao**，选择 XIAO_RA4M1，然后直接选择端口。
    <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/select_port.png" style={{width:800, height:'auto'}}/></div>


## 引导加载程序模式

有时，使用错误的程序可能导致 XIAO 失去端口或无法正常工作。常见问题包括：

- XIAO 连接到计算机，但未找到端口号。
- XIAO 已连接，并且出现了端口号，但程序上传失败。

当您遇到上述两种情况时，可以尝试将 XIAO 放入引导加载程序模式，这可以解决大多数未识别设备和上传失败的问题。具体方法如下：

- **方法1：** 按住 XIAO RA4M1 上的 `BOOT` 按钮，保持不放。
- **方法2：** 按住 `BOOT` 按钮，然后通过数据线连接到计算机。连接到计算机后释放 BOOT 按钮。

## 重置

当程序异常运行时，您可以在上电时按一次 `Reset` 以让 XIAO 重新执行上传的程序。 当您在上电时按住 `BOOT` 键，然后按一次 `Reset` 键时，也可以进入引导加载程序模式。

## 运行您的第一个闪烁程序

到现在为止，我相信您对 XIAO RA4M1 的特性和硬件有了很好的了解。接下来，让我们以最简单的闪烁程序为例，进行 XIAO RA4M1 的第一次闪烁！

- **第一步：** 启动 Arduino 应用程序。
- **第二步：** 导航到 **文件 > 示例 > 01.基础 > Blink**，打开程序。
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/blink_code.png" style={{width:700, height:'auto'}}/></div>

- **第三步：** 选择开发板型号为 **XIAO RA4M1**，并选择正确的端口号以上传程序。
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/choose_board.png" style={{width:700, height:'auto'}}/></div>

程序成功上传后，您将看到以下输出消息，并可以观察到 XIAO RA4M1 右侧的橙色 LED 正在闪烁。
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/blink.gif" style={{width:700, height:'auto'}}/></div>

恭喜您！您已经成功掌握了如何使用 XIAO RA4M1 开发板！
:::note
当 XIAO RA4M1 上的用户 LED 引脚设置为高电平时，LED 才会熄灭，而当引脚设置为低电平时，LED 才会亮起。
:::

## 玩转 RGB LED
XIAO RA4M1 配备了内置 RGB LED，您可以控制它。以下是一个如何平滑地在红色、绿色和蓝色之间切换 LED 颜色的示例。

- **第一步：** 下载 `Adafruit_NeoPixel` 库

Navigate to **Sketch > Include Liarbry > Manage Libraries...**, and search **Adafruit_NeoPixel**, install the lastest version.

- **第二步：** 将以下代码复制到新的草图中：

```cpp
#include <Adafruit_NeoPixel.h>

#define LED_PIN RGB_BUILTIN  // 定义内置 RGB LED 的引脚
#define NUM_PIXELS 1         // WS2812 LED 的数量

Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(PIN_RGB_EN, OUTPUT); // 设置电源引脚
  digitalWrite(PIN_RGB_EN, HIGH); // 打开 LED 电源
  pixels.begin();  // 初始化 NeoPixel 库
}

void loop() {
    // 从红色过渡到绿色
  for (int i = 0; i <= 255; i++) {
    pixels.setPixelColor(0, pixels.Color(255 - i, i, 0));  // 红色减少，绿色增加
    pixels.show();
    delay(10);  // 调整延迟以实现平滑过渡
  }

  // 从绿色过渡到蓝色
  for (int i = 0; i <= 255; i++) {
    pixels.setPixelColor(0, pixels.Color(0, 255 - i, i));  // 绿色减少，蓝色增加
    pixels.show();
    delay(10);  // 调整延迟以实现平滑过渡
  }

  // 从蓝色过渡到红色
  for (int i = 0; i <= 255; i++) {
    pixels.setPixelColor(0, pixels.Color(i, 0, 255 - i));  // 蓝色减少，红色增加
    pixels.show();
    delay(10);  // 调整延迟以实现平滑过渡
  }
}

```
- **第三步：** 选择开发板型号为  **XIAO RA4M1**，并选择正确的端口号以上传程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/rgb_led.gif" style={{width:600, height:'auto'}}/></div>

## 电池与电源管理

是否可以在没有额外组件的情况下读取电池电压？可以，使用 XIAO RA4M1，比以往更简单。在之前的 XIAO 系列产品中，例如 [XIAO ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/#check-the-battery-voltage), 读取电池电压需要手动连接到 A0 并使用电阻。

但在 XIAO RA4M1 中，这个过程简化了。您现在可以直接使用 `BAT_DET_PIN/P105` 引脚读取电池电压水平，简化了设计和开发。只需记得将 `BAT_READ_EN/P400` 引脚设置为高电平，因为这对于启用电池电压读取是必要的。

- **第一步：** 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
        <th>Seeed Studio XIAO 扩展基板与 Grove OLED</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

XIAO 扩展基板上的 OLED 显示器使用 I2C 协议，并通过板上的 I2C 电路连接到 XIAO 的 I2C 接口。因此，我们可以直接将 XIAO 插入扩展基板，并编程在屏幕上显示内容。

- **第二步：** 安装 u8g2 库。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/olikraus/U8g2_Arduino">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

- **Step 3.** Copy the code and stick on the Ardiono IDE.

```cpp
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* 时钟引脚=*/ WIRE_SCL_PIN, /* 数据引脚=*/ WIRE_SDA_PIN, /* 重置引脚=*/ U8X8_PIN_NONE);   // 没有显示重置的 OLED

#define enablePin  BAT_READ_EN  // 启用电池电压读取的引脚
#define adcPin BAT_DET_PIN     // 模拟输入引脚（在您的情况下是 GPIO29）
const float referenceVoltage = 3.3;  // ADC 的参考电压
const float voltageDivider = 2.0;    // 电压分压系数

void setup() {
  Serial.begin(9600);          // 以 9600 波特率初始化串口通信
  pinMode(enablePin, OUTPUT);  // 将启用引脚设置为输出
  digitalWrite(enablePin, HIGH); // 将引脚设置为高电平以启用电池电压读取
  u8x8.begin();
  u8x8.setFlipMode(1);   // 设置旋转模式，值为 1 到 3，屏幕内容将旋转 180 度
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void loop() {
  int rawValue = analogRead(adcPin);    // 读取模拟输入值
  float voltage = rawValue * (referenceVoltage / 1023.0) * voltageDivider; // 计算电压
  // 打印原始值和计算得到的电压
  u8x8.setCursor(0, 0);
  u8x8.print("原始值:0x");
  u8x8.print(rawValue, HEX);
  u8x8.setCursor(0, 2);
  u8x8.print("电压:");
  u8x8.print(voltage, 2);
  u8x8.print("V");

  delay(500); // 延迟 500 毫秒
}
```
- **第四步：** 选择开发板型号为 **XIAO RA4M1**，并选择正确的端口号以上传程序

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/voltage.png" style={{width:600, height:'auto'}}/></div>

## 资源

- 📄 **[PDF]** [RA4M1 数据手册](https://www.renesas.com/us/en/document/dst/ra4m1-group-datasheet)
- 📄 **[PDF]** [Seeed Studio XIAO RA4M1 原理图](https://files.seeedstudio.com/wiki/XIAO-R4AM1/res/XIAO-RA4M1_SCH_PDF_v1.0_240719.pdf)
- 📄 **[ZIP]** [KiCAD 库](https://files.seeedstudio.com/wiki/XIAO-R4AM1/res/XIAO-RA4M1_SCH%26PCB_v1.0_240719.zip)
<!-- - 🗃️ **[ZIP]** [Seeed Studio XIAO RA4M1 KiCAD 文件](https://files.seeedstudio.com/wiki/XIAO-RA4M1/res/Seeeduino-xiao-rp2040-KiCAD-Library.zip) -->
<!-- - 🗃️ **[ZIP]** [Seeed Studio XIAO RA4M1 Eagle 文件](https://files.seeedstudio.com/wiki/XIAO-RA4M1/res/XIAO_RP2040_v1.22_SCH&PCB.zip) -->
<!-- - 📄 **[DXF]** [Seeed Studio XIAO RA4M1 DXF 尺寸](https://files.seeedstudio.com/wiki/XIAO-RA4M1/res/XIAO-RP2040-DXF.zip) -->
<!-- - 📄 **[LBR]** [Seeed Studio XIAO RA4M1 Eagle 封装](https://files.seeedstudio.com/wiki/XIAO-RA4M1/res/Seeed-Studio-XIAO-RP2040-footprint-eagle.lbr) -->
<!-- - 📄 **[XLSX]** [Seeed Studio XIAO RA4M1 引脚图表](https://files.seeedstudio.com/wiki/XIAO-RA4M1/res/XIAO-RP2040-pinout_sheet.xlsx) -->

## 故障排除

### Q1: 焊接引脚时我应该注意什么？

由于 XIAO RA4M1 的微型尺寸，焊接引脚时请小心，避免不同引脚粘连在一起，也不要将焊锡粘在屏蔽或其他组件上。否则，这可能导致 XIAO 短路或无法正常工作，造成的后果将由用户自行承担。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，以确保您在使用我们的产品时获得尽可能顺畅的体验。我们提供多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
