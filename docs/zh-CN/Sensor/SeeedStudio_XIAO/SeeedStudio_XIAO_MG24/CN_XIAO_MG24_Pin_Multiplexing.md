---
title: 使用 Seeed Studio XIAO MG24 进行引脚多路复用
description: 使用 Seeed Studio XIAO MG24 进行引脚多路复用(Sense).
image: https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/top.webp
slug: /cn/xiao_mg24_pin_multiplexing
keywords:
  - XIAO
  - MG24
last_update:
  date: 11/19/2024
  author: Agnes
sidebar_position: 1
---

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/top.png" style={{width:700, height:'auto'}}/></div>

***XIAO MG24*** 具有多达 ***22 个普通引脚***、***18 个模拟引脚***、***18 个数字引脚***、***2 个 SPI***、***2 个 UART***、***2 个 I2C***，并且支持 ***所有 PWM***。它提供了丰富的引脚供我们使用。在本篇文档中，我将教您如何驱动这些引脚，从而使我们能够有效地利用它们 😀！

## 数字引脚

XIAO MG24（Sense）最多具有 22 个普通引脚、18 个模拟引脚、18 个数字引脚、2 个 SPI、2 个 UART、2 个 I2C 和所有 PWM。它提供了丰富的引脚供我们使用。因此，在本示例中，我们将使用 XIAO MG24。

### 硬件准备

<table align="center">
  <tr>
      <th>Seeed Studio XIAO MG24 Sense</th>
        <th>Seeed Studio XIAO 扩展板（带 Grove OLED）</th>
        <th>Grove - 继电器</th>
  </tr>
  <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Relay/img/Twig-Relay.jpg" style={{width:600, height:'auto'}}/></div></td>
  </tr>
    <tr>
      <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
        <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
        </a>
    </div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
        <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
        </a>
    </div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
        <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Relay.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
        </a>
    </div></td>
  </tr>
</table>

请将 XIAO MG24（Sense）或 Sense 安装到扩展板上，然后通过 Grove 电缆将继电器连接到扩展板的 **A0/D0** 接口。最后，通过 USB-C 电缆将 XIAO 连接到计算机。

### 软件实现

在本示例中，我们将使用连接到 XIAO 扩展板上的按钮来实现继电器的开/关控制。当按钮按下时，继电器打开；当按钮松开时，继电器关闭。

```c
const int buttonPin = D1;     // 按钮引脚的编号
int buttonState = 0;          // 读取按钮状态的变量
const int relayPin = D0;

void setup() {
  // 初始化继电器引脚为输出：
  pinMode(relayPin, OUTPUT);
  // 初始化按钮引脚为输入：
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // 读取按钮的状态：
  buttonState = digitalRead(buttonPin);

  // 检查按钮是否被按下。如果按下，按钮状态为 HIGH：
  if (buttonState == HIGH) {
    // 打开继电器：
    digitalWrite(relayPin, HIGH);
  } else {
    // 关闭继电器：
    digitalWrite(relayPin, LOW);
  }
}
```
### 结果图示

如果一切顺利，上传程序后，您应该看到如下效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/2.gif" style={{width:500, height:'auto'}}/></div>

## 数字引脚作为 PWM 输出

XIAO MG24（Sense）上的所有 GPIO 引脚都支持 PWM 输出。因此，您可以使用任意引脚输出 PWM 来调整灯光的亮度、控制舵机等功能。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO MG24 Sense</th>
        <th>Seeed Studio XIAO 扩展板（带 Grove OLED）</th>
        <th>Grove - 可调颜色 LED</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/10.jpg" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Variable-Color-LED-V1-1.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

:::tip
请将 XIAO MG24（Sense）或 Sense 安装到扩展板上，然后使用 Grove 电缆将可调颜色 LED 连接到扩展板的 A0/D0 接口。最后，通过 USB-C 电缆将 XIAO 连接到您的计算机。
:::

### 软件实现

在本示例中，我们将演示如何使用 PWM 输出控制灯光的亮度。

```cpp
int LED_pin = D0;    // LED 连接到数字引脚 10

void setup() {
  // 将 LED 引脚设置为输出
  pinMode(LED_pin, OUTPUT);
}

void loop() {
  // 从最小亮度到最大亮度逐步渐变，每次增量为 3：
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 3) {
    // 设置值（范围从 0 到 255）：
    analogWrite(LED_pin, fadeValue);
    // 等待 30 毫秒以查看调光效果
    delay(30);
  }

  // 从最大亮度到最小亮度逐步渐变，每次递减 3：
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 3) {
    // 设置值（范围从 0 到 255）：
    analogWrite(LED_pin, fadeValue);
    // 等待 30 毫秒以查看调光效果
    delay(30);
  }
}
```
### 结果图示

如果程序运行成功，您将看到如下效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/5.gif" style={{width:500, height:'auto'}}/></div>

## 模拟输入

XIAO MG24（Sense）开发板配备 12 位 ADC，可用于高分辨率读取模拟传感器值，帮助我们读取更准确的值。

接下来，我们将选择两个传感器来展示 ADC 的特点。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO MG24 Sense</th>
         <th>Grove-可调颜色 LED</th>
	    <th>Grove-旋转角度传感器</th>
	    <th>Seeed Studio XIAO 扩展板</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:500, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/10.jpg" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove_Shield_for_Pi_Pico_V1.0/rotary.png" style={{width:500, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Shield-for-Seeeduino-XIAO/img/xiao_-Preview-25.png" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Variable-Color-LED-V1-1.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Rotary-Angle-Sensor.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

### 软件实现

```cpp
const int analogInPin = D1;  // 模拟输入引脚，连接到可调电位器
const int analogOutPin = 9;  // 模拟输出引脚，连接到 LED

int sensorValue = 0;  // 从电位器读取的值
int outputValue = 0;  // 输出到 PWM（模拟输出）的值

void setup() {
  Serial.begin(115200);
}

void loop() {
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 4095, 0, 255);
  analogWrite(analogOutPin, outputValue);

  Serial.print("传感器值 = ");
  Serial.print(sensorValue);
  Serial.print("\t 输出值 = ");
  Serial.println(outputValue);
  delay(100);
}
```
### 结果图示

如果一切顺利，上传程序后，您应该看到如下效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/4.gif" style={{width:500, height:'auto'}}/></div>

## 串口通信

在使用 Arduino IDE 时，串口通信是许多项目中不可或缺的一部分。要在 Arduino IDE 中使用串口通信，您需要首先打开 **串口监视器** 窗口。您可以通过点击工具栏上的 **串口监视器** 图标，或按 **Ctrl+Shift+M** 快捷键来打开。

### 常见用法

一些常用的串口函数包括：

- `Serial.begin()` — 用于初始化串口通信并设置波特率；
- `Serial.print()` — 以可读格式将数据发送到串口；
- `Serial.write()` — 将二进制数据发送到串口；
- `Serial.available()` — 检查是否有数据可以从串口读取；
- `Serial.read()` — 从串口读取一个字节的数据；
- `Serial.flush()` — 等待串口数据的发送完成。

通过使用这些串口函数，您可以在 Arduino 板和计算机之间发送和接收数据，这为创建互动项目提供了许多可能性。

以下是一个示例程序：

```c
void setup() {
  // 初始化串口通信，波特率为9600
  Serial.begin(9600);
}

void loop() {
  // 发送数据到串口
  Serial.println("Hello World!");

  // 从串口读取数据
  if (Serial.available() > 0) {
    // 读取传入的字节
    char incomingByte = Serial.read();
    // 将接收到的字节打印到串口监视器
    Serial.print("我收到: ");
    Serial.println(incomingByte);
  }
  
  // 等待一秒后重复循环
  delay(1000);
}
```

### Serial1 的使用

根据上面的 XIAO MG24(Sense) 引脚图，我们可以观察到有 TX 引脚和 RX 引脚。  
这与串口通信有所不同，但使用方法非常相似，唯一不同的是需要添加一些参数。  
接下来，我们将使用芯片引出的引脚进行串口通信。

```c
#define BAUD 115200

void setup() {
    Serial1.begin(BAUD);
}
 
void loop() {
  if(Serial1.available() > 0)
  {
    char incominByte = Serial1.read();
    Serial1.print("我收到: ");
    Serial1.println(incominByte);
  }
  delay(1000);
}
```

## IIC

XIAO MG24(Sense) 具有 I2C 接口，可以用于多个传感器的数据传输和解析，也可以用于驱动一些 OLED 屏幕。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO MG24 Sense</th>
        <th>Seeed Studio XIAO 扩展板与 Grove OLED</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:400, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:400, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

XIAO 扩展板上的 OLED 屏幕使用 I2C 协议，并通过扩展板上的 I2C 电路与 XIAO 的 I2C 接口连接。因此，我们可以直接将 XIAO 插入扩展板，并编写程序在屏幕上显示内容。

### 软件实现

本示例介绍如何使用 Seeed Studio XIAO MG24(Sense) 上的 OLED 显示器。

***步骤 1. 将 Seeed Studio XIAO MG24(Sense) 安装到扩展板上，然后连接 Type-C 数据线。***

***步骤 2. 安装 u8g2 库。***

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/olikraus/U8g2_Arduino">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库文件</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

***步骤 3. 复制代码并粘贴到 Arduino IDE 中，然后上传。***

- 下载下面的 ZIP 文件

📄 **[ZIP]** [smiley_face Header](https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/smiley_face.zip)

- 创建一个名为 "smiley_face.h" 的头文件，并将下载的 ZIP 文件内容复制到您创建的头文件中
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/8.png" style={{width:800, height:'auto'}}/></div>

```c
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "smiley_face.h" 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int xx = 20; 
int yy = 10; 

void setup() {
    u8g2.begin();
}

void loop() {
    smeil_display();
    delay(500); 
}

void smeil_display() {
    const unsigned char* smileImages[] = {
        semil1, semil2, semil3, semil4, semil5,
        semil6, semil7, semil8, semil9, semil10,
        semil11, semil12, semil13, semil14, semil15,
        semil16, semil17, semil18, semil19, semil20,
        semil21, semil22, semil23, semil24, semil25,
        semil26, semil27
    };

    int delays[] = {
        40, 50, 40, 40, 40,
        40, 40, 50, 40, 40,
        40, 40, 40, 50, 40,
        40, 50, 40, 40, 50,
        40, 50, 40, 40, 50,
        50, 50, 40, 50
    };

    for (int i = 0; i < sizeof(smileImages) / sizeof(smileImages[0]); i++) {
        u8g2.firstPage();
        do {
            u8g2.drawXBMP(xx, yy, 48, 48, smileImages[i]); 
        } while (u8g2.nextPage());
        delay(delays[i]); 
    }
}
```

### 结果图

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/7.gif" style={{width:500, height:'auto'}}/></div>

## SPI

XIAO MG24（Sense）芯片集成了多个外设，其中包括一个SPI接口，可用于连接外部SPI设备，如闪存、显示器、传感器等。

### Arduino库概述

:::tip
如果这是您第一次使用Arduino，我们强烈建议您参考[Arduino入门指南](https://wiki.seeedstudio.com/Getting_Started_with_Arduino/)。
:::
<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/limengdu/XIAO_ST7789V2_LCD_Display/tree/main">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库文件</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div><br />

基于**Waveshare**提供的Arduino示例程序，我们为整个XIAO系列编写了一个Arduino库，您可以通过下面的按钮直接访问Github下载该库。

### 硬件准备

<div class="table-center">
	<table align="center">
		<tr>
			<th>Seeed Studio XIAO MG24 Sense</th>
			<th>1.69英寸LCD SPI显示屏</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:300, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/lcd_spi_display/6.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-MG24-Sense-p-6248.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/1-69inch-240-280-Resolution-IPS-LCD-Display-Module-p-5755.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

### 引脚连接
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/3.png" style={{width:700, height:'auto'}}/></div>

在完成上述硬件准备后，使用跳线连接XIAO和OLED的SPI接口。请参考下面的图示进行接线。

### 安装

下载ZIP格式的库文件后，打开Arduino IDE，点击**草图 > 包含库 > 添加.ZIP库**。选择您刚下载的ZIP文件，如果库安装正确，您将看到**库已添加到您的库中**的提示窗口，这意味着库已成功安装。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Get_Started_With_Arduino/img/Add_Zip.png" style={{width:800, height:'auto'}}/></div>

### 软件实现

正确下载并安装库后，您可以在示例文件夹中找到两个示例程序，分别为**helloworld.ino**和**bgcolor.ino**。其中，bgcolor.ino示例演示了背景颜色的显示，我们将红色设置为默认颜色。helloworld.ino则展示了包含公司logo动画的效果，并且该示例包含了bgcolor示例中的效果。

```cpp
#include <st7789v2.h>
#include "SPI.h"
#include "seeed.h"

st7789v2 Display;

void setup() {
  // 在此处编写您的设置代码，只运行一次：
  Display.SetRotate(270);  // 设置显示器旋转角度
  Display.Init();  // 初始化显示器
  Display.SetBacklight(100);  // 设置背光亮度为100
  Display.Clear(WHITE);  // 清屏，设置为白色背景
}

void loop() {
  // 在此处编写您的主代码，将重复运行：
  // Display.SetPixel(100, 100, RED);  // 在坐标(100, 100)设置红色像素
  // Display.DrawPoint(50, 50, YELLOW, DOT_PIXEL_8X8, DOT_FILL_AROUND);  // 在坐标(50, 50)绘制一个黄色点

  Display.DrawImage(gImage_seeed, 20, 90, 240, 47);  // 绘制图片，指定位置和尺寸

  // 绘制一条粉红色的直线
  Display.DrawLine(15, 65, 65, 65, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Display.DrawLine(15, 70, 80, 70, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  
  // 绘制一个矩形，填充为空心灰色
  Display.DrawRectangle(15, 80, 265, 150, GRAY, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  
  // 绘制几个圆圈，颜色分别为蓝色、黑色、红色和绿色
  Display.DrawCircle(10, 10, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 10, 20, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 10, 15, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 10, 10, GREEN, DOT_PIXEL_2X2, DRAW_FILL_FULL);

  // 在右上角绘制几个圆圈，颜色同上
  Display.DrawCircle(270, 10, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 10, 20, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 10, 15, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 10, 10, GREEN, DOT_PIXEL_2X2, DRAW_FILL_FULL);

  // 在左下角绘制几个圆圈，颜色同上
  Display.DrawCircle(10, 230, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 230, 20, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 230, 15, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(10, 230, 10, GREEN, DOT_PIXEL_2X2, DRAW_FILL_FULL);

  // 在右下角绘制几个圆圈，颜色同上
  Display.DrawCircle(270, 230, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 230, 20, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 230, 15, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Display.DrawCircle(270, 230, 10, GREEN, DOT_PIXEL_2X2, DRAW_FILL_FULL);

  // 绘制两条灰蓝色的直线
  Display.DrawLine(200, 160, 265, 160, GRAYBLUE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Display.DrawLine(215, 165, 265, 165, GRAYBLUE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  
  // 显示文本 "By: Jason"
  Display.DrawString_EN(20, 180, "By: Jason", &Font20, WHITE, BLACK);
  // 显示浮动数字1.00，精度为2
  Display.DrawFloatNum(100, 210, 1.00, 2, &Font16, WHITE, BLACK);
}
```

你将在显示器上动态看到 Seeed Studio 的 logo。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/1.gif"style={{width:500, height:'auto'}}/></div>

## 完成

你已经学习了 XIAO MG24 (Sense) 引脚的基本功能。现在，让我们进一步探索内置传感器。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Pin/6.png" style={{width:1000, height:'auto'}}/></div>

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供各种支持，确保您的产品体验尽可能顺畅。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>

