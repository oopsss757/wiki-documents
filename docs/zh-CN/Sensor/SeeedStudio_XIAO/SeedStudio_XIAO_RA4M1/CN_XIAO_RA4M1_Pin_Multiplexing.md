---
description: Seeed Studio XIAO RA4M1 的引脚复用
title: Seeed Studio XIAO RA4M1 的引脚复用
keywords:
- ra4m1
- xiao
- pin multiple
image: https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/1-102010551-Seeed-Studio-XIAO-RA4M1.jpg
side_position: 1
slug: /cn/xiao_ra4m1_pin_multiplexing
last_update:
  date: 11/01/2024
  author: Agnes
---

## 数字

XIAO RA4M1 具有多达 11 个常规 GPIO 引脚、6 个模拟引脚和 8 个可重用的 IO 端口。在本示例中，我们将使用 XIAO RA4M1、XIAO 扩展板和一个继电器，演示如何使用不同的数字引脚进行读写操作。

### 硬件准备

<table align="center">
  <tr>
      <th>Seeed Studio XIAO R4M1</th>
        <th>Seeed Studio XIAO 扩展基座（带 Grove OLED）</th>
        <th>Grove - 继电器</th>
  </tr>
  <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Relay/img/Twig-Relay.jpg" style={{width:500, height:'auto'}}/></div></td>
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
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
        <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Relay.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
        </a>
    </div></td>
  </tr>
</table>

请将 XIAO RA4M1 或 Sense 安装到扩展板上，并通过 Grove 电缆将继电器连接到扩展板的 **A0/D0** 接口。最后，通过 USB-C 电缆将 XIAO 连接到计算机。

### 软件实现

在本示例中，我们将实现通过连接到 XIAO 扩展板的按钮控制继电器的开关状态。当按下按钮时，继电器开启；当释放按钮时，继电器关闭。

```c
const int buttonPin = D1;     // 按钮引脚编号
int buttonState = 0;          // 读取按钮状态的变量
const int relayPin = D0;

void setup() {
  // 将继电器引脚初始化为输出：
  pinMode(relayPin, OUTPUT);
  // 将按钮引脚初始化为输入：
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // 读取按钮的状态：
  buttonState = digitalRead(buttonPin);

  // 检查按钮是否被按下。如果是，buttonState 为 HIGH：
  if (buttonState == HIGH) {
    // 打开继电器：
    digitalWrite(relayPin, HIGH);
  } else {
    // 关闭继电器：
    digitalWrite(relayPin, LOW);
  }
}
```

如果一切顺利，上传程序后，您应该会看到以下效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/31.gif" style={{width:600, height:'auto'}}/></div>

## 数字作为 PWM

XIAO RA4M1 上的所有 GPIO 引脚都支持 PWM 输出。因此，您可以使用任何引脚输出 PWM，以调节灯光的亮度、控制伺服电机等功能。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
        <th>Seeed Studio XIAO 扩展基座（带 Grove OLED）</th>
        <th>Grove - 可变颜色 LED</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:500, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Variable_Color_LED/img/Variable_Color_LED1.jpg" style={{width:500, height:'auto'}}/></div></td>
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
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Variable-Color-LED-p-852.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

请将 XIAO RA4M1 或 Sense 安装到扩展板上，然后通过 Grove 电缆将可变颜色 LED 连接到扩展板的 A0/D0 接口。最后，通过 USB-C 电缆将 XIAO 连接到计算机。

### 软件实现

在本示例中，我们将演示如何使用 PWM 输出控制灯光的亮度。

```cpp
int LED_pin = D0;    // LED 连接到数字引脚 10

void setup() {
  // 将 LED 引脚声明为输出
  pinMode(LED_pin, OUTPUT);
}

void loop() {
  // 从最小值逐渐增加到最大值，增量为 5：
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 3) {
    // 设置值（范围从 0 到 255）：
    analogWrite(LED_pin, fadeValue);
    // 等待 30 毫秒以查看渐变效果
    delay(30);
  }

  // 从最大值逐渐减少到最小值，增量为 5：
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 3) {
    // 设置值（范围从 0 到 255）：
    analogWrite(LED_pin, fadeValue);
    // 等待 30 毫秒以查看渐变效果
    delay(30);
  }
}
```

如果程序运行成功，您将看到以下运行效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/32.gif" style={{width:600, height:'auto'}}/></div>

## 模拟信号

XIAO RA4M1 开发板具备高达 14 位的 ADC，用于高分辨率读取模拟传感器值，能够帮助我们读取更准确的数值。XIAO RA4M1 开发板上的模数转换器（ADC）默认分辨率设置为 10 位，可以调至 12 位和 14 位以提高模拟读数的精度。

ADC 精度的详细数据
- 10 位: 0~1024
- 12 位: 0~4096
- 14 位: 0~16383

接下来，我们将选择两个传感器来反映 ADC 的特性。

### Hadware Preparation

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
      <th>Grove 可变颜色 LED</th>
	    <th>Grove 旋转角度传感器</th>
	    <th>Grove 旋转角度传感器</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove_Shield_for_Pi_Pico_V1.0/rotary.png" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Variable_Color_LED/img/Variable_Color_LED1.jpg" style={{width:500, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Shield-for-Seeeduino-XIAO/img/xiao_-Preview-25.png" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Variable-Color-LED-p-852.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Rotary-Angle-Sensor.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>


### Software Implementation

``` cpp
#define ADC_Bit_Fourteen 14
#define ADC_Bit_Twelve 12
#define ADC_Bit_Ten 10

const int analogInPin = A1;  // 连接电位器的模拟输入引脚
const int analogOutPin = 9;  // 连接 LED 的模拟输出引脚

int sensorValue = 0;  // 从电位器读取的值
int outputValue = 0;  // 输出到 PWM（模拟输出）的值

void setup() {
  Serial.begin(115200);
  // Ten_Bite_ADC_Config(); // 10 位
  // Twelve_Bite_ADC_Config(); // 12 位
  Fourteen_Bite_ADC_Config(); // 14 位
}

void loop() {
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 4095, 0, 255);
  analogWrite(analogOutPin, outputValue);

  Serial.print("传感器 = ");
  Serial.print(sensorValue);
  Serial.print("\t 输出 = ");
  Serial.println(outputValue);
  delay(100);
}

void Ten_Bite_ADC_Config() {
  analogReadResolution(ADC_Bit_Ten);
}

void Twelve_Bite_ADC_Config() {
  analogReadResolution(ADC_Bit_Twelve);
}

void Fourteen_Bite_ADC_Config() {
  analogReadResolution(ADC_Bit_Fourteen);
}
```
如果一切顺利，上传程序后，您应该会看到以下效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/33.gif" style={{width:650, height:'auto'}}/></div>

## 串口

在 Arduino IDE 中，串口通信是许多项目的重要组成部分。要在 Arduino IDE 中使用串口，您需要首先打开串口监视器窗口。可以通过点击工具栏中的**串口监视器**图标或按**Ctrl+Shift+M**快捷键来完成。

### 一般用法

一些常用的串口函数包括：

- `Serial.begin()` -- 初始化指定波特率的通信；
- `Serial.print()` -- 以可读格式将数据发送到串口；
- `Serial.write()` -- 将二进制数据发送到串口；
- `Serial.available()` -- 检查是否有可读取的数据；
- `Serial.read()` -- 从串口读取一个字节的数据；
- `Serial.flush()` -- 等待传输完成。

使用这些串口函数，您可以在 Arduino 板和计算机之间发送和接收数据，从而为创建交互式项目打开许多可能性。

以下是一个示例程序：

```c
void setup() {
  // 以 9600 位每秒初始化串口通信：
  Serial.begin(9600);
}

void loop() {
  // 发送数据到串口
  Serial.println("Hello World!");

  // 从串口读取数据
  if (Serial.available() > 0) {
    // 读取传入的字节：
    char incomingByte = Serial.read();
    // 将传入的字节打印到串口监视器：
    Serial.print("我收到了: ");
    Serial.println(incomingByte);
  }
  
  // 在重复循环之前等待一秒
  delay(1000);
}
```

### 使用 Serial1

根据上述 XIAO RA4M1 引脚图，我们可以观察到有 TX 引脚和 RX 引脚。这与串口通信不同，但使用方法也非常相似，只需添加一些参数。接下来，我们将使用芯片提供的引脚进行串口通信。

```c

#define BAUD 115200

void setup() {
    Serial1.begin(BAUD);
}
 
void loop() {
  if(Serial1.available() > 0)
  {
    char incominByte = Serial1.read();
    Serial1.print("我收到了：");
    Serial1.println(incominByte);
  }
  delay(1000);
}
```

### 使用软件串口

```c
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
  // 初始化串口通信
  Serial.begin(9600);
  while (!Serial);

  // 初始化软件串口
  mySerial.begin(9600);
}

void loop() {
  // 从软件串口读取数据
  if (mySerial.available()) {
    char data = mySerial.read();
    Serial.print("接收到的数据: ");
    Serial.println(data);
  }

  // 向软件串口写入数据
  mySerial.print("Hello World!");

  // 在重复循环之前等待一秒
  delay(1000);
}
```

在这个程序中，我们首先包含 `SoftwareSerial.h` 库以使用软件串口。然后，使用引脚 2 和 3 创建一个新的 SoftwareSerial 对象 mySerial。

在 `setup()` 函数中，我们初始化硬件串口（`Serial.begin()`）和软件串口（`mySerial.begin()`）。

在 `loop()` 函数中，我们使用 `mySerial.available()` 函数检查是否有可读取的数据。如果有，我们使用 `mySerial.read()` 函数读取传入的字节，并将其存储在名为 data 的变量中。

我们还使用 `mySerial.print()` 函数将 "Hello World!" 写入软件串口。这将把数据从 XIAO 发送到连接到软件串口的设备。

最后，我们添加一个 `delay()` 函数，以便在重复循环之前等待一秒。

## IIC

XIAO RA4M1 具有 I2C 接口，可用于许多传感器的数据传输和解析，以及一些 OLED 屏幕的使用。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
        <th>Seeed Studio XIAO 扩展底座与 Grove OLED</th>
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

XIAO 扩展板上的 OLED 显示屏使用 I2C 协议，通过电路板上的 I2C 接口连接到 XIAO。因此，我们可以直接将 XIAO 插入扩展板并编程在屏幕上显示内容。

### 软件实现

本示例介绍如何在 Seeed Studio XIAO RA4M1 的扩展底座上使用 OLED 显示屏。

#### 步骤 1. 在扩展板上安装 Seeed Studio XIAO RA4M1，然后连接 Type-C 数据线。

#### 步骤 2. 安装 u8g2 库。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/olikraus/U8g2_Arduino">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库文件</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

#### 步骤 3. 复制代码并粘贴到 Arduino IDE 中，然后上传。

```c
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // 没有重置的 OLED

void setup(void) {
  u8x8.begin();
  u8x8.setFlipMode(1);   // 设置为 1 到 3，屏幕文字将旋转 180 度
}

void loop(void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  u8x8.print("i'm XIAO RA4M1");
}
```

在代码的前几行中，我们包括了所需的库，如 Arduino.h、U8x8lib.h 和 Wire.h。U8x8lib.h 库提供了控制 OLED 显示的函数，而 Wire.h 库提供了 I2C 通信的函数。

在 `setup()` 函数中，我们使用 `u8x8.begin()` 函数初始化 OLED 显示。我们还使用 `u8x8.begin()` 函数设置显示的翻转模式，以将屏幕旋转 180 度。

在 `loop() 函数`中，我们使用 `u8x8.setFont()` 函数设置字体，并使用 `u8x8.setCursor()` 函数指定光标在显示器上的位置。最后，我们使用 u8x8.print() 函数在 OLED 显示屏上显示字符串 "Hello World!"。

如果将程序上传到 XIAO RA4M1，您将看到扩展板上的 OLED 显示屏上显示内容。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/21.png" style={{width:800, height:'auto'}}/></div>

## SPI

RA4M1 芯片集成了多个外设，包括一个 SPI 接口，可用于连接外部 SPI 设备，如闪存、显示屏、传感器等。XIAO RA4M1 还支持高速 SPI 传输模式，最大 SPI 传输速率可达 80 MHz，满足大多数 SPI 设备的数据传输需求。

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
      <th>Grove - OLED 显示屏 1.12 (SH1107) V3.0 - SPI/IIC</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-OLED-Display-1.12-(SH1107)_V3.0/img/10402050_Main-02.png" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-OLED-Display-1-12-SH1107-V3-0-p-5011.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> Get One Now 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

在准备好上述硬件后，使用跳线连接 XIAO 和 OLED 的 SPI 接口。请参考以下图示了解接线方式。


### 软件实现

接下来，我们将以以下程序为例，介绍如何使用 SPI 接口控制 OLED 屏幕显示。

安装 u8g2 库。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/olikraus/U8g2_Arduino">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库文件</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

```c
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
 
U8G2_SH1107_128X128_1_4W_HW_SPI u8g2(U8G2_R3, /* cs=*/ D7, /* dc=*/ D4, /* reset=*/ D5);
 
void setup(void) {
  u8g2.begin();
}
 
void loop(void) {
  u8g2.firstPage();

  do {
    u8g2.setFont(u8g2_font_luBIS08_tf);
    u8g2.drawStr(0,24,"Hello Seeed!");
  } while ( u8g2.nextPage() );
}
```

在 `setup()` 函数中，使用适当的构造函数参数实例化 `U8G2_SH1107_128X128_1_4W_HW_SPI` 类，以指定用于片选 (cs)、数据/命令 (dc) 和复位的引脚。然后，调用 `u8g2.begin()` 函数以初始化显示。

在 `loop()` 函数中，使用 `u8g2.firstPage()`、`u8g2.setFont()` 和 `u8g2.drawStr()` 函数更新显示内容。u8g2.firstPage() 函数设置显示缓冲区以进行写入，而 `u8g2.nextPage()` 则显示更新后的内容。do-while 循环确保内容持续显示，直到程序停止。

总体而言，这段代码演示了如何使用 U8g2 库控制 OLED 显示屏并在其上显示文本。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/15.png" style={{width:700, height:'auto'}}/></div>


## CAN

### 硬件准备

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO RA4M1</th>
	    <th>XIAO CAN Bus 扩展板</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/2-102010551-Seeed-Studio-XIAO-RA4M1-45font.jpg" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_can_bus_board/main.jpg" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=seeed&utm_medium=wiki&utm&product=xiao-ra4m1">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-CAN-Bus-Breakout-Board-for-XIAO-and-QT-Py-p-5702.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

#### 步骤 1. 准备两个 CAN Bus 扩展板和 XIAO RA4M1
#### 步骤 2. 将这两个 XIAO RA4M1 分别插入 CAN Bus 扩展板
#### 步骤 3. 准备杜邦线连接。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/36.png" style={{width:400, height:'auto'}}/></div>

### 软件实现
:::tip
不允许同时为两个 XIAO RA4M1 供电并下载程序，这将导致下载串口时出现错误。下载完一个后，拔掉它，然后为另一个 XIAO RA4M1 供电以下载程序，最后同时供电以检查串口信息。
:::

**CAN 写入代码**
```c

/*  从 CAN 总线发送帧

    CAN 波特率,
    
    #define CAN_5KBPS           1
    #define CAN_10KBPS          2
    #define CAN_20KBPS          3
    #define CAN_25KBPS          4 
    #define CAN_31K25BPS        5
    #define CAN_33KBPS          6
    #define CAN_40KBPS          7
    #define CAN_50KBPS          8
    #define CAN_80KBPS          9
    #define CAN_83K3BPS         10
    #define CAN_95KBPS          11
    #define CAN_100KBPS         12
    #define CAN_125KBPS         13
    #define CAN_200KBPS         14
    #define CAN_250KBPS         15
    #define CAN_500KBPS         16
    #define CAN_666KBPS         17
    #define CAN_1000KBPS        18
*/
   
#include <mcp_can.h>
#include <SPI.h>

/* 请根据您的电路板修改 SPI_CS_PIN。

   CANBed V1        - 17
   CANBed M0        - 3
   CAN 总线扩展板   - 9
   CANBed 2040      - 9
   CANBed Dual      - 9
   OBD-2G 开发套件   - 9
   OBD-II 开发套件   - 9
   Hud 开发套件      - 9

   Seeed Studio CAN-Bus 扩展板 - D7
*/

#define SPI_CS_PIN  D7 

MCP_CAN CAN(SPI_CS_PIN);                                    // 设置 CS 引脚

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    
    // 以下代码适用于 OBD-II GPS 开发套件的 Atmega32U4 版本
    // pinMode(A3, OUTPUT);
    // digitalWrite(A3, HIGH);
    
    // 以下代码适用于 OBD-II GPS 开发套件的 RP2040 版本
    // pinMode(12, OUTPUT);
    // digitalWrite(12, HIGH);
    
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // 初始化 CAN 总线：波特率 = 500k
    {
        Serial.println("CAN 总线失败!");
        delay(100);
    }
    Serial.println("CAN 总线正常!");
}

unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
void loop()
{
    CAN.sendMsgBuf(0x00, 0, 8, stmp);
    delay(100);                       // 每 100 毫秒发送一次数据
}

// 文件结束


**CAN 读取代码**
```c
/* 接收来自 CAN 总线的帧

    CAN 波特率，

    #define CAN_5KBPS           1
    #define CAN_10KBPS          2
    #define CAN_20KBPS          3
    #define CAN_25KBPS          4 
    #define CAN_31K25BPS        5
    #define CAN_33KBPS          6
    #define CAN_40KBPS          7
    #define CAN_50KBPS          8
    #define CAN_80KBPS          9
    #define CAN_83K3BPS         10
    #define CAN_95KBPS          11
    #define CAN_100KBPS         12
    #define CAN_125KBPS         13
    #define CAN_200KBPS         14
    #define CAN_250KBPS         15
    #define CAN_500KBPS         16
    #define CAN_666KBPS         17
    #define CAN_1000KBPS        18

    CANBed V1: https://www.longan-labs.cc/1030008.html
    CANBed M0: https://www.longan-labs.cc/1030014.html
    CAN 总线扩展板: https://www.longan-labs.cc/1030016.html
    OBD-II CAN 总线 GPS 开发套件: https://www.longan-labs.cc/1030003.html
*/

#include <SPI.h>
#include "mcp_can.h"

/* 请修改 SPI_CS_PIN 以适应你的开发板。

   CANBed V1        - 17
   CANBed M0        - 3
   CAN 总线扩展板   - 9
   CANBed 2040      - 9
   CANBed Dual      - 9
   OBD-2G 开发套件   - 9
   OBD-II GPS 套件   - 9
   Hud 开发套件      - 9

   Seeed Studio 的 XIAO 和 QT Py 的 CAN 总线扩展板 - D7
*/

#define SPI_CS_PIN  D7 

MCP_CAN CAN(SPI_CS_PIN);                                    // 设置 CS 引脚


void setup()
{
    Serial.begin(115200);
    while(!Serial);
    
    // 以下代码适用于 OBD-II GPS 开发套件的 Atmega32U4 版本
    // pinMode(A3, OUTPUT);
    // digitalWrite(A3, HIGH);
    
    // 以下代码适用于 OBD-II GPS 开发套件的 RP2040 版本
    // pinMode(12, OUTPUT);
    // digitalWrite(12, HIGH);
    
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // 初始化 CAN 总线：波特率 = 500k
    {
        Serial.println("CAN 总线失败!");
        delay(100);
    }
    Serial.println("CAN 总线正常!");
}


void loop()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive())            // 检查是否有数据到来
    {
        CAN.readMsgBuf(&len, buf);    // 读取数据，len：数据长度，buf：数据缓冲区

        unsigned long canId = CAN.getCanId();
        
        Serial.println("-----------------------------");
        Serial.print("从 ID 获取数据: ");
        Serial.println(canId, HEX);

        for(int i = 0; i<len; i++)    // 打印数据
        {
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();
    }
}

// 文件结束

```
:::tip
在此示例中，您需要焊接 CAN 总线扩展板的一个终端引脚 P1，只有这样才能使用任何速度，否则您只能使用低于 125 Kbps 的 CAN 波特率。
:::

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/30.png" style={{width:600, height:'auto'}}/></div>

***何时需要连接终端电阻？***
- 1. 长距离通信：如果 CAN 总线较长（例如超过 1 米），必须在总线的两端连接终端电阻，以避免因信号反射导致的通信问题。
- 2. 多节点通信：如果多个节点连接到同一 CAN 总线上，终端电阻也是不可或缺的。它们确保总线的阻抗稳定，从而防止信号失真。

***何时可以断开终端电阻？***
- 1. 短距离通信：在某些短距离应用中（通常小于 1 米），可以省略终端电阻，因为信号反射对通信的影响相对较小。
- 2. 单节点通信：如果总线上只有一个节点（例如在调试环境中），且距离较短，可以暂时断开终端电阻。

<table align="center">
	<tr>
	    <th>发送器代码结果</th>
	    <th>接收器代码结果</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/25.png" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-R4AM1/img/26.png" style={{width:500, height:'auto'}}/></div></td>
	</tr>
</table>

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时体验尽可能顺畅。我们提供几种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>

