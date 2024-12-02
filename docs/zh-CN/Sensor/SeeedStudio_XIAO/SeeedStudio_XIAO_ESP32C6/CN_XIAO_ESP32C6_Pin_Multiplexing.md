---
description: 使用 Seeed Studio XIAO ESP32C6 进行引脚多路复用
title: 使用 Seeed Studio XIAO ESP32C6 进行引脚多路复用
keywords:
  - esp32c6
  - xiao
  - pin multiple
image: https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg
slug: /cn/xiao_pin_multiplexing_esp33c6
sidebar_position: 2
last_update:
  date: 11/16/2024
  author: Agnes
---

# Arduino 编程与 Seeed Studio XIAO ESP32C6

<div class="table-center">
	<table>
		<tr>
			<th>Seeed Studio XIAO ESP32C6</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html
        ">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

Seeed Studio XIAO ESP32C6 配备了高度集成的 [ESP32-C6 SoC](https://www.espressif.com/en/products/socs/esp32-c6)其内置 **两个 32 位 RISC-V 处理器**其中高性能 (HP) 处理器的主频最高可达 **160 MHz**，低功耗 (LP) 32 位 RISC-V 处理器的主频最高可达 20 MHz。芯片上配备 **512KB SRAM 和 4 MB Flash on the chip**，提供更多的编程空间，带来更多物联网控制场景的可能性。 

## 开始使用

### 引脚概览

在开始之前，让我们回顾一下 XIAO ESP32C6 的所有引脚及其功能，见下图。

<table align="center">
	<tr>
	    <th>XIAO ESP32C6/XIAO ESP32C6 引脚图</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://wdcdn.qpic.cn/MTY4ODg1Nzc0ODUwMjM3NA_556525_Slxs4ARdyuXRrJK-_1711096256?w=9854&h=3367&type=image/png" style={{width:700, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <th>XIAO ESP32C6 引脚功能列表</th>
	</tr>
    <tr>
	    <td><div style={{textAlign:'center'}}><img src="https://wdcdn.qpic.cn/MTY4ODg1Nzc0ODUwMjM3NA_318648_dMoXitoaQiq2N3-a_1711678067?w=1486&h=1228" style={{width:1000, height:'auto'}}/></div></td>
	</tr>
</table>


- 5V - 这是来自 USB 端口的 5V 输出。你也可以将其用作电压输入，但你必须在外部电源与该引脚之间使用某种二极管（肖特基二极管、信号二极管、功率二极管），阳极连接电池，阴极连接 5V 引脚。
- 3V3 - 这是来自板载稳压器的稳压输出，可以提供最高 700mA 的电流。
- GND - 电源/数据/信号地线。 <!-- Need to be confirmed -->

## 串口通信
XIAO ESP32C6 支持两种串口通信方式：`软件串口` 和 `硬件串口`。软件串口通常用于灵活性需求，而硬件串口提供更好的性能。

### 硬件连接

1. 将外部设备的 **TX 引脚** 连接到 XIAO ESP32C6 的 RX 引脚 (`D7`) 
2. 将外部设备的 **RX 引脚** 连接到 XIAO ESP32C6 的 TX 引脚 (`D6`) 

### 代码示例

#### 软件串口

要使用软件串口，请安装 [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial) 库。

:::tip
目前我们推荐使用 **7.0.0** 版本的 EspSoftwareSerial 库。其他版本可能存在各种问题，导致软件串口无法正常工作。
:::

```cpp
#include <SoftwareSerial.h>

SoftwareSerial mySerial(D7, D6); // RX, TX

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if (mySerial.available()) {
    char data = mySerial.read();
    Serial.print("通过软件串口接收到: ");
    Serial.println(data);
  }

  if (Serial.available()) {
    char data = Serial.read();
    mySerial.print("通过硬件串口接收到: ");
    mySerial.println(data);
  }
}
```

此示例在引脚 `D7 (RX)` 和 `D6 (TX)` 上设置了软件串口，波特率为 9600。它监视 USB 串口和软件串口端口，并在两个端口之间回显接收到的数据。

#### 硬件串口

XIAO ESP32C6 配备了硬件 UART（UART0）进行串口通信，使用的引脚是 D7/D6。

```cpp
#include <HardwareSerial.h>

HardwareSerial mySerial(0); // UART0 (Serial0)

void setup() {
  Serial.begin(9600);  // USB 串口
  mySerial.begin(9600); // UART0
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read();
    Serial.print("通过 USB 接收到: ");
    Serial.println(data);
  }
  
  if (mySerial.available()) {
    char data = mySerial.read();
    Serial.print("通过 UART0 接收到: ");
    Serial.println(data);
  }
}
```

此示例使用硬件 UART0（Serial0）进行通信。它初始化 USB 串口和 UART0，然后监视两个端口的输入数据，并将接收到的消息打印到 USB 串口。

#### 使用 Serial1

根据上述 XIAO ESP32C6 引脚图，我们可以观察到 TX 引脚和 RX 引脚的使用方法。与串口通信不同，这里使用的方式也非常类似，只是需要添加一些额外的参数。

以下是一个使用硬件引脚进行串口通信的示例程序：

- `Serial1.begin(BAUD,SERIAL_8N1,RX_PIN,TX_PIN);` -- 启用 Serial1，函数原型为： `<Serial.Type>.begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);`
  - `baud`  :波特率
  - `config`:配置位
  - `rxPin` :接收引脚
  - `txPin` :发送引脚

需要注意的是，如果我们使用数字引脚端口来定义，则应使用 `#define RX_PIN D7` 和 `#define TX_PIN D6`，具体参数请参阅不同 XIAO 系列的引脚图。

以下是一个示例程序：

```c
#define RX_PIN D7
#define TX_PIN D6
#define BAUD 115200

void setup() {
    Serial1.begin(BAUD,SERIAL_8N1,RX_PIN,TX_PIN);
}
 
void loop() {
  if(Serial1.available() > 0)
  {
    char incominByte = Serial1.read();
    Serial1.print("I received : ");
    Serial1.println(incominByte);
  }
  delay(1000);
}
```

上传程序后，打开 Arduino IDE 中的串口监视器并设置波特率为 115200。然后，你可以在串口监视器中输入你想发送的内容，XIAO 将打印出你发送的每一个字节。在这里，我输入的内容是 "Hello Everyone"，结果如图所示：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/114.png" style={{width:600, height:'auto'}}/></div>

## 数字输入输出 (Digital I/O)

XIAO ESP32C6 具有 12 个 GPIO 引脚，可以将它们配置为输入或输出。

### 硬件准备

1. 将一个按钮连接到引脚 `D1`:
   - 使用外部上拉电阻（如果使用内部上拉电阻，可以省略此步骤）。
2. 将一个 LED 连接到引脚 `D10`:
   - 在 LED 与引脚之间串联一个限流电阻。

### 软件实现

GPIO API 提供了配置和与 GPIO 引脚交互的函数。有关更多细节，请参阅 [GPIO API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html) 文档。

```cpp
const int buttonPin = D1; // 按钮引脚
const int ledPin = D10;   // LED 引脚

void setup() {
  pinMode(ledPin, OUTPUT);  // 设置 LED 引脚为输出
  pinMode(buttonPin, INPUT); // 设置按钮引脚为输入
  // 如果不使用外部上拉电阻
  pinMode(buttonPin, INPUT_PULLUP); // 启用内部上拉电阻
}

void loop() {
  int buttonState = digitalRead(buttonPin); // 读取按钮状态
  digitalWrite(ledPin, buttonState); // 将按钮状态写入 LED
}
```

#### 中断方法

您还可以使用中断来更有效地处理按钮按下事件。

```cpp
// 定义按钮和 LED 的引脚
const int buttonPin = D1;
const int ledPin = D10;

// 定义一个结构体来存储按钮相关的数据
struct Button {
    const uint8_t PIN; // 按钮引脚号
    uint32_t numberKeyPresses; // 按钮按下次数计数器
    bool pressed; // 标志，表示按钮是否被按下
};

// 创建一个 Button 结构体的实例
Button my_button = {buttonPin, 0, false};

// 中断服务例程（ISR）处理按钮按下事件
void ARDUINO_ISR_ATTR isr(void* arg) {
    Button* s = static_cast<Button*>(arg); // 将参数转换为 Button 指针
    s->numberKeyPresses += 1; // 增加按钮按下次数
    s->pressed = true; // 设置按钮按下标志
}

void setup() {
    Serial.begin(115200);
    pinMode(my_button.PIN, INPUT_PULLUP); // 设置按钮引脚为输入并启用内部上拉电阻
    attachInterruptArg(my_button.PIN, isr, &my_button, FALLING); // 将 ISR 附加到按钮引脚，触发条件为下降沿
}

void loop() {
    if (my_button.pressed) { // 检查按钮是否被按下
        Serial.printf("按钮已按下 %lu 次\n", my_button.numberKeyPresses); // 打印按钮按下次数
        my_button.pressed = false; // 重置按钮按下标志
    }

    static uint32_t lastMillis = 0; // 用于存储上次断开中断的时间
    if (millis() - lastMillis > 10000) { // 如果已过去 10 秒
        lastMillis = millis(); // 更新最后断开中断的时间
        detachInterrupt(my_button.PIN); // 断开按钮引脚的中断
    }
}
```

在这个例子中，我们使用 `Button` 结构体来存储按钮相关的数据，包括引脚号、按钮按下次数和一个标志，表示按钮当前是否被按下。

`isr` 函数是一个中断服务例程（ISR），用于处理按钮按下事件。它增加按钮按下次数，并将按下标志设置为 `true`。

在 `setup` 函数中，我们初始化串口通信，设置按钮引脚为输入并启用内部上拉电阻，并将 `isr` 函数作为中断处理程序附加到按钮引脚，触发条件为下降沿（按钮按下）。

在 `loop` 函数中，我们检查按钮是否被按下。如果被按下，我们打印按钮按下次数到串口监视器，并重置按下标志。此外，我们每 10 秒断开一次中断，可能是为了允许执行其他操作或防止不必要的中断。

好了，了解这些之后，这是重写的、更容易理解的版本：

## ADC - 模拟到数字转换器

XIAO ESP32C6 具有多个模拟输入引脚，允许读取模拟电压值。

有关更多详细信息，请参阅 [ADC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html) 文档。

### 硬件设置

1. 将一个电位器连接到引脚 A0，一端连接到 3.3V，另一端连接到 GND。

### 软件实现

以下是一个读取模拟值的 Arduino 示例：

```cpp
const int analogPin = A0; 

void setup() {
  // 初始化串口通信，波特率为 115200
  Serial.begin(115200);
  
  // 设置分辨率为 12 位（0-4095）
  analogReadResolution(12);
}

void loop() {
  // 读取模拟值和模拟引脚的毫伏值
  int analogValue = analogRead(analogPin);
  int analogVolts = analogReadMilliVolts(analogPin);
  
  // 打印值到串口监视器
  Serial.printf("ADC 模拟值 = %d\n", analogValue);
  Serial.printf("ADC 毫伏值 = %d\n", analogVolts);
  
  delay(100); // 延迟，方便清晰读取串口数据
}
```

这段代码从指定的引脚读取模拟值，并将其及其对应的毫伏值打印到串口监视器。

## PWM 信号 / LED 控制

XIAO ESP32-C6 具有 6 个 LEDC 通道，可以生成独立的波形，例如用于驱动 RGB LED 设备。

有关更多信息，请参阅 [LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html) 文档。

### 硬件设置

1. 将一个 LED 连接到引脚 `D2`，并在 LED 与引脚之间串联一个限流电阻。

### 软件实现

以下是演示 PWM 输出的 Arduino 示例：

#### 一般 PWM

```cpp
const int ledPin = D2;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    analogWrite(ledPin, dutyCycle);
    delay(10);
  }
}
```

这段代码通过 PWM 控制 LED 的亮度，逐步增加亮度。

#### LED 控制

```cpp
/*
 LEDC 软件渐变

 本例展示了如何使用 ledcWrite 函数通过软件控制 LED 渐变。

 代码改编自原始的 Arduino 渐变例子：
 https://www.arduino.cc/en/Tutorial/Fade

 本例代码属于公有领域。
 */

// 使用 12 位精度的 LEDC 定时器
#define LEDC_TIMER_12_BIT  12

// 使用 5000 Hz 作为 LEDC 基频
#define LEDC_BASE_FREQ     5000

// 渐变 LED 引脚（如果使用内置 LED，请替换为 LED_BUILTIN 常量）
#define LED_PIN            D5

int brightness = 0;    // LED 的亮度
int fadeAmount = 5;    // 每次渐变的亮度变化量

// 类似于 Arduino 的 analogWrite
// 数值必须在 0 到 valueMax 之间
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // 计算占空比，4095 来自 2^12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // 将占空比写入 LEDC
  ledcWrite(pin, duty);
}

void setup() {
  // 设置定时器并将定时器附加到 LED 引脚
  ledcAttach(LED_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
}

void loop() {
  // 设置 LEDC 通道的亮度
  ledcAnalogWrite(LED_PIN, brightness);

  // 改变下次循环的亮度
  brightness = brightness + fadeAmount;

  // 在渐变的两端反转亮度变化方向
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // 等待 30 毫秒，以查看渐变效果
  delay(30);
}
```

这段代码演示了如何使用 ledcWrite 函数来实现 LED 渐变效果。LED 的亮度会在持续的循环中逐渐增加和减少。

## I2C

XIAO ESP32C6 配备硬件 I2C 接口，用于与 I2C 设备进行通信。

有关更多细节，请参阅 [I2C API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html) 文档。

### 硬件准备

1. 将 I2C 设备的 SDA 引脚连接到 XIAO 的 SDA 引脚 (`D4`) 。
2. 将 I2C 设备的 SCL 引脚连接到 XIAO 的 SCL 引脚 (`D5`) 。

### 软件实现

#### 主模式

以下是一个示例，展示如何从 I2C 传感器读取数据：

```cpp
#include <Wire.h>

const int sensorAddress = 0x40;

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  Wire.beginTransmission(sensorAddress);
  Wire.write(0x01);  // 寄存器地址
  Wire.endTransmission();

  Wire.requestFrom(sensorAddress, 2);
  if (Wire.available() >= 2) {
    int data = Wire.read() << 8 | Wire.read();
    Serial.println(data);
  }

  delay(100);
}
```

这段代码从 I2C 传感器的寄存器 `0x01` 读取一个 16 位值。

#### 从模式

以下是一个示例，展示如何将 XIAO ESP32C6 用作 *I2C 从设备*：

```cpp
#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len) {
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
}

void loop() {
  // 从设备代码部分
}
```

在这个从设备模式示例中，XIAO ESP32C6 被配置为 I2C 从设备，地址为 `0x55`。当从设备接收到主设备的数据时，`onReceive` 回调函数会被调用；当主设备请求数据时，`onRequest` 回调函数会被调用。

## SPI

XIAO ESP32C6 微控制器板具有内建的 SPI 接口，能够与其他支持 SPI 的设备进行快速的数据交换。这在需要多个设备之间进行快速通信的项目中非常有用。

- 有关详细的技术规格，请参阅 [XIAO ESP32C6 数据手册](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)。
- 有关如何使用 SPI 接口与 XIAO ESP32C6 配合使用的更多信息，请参考 [SPI API 文档](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/spi.html#)。

### 硬件准备

要将 XIAO ESP32C6 连接到另一个 SPI 设备，请按照以下步骤操作：

1. **MOSI (Master Out Slave In):** 将 SPI 设备的 `MOSI` 引脚连接到 XIAO 的 `D10` 引脚。
2. **MISO (Master In Slave Out):** 将 SPI 设备的 `MISO` 引脚连接到 XIAO 的 `D9` 引脚。
3. **SCK (Serial Clock):** 将 SPI 设备的 `SCK` 引脚连接到 XIAO 的 `D8` 引脚。
4. **CS (Chip Select):** 将 SPI 设备的 `CS` 引脚连接到 XIAO 的一个数字引脚（例如，`D3`）。

```
MOSI -> D10
MISO -> D9
SCK -> D8
CS -> D3 (例如)
```

### 软件实现

以下是一个简化的 Arduino 程序，展示了如何使用 XIAO ESP32C6 与 SPI 设备进行基本的 SPI 通信。该程序向 SPI 设备发送命令并读取响应（从 SPI 设备读取数据）。

```cpp
#include <SPI.h>

const int csPin = 3;  // 使用 D3 引脚作为芯片选择（CS）

void setup() {
  // 初始化 SPI 通信
  SPI.begin();             
  // 设置 CS 引脚为输出
  pinMode(csPin, OUTPUT); 
  // 将 CS 引脚设置为高电平，表示没有正在进行的通信
  digitalWrite(csPin, HIGH);
}

void loop() {
  // 启动与设备的通信
  digitalWrite(csPin, LOW);
  SPI.transfer(0x01);  // 向设备发送命令
  int data = SPI.transfer(0);  // 读取响应
  digitalWrite(csPin, HIGH);  // 结束通信

  // 打印接收到的数据
  Serial.println(data);       
  delay(100);  // 等待短暂时间
}
```
:::note
请确保程序中的引脚分配与硬件设置中的物理连接匹配。上面的示例使用了基于 `pin_arduino.h` 文件的预定义引脚编号，并为 CS 引脚定义了额外的设置。
:::

## 资源

- [XIAO ESP32C6 文档](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)
- [ESP32-C6 数据手册](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)
- [Arduino 参考文档](https://www.arduino.cc/reference/en/)
- [Arduino ESP32 参考文档](https://docs.espressif.com/projects/arduino-esp32/en/latest/index.html)
