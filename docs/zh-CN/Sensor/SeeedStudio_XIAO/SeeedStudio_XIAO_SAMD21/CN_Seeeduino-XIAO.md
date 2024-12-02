---
description: Getting Started with Seeed Studio XIAO SAMD21
title: Getting Started with Seeed Studio XIAO SAMD21 开发板
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/Seeeduino-XIAO
last_update:
  date: 11/13/2024
  author: Agnes
---

# Seeed Studio XIAO SAMD21 开发

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Seeeduino-XIAO-preview-1.jpg" alt="pir" width={600} height="auto" /></p>

Seeed Studio XIAO SAMD21（原名Seeeduino XIAO）是[Seeed Studio XIAO](https://www.seeedstudio.com/xiao-series-page)系列的首次亮相，该系列包括一系列功能强大的拇指大小开发板，兼容Arduino。它搭载了强大的ATSAMD21G18A-MU，这是一款低功耗微控制器。另一方面，这块小板在处理性能上表现出色，但功耗较低。它的尺寸非常紧凑，适合用于可穿戴设备和小型项目。

Seeed Studio XIAO SAMD21有14个引脚，其中包括11个数字接口、11个模拟接口、10个PWM接口（d1-d10）、1个DAC输出引脚D0、1个SWD调试接口、1个I2C接口、1个SPI接口、1个UART接口、串行通信指示灯（T/R）、通过引脚复用的闪烁灯（L）。LED灯（电源、L、RX、TX）的颜色分别为绿色、黄色、蓝色和蓝色。此外，Seeed Studio XIAO SAMD21还配备了一个Type-C接口，可以供电并下载代码。板子上有两个重置按钮，您可以将它们短接来重置板子。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    </a>
</div>

## 文档

关于**Seeed Studio XIAO SAMD21**的使用，有两篇文档分别关注了不同的领域，可以参考下表:

| [**Seeed文档**](https://wiki.seeedstudio.com/Seeeduino-XIAO/) | [**Documentation by Nanase**](https://wiki.seeedstudio.com/Seeeduino-XIAO-by-Nanase/) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Pinout Digram                                                | Interface（接口）                                            |
| Seeed Studio XIAO SAMD21 Getting Started                     | Seeed Studio XIAO SAMD21 with MicroSD Card(SPI)              |
| Seeed Studio XIAO SAMD21 GPIO Usage                          | Seeed Studio XIAO SAMD21 with GPS(UART)                      |
| Seeed Studio XIAO SAMD21 Resources                           | Single Cycle IOBUS                                           |

### **CircuitPython 在 Seeed Studio XIAO SAMD21上的运用**

- 可以访问 [**CircuitPython on Seeed Studio XIAO SAMD21 **](http://wiki.seeedstudio.com/Seeeduino-XIAO-CircuitPython).

## **特征**

- **强大的中央处理器**: ARM® Cortex®-M0+ 32bit 48MHz 微控制器(SAMD21G18)，带有 256KB Flash 和 32KB SRAM 。
- **兼容性强**: 兼容 Arduino IDE.
- **项目易于操作**: 适合面包板的设计。
- **迷你尺寸**: 适用于可穿戴设备和小型项目。
- **多种开发接口**: 11 个数字/模拟引脚，10 个 PWM 引脚，1 个 DAC 输出，1 个 SWD Bonding pad 接口，1 个 I2C 接口，1 个 UART 接口，1 个 SPI 接口。

## **规格参数**

|项目|Value|
|---|---|
|CPU（中央处理器）|运行频率高达 48MHz 的 ARM Cortex-M0+ CPU(SAMD21G18|
|闪存|256KB|
|SRAM（静态随机存取存储器）|32KB|
|数字 I/O 引脚|11|
|模拟 I/O 引脚|11|
|I2C 接口|1|
|SPI 接口|1|
|QTouch|7 (A0,A1,A6,A7,A8,A9,A10)|
|串口接口|1|
|电源下载接口| Type-C|
|工作电压|3.3V/5V DC|
|尺寸|20×17.5×3.5mm|

## 硬件概述

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Seeeduino-XIAO-pinout-1.jpg" alt="pir" width={600} height="auto" /></p>

<p style={{textAlign: 'center'}}><img src="https:///files.seeedstudio.com/wiki/Seeeduino-XIAO/img/new1.png" alt="pir" width={600} height="auto" /></p>

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Seeeduino%20XIAO%20pinout%202.png" alt="pir" width={600} height="auto" /></p>

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/regulator_to_3.3v.png" alt="pir" width={600} height="auto" /></p>

:::note
**对于普通的 I/O 引脚**：
MCU的工作电压为3.3V。如果将高于 3.3V 的电压输入连接到普通的 I/O 引脚，可能会导致芯片损坏。

**对于电源引脚**：
内置的DC-DC转换电路能够将5V电压转换成3.3V电压，因此可以通过VIN引脚和5V引脚使用5V供电来为设备供电。请注意使用，不要拆卸防护盖。
:::

### **进入Bootloader模式**

当用户编程过程失败时，Seeed Studio XIAO SAMD21 端口可能会消失。我们可以通过以下操作解决这个问题：

- 将 Seeed Studio XIAO SAMD21 连接到您的计算机。
- 用镊子或短线将图中的RST管脚短接**两次**。
- 橙色 LED 灯亮起并闪烁。

此时，芯片进入Bootloader模式，也就是引导加载程序模式，并且烧录端口再次出现。因为SAMD21芯片有两个分区，一个是Bootloader，另一个是用户程序。产品出厂时会在系统内存中烧录引导加载程序代码。我们可以通过执行上述步骤来切换模式。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/XIAO-reset.gif" alt="pir" width={600} height="auto" /></p>

### **重置**

如果要重置 Seeed Studio XIAO SAMD21，请执行以下步骤：

- 将 Seeed Studio XIAO SAMD21 连接到您的电脑上。
- 使用镊子或短线将RST引脚短接**一次**即可
- 橙色 LED 灯亮起并闪烁。

**请注意**：内置LED的行为方式与Arduino上的相反。在Seeed Studio XIAO SAMD21上，该引脚必须被拉低以点亮LED；而在其他微控制器上，它必须被拉高。

### **中断**

Seeed Studio XIAO SAMD21上的所有引脚都支持中断，但是5号引脚和7号引脚不能同时使用。有关中断的更多详细信息，请在[这里](https://github.com/Seeed-Studio/ArduinoCore-samd/blob/master/variants/XIAO_m0/variant.cpp)查看。

### **引脚多路复用**

我们无需自己配置引脚，在使用完引脚后，可以直接调用函数。

#### **数字输入和输出**

- 使用引脚 6 作为数字引脚：

```c
const int buttonPin = 6;     // 按钮引脚的编号
const int ledPin =  13;      // LED引脚的编号

int buttonState = 0;         // 变量用于读取按钮状态

void setup() {
  // 初始化LED引脚为输出模式：
  pinMode(ledPin, OUTPUT);
  // 初始化按钮引脚为输入模式：
  pinMode(buttonPin, INPUT);
}

void loop() {
  // 读取按钮的状态：
  buttonState = digitalRead(buttonPin);

  // 检查按钮是否被按下。如果按下，buttonState为HIGH：
  if (buttonState == HIGH) {
    // 打开LED：
    digitalWrite(ledPin, HIGH);
  } else {
    // 关闭LED：
    digitalWrite(ledPin, LOW);
  }
}
```

#### **模拟读取**

- 使用引脚 6 作为模拟引脚:

```c


void setup() {
  // 将ledPin声明为输出模式：
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // 从传感器读取值：
  sensorValue = analogRead(sensorPin);
  // 打开LED：
  digitalWrite(ledPin, HIGH);
  // 停止程序，延时<sensorValue>毫秒：
  delay(sensorValue);
  // 关闭LED：
  digitalWrite(ledPin, LOW);
  // 停止程序，再延时<sensorValue>毫秒：
  delay(sensorValue);
}
```

#### **串口**

- 使用引脚 6 作为 UART 的 TX 引脚（UART 的 RX 引脚为引脚 7）：

```c

void setup() {
    Serial1.begin(115200);
    while (!Serial);
}

void loop() {
    Serial1.println("Hello,World");
    delay(1000);
}
```

#### **I2C**

- 使用引脚5作为I2C的SCL引脚（SDA引脚为引脚4）：

```c
// I2C主机写入示例
// by Nicholas Zambetti <http://www.zambetti.com>

// 演示使用Wire库
// 向I2C/TWI从设备写入数据
// 请参考“Wire Slave Receiver”示例以配合使用

// 创建于2006年3月29日

// 该示例代码属于公共领域。

#include <Wire.h>

void setup()
{
  Wire.begin(); // 加入I2C总线（对于主机来说，地址是可选的）
}

byte x = 0;

void loop()
{
  Wire.beginTransmission(4); // 向设备#4传输数据
  Wire.write("x is ");        // 发送5个字节
  Wire.write(x);              // 发送1个字节  
  Wire.endTransmission();    // 结束传输
  x++;
  delay(500);
}
```

#### **SPI**

- 使用8号引脚作为SPI的SCK引脚（SPI的MISO引脚为9号引脚，MOSI引脚为10号引脚）:

```c
#include <SPI.h>
const int CS = 7;

void setup (void) {
   digitalWrite(CS, HIGH); // 禁用从设备选择
   SPI.begin ();
   SPI.setClockDivider(SPI_CLOCK_DIV8); // 将时钟分频为8
}

void loop (void) {
   char c;
   digitalWrite(CS, LOW); // 启用从设备选择
   // 发送测试字符串
   for (const char * p = "Hello, world!\r" ; c = *p; p++) {
      SPI.transfer (c);
   }
   digitalWrite(CS, HIGH); // 禁用从设备选择
   delay(2000);
}
```

#### **QTouch**

我们提供了一个示例工程关于如何使用 QTouch： [如何利用Q-Touch 功能在Seeed Studio XIAO SAMD21 上制作水果钢琴](https://www.seeedstudio.com/blog/2020/07/20/how-to-make-a-fruit-piano-on-seeeduino-xiaos-q-touch-function-m/).

#### **模拟输入和输出**

尽管 SAM21 仍然具有基于脉宽调制（PWM）的“模拟输出”，但它还配备了数字模拟转换器（DAC），提供真正的模拟输出功能。该模块可在 0 到 3.3V 之间生成模拟电压。它可用于产生更自然的音频声音，或作为“数字电位器”来控制模拟设备。

DAC 仅可用于 Arduino 引脚 A0 ，并可使用 analogWrite（A0，`<value>`）进行控制。 DAC 可以设置为 10 位分辨率（确保在设置中调用 [**analogWriteResolution(10)**](https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogwriteresolution/) ),这意味着 0 到 1023 之间的值将电压设置在 0 到 3.3V 之间的某个位置。

除了DAC，SAMD21的ADC通道也与ATmega328有所不同：它们支持高达12位的分辨率。这意味着模拟输入值的范围为0到4095，表示的电压范围为0到3.3V。要在12位模式下使用ADC，请确保调用[**analogReadResolution(12)**](https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/)，这样才能在12位模式下正确使用ADC。

**串行绘制 DAC**

下面是一个演示 DAC 和 ADC 的示例。接下来开始实验，请将 A0 连接到 A1 -- 我们将用模拟电压驱动 A0，然后使用 A1 读取它。这是我们在教程中介绍过的最简单的电路：

<div align="center"><img width="{600}" src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/AO_A1.jpg" /></div>

:::note
上述图片中 Seeed Studio XIAO SAMD21 正在使用 [**Seeed Studio XIAO SAMD21 拓展板**](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html)
:::

这个程序在引脚A0上产生一个 0 到 3.3V 范围内的正弦波输出，然后使用 A1 读取该输出并将其转换为 0 到 3.3V 之间的电压值。

您可以打开串口监视器查看电压值的流动。但是，如果正弦波很难通过文本可视化，请尝试打开 Arduino 的新串口绘图工具，方法是： Tools > Serial Plotter.

<div align="center"><img width="{600}" src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Serial%20poltting.png" /></div>

<div align="center"><img width="{600}" src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/XIAO_DAC_wave.gif" /></div>

代码示例

```cpp
#define DAC_PIN A0 // 使代码更具可读性

float x = 0; // 要取正弦值的变量
float increment = 0.02;  // 每次增加的x值
int frequency = 440; // 正弦波的频率

void setup() 
{
  analogWriteResolution(10); // 设置模拟输出分辨率为最大值，10位
  analogReadResolution(12); // 设置模拟输入分辨率为最大值，12位

  SerialUSB.begin(9600);
}

void loop() 
{
  // 生成一个0到1023之间的电压值。
  // 让我们将正弦波的值缩放到这些值之间：
  // 偏移量为511.5，然后将正弦值乘以511.5。
  int dacVoltage = (int)(511.5 + 511.5 * sin(x));
  x += increment; // 增加x的值

  // 生成一个0到3.3V之间的电压值。
  // 0 = 0V，1023 = 3.3V，512 = 1.65V，依此类推。
  analogWrite(DAC_PIN, dacVoltage);

  // 现在读取A1（连接到A0），并将该
  // 12位ADC值转换为0到3.3V之间的电压。
  float voltage = analogRead(A1) * 3.3 / 4096.0;
  SerialUSB.println(voltage); // 打印电压值。
  delay(1); // 延时1毫秒
}
```

## **上手指南**

### **硬件**

**所需材料**

- Seeed Studio XIAO SAMD21 x1
- 电脑x1
- USB typc数据线 x1

:::tip

有些USB数据线只能供电，不能传输数据。如果你没有USB线或者不知道你的USB线是否可以传输数据，你可以查看 [seeed USB type C support USB 3.1](https://www.seeedstudio.com/USB-Type-C-to-A-Cable-1Meter-p-4085.html).
:::

- Step 1. 准备一根 Seeed Studio XIAO SAMD21和一根 Type-C 数据线。

- Step 2. 将 Seeed Studio XIAO SAMD21 连接到您的计算机，紧接着会亮起黄色的 LED 灯。

### 软件

:::note

如果这是您第一次使用 Arduino，我们建议您可以参考学习 [Getting Started with Arduino](https://wiki.seeedstudio.com/Getting_Started_with_Arduino)
:::

- **Step 1. 安装Arduino 软件**

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>Download Arduino IDE</font></span></strong>
    </a>
</div>
**启动Arduino应用程序**

双击您之前下载的 Arduino 应用程序 (arduino.exe)。

:::note

如果 Arduino 软件加载的语言与您需要的不同，您可以在首选项对话框中更改。有关详细信息，请参见 [Arduino Software (IDE) page](https://www.arduino.cc/en/Guide/Environment#languages) 。
:::

- **Step 2. 打开 Blink 示例**  

打开 LED 闪烁示例:**File > Examples >01.Basics > Blink**.

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino_GPRS/img/select_blink.png" alt="pir" width={600} height="auto" /></p>

- **Step 3. 将 Seeeduino板的URL 添加到您的 Arduino IDE**

点击 **File > Preference**, 然后再“Additional Boards Manager URLs”内添加以下网址 ：

*<https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json>*

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Wio-Terminal/img/Boardurl.png" alt="pir" width={600} height="auto" /></p>

请点击 **Tools-> Board-> Boards Manager...**, 在搜索框中输入关键词"**Seeed Studio XIAO SAMD21**" 出现 "**Seeed SAMD Boards**" 后，请进行安装。

<!-- ![](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/XIAO-board.png) -->
  <p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/XIAO-board.png" alt="pir" width={600} height="auto" /></p>

- **Step 4. 选择您的板和端口**

安装好板子后，点击 **工具-> 板**，找到 **"Seeed Studio XIAO"** 并选择它。现在，您已经为Arduino IDE设置了Seeed Studio XIAO SAMD21板。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino_GPRS/img/1.jpg" alt="pir" width={600} height="auto" /></p>

从 **工具 | 串口** 菜单中选择Arduino板的串口设备。它很可能是 COM3 或更高的端口（**COM1** 和 **COM2** 通常保留给硬件串口）。您可以断开Arduino板，重新打开菜单；消失的那一项应该是Arduino板。重新连接板子并选择该串口。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino_GPRS/img/2.jpg" alt="pir" width={600} height="auto" /></p>

- **Step 5.上传程序**

现在，只需在环境中点击 "`上传`" 按钮。等待几秒钟，如果上传成功，状态栏中将出现消息 "Done uploading."。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino_GPRS/img/upload_image.png" alt="pir" width={600} height="auto" /></p>

上传完成后几秒钟，您应该会看到开发板上的 13 号引脚（L）LED 开始闪烁（呈橙色）。如果是这样，恭喜您！您已经成功运行了 Arduino。如果出现问题，请参考故障排除建议。

:::note
闪存的最大大小为 8KB。 有关更多信息，请参阅资源中的 ATSAMD218A-MU 数据表
:::

## 其他教程

- [如何使用Seeed Studio XIAO SAMD21登录你的树莓派I](https://wiki.seeedstudio.com/How-to-use-Seeeduino-XIAO-to-log-in-to-your-Raspberry-PI)

- [SPI 通讯接口](https://wiki.seeedstudio.com/XIAO-SPI-Communication-Interface)

- [如何使用树莓派解除砖化的Xiao开发板](https://forum.seeedstudio.com/t/how-to-unbrick-a-dead-xiao-using-raspberry-pi-guide-openocd/253990). 感谢 John_Doe 的分享

## 资源下载

- **[PDF]** [ATSAMD218A-MU datasheet](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/ATSAMD21G18A-MU-Datasheet.pdf)

- **[PDF]** [Seeed Studio XIAO SAMD21 Schematic](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-v1.0-SCH-191112.pdf)

- **[ZIP]** [Seeed Studio XIAO SAMD21 KiCAD file](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-KICAD.zip)

- **[ZIP]** [Seeed Studio XIAO SAMD21 Eagle file](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-v1.0.zip)

- **[DXF]** [Seeed Studio XIAO SAMD21 Dimension in DXF](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/102010328_Seeeduino_XIAO_Dimension.rar)

- **[LBR]** [Seeed Studio XIAO SAMD21 Eagle footprint](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-footprint-eagle.lbr)

- **[ZIP]** [Seeed Studio XIAO SAMD21 Factory firmware](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/102010328_Seeeduino_XIAO_final_firmware.zip)

- **[XLSX]** [Seeed Studio XIAO SAMD21 pinout sheet](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/XIAO-SAMD21-pinout_sheet.xlsx)

- **[STEP]** [Seeed Studio XIAO SAMD21 3D Model](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/seeeduino-xiao-samd21-3d-model.zip)

- **[ZIP]** [Seeed Studio XIAO SAMD21 Certification files](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/XIAO-SAMD21-Certification.zip)

<!-- ## 课程资料

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO.jpg" /></div>

- **[PDF]** [Seeed Studio XIAO SAMD21 in Action——Minitype ＆ Wearable Projects Step by Step](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-in-Action-Minitype＆Wearable-Projects-Step-by-Step.pdf)
- **[ZIP]** [Codes](https://files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-Codes.rar)
- **[ZIP]** [Document](https:///files.seeedstudio.com/wiki/Seeeduino-XIAO/res/Seeeduino-XIAO-Document.rar) -->

## 课程资料

<div align="middle"><img width="400" src="https://mjrovai.github.io/XIAO_Big_Power_Small_Board-ebook/cover.jpg" /></div>

- **[电子书]** [Arduino小型化与TinyML应用 从入门到精通](https://tinkergen.cn/book_xiao)

## 技术支持和产品讨论

感谢您选择我们的产品!我们在这里为您提供不同的支持，以确保您使用我们的产品的体验尽可能顺利。我们提供多种沟通渠道，以满足不同的喜好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
