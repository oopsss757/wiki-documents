---
description: Seeed Studio XIAO ESP32C3的引脚复用
title: 引脚串口硬件使用教程
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/XIAO_ESP32C3_Pin_Multiplexing
last_update:
  date: 11/13/2024
  author: Agnes
---

# XIAO ESP32C3 引脚串口硬件使用教程

Seeed Studio XIAO ESP32C3接口丰富。有**11个数字I/O**可以用作**PWM引脚**和**4个模拟输入**可以用作**ADC引脚**。支持**UART、I2C、SPI、I2S**等4种串行通信接口。这个wiki将有助于学习这些接口并在您的下一个项目中实现它们!

## 数字

将按钮连接到引脚D6，并将LED连接到引脚D10。然后上传以下代码，使用按钮控制LED的开/关。

```c
const int buttonPin = D6;     // 按钮连接到数字引脚6
const int ledPin =  D10;      // LED连接到数字引脚10
 
int buttonState = 0;         // 用于读取按钮状态的变量
 
void setup() {
  // 将LED引脚初始化为输出：
  pinMode(ledPin, OUTPUT);
  // 将按钮引脚初始化为输入：
  pinMode(buttonPin, INPUT);
}

void loop() {
  // 读取按钮的状态：
  buttonState = digitalRead(buttonPin);
 
  // 检查按钮是否被按下。如果按下，按钮状态为HIGH：
  if (buttonState == HIGH) {
    // 打开LED：
    digitalWrite(ledPin, HIGH);
  } else {
    // 关闭LED：
    digitalWrite(ledPin, LOW);
  }
}
```

## 数字式PWM

将LED连接到引脚D10。然后上传以下代码，看到LED逐渐褪色。
```cpp
int ledPin = D10;    // LED连接到数字引脚10

void setup() {
  // 将LED引脚声明为输出
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // 从最小值逐渐增大到最大值，增量为5：
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // 设置值（范围从0到255）：
    analogWrite(ledPin, fadeValue);
    // 等待30毫秒以看到渐变效果
    delay(30);
  }

  // 从最大值逐渐减小到最小值，增量为5：
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // 设置值（范围从0到255）：
    analogWrite(ledPin, fadeValue);
    // 等待30毫秒以看到渐变效果
    delay(30);
  }
}
```

## 模拟

将电位器连接到引脚A0，并将LED连接到引脚D10。然后上传以下代码，通过旋转电位器旋钮来控制LED的闪烁间隔。
```c
const int sensorPin = A0;
const int ledPin =  D10; 

void setup() {
  pinMode(sensorPin, INPUT);  // 将传感器引脚声明为输入
  pinMode(ledPin, OUTPUT);   // 将LED引脚声明为输出
}

void loop() {
  // 读取传感器的值：
  int sensorValue = analogRead(sensorPin);
  // 打开LED引脚
  digitalWrite(ledPin, HIGH);
  // 程序暂停<sensorValue>毫秒：
  delay(sensorValue);
  // 关闭LED引脚：
  digitalWrite(ledPin, LOW);
  // 程序暂停<sensorValue>毫秒：
  delay(sensorValue);
}
```

## 串口

### 常规方法-选择USB串口或UART0串口之一使用

该开发板上有2个串口:

- USB 串口
- UART0 串口

:::note
XIAO ESP32 C3 没有 `Serial2`。
另外，如果您需要使用 `Serial1`，则必须定义引脚;否则，它可能不会接收数据。对于 XIAO ESP32 系列，使用 `Serial1` 如下：

```cpp
Serial1.begin(115200, SERIAL_8N1, D7, D6); // RX, TX
```
:::

默认情况下，USB串口是启用的，这意味着您可以通过USB Type-C将板连接到PC上，并在Arduino IDE上打开串口监视器以查看通过串口发送的数据。
但是，如果要使用UART0作为串口，则需要使用usb -串口适配器将引脚D6连接为TX引脚，将引脚D7连接为RX引脚。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-3.png" alt="pir" width={1000} height="auto" /></div>

此外，您需要将**USB CDC On Boot**设置为**Disabled**从Arduino IDE。

**注意:更改照片时，板显示在Arduino板管理器**。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-1.png" alt="pir" width={600} height="auto" /></div>

将以下代码上传到Arduino IDE，通过串口发送字符串“Hello World!

```cpp
void setup() {
    Serial.begin(115200);
    while (!Serial);
}
 
void loop() {
    Serial.println("Hello World!");
    delay(1000);
}
```

Arduino Serial Monitor上的输出如下所示

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-2.jpg" alt="pir" width={450} height="auto" /></div>

### 特殊方式-同时使用USB串口和UART0/UART1

很多时候，我们需要使用UART传感器连接到XIAO ESP32C3硬件串口来获取数据，同时可能需要使用USB串口在串口监视器上显示数据。这可以通过一些特殊的方法来实现。

- 示例程序:

```c
// 需要此库以进行低级别访问设置
#include <HardwareSerial.h>

// 定义两个串口设备，映射到两个内部UART
HardwareSerial MySerial0(0);
HardwareSerial MySerial1(1);

void setup()
{
    // 对于USB，正常使用Serial：
    Serial.begin(115200);

    // 配置MySerial0，TX=D6，RX=D7（-1, -1表示使用默认设置）
    MySerial0.begin(9600, SERIAL_8N1, -1, -1);
    MySerial0.print("MySerial0");

    // 配置MySerial1，RX=D9，TX=D10
    MySerial1.begin(115200, SERIAL_8N1, 9, 10);
    MySerial1.print("MySerial1");
}

void loop()
{
 
}
```

正如您所看到的，XIAO ESP32C3实际上有三个可用的uart。
下面，我们将以已经上市的[60GHz毫米波传感器-人类静息呼吸和心跳模块](https://www.seeedstudio.com/60GHz-mmWave-Radar-Sensor-Breathing-and-Heartbeat-Module-p-5305.html)为例，说明D6和D7硬件串口和USB串口的使用方法。

请做好以下准备。

<table align="center">
 <tr>
     <th>XIAO ESP32C3</th>
        <th>60GHz毫米波传感器  -<br/>人体静息呼吸<br/>和心跳模块</th>
 </tr>
    <tr>
        <td><div align="center"><img width = {120} src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png"/></div></td>
        <td><div align="center"><img width = {240} src="https://files.seeedstudio.com/wiki/60GHzradar/newpic.png"/></div></td>
    </tr>
 <tr>
        <td align = "center"><a href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">点击购买</a></td>
        <td align = "center"><a href="https://www.seeedstudio.com/60GHz-mmWave-Radar-Sensor-Breathing-and-Heartbeat-Module-p-5305.html">点击购买</a></td>
 </tr>
</table>

将传感器库下载到计算机上。并将其添加到Arduino IDE中。

<p style={{textAlign: 'center'}}><a href="https://github.com/limengdu/Seeed-Studio-MR60BHA1-Sensor/" target="_blank"><div align="center"><img width = {300} src="https://files.seeedstudio.com/wiki/seeed_logo/github.png" /></div></a></p>

这里，我们要解析心跳和呼吸数据信息，然后您可以像这样重写您的程序。

```c
#include "Arduino.h"
#include <60ghzbreathheart.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(0);   // 创建新的HardwareSerial类 -- D6/D7

// 也可以尝试使用硬件串口
BreathHeart_60GHz radar = BreathHeart_60GHz(&MySerial);

void setup() {
  // 放入你的设置代码，只运行一次：
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, -1, -1); // CPU频率为40MHz，工作在定义速度的一半。

  while(!Serial);   // 当串口打开时，程序开始执行。

  Serial.println("准备就绪");

  // radar.ModeSelect_fuc(1);  // 1: 表示实时传输模式，2: 表示睡眠模式。
  // 设置模式后，如果没有返回数据，可能需要重新为传感器供电。
}

void loop()
{
  // 放入你的主循环代码，重复运行：
  radar.Breath_Heart();           // 输出呼吸和心跳信息
  if(radar.sensor_report != 0x00){
    switch(radar.sensor_report){
      case HEARTRATEVAL:
        Serial.print("传感器监测到当前心率值为: ");
        Serial.println(radar.heart_rate, DEC);
        Serial.println("----------------------------");
        break;
      case HEARTRATEWAVE:  // 仅在实时数据传输模式下有效
        Serial.print("心率波形（正弦波）-- 点1: ");
        Serial.print(radar.heart_point_1);
        Serial.print(", 点2: ");
        Serial.print(radar.heart_point_2);
        Serial.print(", 点3: ");
        Serial.print(radar.heart_point_3);
        Serial.print(", 点4: ");
        Serial.print(radar.heart_point_4);
        Serial.print(", 点5: ");
        Serial.println(radar.heart_point_5);
        Serial.println("----------------------------");
        break
    }
  }
  delay(200);                       //增加时间延迟以避免程序卡顿
}
```

请上传程序，然后打开串口监视器，将波特率设置为115200。
接下来，我们可以使用以下连接方法将传感器连接到XIAO ESP32C3。

<div align="center"><img width = {700} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/62.jpg"/></div>

如果一切顺利，您将在串行监视器上看到数据消息。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/2.png" alt="pir" width="800" height="auto"/></div>

### Serial1 用法

根据上面的 XIAO ESP32C3 引脚图，我们可以看到有 TX 引脚和 RX 引脚。这与串口通信不同，但用法也非常相似，只是需要添加一些参数。接下来，我们将使用芯片引出的引脚进行串口通信。

核心功能需要包含：

- `Serial1.begin(BAUD,SERIAL_8N1,RX_PIN,TX_PIN);` — 启用 Serial1，函数原型： `<Serial.Type>.begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);`
  - `baud`  ：波特率
  - `config`：配置位
  - `rxPin` ：接收引脚
  - `txPin` ：发送引脚

值得一提的是，如果我们用数字引脚端口来定义，这个地方应该是 `#define RX_PIN D7`、`#define TX_PIN D6`，具体参数请参考不同 XIAO 系列的引脚图。

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

上传程序后，在 Arduino IDE 中打开串口监视器，并将波特率设置为 115200。然后，您可以通过串口监视器向 XIAO ESP32C3 发送内容，XIAO 将打印出您发送的每个字节。在这里，我输入的内容是 "Hello Everyone"，我的结果如下图所示：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/114.png" style={{width:600, height:'auto'}}/></div>


### 软件串口

要使用软件串口，需要安装 [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial) 库。

:::tip
目前推荐使用 EspSoftwareSerial 库的 7.0.0 版本。其他版本可能存在不同程度的问题，导致软件串口无法正常工作。
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
    Serial.print("Received via software serial: ");
    Serial.println(data);
  }

  if (Serial.available()) {
    char data = Serial.read();
    mySerial.print("Received via hardware serial: ");
    mySerial.println(data);
  }
}
```

此示例在引脚 `D7 (RX)` 和 `D6 (TX)` 上设置了 9600 波特率的软件串口。它监视硬件串口（USB）和软件串口之间的数据，并回显收到的数据。 

## I2C

### 硬件连接

将[Grove -OLED OLED 黄蓝双色显示屏 0.96 (SSD1315)](https://www.seeedstudio.com/Grove-OLED-Yellow-Blue-Display-0-96-SSD1315-V1-0-p-5010.html)连接到XIAO ESP32C3，硬件连接如下。

| Grove - OLED黄蓝显示屏0.96 (SSD1315)|  XIAO ESP32C3 |
|-----------|-----------|
| SCL       | SCL       |
| SDA       | SDA       |
| VCC       | 5V        |
| GND       | GND       |

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-7.png" alt="pir" width={1000} height="auto" /></div>

### 软件设置

- **步骤 1.** 打开Arduino IDE，导航到 `Sketch > Include Library > Manage Libraries…`

- **步骤  2.** 搜索**u8g2**并安装它

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE/u8g2-install.png" alt="pir" width={600} height="auto" /></p>

- **步骤  3.** 上传以下代码以在OLED显示器上显示文本字符串

```cpp
//#include <Arduino.h>
#include <U8g2lib.h>
 
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* 时钟引脚=*/ SCL, /* 数据引脚=*/ SDA, /* 重置引脚=*/ U8X8_PIN_NONE);    // 低速 I2C
 
void setup(void) {
  u8g2.begin();
//  u8x8.setFlipMode(1);   // 设置数值从 1 到 3，屏幕内容会旋转 180°
}
 
void loop(void) {
  u8g2.clearBuffer();                   // 清空内部缓冲区
  u8g2.setFont(u8g2_font_ncenB08_tr);   // 选择合适的字体
  u8g2.drawStr(0,15,"Hello World!");    // 将文本写入内部缓冲区
  u8g2.drawStr(0,30,"Hello World!");
  u8g2.drawStr(0,40,"Hello World!");
  u8g2.sendBuffer();                    // 将内部缓冲区的数据传输到显示屏
//  delay(1000);  
}
```

## SPI

### 硬件连接

将[Grove -高精度气压传感器(DPS310)](https://www.seeedstudio.com/Grove-High-Precision-Barometer-Sensor-DPS310-p-4397.html)连接到XIAO ESP32C3，硬件连接如下。

| Grove -高精度气压传感器(DPS310) | XIAO ESP32C3 |
|-----------|------------|
| 3V3        | 3V3       |
| SDI        | MOSI      |
| GND        | GND       |
| SDO        | MISO      |
| CSK        | SCK       |
| CS         | CS        |

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-4.png" alt="pir" width={1000} height="auto" /></div>

### 软件设置

- **步骤 1**. 下载[Seeed_Arduino_DPS310库](https://github.com/Seeed-Studio/Seeed_Arduino_DPS310)作为zip文件

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-5.png" alt="pir" width={1000} height="auto" /></div>

- **步骤 2**.打开Arduino IDE，导航到`Sketch > Include Library > Add .ZIP Library…`，并打开下载的zip文件

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE/add-zip.png" alt="pir" width={600} height="auto" /></p>

- **步骤 3.** 导航到` File > Examples > digitalpressuressensor > spi_background `打开**spi_background**示例

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-6.png" alt="pir" width={450} height="auto" /></div>

或者，您也可以从下面复制代码

```cpp
//#include <Dps310.h>

// Dps310 对象
Dps310 Dps310PressureSensor = Dps310();

void setup() {
    // 从设备选择引脚的引脚号
    // XMC2GO
    int16_t pin_cs = SS;
    // 对于 XMC 1100 Bootkit 和 XMC4700 Relax Kit，请取消注释以下行
    // int16_t pin_cs = 10;

    Serial.begin(9600);
    while (!Serial);

    // 调用 begin 初始化 Dps310
    // 参数 pin_nr 是微控制器上 CS 引脚的编号
    Dps310PressureSensor.begin(SPI, pin_cs);

    // 温度测量速率（值范围从 0 到 7）
    // 2^temp_mr 每秒温度测量结果
    int16_t temp_mr = 2;
    // 温度过采样率（值范围从 0 到 7）
    // 2^temp_osr 内部温度测量结果每次
    // 较高的值提高精度
    int16_t temp_osr = 2;
    // 压力测量速率（值范围从 0 到 7）
    // 2^prs_mr 每秒压力测量结果
    int16_t prs_mr = 2;
    // 压力过采样率（值范围从 0 到 7）
    // 2^prs_osr 内部压力测量结果每次
    // 较高的值提高精度
    int16_t prs_osr = 2;
    // startMeasureBothCont 启用后台模式
    // 温度和压力将自动测量
    // 高精度和高测量速率同时进行时不可用
    // 请参考数据手册（或通过试验）获取更多信息
    int16_t ret = Dps310PressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
    // 若只测量温度或压力，可以使用以下注释行之一
    // int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
    // int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);

    if (ret != 0) {
        Serial.print("初始化失败！ret = ");
        Serial.println(ret);
    } else {
        Serial.println("初始化完成！");
    }
}

void loop() {
    uint8_t pressureCount = 20;
    float pressure[pressureCount];
    uint8_t temperatureCount = 20;
    float temperature[temperatureCount];

    // 此函数将连续测量结果写入传递的数组
    // 参数 temperatureCount 和 pressureCount 应当保存 temperature 和 pressure 数组的大小
    // 在函数执行后，temperatureCount 和 pressureCount 将保存写入数组的值的数量
    // 注意：Dps310 不能保存超过 32 个结果。当其结果缓冲区已满时，它将不保存任何新的测量结果
    int16_t ret = Dps310PressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);

    if (ret != 0) {
        Serial.println();
        Serial.println();
        Serial.print("失败！ret = ");
        Serial.println(ret);
    } else {
        Serial.println();
        Serial.println();
        Serial.print(temperatureCount);
        Serial.println(" 个温度值：");
        for (int16_t i = 0; i < temperatureCount; i++) {
            Serial.print(temperature[i]);
            Serial.println(" 摄氏度");
        }

        Serial.println();
        Serial.print(pressureCount);
        Serial.println(" 个压力值：");
        for (int16_t i = 0; i < pressureCount; i++) {
            Serial.print(pressure[i]);
            Serial.println(" 帕斯卡");
        }
    }

    // 等待一段时间，以便 Dps310 可以重新填充其缓冲区
    delay(10000);
}
```

- **步骤 4.** 上传代码并打开“串口监视器

**注意:**上传代码后，需要点击Arduino窗口右上角的**Serial Monitor**才会自动执行。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pins-8.jpg" alt="pir" width={600} height="auto" /></div>

现在你会看到温度和压力数据一个接一个地显示在串行监视器上，如上所示!


## 关于XIAO ESP32C3 IO分配的说明
### D9

IAO ESP32C3的D9连接ESP32-C3的GPIO9(15)，上拉电阻(R6)和BOOT按钮。BOOT按钮(和RESET按钮)用于手动切换ESP32-C3的启动模式。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/10.png" alt="pir" width={600} height="auto" /></div>

按BOOT键将D9连接到GND。所以最好使用D9作为开关输入。

### D6

Seeeed ESP32C3的D6接ESP32-C3的U0TXD(28)。第一/第二阶段引导加载程序的运行状态作为文本输出到U0TXD。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/11.png" alt="pir" width={400} height="auto" /></div>

D6在启动时被设置为UART输出，因此如果您使用D6作为输入，可能会意外地产生大电流。**因此建议仅在输出模式下使用D6引脚。**

然而，由于这个D6是UART输出，您必须注意一些事情:一个是它在待机模式下不通信时为HIGH。另一个是第一/第二阶段引导加载程序的文本输出。信号在启动后立即振荡HIGH/LOW，必要时必须抵消。

所以尽量不要使用D6。(当然，在你理解它之后使用它是可以的。)

### D8
seed Studio XIAO ESP32C3的D8连接ESP32-C3的GPIO8(14)。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/12.png" alt="pir" width={300} height="auto" /></div>

当启动模式设置为长按boot键下载启动时，引用GPIO8，此时必须为HIGH。[这里](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf):“GPIO8 = 0和GPIO9 = 0的捆绑组合无效，将触发意外行为。”

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/13.png" alt="pir" width={700} height="auto" /></div>

如果你使用下载引导，**添加一个上拉电阻使GPIO8在引导时为HIGH**。

特别感谢SeeedJP的同事**李新平**对本节进行了测试和贡献。这里是原文的参考链接。

- [Seeed Studio XIAO ESP32C3的I/O分配注意](https://lab.seeed.co.jp/entry/2023/04/03/120000)

