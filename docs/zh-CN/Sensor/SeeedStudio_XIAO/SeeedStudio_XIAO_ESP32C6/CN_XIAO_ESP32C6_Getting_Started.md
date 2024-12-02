---
description: Seeed Studio XIAO ESP32C6 入门.
title: Seeed Studio XIAO ESP32C6 入门
image: https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/2/-/2-113991254-seeedxiao-esp32c6-font.jpg
keywords:
- XIAO
- ESP32C6
- Seeeduino
slug: /cn/xiao_esp32c6_getting_started
toc_max_heading_level: 4
sidebar_position: 1
last_update:
  date: 11/15/2024
  author: Agnes
---

# Seeed Studio XIAO ESP32C6 入门

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

## 介绍

Seeed Studio XIAO ESP32C6 由高集成的 [ESP32-C6 SoC](https://www.espressif.com/en/products/socs/esp32-c6)驱动，内置 **两个 32 位 RISC-V 处理器**，其中一个为高性能（HP）处理器，主频可达到 **160 MHz**，另一个为低功耗（LP）32 位 RISC-V 处理器，主频可达到 20 MHz. 该芯片还配备 **512KB SRAM 和 4 MB Flash**，为编程提供更大的空间，带来了更多物联网控制场景的可能性。

XIAO ESP32C6 由于其增强的无线连接性， **原生支持 Matter**。其无线协议栈支持 **2.4 GHz WiFi 6, Bluetooth® 5.3, Zigbee, and Thread (802.15.4)**。作为首个兼容 Thread 的 XIAO 成员，它非常适合构建符合 Matter 标准的项目，从而实现智能家居设备之间的互操作性。

export const Highlight = ({children, color}) => (
  <span
    style={{
      backgroundColor: color,
      borderRadius: '2px',
      color: '#fff',
      padding: '0.2rem',
    }}>
    {children}
  </span>
);

## 规格对比

<table class="sp-table-c6">
    <thead>
        <tr>
            <th colspan="2">产品</th>
            <th><Highlight color="#92c52a">XIAO ESP32C6</Highlight></th>
            <th>XIAO ESP32C3</th>
            <th>XIAO ESP32S3</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <th colspan="2" rowspan="2">处理器</th>
            <td>Espressif ESP32-C6 SoC</td>
            <td>Espressif ESP32-C3 SoC</td>
            <td>Espressif ESP32-S3R8</td>
        </tr>
        <tr>
            <td>两个 32 位 RISC-V 处理器，高性能处理器最高可运行至 160 MHz，低功耗处理器最高可运行至 20 MHz</td>
            <td>单核 32 位 RISC-V 处理器，具有四级流水线，运行频率最高为 160 MHz</td>
            <td>Xtensa LX7 双核 32 位处理器，最高可运行至 240 MHz</td>
        </tr>
        <tr>
            <th colspan="2" rowspan="3">无线</th>
            <td>完整的 2.4GHz <strong>Wi-Fi 6</strong> subsystem</td>
            <td colspan="2">完整的 2.4GHz Wi-Fi subsystem</td>
        </tr>
        <tr>
            <td>BLE: Bluetooth 5.0, Bluetooth Mesh</td>
            <td>BLE: Bluetooth 5.0, Bluetooth Mesh</td>
            <td>BLE: Bluetooth 5.0, Bluetooth Mesh</td>
        </tr>
        <tr>
            <td><strong>Zigbee,Thread,IEEE 802.15.4</strong></td>
            <td>/</td>
            <td>/</td>
        </tr>
        <tr>
            <th colspan="2" rowspan="1" >片上内存</th>
            <td>512KB SRAM &amp; 4MB Flash</td>
            <td>400KB SRAM &amp; 4MB Flash</td>
            <td>8M PSRAM &amp; 8MB Flash</td>
        </tr>
        <tr>
            <th colspan="2" rowspan="2" >接口</th>
            <td>1x UART,1x LP_UART, 1x IIC, 1x LP_IIC, 1x SPI,11x GPIO(PWM), 7x ADC, 1xSDIO</td>
            <td>1x UART, 1x IIC, 1x SPI,11x GPIO(PWM), 4x ADC</td>
            <td>1x UART, 1x IIC, 1x IIS, 1x SPI,11x GPIO(PWM), 9x ADC, 1x User LED, 1x Charge LED</td>
        </tr>
        <tr>
            <td colspan="3">1x Reset 按钮, 1x Boot 按钮</td>
        </tr>
        <tr>
            <th colspan="2" rowspan="1">尺寸</th>
            <td colspan="3">21 x 17.8 mm</td>
        </tr>
        <tr>
            <th colspan="1" rowspan="3">电源</th>
            <th colspan="1">Input voltage</th>
            <td colspan="3">Type-C: 5V<br></br>BAT: 4.2V</td>
        </tr>
        <tr>
            <th>电源电压（待机）</th>
            <td colspan="2">USB:5V@9mA<br></br>BAT:3.8V@9mA</td>
            <td>Type-C: 5V@19mA<br></br>BAT: 3.8V@22mA</td>
        </tr>
        <tr>
            <th>电池充电电流</th>
            <td>100mA</td>
            <td>350mA</td>
            <td>100mA</td>
        </tr>
        <tr>
            <th colspan="1" rowspan="3">功耗模式（供电 3.8V）</th>
            <th>调制解调器睡眠模式</th>
            <td>~ 30 mA</td>
            <td>~ 24 mA</td>
            <td>~ 25 mA</td>
        </tr>
        <tr>
            <th> 浅睡眠模式</th>
            <td>~ 2.5 mA</td>
            <td>~ 3 mA</td>
            <td>~ 2 mA</td>
        </tr>
        <tr>
            <th> 深度睡眠模式</th>
            <td>~ 15 μA</td>
            <td>~ 44 μA</td>
            <td>~ 14 μA</td>
        </tr>
        <tr>
            <th colspan="2">工作温度</th>
            <td colspan="2">-40°C ~ 85°C</td>
            <td>-40°C ~ 65°C</td>
        </tr>
    </tbody>
</table>

## 特性

- **增强的连接性**: 集成 ***2.4*** GHz Wi-Fi 6（802.11ax）、Bluetooth 5（LE）和 IEEE 802.15.4 无线连接，支持 **Thread** 和 **Zigbee** 协议。
- **原生支持 Matter**: 支持构建符合 Matter 标准的智能家居项目，确保不同智能设备之间的互操作性。
- **芯片级安全加密**: 利用 ESP32-C6 提供安全启动、加密和受信执行环境（TEE）功能，增强智能家居项目的安全性。
- **卓越的 RF 性能**: 配备板载天线，提供高达 80m 的 BLE/Wi-Fi 范围，并提供外接 UFL 天线接口，确保可靠的连接性。
- **优化的功耗管理n**: 提供四种工作模式，包括深度睡眠模式，功耗低至 15 μA，同时支持锂电池充电管理。
- **双 RISC-V 处理器**: 集成两个 32 位 RISC-V 处理器，高性能处理器主频可达 160 MHz，低功耗处理器主频可达 *20 MHz*。
- **Classic XIAO Designs**: 保持 21 x 17.8mm 的指尖大小外形和单面安装设计，适用于空间有限的项目，如可穿戴设备。

## 硬件概览

<table align="center">
 <tr>
     <th>XIAO ESP32C6 指示图</th>
 </tr>
 <tr>
     <td><div style={{textAlign:'center'}}><img src="https://wdcdn.qpic.cn/MTY4ODg1Nzc0ODUwMjM3NA_556525_Slxs4ARdyuXRrJK-_1711096256?w=9854&h=3367&type=image/png" style={{width:700, height:'auto'}}/></div></td>
 </tr>
    <tr>
     <th>XIAO ESP32C6 引脚图</th>
 </tr>
    <tr>
     <td><div style={{textAlign:'center'}}><img src="https://wdcdn.qpic.cn/MTY4ODg1Nzc0ODUwMjM3NA_318648_dMoXitoaQiq2N3-a_1711678067?w=1486&h=1228" style={{width:1000, height:'auto'}}/></div></td>
 </tr>
</table>

:::tip
GPIO14 用于选择使用内置天线还是外接天线。首先，你需要将 GPIO3 设置为低电平，启用此功能。如果 GPIO14 设置为低电平，则使用内置天线；如果设置为高电平，则使用外接天线。默认情况下为低电平。若要设置为高电平，可以参考以下代码：

```cpp
void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); // 启用此功能
  delay(100);
  pinMode(14, OUTPUT); 
  digitalWrite(14, HIGH); // 使用外接天线
}
```

:::

## 入门

为了让您更快开始使用 XIAO ESP32C6，请阅读以下硬件和软件准备内容，帮助您准备好 XIAO。

### 硬件准备

您需要准备以下物品：

- 1 x [Seeed Studio XIAO ESP32C6](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html)
- 1 x 计算机
- 1 x USB Type-C 数据线

:::tip
某些 USB 数据线仅能提供电力，无法传输数据。如果您没有 USB 数据线，或者不确定您的 USB 数据线是否支持数据传输，您可以查看 [Seeed USB Type-C 支持 USB 3.1](https://www.seeedstudio.com/USB-3-1-Type-C-to-A-Cable-1-Meter-3-1A-p-4085.html).
:::

#### 焊接引脚

XIAO ESP32C6 默认不带引脚，您需要准备好引脚并将其焊接到 XIAO 对应的引脚上，以便您能连接扩展板或传感器。

由于 XIAO ESP32C6 的尺寸非常小，请在焊接引脚时小心，避免将不同的引脚焊接在一起，也不要将焊锡粘到屏蔽或其他组件上，否则可能会导致 XIAO 短路或无法正常工作，造成的后果由用户自行承担。

### BootLoader 模式

有时，我们使用了错误的程序导致 XIAO 看起来丢失了端口或无法正常工作，具体表现为：

- 连接到计算机后，无法找到 XIAO 的端口。
- 计算机连接成功，但端口号显示正常，上传程序失败。

遇到以上两种情况时，您可以尝试将 XIAO 置于 BootLoader 模式，这可以解决大多数设备无法识别或上传失败的问题。具体方法如下：

- **步骤 1**. 按住 XIAO ESP32C6 的 BOOT 按钮，保持按压。
- **步骤 2**. 保持按压 BOOT 按钮的同时，通过数据线连接计算机，连接完成后松开 BOOT 按钮。
- **步骤 3**. 上传 **Blink** 程序，检查 XIAO ESP32C6 是否正常运行。

### 重置

当程序运行异常时，您可以在开机时按一次 Reset 按钮，让 XIAO 重新执行上传的程序。

在上电时按住 BOOT 按钮，同时按一次 Reset 按钮，也可以进入 BootLoader 模式。

### 软件准备

推荐使用 Arduino IDE 作为 XIAO ESP32C6 的编程工具，因此您需要先完成 Arduino 安装作为软件准备的一部分。

:::tip
如果您是第一次使用 Arduino，强烈建议您参考 [Arduino 入门指南](https://wiki.seeedstudio.com/Getting_Started_with_Arduino/)。

并且 XIAO ESP32C6 的板载包需要至少 **2.0.8** 版本才能使用。

:::

- **步骤 1.** 根据您的操作系统下载并安装 Arduino IDE 稳定版本。

  <div class="download_arduino_container" style={{textAlign: 'center'}}>
      <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>Download Arduino IDE</font></span></strong>
      </a>
  </div>

  <br></br>
- **步骤 2.** 启动 Arduino 应用。
- **[步骤 3](#add-board).**  将 XIAO ESP32C6 的板载包添加到 Arduino IDE 中，并点击 `Ok`。
- **步骤 4.** 关闭 Arduino IDE 后重新打开。

#### 添加 XIAO-C6 开发板 {#add-board}

要安装 XIAO ESP32C6 开发板，请按照以下步骤操作：

```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

1. 将上述开发板管理器的 URL 添加到 Arduino IDE 的首选项中，具体方法可以参考 [Arduino ESP32安装说明](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html#installing-using-arduino-ide)。

<div style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/boards_url.png" style={{width: 'auto', height: 'auto'}}/></div>

2. 下载 XIAO ESP32C6 开发板包。

:::note
只有当 esp32 开发板版本大于 `3.0.0` 时，才能安装。
:::

<div style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/install_board.png" style={{width: 'auto', height: 'auto'}}/></div>

3. 选择 `XIAO_ESP32C6` 变体。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/select_xiao_c6.png" style={{width:1000, height:'auto'}}/></div>

现在可以开始编程了 ✨.

## 运行第一个 Blink 程序

- **步骤 1.** 启动 Arduino 应用。

- **步骤 2.** 导航至 **File > Examples > 01.Basics > Blink**，打开程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/11.png" style={{width:700, height:'auto'}}/></div>

- **步骤 3.** 选择开发板模型为 **XIAO ESP32C6**，并选择正确的端口号上传程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/upload_program.png" style={{width:1000, height:'auto'}}/></div>

程序上传成功后，您将看到以下输出消息，并且可以观察到 XIAO ESP32C6 右侧的橙色 LED 在闪烁。

<table>
 <tr>
     <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/XIAOC6_flash_firmware.png" style={{width:680, height:'auto'}}/></div></td>
     <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/XIAOC6-blink.gif" style={{width:400, height:'auto'}}/></div></td>
 </tr>
</table>

## 电池使用

XIAO ESP32C6 系列内置电源管理芯片，支持通过电池独立供电，或通过 USB 端口为电池充电。

为了将电池连接到 XIAO，建议使用合格的可充电 3.7V 锂电池。在焊接电池时，仔细区分正负极。负极应连接到靠近 USB 端口的一侧，正极应连接到远离 USB 端口的一侧。

:::caution
当使用电池供电时，5V 引脚不会输出电压。
:::

:::tip 红色指示灯

XIAO ESP32C6 配有红色指示灯用于电池充电，类似于 [XIAO ESP32S3](/xiao_esp32s3_getting_started/#battery-usage):

XIAO ESP32C6 红色指示灯的工作状态如下：

- 没有连接电池时：
  - 当 Type-C 数据线连接时，红灯亮起，并在 30 秒后熄灭。
- 连接电池且插入 Type-C 数据线进行充电时：
  - 红灯闪烁。
- 电池通过 Type-C 连接充满电时：
  - 红灯熄灭。
:::

## 读取电池电压

要监控 XIAO ESP32C6 的电池电压，类似于 [XIAO ESP32C3](/XIAO_ESP32C3_Getting_Started/#check-the-battery-voltage)，您需要在电路中焊接一个 200k 电阻，形成 1:2 的电压分压器。这种设置会将电压降低一半，从而可以通过 A0 模拟端口安全地监控电池电压。

### 示例代码

以下代码初始化 A0 端口的 ADC，平均 16 次读取值来计算电池电压，并根据电压分压器的 1:2 比例调整。

```cpp
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);         // 配置 A0 为 ADC 输入
}

void loop() {
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(A0); // 读取并累加 ADC 电压值
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // 根据 1:2 分压比例调整并转换为伏特
  Serial.println(Vbattf, 3);                  // 输出电压，精确到小数点后三位
  delay(1000);                                // 等待 1 秒
}
```

此代码从 ADC 获取 16 次测量值，并对其进行平均处理，然后根据电压分压器的 1:2 比例进行补偿，最终以伏特为单位输出电池电压，精确到三位小数。

## 深度睡眠模式与唤醒

XIAO ESP32C6 提供了完整的深度睡眠模式和唤醒功能。这里展示了 ESP 提供的两个常见示例。

### 示例1：外部唤醒的深度睡眠

该代码演示了如何使用外部触发器唤醒 ESP32，并如何将数据存储在 RTC 内存中，以便在重启后使用。

```cpp
/*
硬件连接
=======================
将按钮连接到 GPIO 0，使用 10K 欧姆电阻拉低

注意：
======
GPIO 引脚的位掩码，指定哪些 GPIO 引脚可用于唤醒。只有具备 RTC 功能的 GPIO 引脚可用。
对于不同的 SoC，相关的 GPIO 引脚为：
- ESP32: 0, 2, 4, 12-15, 25-27, 32-39
- ESP32-S2: 0-21
- ESP32-S3: 0-21
- ESP32-C6: 0-7
- ESP32-H2: 7-14
*/

#define BUTTON_PIN_BITMASK (1ULL << GPIO_NUM_0) // GPIO 0 位掩码，用于外部唤醒

RTC_DATA_ATTR int bootCount = 0;

/*
打印 ESP32 唤醒原因
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("通过外部信号 RTC_IO 唤醒"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("通过外部信号 RTC_CNTL 唤醒"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("通过定时器唤醒"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("通过触摸板唤醒"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("通过 ULP 程序唤醒"); break;
    default : Serial.printf("唤醒不是由深度睡眠引起的: %d\n",wakeup_reason); break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(1000); // 给串口监视器一些时间

  // 增加启动次数并在每次重启时打印
  ++bootCount;
  Serial.println("启动次数: " + String(bootCount));

  // 打印 ESP32 的唤醒原因
  print_wakeup_reason();

  /*
  首先配置唤醒源
  我们将 ESP32 设置为通过外部触发器唤醒。
  ESP32 支持两种唤醒方式：ext0 和 ext1。由于 ESP32C6 不支持 ext0，因此我们使用 ext1。
  */

  // 如果使用 ext1，可以使用以下代码：
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);

  // 进入深度睡眠
  Serial.println("进入深度睡眠");
  esp_deep_sleep_start();
  Serial.println("这条信息将不会被打印");
}

void loop(){
  // 这个部分不会被调用
}
```

### 示例2：使用定时器唤醒的深度睡眠

ESP32 提供了一种深度睡眠模式，用于有效节省电力，因为电源消耗是物联网应用中的一个重要因素。在该模式下，CPU、大部分 RAM 以及所有从 APB_CLK 时钟供电的数字外设都会关闭。只有 RTC 控制器、RTC 外设和 RTC 内存这几个部分能够保持供电。

以下代码演示了最基本的深度睡眠模式，并使用定时器来唤醒，以及如何将数据存储在 RTC 内存中以便在重启时使用。

```cpp
/*
简单的定时器唤醒深度睡眠
=====================================
此代码遵循公共领域许可证。

作者：
Pranav Cherukupalli <cherukupallip@gmail.com>
*/

#define uS_TO_S_FACTOR 1000000ULL  /* 微秒到秒的转换因子 */
#define TIME_TO_SLEEP  5        /* ESP32 进入睡眠的时间（秒） */

RTC_DATA_ATTR int bootCount = 0;

/*
方法：打印 ESP32 唤醒的原因
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("通过外部信号 RTC_IO 唤醒"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("通过外部信号 RTC_CNTL 唤醒"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("通过定时器唤醒"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("通过触摸板唤醒"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("通过 ULP 程序唤醒"); break;
    default : Serial.printf("唤醒不是由深度睡眠引起的: %d\n",wakeup_reason); break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(1000); // 等待串口监视器启动

  // 增加启动次数并在每次重启时打印
  ++bootCount;
  Serial.println("启动次数: " + String(bootCount));

  // 打印 ESP32 唤醒原因
  print_wakeup_reason();

  /*
  首先配置唤醒源
  我们将 ESP32 设置为每 5 秒唤醒一次
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("设置 ESP32 每 " + String(TIME_TO_SLEEP) + " 秒进入深度睡眠");

  /*
  接下来，我们决定关闭/保持哪些外设
  默认情况下，ESP32 会自动关闭不需要的外设，但如果你是一个高级用户，
  你可以进行更细致的控制。详细内容请参考 API 文档
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  下面的代码示例展示了如何关闭深度睡眠中的所有 RTC 外设。
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("配置所有 RTC 外设在睡眠中关闭");

  /*
  配置好唤醒原因后，如果需要设置外设状态，我们就可以开始进入
  深度睡眠了。
  如果没有提供唤醒源，深度睡眠将一直进行，直到硬件复位发生。
  */
  Serial.println("进入深度睡眠");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("这条信息不会被打印");
}

void loop(){
  // 这个部分不会被调用
}
```

:::tip
如果你想了解更多关于深度睡眠模式和唤醒功能的使用，可以在 Arduino IDE 中找到更多由 ESP 官方编写的示例程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/16.png" style={{width:600, height:'auto'}}/></div>
:::

## 资源

- **[PDF]** [ESP32C6 数据手册](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/res/esp32-c6_datasheet_en.pdf)

- **[ZIP]** [Seeed Studio XIAO ESP32C6 KiCAD 库](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/XIAO-ESP32-C6_v1.0_SCH&PCB_24028.zip)

- **[PDF]** [Seeed Studio XIAO ESP32C6 原理图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/XIAO-ESP32-C6_v1.0_SCH_PDF_24028.pdf)

- **[XLSX]** [Seeed Studio XIAO ESP32C6 pinout 引脚图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/res/XIAO_ESP32C6_Pinout.xlsx)

- **[ZIP]** [Seeed Studio XIAO ESP32C6 认证文件](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/res/SeeedStudio_XIAO_ESP32C6_Certification.zip)

## 课程资源

<div align="middle"><img width="400" src="https://mjrovai.github.io/XIAO_Big_Power_Small_Board-ebook/cover.jpg" /></div>

- **[电子书]** [XIAO: Big Power, Small Board Mastering Arduino and TinyML](https://mjrovai.github.io/XIAO_Big_Power_Small_Board-ebook/)

## 技术支持与产品讨论

感谢您选择我们的产品！我们将为您提供不同的支持，以确保您使用我们的产品时体验顺利。我们提供多种沟通渠道，满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
