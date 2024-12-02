---
title: 开始使用 Seeed Studio XIAO MG24(Sense)
description: |
image: https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/top.jpg
slug: /cn/xiao_mg24_getting_started
keywords:
  - XIAO
  - MG24
last_update:
  date: 11/19/2024
  author: Agnes
sidebar_position: 0
---

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/top.jpg" style={{width:600, height:'auto'}}/></div>
<br />


# 开始使用 Seeed Studio XIAO MG24(Sense)

<div class="table-center">
	<table align="center">
		<tr>
			<th>Seeed Studio XIAO MG24</th>
			<th>Seeed Studio XIAO MG24 Sense</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop0.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/shop.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-MG24-p-6247.html">
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
</div>

## 简介

**Seeed Studio XIAO MG24** 是一款基于 Silicon Labs 的 MG24 的迷你开发板。XIAO MG24 基于 **ARM Cortex-M33 核心**，采用 **32位 RISC** 架构，最大时钟速度为 78MHz，支持 DSP 指令和 FPU 浮动点运算，具有强大的计算能力，并内置 **AL/ML 硬件加速器 MVP**，可以高效处理 AI/机器学习算法。其次，它具有优异的 RF 性能，传输 **功率最高可达 +19.5 dBm**，接收灵敏度低至 **-105.4 dBm**。它支持多种物联网和无线传输协议，如 **Matter、Thread、Zigbee、Bluetooth LE 5.3、Bluetooth mesh** 等。

### 规格

<table align="center">
	<tr>
	    <th>项目</th>
	    <th>Seeed Studio XIAO MG24</th>
        <th>Seeed Studio XIAO MG24 Sense</th>
	</tr>
	<tr>
	    <th>处理器</th>
	    <td align="center" colspan="2">Silicon Labs EFR32MG24 <br></br>ARM Cortex-M33 @ 78MHz </td>
	</tr>
	<tr>
	    <th>无线</th>
	    <td align="center" colspan="2">完整的 2.4GHz Wi-Fi 子系统 <br></br> BLE: 蓝牙 5.3, 蓝牙 Mesh</td>
	</tr>
    <tr>
	    <th>内置传感器</th>
	    <td align="center"> - </td>
        <td align="center">6轴 IMU(LSM6DS3TR-C) <br></br>模拟麦克风(MSM381ACT001)</td>
	</tr>
    <tr>
	    <th>内存</th>
	    <td align="center">256kB RAM & 1538KB + 4MB Flash</td>
        <td align="center">256kB RAM & 1538KB + 4MB Flash</td>
	</tr>
    <tr>
	    <th>接口</th>
	    <td>2x UART, 2x IIC, 2x SPI, 18x 模拟, 18x 数字, 全部为 PWM, 1x 用户 LED, 1x 充电 LED <br></br> 1x 重置按钮, </td>
	    <td>2x UART, 2x IIC, 2x SPI, 18x 模拟, 18x 数字, 全部为 PWM, 1x 用户 LED, 1x 充电 LED <br></br> 1x 重置按钮, </td>
	</tr>
    <tr>
	    <th>尺寸</th>
	    <td align="center">21 x 17.8mm</td>
        <td align="center">21 x 17.8mm</td>
	</tr>
    <tr>
	    <th rowspan="2">电源</th>
	    <td colspan="2" align="center">输入电压 (Type-C): 5V@14mA <br></br> 输入电压 (BAT): 3.7V@7mA</td>
	</tr>
    <tr>
	    <td align="center">充电电池电流: <strong>200mA</strong></td>
		<td align="center">充电电池电流: <strong>200mA</strong></td>
	</tr>
    <tr>
        <th>低功耗模式</th>
        <td>常规模式: <strong>3.7V/6.71 mA</strong> <br></br> 睡眠模式: <strong>3.7V/1.91mA</strong> <br></br> 深度睡眠模式: <strong>3.7V/4.66μA</strong></td>
        <td>常规模式: <strong>3.7V/6.71 mA</strong> <br></br> 睡眠模式: <strong>3.7V/1.91mA</strong> <br></br> 深度睡眠模式: <strong>3.7V/4.66μA</strong></td>
    </tr>
    <tr>
        <th>工作温度</th>
        <td colspan="2" align="center">-20°C ~ 70°C</td>
    </tr>
</table>

## 特点

- **强大的 CPU**：ARM Cortex-M33 核心，最大时钟速度 78MHz，支持 DSP 指令和 FPU 浮动点运算，32 位 RISC 架构。
- **超低功耗**：接收电流 4.6mA / 发送电流 5mA (0dBm)，多种低功耗睡眠模式。
- **强大的 AI**：内置 AI/ML 硬件加速器 MVP，能够高效处理 AI/机器学习算法。
- **多样化的无线传输**：集成 2.4GHz 多协议无线收发器，支持 Matter、OpenThread、Zigbee、Bluetooth LE 5.3、Bluetooth mesh 等多种物联网协议。
- **更好的 RF 性能**：优异的 RF 性能，传输功率最高可达 +19.5 dBm，接收灵敏度低至 -105.4 dBm (250kbps DSSS)。
- **强大的安全性**：Secure Vault 提供强大的安全功能，包括安全启动、加密、随机数生成、防篡改、安全调试等。
- **超小尺寸**：
- **丰富的片上资源**：最大 1536KB Flash 和 256KB RAM，提供充足的存储空间。
- **丰富的接口**：集成 12 位 1Msps ADC、温度传感器、模拟比较器、DCDC 和其他丰富外设，最多可支持 22 个引脚、2 个 USART、2 个低功耗 UART、2 个 I2C 等接口。




## 硬件概述

<table align="center">
	<tr>
	    <th>XIAO MG24 Sense 指示图</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/aaa.png" style={{width:700, height:'auto'}}/></div></td>
	</tr>
  	<tr>
	    <th>XIAO MG24 指示图</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/bbb.png" style={{width:700, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <th>XIAO MG24 / XIAO MG24(Sense) 引脚列表</th>
	</tr>
    <tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/ccc.png" style={{width:1000, height:'auto'}}/></div></td>
	</tr>
</table>

:::tip
  两款开发板的区别在于：MG24 Sense 配有麦克风传感器和六轴加速度传感器，而 MG24 不具备这些传感器。
:::

- 5V - 这是来自 USB 端口的 5V 输出。你也可以将其用作电压输入，但必须在外部电源和此引脚之间放置某种二极管（肖特基二极管、信号二极管、电源二极管），阳极接电池，阴极接 5V 引脚。
- 3V3 - 这是来自板载稳压器的受控输出。
- GND - 电源/数据/信号地线。

## 快速入门

为了让你更快地开始使用 XIAO MG24，请阅读以下硬件和软件准备部分，准备好你的 XIAO。

### 出厂程序

我们会在每个新 XIAO MG24 和 XIAO MG24 Sense 上预装一个简单的出厂程序。

1. **XIAO MG24**

  常规版本的出厂程序是闪烁灯。当你为 XIAO 供电时，橙色的用户指示灯将会亮起。

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/00.gif" style={{width:500, height:'auto'}}/></div>

2. **XIAO MG24 Sense**

  常规版本的出厂程序是“你喊得越大，灯光越亮”。

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/99.gif" style={{width:500, height:'auto'}}/></div>

### 硬件准备

你需要准备以下物品：

- 1 x [Seeed Studio XIAO MG24](https://www.seeedstudio.com/Seeed-Studio-XIAO-MG24-p-6247.html)
- 1 x 计算机
- 1 x USB Type-C 数据线

:::tip
有些 USB 数据线只能提供电力，而不能传输数据。如果你没有 USB 数据线或不确定你的 USB 数据线是否支持数据传输，可以参考 [Seeed USB Type-C 支持 USB 3.1](https://www.seeedstudio.com/USB-3-1-Type-C-to-A-Cable-1-Meter-3-1A-p-4085.html)。
:::

### 软件准备

XIAO MG24 推荐使用 Arduino IDE 作为编程工具，因此你需要完成 Arduino 的安装，作为软件准备的一部分。

:::tip
如果这是你第一次使用 Arduino，我们强烈推荐你参考 [Arduino 入门指南](https://wiki.seeedstudio.com/Getting_Started_with_Arduino/)。
:::

- **步骤 1.** 根据你的操作系统下载并安装稳定版 Arduino IDE。

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载 Arduino IDE</font></span></strong>
    </a>
</div>

<br></br>

- **步骤 2.** 启动 Arduino 应用程序。
- **[步骤 3](#add-board).** 将 XIAO MG24 板载包添加到 Arduino IDE 中并点击 `OK`。
- **步骤 4.** 关闭 Arduino IDE，然后重新打开它。

#### 添加 XIAO MG24 开发板 {#add-board}

要安装 XIAO MG24 开发板，请按照以下步骤操作：

```
https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json 
```

1. 将上述开发板管理器的 URL 添加到 Arduino IDE 的偏好设置中。

<div style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/13(1).png" style={{width: 'auto', height: 'auto'}}/></div>

2. 下载 XIAO MG24 开发板包。

<div style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/14(1).png" style={{width: 'auto', height: 'auto'}}/></div>

:::tip 
如果进入后找不到它，请重新打开 Arduino IDE。
:::
3. 选择 `XIAO_MG24` 版本。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/15.png" style={{width:1000, height:'auto'}}/></div>

现在，享受编程的乐趣吧 ✨。

## 运行你的第一个 Blink 程序

- **步骤 1.** 启动 Arduino 应用程序。

- **步骤 2.** 导航到 **文件 > 示例 > 01.Basics > Blink**，打开程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/12(1).png" style={{width:1000, height:'auto'}}/></div>

- **步骤 3.** 选择开发板型号为 **XIAO MG24**，并选择正确的端口号上传程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/16.png" style={{width:1000, height:'auto'}}/></div>

程序上传成功后，你将看到以下输出信息，并且你可以观察到 XIAO MG24 右侧的橙色 LED 正在闪烁。

<div class="table-center">
	<table align="center">
		<tr>
			<th>MG24 Blink 代码</th>
			<th>LED 闪烁显示</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/7.png" style={{width:700, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/8.gif" style={{width:400, height:'auto'}}/></div></td>
		</tr>
	</table>
</div>

## 电池使用

XIAO MG24 可以使用 3.7V 锂电池作为电源输入。你可以参考以下电路图进行接线。

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/4444.png" alt="pir" width="800" height="auto"/></div>

:::caution
焊接时请小心不要短路正负极，以免烧毁电池和设备。
:::

**电池使用说明：**

1. 请使用符合规格的合格电池。
2. 使用电池时，XIAO 可以通过数据线连接到计算机设备，放心使用，XIAO 内置有电路保护芯片，确保安全。
3. 当 XIAO MG24 使用电池供电时，LED 灯不会亮起（除非你编写了特定程序），请不要通过 LED 的状态判断 XIAO MG24 是否工作，而是根据你编写的程序合理判断。
4. 抱歉，我们目前无法通过软件帮助你检查剩余电池电量（因为没有更多的芯片引脚可用），你需要定期给电池充电，或者使用万用表检查电池电量。

## 电压测试
### 软件代码
```cpp
/*
  AnalogReadSerial

  读取引脚 0 上的模拟输入，将结果打印到串口监视器。
  使用串口绘图工具 (工具 > 串口绘图器) 可查看图形表示。
  将电位器的中间引脚连接到引脚 A0，外侧引脚连接到 +5V 和地线。

  该示例代码属于公共领域。

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

// 设置程序，按下重置按钮时运行一次：
void setup() {
  Serial.begin(115200);
  pinMode(PD3, OUTPUT);
  digitalWrite(PD3, HIGH);
}

void loop() {
  int voltageValue = analogRead(PD4);
  float voltage = voltageValue * (5.0 / 4095.0);
  
  Serial.print("电压: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  delay(1000);  // 读取间隔延时，确保稳定性
}
```
### 显示结果

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/55.png" alt="pir" width="800" height="auto"/></div>

## 深度睡眠和睡眠模式示例

#### 示例1：睡眠模式与唤醒

```cpp

/*
   ArduinoLowPower 定时睡眠示例

   本示例展示了 Arduino Low Power 库的基本用法，通过让设备进入睡眠模式一段时间。
   设备将进入睡眠模式 2000ms。在睡眠期间，CPU 会停止工作，但 RAM 保留其内容。

   该示例与所有 Silicon Labs Arduino 开发板兼容。

   作者: Tamas Jozsi (Silicon Labs)
 */

#include "ArduinoLowPower.h"

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  Serial.println("定时唤醒睡眠模式");
}

void loop()
{
  digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
  delay(500);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  delay(500);

  Serial.printf("在 %lu 时进入睡眠\n", millis());
  LowPower.sleep(2000);
  Serial.printf("在 %lu 时醒来\n", millis());
}
```

#### 示例2：深度睡眠模式与唤醒

```cpp
/*
   ArduinoLowPower 深度睡眠示例，支持外部或定时唤醒

   本示例展示了 Arduino Low Power 库的基本用法，通过让设备进入深度睡眠模式。
   设备将保持深度睡眠，直到睡眠计时器到期。
   在深度睡眠期间，除了一些最小的外设（如备份 RAM 和 RTC）外，设备的电源全部关闭。
   这意味着 CPU 停止工作，RAM 内容会丢失——设备在唤醒后将从程序的开始部分重新启动。

   该示例与所有 Silicon Labs Arduino 开发板兼容。

   作者: Tamas Jozsi (Silicon Labs)
 */

#include "ArduinoLowPower.h"

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  Serial.println("深度睡眠定时唤醒");
}

void loop()
{
  digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
  delay(500);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  delay(500);

  Serial.printf("将在 %lu 时进入深度睡眠 10 秒\n", millis());
  LowPower.deepSleep(10000);
}



```

#### 示例3：深度睡眠模式与闪光灯及唤醒

:::tip
如果你想让闪光灯进入深度睡眠模式，你需要启用 0xb9 寄存器。
:::

```cpp
/*
   ArduinoLowPower 深度睡眠示例，支持外部或定时唤醒

   本示例展示了 Arduino Low Power 库的基本用法，通过让设备进入深度睡眠模式。
   设备将保持深度睡眠，直到睡眠计时器到期。
   在深度睡眠期间，除了一些最小的外设（如备份 RAM 和 RTC）外，设备的电源全部关闭。
   这意味着 CPU 停止工作，RAM 内容会丢失——设备在唤醒后将从程序的开始部分重新启动。

   该示例与所有 Silicon Labs Arduino 开发板兼容。

   作者: Tamas Jozsi (Silicon Labs)
 */
#include <Arduino.h>
#include "ArduinoLowPower.h"

#define CS_PIN PA6
#define CLK_PIN PA3
#define MOSI_PIN PA5
#define MISO_PIN PA4

#define READ_DATA 0x03
#define WRITE_ENABLE 0x06
#define PAGE_PROGRAM 0x02
#define SECTOR_ERASE 0x20

void sendSPI(byte data) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(MOSI_PIN, data & 0x80);
    data <<= 1;
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(1);
  }
}

void writeEnable() {
  digitalWrite(CS_PIN, LOW);
  sendSPI(WRITE_ENABLE);
  digitalWrite(CS_PIN, HIGH);
}

void setup()
{
  //Serial.begin(115200);
  pinMode(PA7, OUTPUT);
  digitalWrite(PA7, LOW);

  pinMode(CS_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);


  //SW
  pinMode(PD3, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB0, OUTPUT);
  pinMode(PA6, OUTPUT);
  digitalWrite(PD3, LOW); //VBAT
  digitalWrite(PB5, LOW); //RF_SW
  digitalWrite(PB1, LOW); //IMU
  digitalWrite(PB0, LOW); //MIC
  digitalWrite(PA6, HIGH);  //FLASH

  //Serial.println("深度睡眠定时唤醒");
  writeEnable();
  digitalWrite(CS_PIN, LOW);
  sendSPI(0xB9);
  digitalWrite(CS_PIN, HIGH);
}

void loop()
{
  delay(12000);  
  digitalWrite(PA7, HIGH);
  delay(500);

  //Serial.printf("将在 %lu 时进入深度睡眠 10 秒\n", millis());
  LowPower.deepSleep(600000);
}

```
## 资源

### 对于 Seeed Studio XIAO MG24 Sense

- 📄 **[PDF]** [MG24 数据手册](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/mg24-group-datasheet.PDF)
- 📄 **[PDF]** [Seeed Studio XIAO MG24 Sense 原理图](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/XIAO_MG24_SCH.pdf)
- 📄 **[PDF]** [MG24 无线 SoC](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/efr32xg24_rm.pdf)
- 📄 **[Kicad]** [MG24 FootOut](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/XIAO_MG24_SMD.kicad_mod)

### 对于 Seeed Studio XIAO MG24
- 📄 **[PDF]** [MG24 数据手册](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/mg24-group-datasheet.PDF)
- 📄 **[PDF]** [Seeed Studio XIAO MG24 原理图](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/XIAO_MG24_SCH.pdf)
- 📄 **[PDF]** [MG24 无线 SoC](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/efr32xg24_rm.pdf)
- 📄 **[Kicad]** [MG24 FootOut](https://files.seeedstudio.com/wiki/XIAO_MG24/Getting_Start/XIAO_MG24_SMD.kicad_mod)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时能够顺畅无忧。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
