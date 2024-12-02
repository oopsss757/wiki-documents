---
title: Seeed Studio XIAO RP2350 入门
description: |
  XIAO RP2350，来自 Seeed Studio 的尖端微控制器。它具有双核处理器、增强的 SRAM 和闪存以及增强的连接性。
image: https://files.seeedstudio.com/wiki/XIAO-RP2350/img/RP2350-thumbnail.png
slug: /cn/getting-started-xiao-rp2350
keywords:
  - xiao
  - RP2350
sidebar_position: 0
last_update:
  author: Agnes
  date: 11/18/2024
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Seeed Studio XIAO RP2350

<div style={{ textAlign: 'center' }}>
  <img 
    src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/2-102010550%20XIAO%20RP2350-45font.jpg" 
    style={{ width: 480, height: 'auto', "border-radius": '12.8px' }} 
  />
</div>

<div class="get_one_now_container" style={{textAlign: 'center'}}>
  <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-RP2350-p-5944.html?utm_source=seeed&utm_medium=wiki">
  <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
  </a>
</div><br></br>

XIAO RP2350 将 Raspberry Pi RP2350（双 Cortex-M33 核心，运行频率 150MHz，支持浮点运算单元 FPU，增强的安全性和加密功能）强大性能，融入了经典的 XIAO 设计中。它的尺寸仅为 21x17.8mm，拥有 19 个多功能 GPIO、RGB LED，以及一个具有超低功耗（27μA）的电池管理系统，支持电池供电和直接电池电压测量。得益于 XIAO 生态系统，XIAO RP2350 可以兼容多种扩展模块，包括显示器、LED 矩阵、Grove 模块、CAN 总线、视觉 AI 传感器和毫米波传感器等。它原生支持 MicroPython、C 和 C++，是各类开发者打造智能控制、可穿戴设备、DIY 键盘等紧凑型电池驱动应用的理想选择。

## 特点

- **强大的 MCU 板卡：** 配备 Raspberry Pi RP2350 芯片，具有双 Cortex-M33 核心，运行频率 150MHz，支持 FPU。
- **增强的安全功能：** 内置安全启动和加密引导加载程序，确保应用程序的安全性。
- **软件支持：** 支持 C/C++ 和 MicroPython，方便项目开发和原型设计。
- **丰富的板载资源：** 集成 RGB LED、2MB Flash、520kB SRAM 和 19 个多功能 GPIO（模拟、数字、I²C、UART、SPI、PWM）。
- **新增 8 个 IO 引脚：** 相比之前的 XIAO MCU，新增的 8 个 IO 引脚支持更复杂的应用。
- **高效的电源设计：** 睡眠模式下功耗仅为 27μA，支持电池供电。通过内置 IO 进行直接电池电压测量，增强电池管理系统（BMS）。
- **紧凑的拇指尺寸设计：** 尺寸为 21 x 17.8mm，采用 Seeed Studio 经典的 XIAO 外形，适用于空间受限的应用。
- **适合量产：** 表面贴装设计（SMD），所有组件位于板面，并且两侧有冲压孔，便于高效的大规模生产。

## 规格

<table align="center">
    <tr>
        <td>产品</td>
        <td>XIAO RP2040</td>
        <td><b>XIAO RP2350</b></td>
    </tr>
    <tr>
        <td rowspan="2">处理器</td>
        <td>Raspberry Pi RP2040</td>
        <td>Raspberry Pi RP2350</td>
    </tr>
    <tr>
        <td>双 Cortex-M0+ @ 133MHz</td>
        <td>双 Cortex-M33 @ 150MHz, FPU</td>
    </tr>
    <tr>
        <td>RAM</td>
        <td>264kB SRAM</td>
        <td>520kB SRAM</td>
    </tr>
    <tr>
        <td>Flash</td>
        <td>2MB 板载</td>
        <td>2MB PSRAM</td>
    </tr>
    <tr>
        <td>LED</td>
        <td>1 个用户 LED，1 个电源 LED，1 个 RGB LED<br></br>用于串口下载的两个 LED</td>
        <td>1 个用户 LED，1 个电源 LED，1 个 RGB LED</td>
    </tr>
    <tr>
        <td>接口</td>
        <td>11 引脚：4 个模拟引脚，11 个数字引脚，1 个 I²C，1 个 UART，1 个 SPI，所有引脚支持 PWM</td>
        <td><b>19 引脚：3 个模拟引脚，19 个数字引脚，2 个 I²C，2 个 UART，2 个 SPI，所有引脚支持 PWM</b></td>
    </tr>
    <tr>
        <td>按钮</td>
        <td align="center" colspan="2">1 个 RESET 按钮，1 个 BOOT 按钮</td>
    </tr>
    <tr>
        <td>安全性</td>
       <td align="center"> - </td>
        <td>OTP、Secure Boot、Arm TrustZone</td>
    </tr>
    <tr>
        <td>低功耗</td>
       <td align="center"> - </td>
        <td>27μA</td>
    </tr>
    <tr>
        <td>软件兼容性</td>
        <td>支持 Micropython/CircuitPython，Arduino</td>
        <td>支持 Micropython，C/C++</td>
    </tr>
    <tr>
        <td>工作温度</td>
        <td align="center" colspan="2">-20°C~70°C</td>
    </tr>
    <tr>
        <td>尺寸</td>
        <td align="center" colspan="2">21x17.8 mm</td>
    </tr>
</table>

## 硬件概览

<div class="table-center">
<table align="center">
	<tr>
	    <th>XIAO RP2350 正面引脚图</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/XIAO-RP2350-front.png" style={{width:680, height:'auto'}} alt="XIAO RP2350 正面引脚图" /></div></td>
	</tr>
    <tr>
	    <th>XIAO RP2350 背面引脚图</th>
	</tr>
    <tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/XIAO-RP2350-back.png" style={{width:680, height:'auto'}} alt="XIAO RP2350 背面引脚图" /></div></td>
	</tr>
    <tr>
	    <th>XIAO RP2350 组件分布</th>
	</tr>
    <tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/XIAO-RP2350-components.png" style={{width:480, height:'auto'}} alt="XIAO RP2350 组件分布" /></div></td>
	</tr>
</table>
</div>

需要更多引脚图信息？请跳转到下方的 [资源与资料](#assets--resources)。

## 支持的平台

XIAO RP2350 由 RP2350 提供支持，兼容 MicroPython 和 Raspberry Pi 提供的 C/C++ SDK。这种灵活性使开发者可以选择自己喜欢的编程语言和环境进行原型设计和开发。

<div class="table-center">
  <table align="center">
    <tr>
      <th>C/C++ SDK</th>
      <th>MicroPython</th>
    </tr>
    <tr>
      <td style={{ textAlign: 'center' }}>
        <img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/C%2B%2B-Logo.wine.png" alt="C/C++ Logo" width={200} height="auto" />
      </td>
      <td style={{ textAlign: 'center' }}>
        <img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/micropython/MicroPython-Logo.png" alt="MicroPython Logo" width={200} height="auto" />
      </td>
    </tr>
  </table>
</div>

## 入门指南▶️

:::info attention
本页面主要面向 MicroPython 用户。如果您有兴趣学习 SDK 编程，或者是高级用户，您可以从 [Raspberry Pi Pico 系列 C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) 开始。该指南将帮助您搭建环境并从示例代码开始。此外，您还可以访问 [XIAO RP2350 与 C/C++ SDK](/xiao-rp2350-c-cpp-sdk) 以获取更多关于 XIAO RP2350 的具体指导。
:::

:::note micropython is still not released
截至 2024 年 8 月 9 日，XIAO RP2350 的稳定版 MicroPython 固件仍在等待 Raspberry Pi 的发布。在此期间，您可以使用 [RPI_PICO2 | MicroPython.org](https://micropython.org/download/RPI_PICO2/) 提供的预览版 MicroPython 固件。

有关最新文档，您可以随时参考 [Raspberry Pi Python SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf)。
:::

### 第一步：在 XIAO RP2350 上安装 MicroPython

要在 XIAO RP2350 上安装 MicroPython 固件，请按照以下步骤操作：

**步骤 1.1 下载 MicroPython 固件：**  
   - 前往 [MicroPython 下载页面](https://micropython.org/download/RPI_PICO2/)。
   - 下载与 XIAO RP2350 兼容的最新 `.uf2` 固件文件。

**步骤 1.2 进入 BOOTSEL 模式：**  

您可以通过以下两种方法之一让 XIAO RP2350 进入 BOOTSEL 模式：

<Tabs>
<TabItem value="method1" label="方法一：连接电脑前" default>

1. **按住 BOOT 按钮：**  
   在 XIAO RP2350 未连接到电脑时，**按住** BOOT 按钮。
2. **连接到电脑：**  
   按住 BOOT 按钮的同时，通过 USB 线将 XIAO RP2350 连接到电脑。
3. **松开 BOOT 按钮：**  
   板子连接到电脑后，可以松开 BOOT 按钮。此时，XIAO RP2350 应进入 BOOTSEL 模式，电脑会将其识别为一个可移动存储设备。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-no-charge.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/>
<div style={{ marginTop: '-8px' }}><em>Hold Boot-> Plug in Cable-> Release Boot</em></div>
</div>

</TabItem>

<TabItem value="method2" label="Method 2: While Connected to Computer">

1. **按住 BOOT 按钮：**  
   在 XIAO RP2350 已连接到电脑的情况下，按住 BOOT 按钮。
2. **按下 RESET 按钮：**  
   按住 BOOT 按钮的同时，按下并释放 RESET 按钮（在上图中，位于电路板右下角标记为 “B” 的按钮）。
3. **松开 BOOT 按钮：**  
   按下 RESET 按钮后，松开 BOOT 按钮。此时，XIAO RP2350 应进入 BOOTSEL 模式，电脑会将其识别为一个可移动存储设备。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-charged.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/>
<div style={{ marginTop: '-8px' }}><em>按住 BOOT -> 点击 RESET -> 松开 BOOT</em></div>
</div>

</TabItem>
</Tabs>

**步骤 1.3 安装固件：**  
   - **拖放**下载的 `.uf2` 文件到 XIAO RP2350 的可移动存储驱动器中。
   - 文件复制完成后，开发板将自动重启，完成固件安装。

### 第二步：安装 Thonny IDE

:::tip 关于 MicroPython

[MicroPython](https://micropython.org/) 是一种类似于 [Python](https://www.python.org/) 的解释性语言。然而，与 Python 不同，MicroPython 可直接运行在硬件上（裸机），提供一个交互式提示符（REPL）来立即执行命令，还可以运行和导入存储在内置文件系统中的脚本。

要连接到 XIAO RP2350 开发板并开始编写和运行 Python 代码，您可以使用任何支持串行连接的终端工具，如 minicom、PuTTY、electerm、warp 等。如果想获得更加*用户友好的体验*，可以使用 **[Thonny](https://thonny.org/)**，因为它易于使用，功能集成且界面友好，特别适合初学者。这种方式可以让您直接在设备上编写和运行 Python 代码。

:::

Thonny IDE 是一个非常适合 MicroPython 开发的初学者友好型 Python 编辑器。以下是安装步骤：

1. **下载 Thonny：**  
   - 访问 [Thonny 下载页面](https://thonny.org/)。
   - 根据您的操作系统（Windows、macOS 或 Linux）选择合适的安装程序并下载。

2. **安装 Thonny：**  
   - **运行**下载的安装程序。
   - **按照**屏幕上的指引完成安装过程。

3. **配置 Thonny 以支持 MicroPython：**  
   - **打开** Thonny IDE。
   - 查看 Thonny 窗口右下角。
   - 点击 **解释器** 选择区域。
   - 从下拉菜单中选择 **'MicroPython (RP2040)'**。
   - 确保选择正确的 **端口** —— Thonny 通常会自动检测到。

现在，您可以使用 Thonny IDE 开始为 XIAO RP2350 编写和上传 MicroPython 代码了！

<Tabs>
  <TabItem value="thonny-mpy" label="Thonny IDE" default>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/thonny-mpy.png" style={{width:680, height:'auto'}}/></div>

  </TabItem>
  <TabItem value="putty-mpy" label="PuTTY 控制台">

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/putty-mpy.png" style={{width:680, height:'auto'}}/></div>

  </TabItem>
</Tabs>

如果您的设备已经准备好运行 MicroPython，让我们从一个简单的项目开始：

### 点亮一个 LED ✨

让开发板闪烁一个 LED 通常是大家运行的第一个程序。XIAO RP2350 也不例外。

:::note
根据电路图，XIAO RP2350 上的 `用户 LED`（黄色 LED）连接到 `GPIO25/D19`。  
对于所有 XIAO 系列开发板，`用户 LED` 设置为 **低电平** 时会 **点亮**，设置为 **高电平** 时会 **熄灭**。
:::

<Tabs>
  <TabItem value="blink" label="闪烁 LED" default>

```python showLineNumbers
from machine import Pin # 从 machine 模块中导入 Pin 类
from time import sleep  # 从 time 模块中导入 sleep 函数

# 初始化 GPIO25 为输出引脚，控制用户 LED
led = Pin(25, Pin.OUT) 

# 初始化时关闭 LED
led.value(1) # led.on() -> 高电平 -> 灯灭
sleep(0.5) # 等待 0.5 秒

# 打开 LED
led.value(0) # led.off() -> 低电平 -> 灯亮
sleep(0.5) # 等待 0.5 秒

# 进入无限循环
while True:
    # 切换 LED 状态（从亮到灭或从灭到亮）
    led.toggle() 
    # 打印当前 LED 的状态
    print(f"LED {'亮' if led.value() == 0 else '灭'}")
    sleep(0.5) # 等待 0.5 秒后切换
    ```

<table>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/tonny-blink-led.png" style={{width:680, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/rp2350-blink.gif" style={{width:400, height:'auto'}}/></div></td>
	</tr>
</table>

  </TabItem>
  <TabItem value="pwm" label="Fading a LED" default>

```python title="examples/rp2/pwm_fade.py"
# 使用 PWM 让 LED 实现渐亮和渐暗的示例。
# 注意：此示例不适用于 Pico W，因为其使用 Pin(25) 控制 LED 输出。
# 此示例代码来源：https://github.com/micropython/micropython
# ruff: noqa: F821 - @asm_pio 装饰器会将名称添加到函数作用域中

import time
from machine import Pin, PWM

# 创建 PWM 对象，将 LED 连接到 Pin(25)。
pwm = PWM(Pin(25))

# 设置 PWM 频率。
pwm.freq(1000)

# 让 LED 多次实现渐亮和渐暗效果。
duty = 0
direction = 1
for _ in range(8 * 256):
    duty += direction
    if duty > 255:
        duty = 255
        direction = -1
    elif duty < 0:
        duty = 0
        direction = 1
    pwm.duty_u16(duty * duty)
    time.sleep(0.001)
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/rp2350-mpy-fade-led.gif" style={{width:240, height:'auto', "border-radius": '12.8px'}}/></div>

  </TabItem>
</Tabs>

一旦你将代码复制到 Thonny IDE 中，如下图所示，只需点击 `运行当前脚本` 按钮或按下 `F5` 键。此操作将执行代码片段，你会看到 XIAO RP2350 上的 LED 开始闪烁。

### 玩转 RGB LED

XIAO RP2350 配备了内置 RGB LED，您可以通过 MicroPython 控制它。以下是一个切换不同颜色的示例：

```python
import array, time, random
from machine import Pin
import rp2

NUM_LEDS = 1
LED_PIN = 22  # 默认 WS2812 LED 数据引脚
POWER_PIN = 23  # 默认 WS2812 LED 电源引脚

# 全局亮度变量 (0.0 到 1.0)
BRIGHTNESS = 0.1

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

# 设置电源引脚
power_pin = Pin(POWER_PIN, Pin.OUT)
power_pin.value(1)  # 打开 LED 电源

# 使用 ws2812 程序创建状态机，输出到 LED_PIN
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(LED_PIN))

# 启动状态机，它将等待 FIFO 数据。
sm.active(1)

def set_led_color(color):
    sm.put(array.array("I", [color]), 8)

def random_color():
    return random.randint(0, 255) | (random.randint(0, 255) << 8) | (random.randint(0, 255) << 16)

def interpolate(color1, color2, factor):
    r1, g1, b1 = color1 & 255, (color1 >> 8) & 255, (color1 >> 16) & 255
    r2, g2, b2 = color2 & 255, (color2 >> 8) & 255, (color2 >> 16) & 255
    r = int(r1 + factor * (r2 - r1))
    g = int(g1 + factor * (g2 - g1))
    b = int(b1 + factor * (b2 - b1))
    return (b << 16) | (g << 8) | r

def apply_brightness(color, brightness):
    r, g, b = color & 255, (color >> 8) & 255, (color >> 16) & 255
    r = int(r * brightness)
    g = int(g * brightness)
    b = int(b * brightness)
    return (b << 16) | (g << 8) | r

print("开始随机颜色渐变，并可调节亮度...")

# 主循环
current_color = random_color()
while True:
    next_color = random_color()
    for i in range(100):  # 100 步实现平滑过渡
        transition_color = interpolate(current_color, next_color, i / 100)
        final_color = apply_brightness(transition_color, BRIGHTNESS)
        set_led_color(final_color)
        time.sleep_ms(20)  # 调整此值改变渐变速度
    current_color = next_color

    # 可选：您可以在此调整亮度以进行演示
    # BRIGHTNESS = random.random()  # 这会在每个循环中随机设置亮度
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/rp2350-mpy-rgb-led.gif" style={{width:240, height:'auto', "border-radius": '12.8px'}}/></div>

### 电池与电源管理

是否可以在不使用额外组件的情况下读取电池电压？答案是肯定的。使用 XIAO RP2350，这比以往更加简单。在之前的 XIAO 系列产品中（如 [XIAO ESP32C3](/XIAO_ESP32C3_Getting_Started/#check-the-battery-voltage)），读取电池电压需要通过电阻手动连接到 *A0*。

而在 XIAO RP2350 中，这一过程被简化了。现在您可以直接使用 `A3/GPIO29` 引脚读取电池电压水平，从而简化您的设计和开发。只需记住将 `GPIO19` 引脚设置为高电平，这是启用电池电压读取所必需的。

按照以下代码示例，使用 Pico SDK 读取电池电压：

<Tabs>
  <TabItem value="python" label="MicroPython" default>

```python
from machine import Pin, ADC
import time

# 初始化 GPIO 引脚以启用电池电压读取
def init_gpio():
    enable_pin = Pin(19, Pin.OUT)
    enable_pin.value(1)  # 将引脚设置为高电平以启用电池电压读取

def main():
    print("ADC 电池示例 - GPIO29 (A3)")
    
    init_gpio()  # 初始化启用引脚
    adc = ADC(Pin(29))  # 初始化 GPIO29 上的 ADC

    conversion_factor = 3.3 / (1 << 12)  # 12 位 ADC 和 3.3V 参考电压的转换因子
    
    while True:
        result = adc.read_u16()  # 读取 ADC 值
        voltage = result * conversion_factor * 2  # 计算电压，考虑到电压分压器（系数为 2）
        print("原始值: 0x{:03x}, 电压: {:.2f} V".format(result, voltage))
        time.sleep(0.5)  # 延迟 500 毫秒

if __name__ == '__main__':
    main()
```

  </TabItem>
  <TabItem value="sdk" label="C/C++ SDK">

```c title='adc_bat.c'
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// 初始化 GPIO 引脚以启用电池电压读取
void init_gpio() {
    const int enable_pin = 19; // 用于启用电池电压读取的引脚

    gpio_init(enable_pin); // 初始化引脚
    gpio_set_dir(enable_pin, GPIO_OUT); // 设置引脚为输出
    gpio_put(enable_pin, 1); // 将引脚设置为高电平以启用电池电压读取
}

int main() {
    stdio_init_all(); // 初始化标准输入/输出
    printf("ADC 电池示例 - GPIO29 (A3)\n");

    init_gpio(); // 初始化启用引脚
    adc_init(); // 初始化 ADC

    // 初始化 ADC GPIO 引脚（GPIO29）
    adc_gpio_init(29);
    // 选择 ADC 输入 3（对应 GPIO29）
    adc_select_input(3);

    while (1) {
        // 12 位转换，假设最大值 == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12); // 12 位 ADC 和 3.3V 参考电压的转换因子
        uint16_t result = adc_read(); // 读取 ADC 值
        // 计算电压，考虑到电压分压器（系数为 2）
        printf("原始值: 0x%03x, 电压: %f V\n", result, result * conversion_factor * 2); 
        sleep_ms(500); // 延迟 500 毫秒
    }
}
```

  </TabItem>
</Tabs>

## 资源与参考资料

XIAO RP2350 借助 Raspberry Pi RP2350 的强大功能，充分利用了 Raspberry Pi 社区丰富的共享资源。这为您在这个小巧的开发板上创造无限创意打开了大门。以下是帮助您快速入门的重要资源与参考资料。

***数据手册与原理图***

- 📄 **[PDF]** [RP2350 数据手册](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- 📄 **[PDF]** [Seeed Studio XIAO RP2350 原理图](https://files.seeedstudio.com/wiki/XIAO-RP2350/res/Seeed-Studio-XIAO-RP2350-v1.0.pdf)
- 📄 **[XLSX]** [Seeed Studio XIAO RP2350 引脚图](https://files.seeedstudio.com/wiki/XIAO-RP2350/res/XIAO-RP2350-pinout-sheet.xlsx)
- 📄 **[DXF]** [Seeed Studio XIAO RP2350 尺寸图 (DXF)](https://files.seeedstudio.com/wiki/XIAO-RP2350/res/XIAO-RP2350-dimension-v1.0.dxf)
- 🔗 **[链接]** [Seeed Studio XIAO RP2350 3D STEP 文件](https://grabcad.com/library/seeed-studio-xiao-rp2350-1)
- 📄 **[ZIP]** [Seeed Studio XIAO RP2350 v1.0 SCH & PCB 文件](https://files.seeedstudio.com/wiki/XIAO-RP2350/res/XIAO_RP2350_v1.0_SCH&PCB_240626.zip)

***相关资源***

- 📄 **[PDF]** [Raspberry Pi Pico 系列入门指南](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf): 为初学者提供设置与编程 Raspberry Pi Pico 板的完整指南，适用于学习 MicroPython 或 C/C++。
- 📄 **[PDF]** [Raspberry Pi Pico 系列 Python SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf)：记录了 MicroPython 设置教程和 API 的文档。
- 📄 **[PDF]** [Raspberry Pi Pico 系列 C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)：记录 Pico C/C++ SDK API 的文档。

### 拓展与应用

[XIAO 系列](/xiao_topic_page) 拥有丰富的外设和外围配件，供您学习和使用。无论是支持完美交互的彩色屏幕，还是配备简单明亮 RGB 灯的集成板，都在等待您的探索。

作为 XIAO 家族的一员，XIAO RP2350 同样延续了这一特点。当然，为了更好地利用扩展引脚，新的*外设与扩展板*将陆续推出，充分发挥其性能。

- 🌟 **[扩展配件](/SeeedStudio_XIAO_Series_Introduction/#seeed-studio-xiao-series-compatible-accessories)**  
  探索 XIAO 系列兼容的各种附加模块和配件，从显示屏和 LED 点阵到 Grove 模块和传感器，学习如何利用它们提升您的项目。

### 社区与学习

此外，深入活跃的 Raspberry Pi 社区，拓展您的知识，发现新的项目创意。通过社区共享资源、论坛和教程，增强您对 XIAO RP2350 的使用体验。除了 Seeed Studio 的 Wiki 以外，我们还推荐以下学习资源：

- **[Raspberry Pi 文档](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)**：获取有关 RP2350 的可靠且最新的信息。
- **[Raspberry Pi 论坛](https://www.raspberrypi.org/forums/)**：与其他爱好者互动，提出问题并分享项目。
- **[XIAO GitHub 仓库](https://github.com/Seeed-Studio/OSHW-XIAO-Series)**：探索官方 XIAO 仓库，获取更全面的文档，与我们的团队互动，**欢迎加入！**
- **[Reddit 的嵌入式社区](https://www.reddit.com/r/embedded/)**：加入嵌入式系统社区，分享见解并讨论各种话题。
- **[GitHub 的 Pico 主题](https://github.com/topics/pico)**：探索与 Pico 相关的代码仓库和讨论。
- **[Hackster.io](https://www.hackster.io/)**：发现与 XIAO 和 Raspberry Pi 相关的项目与教程。
- **[Instructables](https://www.instructables.com/)**：查找 DIY 项目和逐步指南，创建 XIAO 及其他硬件相关的作品。
- **[Element14 社区](https://www.element14.com/community/)**：参与有关电子与嵌入式系统的讨论、网络研讨会和项目。

此外，欢迎您在我们的 [Seeed Studio Discord](https://discord.com/invite/kpY74apCWj) 和 [Seeed Studio 论坛](https://forum.seeedstudio.com/) 上分享您的项目。这些平台为您提供了与其他创作者互动、获得反馈和寻找灵感的绝佳机会。无论是解决问题、展示作品，还是加入支持性社区，*Seeed Studio 的 Discord 和论坛*都是完美的参与和合作场所。

## 技术支持与产品讨论

感谢您选择我们的产品！我们致力于为您提供多种支持，以确保您使用产品的体验尽可能顺畅。我们提供了多种沟通渠道，满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>