---
description: Seeed Studio XIAO ESP32C3与CircuitPython
title: XIAO ESP32C3 与 CircuitPython
keywords:
- xiao
- esp32c3
- circuitpython
image: https://files.seeedstudio.com/wiki/esp32c3_circuitpython/title.png
slug: /cn/xiao_esp32c3_with_circuitpython
last_update:
  date: 11/15/2024
  author: Agnes
---

# **Seeed Studio XIAO ESP32C3与CircuitPython**

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/title.png" /></div>

本文介绍如何在Seeed Studio XIAO ESP32C3开发板上安装并运行Adafruit Industries官方的CircuitPython！
CircuitPython是一种编程语言，旨在简化低成本微控制器板上的实验和编程学习。它使得入门比以往任何时候都更容易，不需要预先下载桌面软件。一旦设置好开发板，你只需打开任何文本编辑器，开始编辑代码即可。如需更多信息，请参考 [这里](https://learn.adafruit.com/welcome-to-circuitpython/what-is-circuitpython)。

## 安装CircuitPython

### 方法 1：使用命令行的esptool

#### 安装Esptool
如果尚未安装esptool.py，可以通过pip在你的PC上安装：

``` linux
pip install esptool
```

#### 下载XIAO ESP32C3的CircuitPython固件
你需要从 [circirtpython.org](https://circuitpython.org/board/seeed_xiao_esp32c3/)下载固件二进制文件。
下载正确的bin文件后，进入该文件所在的文件夹，并在该位置打开命令行终端。
截至当前版本，最新的bin文件为：

```
adafruit-circuitpython-seeed_xiao_esp32c3-en_GB-9.1.1.bin
```

#### 连接XIAO ESP32C3到PC

你需要按住XIAO ESP32C3板上的BOOT按钮，进入“bootloader”模式，同时将Type C USB数据线连接到PC。

#### 检查端口

查看PC上的所有串口设备。


* Linux 

在Linux上，你可以使用 dmesg 命令查看连接的设备：

```Linux
dmesg | grep tty
```

或者，你可以使用 ls 命令列出串口设备：

```
ls /dev/ttyS* /dev/ttyUSB*
```

* Window

在Windows上，你可以通过设备管理器检查串口。在“端口（COM与LPT）”部分查看可用的串口。也可以使用命令提示符中的mode命令列出串口：

```
mode
```

* macOS

在macOS上，你可以使用 ls 命令列出可用的串口：

```
ls /dev/cu*
```

这将显示所有串口设备。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/1.png" /></div>

:::tip
如果端口忙碌，你可以使用以下命令查找并终止占用该端口的进程（在macOS上）：
查找占用端口的进程：

```
lsof | grep port
```

该命令列出打开的文件，并搜索任何使用指定端口的进程。
从输出中找到进程ID（PID），然后终止进程：

```
kill -9 <PID>
```

将 *PID* 替换为实际的进程ID。
:::


#### 擦除闪存

```linux
esptool.py --chip esp32c3 --port /dev/cu.usbmodem11301 erase_flash
```

将`/dev/cu.usbmodem11301`替换为你系统中正确的端口名（例如Windows中的`COM3`，Linux中的`/dev/ttyUSB0`）。


#### 写入闪存
将固件写入XIAO ESP32C3：

```linux
esptool.py --chip esp32c3 --port /dev/cu.usbmodem11301 --baud 460800 write_flash -z 0x0 adafruit-circuitpython-seeed_xiao_esp32c3-en_GB-9.1.1.bin
```

再次，将`/dev/cu.usbmodem11301`替换为正确的端口名，将`adafruit-circuitpython-seeed_xiao_esp32c3-en_GB-9.1.1.bin`替换为你下载的固件文件路径。
通过RTS引脚进行硬复位…


### 方法 2: Web Serial esptool
WebSerial ESPTool是一个针对具有串行ROM引导加载程序的Espressif ESP系列微控制器板设计的Web编程工具。它允许擦除微控制器的内容并以不同的偏移量编程最多4个文件。请参考 [Web Serial ESPtool](https://learn.adafruit.com/circuitpython-with-esp32-quick-start/web-serial-esptool)。

然后，你可以使用你喜欢的工具开始编写脚本并将其上传到XIAO ESP32C3！

## 推荐的CircuitPython编辑器

通常，当CircuitPython安装完成后，或者你将已经安装CircuitPython的开发板插入计算机时，开发板会以名为CIRCUITPY的USB驱动器的形式出现在计算机中。
然而，不支持本机USB的ESP32或ESP32-C3微控制器无法呈现CIRCUITPY驱动器。
在这些板上，可以通过其他方式传输和编辑文件。你可以使用 [Thonny](https://thonny.org/), ，它通过隐藏命令发送到REPL来读取和写入文件。或者，你可以使用 [CircuitPython web 工作流程](https://code.circuitpython.org/)该功能在CircuitPython 8中引入。Web工作流程提供基于浏览器的WiFi访问CircuitPython文件系统，更多信息请参考 [使用代码编辑器进行Web工作流程入门](https://learn.adafruit.com/getting-started-with-web-workflow-using-the-code-editor/overview.)。

### 1. Thonny
安装并打开Thonny，然后按照以下说明配置Thonny：

```
pip install thonny
#open thonny after installation
thonny
```

进入 "Run" -> "Configure Interpreter"，确保Thonny选项中的"Interpreter"标签如图所示，选择“CircuitPython (generic)”和端口：

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/2.png" /></div>

点击对话框中的“OK”，你应该会在Thonny窗口底部看到Micropython shell，如下图所示。
然后，你可以使用**R**ead-**E**valuate-**P**rint-**L**oop（REPL）进行串口连接，它允许你输入单行代码并立即在shell中运行。如果你在某个程序中遇到问题并且不知道原因，REPL非常有用。它是交互式的，非常适合测试新想法。更多信息请参考 [REPL](https://learn.adafruit.com/welcome-to-circuitpython/the-repl) 。

你可以通过输入*help()*与REPL进行交互，帮助你开始探索REPL。要在REPL中运行代码，直接在REPL提示符旁边输入。
输入*help("modules")*可以列出所有内置模块，包括*board*模块。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/3.png" /></div>

接下来，输入*"import board"*到REPL并按Enter。然后，输入*"dir(board)"*，它将列出你板上的所有引脚。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/4.png" /></div>

### 2. CircuitPython Web Workflow

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/5.png" /></div>

[CircuitPython代码编辑器](https://code.circuitpython.org/) 提供了一个更全面、丰富的体验，适用于在运行最新版本CircuitPython的ESP32设备上编辑文件。
该编辑器支持通过Web蓝牙、USB和WiFi Web工作流来编辑文件。

## 引脚/端口信息

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/6.png" /></div>

* 更多信息请参阅 [硬件概述](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/#hardware-overview)
* [Seeed Studio XIAO ESP32C3 原理图](https://files.seeedstudio.com/wiki/XIAO_WiFi/Resources/Seeeduino-XIAO-ESP32C3-SCH.pdf)



## 在XIAO ESP32C3上开始使用CircuitPython

### 网络-WLAN

对于没有本机USB支持的开发板（如ESP32-C3或ESP32），你需要通过REPL连接到Wi-Fi。当文件名为*settings.toml*的文件被添加到CircuitPython文件系统的根目录时，将启用Wi-Fi功能。
通过REPL创建*settings.toml*文件：

```r
f = open('settings.toml', 'w')
f.write('CIRCUITPY_WIFI_SSID = "wifissid"\n')
f.write('CIRCUITPY_WIFI_PASSWORD = "wifipassword"\n')
f.write('CIRCUITPY_WEB_API_PASSWORD = "webpassword"\n')
f.close()
```

* 将*wifissid*替换为你本地Wi-Fi网络的名称。
* 将*wifipassword*替换为你本地Wi-Fi的密码。
* 另一个密码*webpassword*用于通过Web浏览器访问开发板。你可以根据需要设置这个密码。

连接后，你可以按**重置**按钮启动固件，然后按几次回车键进入REPL提示符。然后重新连接设备到Thonny，你的XIAO ESP32C3的IP地址将会显示出来。

:::note
请记住，ESP32不支持5 GHz网络，因此如果你有两个网络，使用2.4 GHz的SSID。
:::

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/7.png" /></div>

### 延迟和定时

使用*time*模块：

```python
import time
time.sleep(1)           # 延时1秒
time.sleep_ms(500)      # 延时500毫秒
time.sleep_us(10)       # 延时10微秒
start = time.ticks_ms() # 获取毫秒计数器
delta = time.ticks_diff(time.ticks_ms(), start) # 计算时间差
```


### 引脚和GPIO

你可以使用*board*和*microcontroller*模块控制GPIO，以下是连接LED到D5引脚的示例代码：
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/14.png" /></div>

```python
# 使用board模块
import board
import digitalio
import time

led = digitalio.DigitalInOut(board.D5)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True  # 打开LED
    time.sleep(1)
    led.value = False  # 关闭LED
    time.sleep(1)
    
# 使用microcontroller模块
import microcontroller
import digitalio
import time

led = digitalio.DigitalInOut(microcontroller.pin.GPIO7)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True  # 打开LED
    time.sleep(1)
    led.value = False  # 关闭LED
    time.sleep(1)
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/8.png" /></div>

### UART（串行总线）

使用*busio*模块：

```python
import board
import busio

# 初始化UART
uart = busio.UART(board.TX, board.RX, baudrate=9600)

# 发送数据
uart.write(b"Hello UART\n")

# 接收数据
while True:
    if uart.in_waiting > 0:
        data = uart.read()
        print("Received:", data)

```

XIAO ESP32C3有一个硬件UART，下面是对应的引脚：

| UART | Pin |
|------|-----|
| TX   | D6  |
| RX   | D7  |

### PWM（脉冲宽度调制）

使用 *pwmio* 模块:

```python
import board
import pwmio
from digitalio import DigitalInOut
import time

# 初始化PWM
pwm = pwmio.PWMOut(board.D5, frequency=5000, duty_cycle=0)

# 调节LED亮度
while True:
    for duty_cycle in range(0, 65535, 1000):
        pwm.duty_cycle = duty_cycle
        time.sleep(0.1)

```

### ADC（模拟到数字转换）

使用 *analogio* 模块:

```python
import board
import analogio
import time

# 初始化ADC
adc = analogio.AnalogIn(board.A0)

while True:
    value = adc.value
    print("ADC值:", value)
    time.sleep(1)

```

### SPI

```python
import board
import busio
import digitalio

# 初始化SPI
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

# 选择一个芯片
cs = digitalio.DigitalInOut(board.D5)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True 

# 发送和接收数据
data_out = bytearray([0x01, 0x02, 0x03])
data_in = bytearray(3)
spi.write_readinto(data_out, data_in)
print("接收到的数据:", data_in)
```

| SPI  | 引脚 |
|------|-----|
| SCK  | D8  |
| MOSI | D10 |
| MISO | D9  |

### I2C

```python
import board
import busio

# 初始化I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
```

### XIAO扩展板基础

*前提条件：*:

<table align="center">
  <tbody><tr>
      <th>XIAO ESP32C3<br /> 带焊接插针</th>
      <th>扩展板基础</th>
      <th>Grove光传感器</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:100, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/15.png" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/16.png" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Light-Sensor-v1-2-LS06-S-phototransistor.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

#### 读取光传感器数据

<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/9.png" /></div>

```python
import time
import board
import analogio

# 初始化A0上的模拟输入
analog_in = analogio.AnalogIn(board.A0)

def get_voltage(pin):
    return (pin.value * 3.3) / 65536

while True:
    # 读取原始模拟值
    raw_value = analog_in.value
    # 将原始值转换为电压
    voltage = get_voltage(analog_in)
    
    # 打印原始值和电压
    print("[光传感器] 原始值: {:5d} 电压: {:.2f}V".format(raw_value, voltage))
    
    # 延时一段时间再读取
    time.sleep(1)
```

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/10.png" /></div>

#### 点亮OLED显示屏

**下载并解压库包：**:
* 访问 [CircuitPython库](https://circuitpython.org/libraries) 并下载适用于你的CircuitPython版本的库包。

**将库复制到CIRCUITPY**:

* 解压库包ZIP文件，你将找到一个名为lib的文件夹，里面包含多个*.mpy文件。
* 打开Thonny->查看->文件，然后将所需的.mpy文件和lib文件夹复制到CircuitPython设备的/lib目录下。
你需要手动安装库包中的必要库：
  * adafruit_ssd1306
  * adafruit_bus_device
  * adafruit_register
  * adafruit_framebuf.mpy

<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/11.png" /></div>

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/12.png" /></div>

**编写CircuitPython代码**:

* 创建一个code.py（或main.py）文件，这个文件将包含你的CircuitPython代码。

```python
import board
import busio
import displayio
import adafruit_ssd1306
import terminalio

# 初始化I2C
i2c = busio.I2C(board.SCL, board.SDA)

# 定义显示屏参数
oled_width = 128
oled_height = 64

# 初始化OLED显示屏
oled = adafruit_ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

# 清空显示屏
oled.fill(0)
# 设置第一个像素为白色
oled.pixel(0, 0, 1)
oled.show()
```
<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/13.png" /></div>


## "卸载" CircuitPython

我们的许多开发板可以与多种编程语言一起使用。例如，Circuit Playground Express 可以与 MakeCode、Code.org CS Discoveries、CircuitPython 和 Arduino 一起使用。你可能想要回到 Arduino 或 MakeCode 环境。其实并不需要卸载 CircuitPython。CircuitPython 只是加载到开发板上的“另一种程序”。因此，你只需要加载另一个程序（Arduino 或 MakeCode），它将覆盖 CircuitPython。

### 备份你的代码

在替换 CircuitPython 之前，请不要忘记备份你在 CIRCUITPY 驱动器上的代码。这意味着你需要备份 code.py 及任何其他文件，如 lib 文件夹等。移除 CircuitPython 时，这些文件可能会丢失，因此备份是非常重要的！只需像使用 USB 驱动器一样，将文件拖到笔记本电脑或桌面计算机的文件夹中即可。

### 转向 Arduino

如果你想使用 Arduino，只需使用 Arduino IDE 来加载一个 Arduino 程序。这里是上传一个简单的“Blink” Arduino 程序的示例，但你不必使用这个特定的程序。 首先，插入你的开发板，双击重置按钮，直到看到板载 LED 闪烁。

感谢你阅读本文！欢迎在评论中分享你的想法。

## 资源

* [XIAO ESP32C3 的 CircuitPython 固件二进制文件](https://circuitpython.org/board/seeed_xiao_esp32c3/) for XIAO ESP32C3 with CircuitPython
* [CircuitPython 的库包](https://circuitpython.org/libraries)

## 技术支持与产品讨论

感谢你选择我们的产品！我们提供多种支持方式，确保你在使用我们的产品时能够获得顺畅的体验。我们为不同的需求和偏好提供了多个沟通渠道。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
