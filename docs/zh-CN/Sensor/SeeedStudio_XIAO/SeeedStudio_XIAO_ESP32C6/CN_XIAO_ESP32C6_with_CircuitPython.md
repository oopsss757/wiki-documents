---
description: Seeed Studio XIAO ESP32C6 与 CircuitPython
title: 使用 CircuitPython 的 XIAO ESP32C6
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/esp32c6_circuitpython/title.png
slug: /cn/xiao_esp32c6_with_circuitpython
last_update:
  date: 11/16/2024
  author: Agnes
---

# **Seeed Studio XIAO ESP32C6与CircuitPython**

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/title.png" /></div>

本页面介绍了如何在Seeed Studio XIAO ESP32C6开发板上安装并运行由Adafruit Industries官方提供的CircuitPython！
CircuitPython是一种编程语言，旨在简化低成本微控制器板的实验和学习编程。它让入门变得更加容易，无需预先下载桌面软件。一旦你设置好开发板，打开任何文本编辑器，开始编写代码即可。更多信息请参考[这里](https://learn.adafruit.com/welcome-to-circuitpython/what-is-circuitpython)。

## 安装CircuitPython

### 方法 1: 命令行esptool

#### 安装 Esptool
如果你还没有安装esptool.py，可以通过以下命令在PC上安装：
``` linux
pip install esptool
```

#### 下载ESP32C6 CircuitPython固件
你需要从 [circirtpython.org](https://circuitpython.org/board/seeed_xiao_esp32c6/)下载固件二进制文件。
下载正确的bin文件后，进入文件夹并打开命令行终端。
截至目前，最新版本的bin文件为：
```
adafruit-circuitpython-seeed_xiao_esp32c6-en_GB-9.1.1.bin
```

#### 连接XIAO ESP32C6到PC
你需要在将USB Type-C线连接到PC时，按住XIAO ESP32C6开发板上的BOOT按钮，进入“引导加载程序”模式。

#### 检查端口
查找PC上的所有串口设备。

* Linux 

在Linux上，你可以使用*dmesg*命令查看已连接的设备：
```Linux
dmesg | grep tty
```
或者，使用*ls*列出串口设备：
```
ls /dev/ttyS* /dev/ttyUSB*
```

* Window

在Windows上，你可以通过设备管理器检查串口，查看“端口 (COM & LPT)”部分，找到可用的串口。你也可以在命令提示符中使用*mode*命令列出串口：
```
mode
```

* macOS

在macOS上，可以使用*ls*命令列出可用的串口：
```
ls /dev/cu*
```
这将显示所有串口设备。
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/1.png" /></div>

:::tip
如果端口被占用，可以使用以下命令找到并终止占用端口的进程（在macOS上）：
首先识别占用端口的进程：
```
lsof | grep port
```
该命令会列出所有打开的文件，并查找使用指定端口的进程。
从输出中找到进程ID（PID），并终止该进程：
```
kill -9 <PID>
```
将*PID*替换为实际的进程ID。
:::

#### 擦除闪存
```linux
esptool.py --chip esp32c6 --port /dev/cu.usbmodem11301 erase_flash
```
将/dev/cu.usbmodem11301替换为你系统中正确的端口名称（例如，在Windows上为`COM3`，在Linux上为`/dev/ttyUSB0）`。

#### 写入闪存
将固件写入XIAO ESP32C6：
```linux
esptool.py --chip esp32c6 --port /dev/cu.usbmodem11301 --baud 460800 write_flash -z 0x0 adafruit-circuitpython-seeed_xiao_esp32c6-en_GB-9.1.1.bin
```
同样，将`/dev/cu.usbmodem11301`替换为正确的端口名称，将`adafruit-circuitpython-seeed_xiao_esp32c6-en_GB-9.1.1.bin`替换为你固件文件的路径。
通过RTS引脚进行硬重置...


### 方法 2: Web Serial esptool
WebSerial ESPTool是为具有串口ROM引导加载程序的Espressif ESP系列微控制器设计的Web选项。它允许你擦除微控制器的内容并编程最多4个文件。更多信息请参考 [Web Serial ESPtool](https://learn.adafruit.com/circuitpython-with-esp32-quick-start/web-serial-esptool)。

然后，你可以使用你喜欢的工具开始为ESP32C6编写脚本！

## 推荐的CircuitPython编辑器

通常情况下，当CircuitPython安装完成后，或者你将已经安装CircuitPython的开发板插入计算机时，开发板会以CIRCUITPY的USB驱动器形式出现在你的电脑上。
然而，不支持原生USB的ESP32 / ESP32-C3 / ESP32-C6微控制器无法呈现CIRCUITPY驱动器。
在这些开发板上，可以使用其他方式传输和编辑文件。你可以使用 [Thonny](https://thonny.org/), ，它通过发送隐藏命令到REPL来读取和写入文件。或者你可以使用 [CircuitPython web 工作流](https://code.circuitpython.org/)，该工作流在CircuitPython 8中首次引入，提供基于浏览器的WiFi访问CircuitPython文件系统。请参考 [使用代码编辑器的Web工作流入门](https://learn.adafruit.com/getting-started-with-web-workflow-using-the-code-editor/overview.)。

### 1. Thonny
安装并打开Thonny，然后按照以下指示配置Thonny：
```
pip install thonny
#安装完成后，打开Thonny
thonny
```
进入 运行 -> 配置解释器，确保Thonny选项中的“解释器”标签页如下所示，选择“CircuitPython (generic)”和端口：
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/2.png" /></div>

点击“确定”后，你将在Thonny窗口底部看到MicroPython Shell。
此时你可以使用**R**ead-**E**valuate-**P**rint-**L**oop（REPL）进行串口连接，这使你可以输入单行代码并立即在Shell中运行，非常方便调试代码。如果你遇到程序问题，无法理解为什么出现错误，可以通过REPL进行交互式调试。更多信息请参考 [REPL](https://learn.adafruit.com/welcome-to-circuitpython/the-repl) 。

与REPL交互时，可以使用*help()*命令来获取帮助，了解从哪里开始探索REPL。
要在REPL中运行代码，可以直接输入并回车。
要列出内置模块，可以输入*help("modules")*，它会显示CircuitPython中所有内置模块的列表，包括*board*模块。
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/3.png" /></div>

接着你可以在REPL中输入*import board*并回车。然后，输入*dir(board)*来获取你板上的所有引脚列表。

```
# 使用以下命令检查引脚。更多REPL细节，请参见《欢迎使用CircuitPython》 
# 在Thonny的Shell中输入：
>>> import board
>>> dir(board)
['__class__', '__name__', 'A0', 'A1', 'A2', 'A4', 'A5', 'A6', 'D0', 'D1', 'D10', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7', 'D8', 'D9', 'I2C', 'LP_I2C_SCL', 'LP_I2C_SDA', 'LP_UART_RXD', 'LP_UART_TXD', 'MISO', 'MOSI', 'MTCK', 'MTDI', 'MTDO', 'MTMS', 'RX', 'SCK', 'SCL', 'SDA', 'SPI', 'TX', 'UART', '__dict__', 'board_id']
```


### 2. CircuitPython Web 工作流
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/5.png" /></div>

[CircuitPython代码编辑器](https://code.circuitpython.org/)在编辑运行最新版本CircuitPython的ESP32设备文件时，提供了更加全面和丰富的体验。
该编辑器支持通过Web蓝牙、USB和WiFi的Web工作流程编辑文件。

## 引脚图/端口信息
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/5.png" /></div>

* 更多信息，请参考 [硬件概览](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#hardware-overview)
* [Seeed Studio XIAO ESP32C6原理图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/XIAO-ESP32-C6_v1.0_SCH_PDF_24028.pdf)

## 在XIAO ESP32C6上使用CircuitPython入门

### 网络-WLAN

对于没有原生USB的板子（如ESP32-C6或ESP32），你需要使用REPL连接Wi-Fi。Wi-Fi功能会在CircuitPython文件系统的根目录中添加名为settings.toml的文件时启用。

#### 方法 1:通过REPL创建setting.toml文件

通过REPL创建settings.toml文件：
```r
f = open('settings.toml', 'w')
f.write('CIRCUITPY_WIFI_SSID = "wifissid"\n')
f.write('CIRCUITPY_WIFI_PASSWORD = "wifipassword"\n')
f.write('CIRCUITPY_WEB_API_PASSWORD = "webpassword"\n')
f.close()
```
* 用你的本地Wi-Fi网络名称替换 *wifissid*
* 用你的本地Wi-Fi密码替换 *wifipassword*
* 另一个密码 *webpassword* 用于通过网页浏览器访问开发板，可以自定义设置。

连接后，按下**Reset**按钮重新启动固件，然后按回车几次进入REPL提示符。此时重新连接设备到Thonny，你的XIAO ESP32C6的IP地址会显示出来。

#### 方法 2: 通过Thonny编辑*setting.toml*文件
打开Thonny --> 查看 --> 文件，编写Wi-Fi网络、密码和web密码到setting.toml文件。记得保存文件并按Reset按钮重启固件，再重新打开Thonny。

<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/6.png" /></div>
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/7.png" /></div>
:::note
别忘了，ESP32不支持5GHz网络，如果你有两个网络，使用2.4GHz的SSID。
:::
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/8.png" /></div>

### 延迟和计时
使用 *time* 模块：
```python
import time
dir(time)
time.sleep(1)           # 睡眠1秒
time.localtime()        # 获取本地时间
```

### 引脚和GPIO
可以使用*board*和*microcontroller*模块控制GPIO，以下代码将LED连接到D5引脚：
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/16.png" /></div>

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

led = digitalio.DigitalInOut(microcontroller.pin.GPIO23)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True  # 打开LED
    time.sleep(1)
    led.value = False  # 关闭LED
    time.sleep(1)
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/9.png" /></div>

### UART（串行总线）
使用 *busio* 模块：
```python
import board
import busio

# 初始化UART
uart = busio.UART(board.LP_UART_TXD, board.LP_UART_RXD, baudrate=9600)

# 发送数据
uart.write(b"Hello UART\n")

# 接收数据
while True:
    if uart.in_waiting > 0:
        data = uart.read()
        print("接收到:", data)

```
XIAO ESP32C6具有一个硬件UART，以下是对应的引脚：
| UART       | 引脚   | GPIO  |
|------------|-------|-------|
| LP_UART_TXD | A5    | GPIO5 |
| LP_UART_RXD | A4    | GPIO4 |

### PWM（脉宽调制）

使用 *pwmio* 模块：
```python
import board
import pwmio
from digitalio import DigitalInOut
import time

# 初始化PWM
pwm = pwmio.PWMOut(board.D5, frequency=5000, duty_cycle=0)

# 调暗LED
while True:
    for duty_cycle in range(0, 65535, 1000):
        pwm.duty_cycle = duty_cycle
        time.sleep(0.1)
```

### ADC（模拟到数字转换）
使用 *analogio* 模块：
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
# 调用try_lock（然后使用unlock）确保SPI总线只由你来使用
spi.try_lock()

# 选择一个芯片选择引脚
cs = digitalio.DigitalInOut(board.D3)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True

# 确保在通信前芯片选择是激活的（低电平）
cs.value = False

# 发送和接收数据
data_out = bytearray([0x01, 0x02, 0x03])
data_in = bytearray(3)

try:
    # 写入和读取数据
    spi.write_readinto(data_out, data_in)
    print("接收到:", data_in)
finally:
    # 确保在通信后芯片选择处于非激活状态（高电平）
    cs.value = True
```
XIAO ESP32C6有一个硬件SPI，以下是引脚信息：

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
*先决条件*:

<table align="center">
  <tbody><tr>
      <th>XIAO ESP32C6<br />  带焊接头</th>
      <th>XIAO扩展板基础</th>
      <th>Grove光传感器</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/15.png" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/16.png" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
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
    
    # 打印原始值和电压到串口控制台
    print("[光传感器] 原始值: {:5d} 电压: {:.2f}V".format(raw_value, voltage))
    
    # 延迟一段时间后再次读取
    time.sleep(1)
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/11.png" /></div>

#### 点亮OLED屏幕
**下载并解压库包**:
* 访问 [CircuitPython库](https://circuitpython.org/libraries) 并下载适合你的CircuitPython版本的库包。

**将库文件复制到CIRCUITPY：**:
* 解压库包ZIP文件，你会找到一个名为lib的文件夹，里面包含各种*.mpy*文件。
* 打开Thonny --> 查看 --> 文件，然后将所需的.mpy文件和lib文件夹复制到CircuitPython设备的/lib目录下。
你需要手动安装库包中的必要库：
  * adafruit_ssd1306
  * adafruit_bus_device
  * adafruit_register
  * adafruit_framebuf.mpy
  
**将font5x8.bin复制到CIRCUITPY**:
* 从 [这里](https://github.com/adafruit/Adafruit_CircuitPython_framebuf/blob/main/examples/font5x8.bin)下载font5x8.bin文件。
* 打开Thonny --> 查看 --> 文件，然后将font5x8.bin文件复制到CircuitPython设备。

<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/12.png" /></div>
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/13.png" /></div>
<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/14.png" /></div>

**创建你的CircuitPython代码**:
* 创建一个code.py文件（或者main.py）。该文件应包含你的CircuitPython代码。
```python 
import board
import busio
import displayio
import adafruit_ssd1306
import terminalio

# 初始化I2C
i2c = busio.I2C(board.SCL, board.SDA)

# 定义显示参数
oled_width = 128
oled_height = 64

# 初始化OLED显示器
oled = adafruit_ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

# 用颜色0填充显示器
oled.fill(0)
# 设置第一个像素为白色
oled.pixel(0, 0, 1)
oled.show()
```
<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/esp32c6_circuitpython/15.png" /></div>

## "卸载" CircuitPython
我们的许多开发板可以使用多种编程语言。例如，Circuit Playground Express可以与MakeCode、Code.org CS Discoveries、CircuitPython和Arduino一起使用。如果你想切换回Arduino或MakeCode，并不需要卸载CircuitPython。CircuitPython只是一个被加载到你的开发板中的程序。因此，你只需要加载另一个程序（例如Arduino或MakeCode），它将覆盖CircuitPython。

### 备份你的代码
在替换CircuitPython之前，别忘了备份你在CIRCUITPY驱动器上的代码。这意味着你的code.py文件和其他文件，lib文件夹等。当你删除CircuitPython时，这些文件可能会丢失，所以备份非常重要！只需像使用任何USB驱动器一样，将文件拖动到笔记本或台式计算机的文件夹中。

### 切换到Arduino
如果你希望改用Arduino，只需使用Arduino IDE加载一个Arduino程序。以下是上传一个简单的“闪烁”Arduino程序的示例，但你不必使用这个特定程序。 首先插入你的开发板，然后双击重置按钮，直到看到板载LED灯亮起。

感谢阅读本文！欢迎在评论中分享你的想法。

## 资源
* [XIAO ESP32C6的CircuitPython固件二进制文件](https://circuitpython.org/board/seeed_xiao_esp32c6/) for XIAO ESP32C6 with CircuitPython
* [CircuitPython的库包](https://circuitpython.org/libraries)


## 技术支持与产品讨论

感谢你选择我们的产品！我们在这里提供不同的支持，确保你使用我们产品的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>