---
description: Seeed Studio XIAO ESP32S3 with MicroPython
title: XIAO ESP32S3 和 MicroPython
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/esp32s3_micropython/title.png
slug: /cn/xiao_esp32s3_with_micropython
last_update:
  date: 08/14/2024
  author: Agnes
---

# **Seeed Studio XIAO ESP32S3 与 MicroPython**

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/title.png" /></div>

[MicroPython](https://github.com/micropython/micropython/wiki) 是一种带有部分原生代码编译功能的 Python 解释器。它提供了 Python 3.5 的子集，专为嵌入式处理器和受限系统实现。它与 CPython 不同，您可以在 [这里](https://github.com/micropython/micropython/wiki/Differences) 了解更多区别。

## 安装 MicroPython

#### 安装 Esptool
如果您尚未安装 esptool.py，可以通过 pip 在您的 PC 上安装：
``` linux
pip install esptool
```
#### 下载 XIAO ESP32S3 MicroPython 固件
您需要从 [micropython.org](https://micropython.org/download/ESP32_GENERIC_S3/) 下载固件二进制文件。  
下载正确的 bin 文件后，导航到该文件所在的文件夹，并在该位置打开命令行终端。  
截至最终版本，最新的 bin 文件版本为：
```
ESP32_GENERIC_S3-20230602-v1.23.0.bin
```
#### 将 XIAO ESP32S3 连接到您的 PC
您需要按住 XIAO ESP32S3 板上的 BOOT 按钮，进入 'bootloader' 模式，同时将 Type-C USB 数据线插入到您的 PC。

#### 检查端口
查找 PC 上的所有串口设备。

* Linux

在 Linux 上，您可以使用 *dmesg* 命令查看已连接的设备：
```Linux
dmesg | grep tty
```
或者，您可以使用 *ls* 列出串行设备：
```
ls /dev/ttyS* /dev/ttyUSB*
```

* Windows

在 Windows 上，您可以通过打开设备管理器来找到 USB 到串口的地址。
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/2.png" /></div>

* macOS

在 macOS 上，您可以使用 *ls* 命令列出可用的串口：
```
ls /dev/cu*
```
这将显示所有串行端口设备。
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/1.png" /></div>

:::tip
如果端口繁忙，您可以使用以下命令查找 dkill 并使用 port（在 macOS 上）：
使用端口识别进程：
```
lsof | grep port
```
此命令列出打开的文件，并使用指定端口搜索任何进程。
从输出中找到进程 ID （PID） 并终止进程：
```
kill -9 <PID>
```
将 *PID* 替换为实际的进程 ID。
:::

#### 擦除闪存
```linux
esptool.py --chip esp32s3 --port /dev/cu.usbmodem11301 erase_flash
```
将 '/dev/cu.usbmodem11301' 替换为您系统中正确的端口名称（例如 Windows 上是 `COM3`，Linux 上是 `/dev/ttyUSB0`）。

#### 写入闪存
将固件刷入 XIAO ESP32S3：
```linux
esptool.py --chip esp32s3 --port /dev/cu.usbmodem11301 --baud 460800 write_flash -z 0x0 ESP32_GENERIC_S3-20240602-v1.23.0.bin
```
同样，将 '/dev/cu.usbmodem11301' 替换为正确的端口名称，并将 'ESP32_GENERIC_S3-20240602-v1.23.0.bin' 替换为您的固件文件路径。  
别忘了通过 RTS 引脚进行硬重置。  
然后，可以使用您喜欢的工具开始编译脚本并上传到 ESP32！


## 推荐的 MicroPython 编辑器
以下是一些流行的工具：
* [Thonny IDE](https://randomnerdtutorials.com/getting-started-thonny-micropython-python-ide-esp32-esp8266/)
* [Arduino for micropython](https://docs.arduino.cc/micropython/)
* [Visual Studio Code pymakr](https://randomnerdtutorials.com/micropython-esp32-esp8266-vs-code-pymakr/)
* [uPyCraft IDE](https://randomnerdtutorials.com/install-upycraft-ide-windows-pc-instructions/)
  
### 1. Thonny
安装并打开 Thonny，然后按照以下说明配置 Thonny
```
pip install thonny
#open thonny after installation
thonny
```
前往 **运行** -> **配置解释器**，确保 Thonny 选项中的 **解释器** 标签如下所示，选择 "CircuitPython (generic)" 和端口：
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_micropython/4.png" /></div>

点击对话框中的 "OK"，然后您应该能看到如下图所示的 MicroPython shell 出现在 Thonny 窗口的底部。  
逐行输入脚本到 Shell 中，以获取 flash 和 sram 的大小：
```python
import esp
esp.flash_size()
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/4.png" /></div>

恭喜您成功在 XIAO ESP32S3 上使用 Thonny 设置了 MicroPython！

### 2. Arduino MicroPython 实验室
下载 Arduino MicroPython 实验室并将设备连接到您的 PC。
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_micropython/7.png" /></div>

像这样编写代码：
```python
from machine import Pin
import time

# 定义 LED 引脚
led = Pin(6, Pin.OUT)  # 使用正确的 GPIO

# 循环闪烁 LED
while True:
    led.value(1)   # 打开 LED
    time.sleep(1)  # 等待一秒
    led.value(0)   # 关闭 LED
    time.sleep(1)  # 等待一秒
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/7.png" /></div>

### 3. 在 Visual Studio Code 上使用 Pymakr

* 安装 Pymakr  
按照[安装说明](https://randomnerdtutorials.com/micropython-esp32-esp8266-vs-code-pymakr/)安装 Pymakr。
* 将 XIAO ESP32S3 连接到计算机。  
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/8.png" /></div>

* 创建一个新项目  
打开 VS Code 并为您的微控制器创建一个新项目。
* 添加一个新的 Python 文件  
在您的项目中创建一个新的 Python 文件。
* 将脚本上传到 MCU 并编译脚本  
<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/esp32c3_micropython/10.png" /></div>

### 4. uPtCraft IDE
* [macOS](https://randomnerdtutorials.com/install-upycraft-ide-mac-os-x-instructions/)
* [Windows](PChttps://randomnerdtutorials.com/install-upycraft-ide-windows-pc-instructions/)
* [Linux](https://randomnerdtutorials.com/install-upycraft-ide-linux-ubuntu-instructions/)

## 引脚/端口信息
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/10.png" /></div>

* 更多信息请参考[硬件概述](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#hardware-overview)
* [Seeed Studio XIAO ESP32S3 原理图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_SCH_v1.2.pdf)

## 在 XIAO ESP32S3 上开始使用 MicroPython

这是[ESP32 使用 MicroPython 的快速参考](https://docs.micropython.org/en/latest/esp32/quickref.html#installing-micropython)。  
更多关于[MicroPython 库](https://docs.micropython.org/en/latest/library/index.html#python-standard-libraries-and-micro-libraries)的知识。

### 一般板控制
MicroPython 的 REPL（*R*ead-*E*val-*P*rint-*L*oop）位于 UART0（GPIO1=TX，GPIO3=RX），波特率为 115200。Tab 补全功能对于查找对象的方法很有用。粘贴模式（ctrl-E）适合将大量 Python 代码粘贴到 REPL 中。  
可以使用 MicroPython 中的 *dir()* 函数（Python 中也类似）列出对象的属性和方法。  
例如，在 shell 中输入 *dir(machine)*：
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/11.png" /></div>

*machine* 模块：
```python
import machine
machine.freq()          # 获取 CPU 当前频率，ESP32S3 为 240000000
machine.freq(160000000) # 设置 CPU 频率为 160 MHz
# 频率必须为 20MHz, 40MHz, 80MHz, 160MHz 或 240MHz
```
*esp* 模块：
```python
import esp

esp.osdebug(None)       # 关闭厂商操作系统调试信息
esp.osdebug(0)          # 将厂商操作系统调试信息重定向到 UART(0)

# 与闪存存储交互的低级方法
esp.flash_size()
esp.flash_user_start()
esp.flash_erase(sector_no)
esp.flash_write(byte_offset, buffer)
esp.flash_read(byte_offset, buffer)
```

*esp32*模块：ESP32C3、ESP32S2 和 ESP32S3 有一个可用的内部温度传感器，并以摄氏度为单位返回温度：
```
import esp32
esp32.mcu_temperature() # 读取 MCU 的内部温度，单位为摄氏度
```

### 网络-WLAN
*Network* 模块：更多信息请参考 [这里](https://docs.micropython.org/en/latest/library/network.WLAN.html)。

```python
import network

wlan = network.WLAN(network.STA_IF)  # 创建站点接口
wlan.active(True)                    # 激活接口
wlan.scan()                          # 扫描接入点
wlan.isconnected()                   # 检查站点是否连接到接入点
wlan.connect('ssid', 'key')          # 连接到接入点
wlan.config('mac')                   # 获取接口的 MAC 地址
wlan.ifconfig()                      # 获取接口的 IPv4 地址

ap = network.WLAN(network.AP_IF)     # 创建接入点接口
ap.config(ssid='ESP-AP')             # 设置接入点的 SSID
ap.config(max_clients=10)            # 设置最多可连接的客户端数
ap.active(True)                      # 激活接口
```
连接到本地 WiFi 网络的一个有用功能是：	
```python
def do_connect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('ssid', 'key') #更换 the ssid 和 key
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())
```

### 延迟与计时
*time* 模块：
```python
import time

time.sleep(1)           # 暂停 1 秒
time.sleep_ms(500)      # 暂停 500 毫秒
time.sleep_us(10)       # 暂停 10 微秒
start = time.ticks_ms() # 获取毫秒计数器
delta = time.ticks_diff(time.ticks_ms(), start) # 计算时间差
```

### 定时器
ESP32 端口有四个硬件定时器。使用带有定时器 ID 的 [类](https://docs.micropython.org/en/latest/library/machine.Timer.html#machine-timer)，ID 范围为 0 到 3（包含 0 和 3）：
```python
from machine import Timer

tim0 = Timer(0)
tim0.init(period=5000, mode=Timer.ONE_SHOT, callback=lambda t:print(0))

tim1 = Timer(1)
tim1.init(period=2000, mode=Timer.PERIODIC, callback=lambda t:print(1))
```
周期单位为毫秒。
目前此端口不支持虚拟定时器。

### 引脚与 GPIO
*machine.Pin* 类
```python
from machine import Pin

p2 = Pin(2, Pin.OUT)    # 在 GPIO2 上创建输出引脚
p2.on()                 # 将引脚设置为 "开"（高电平）
p2.off()                # 将引脚设置为 "关"（低电平）
p2.value(1)             # 将引脚设置为开/高电平

p3 = Pin(3, Pin.IN)     # 在 GPIO3 上创建输入引脚
print(p3.value())       # 获取引脚值，0 或 1

p4 = Pin(4, Pin.IN, Pin.PULL_UP) # 启用内部上拉电阻
p5 = Pin(5, Pin.OUT, value=1) # 在创建时将引脚设置为高电平
p6 = Pin(6, Pin.OUT, drive=Pin.DRIVE_3) # 设置最大驱动能力

```
可用的引脚范围如下（包括端点）：1, 2, 3, 4, 5, 6, 7, 8, 9, 43, 44。这些引脚对应于 ESP32S3 芯片的实际 GPIO 引脚号。

### UART（串行总线）
*machine.UART* 类：
```python
from machine import UART

uart1 = UART(1, baudrate=9600, tx=43, rx=44)
uart1.write('hello')  # 写入 5 个字节
uart1.read(5)         # 读取最多 5 个字节
```
ESP32C3 具有一个硬件 UART。下列引脚如下：

| UART | 引脚 |
|------|------|
| TX   | 43   |
| RX   | 44   |

### PWM（脉宽调制）
PWM 可以在所有启用输出的引脚上启用。基本频率可以从 1Hz 到 40MHz，但有一个权衡；随着基本频率的增加，工作周期的分辨率会降低。
*machine.PWM* 类：
```python
from machine import Pin, PWM

pwm2 = PWM(Pin(2), freq=5000, duty_u16=32768) # 从引脚创建 PWM 对象
freq = pwm2.freq()         # 获取当前频率
pwm2.freq(1000)            # 设置 PWM 频率范围从 1Hz 到 40MHz

duty = pwm2.duty()         # 获取当前占空比，范围 0-1023（默认 512，50%）
pwm2.duty(256)             # 设置占空比范围从 0 到 1023，作为占空比/1023 的比例（现在是 25%）

duty_u16 = pwm2.duty_u16() # 获取当前占空比，范围 0-65535
pwm2.duty_u16(2**16*3//4)  # 设置占空比范围从 0 到 65535，作为占空比_u16/65535 的比例（现在是 75%）

duty_ns = pwm2.duty_ns()   # 获取当前脉冲宽度（单位：纳秒）
pwm2.duty_ns(250_000)      # 设置脉冲宽度，单位为纳秒，范围从 0 到 1_000_000_000/freq（现在是 25%）

pwm2.deinit()              # 关闭引脚上的 PWM

pwm3 = PWM(Pin(3), freq=20000, duty=512)  # 一次性创建并配置
print(pwm3)                               # 查看 PWM 设置
```
### ADC（模拟到数字转换）
在 XIAO ESP32S3 上，ADC 功能可用于引脚 1, 2, 3, 4, 5, 6, 7, 8, 9。
*machine.ADC* 类：
```python
from machine import ADC

adc = ADC(pin)        # 创建一个作用于某引脚的 ADC 对象
val = adc.read_u16()  # 读取原始模拟值，范围为 0-65535
val = adc.read_uv()   # 读取微伏（uV）级别的模拟值
```
### SPI

#### 软件 SPI 总线
软件 SPI（使用位操作）可以在所有引脚上工作，并通过 *machine.SoftSPI* 类访问：
```python
from machine import Pin, SoftSPI

# 在给定引脚上构建一个 SoftSPI 总线
# 极性是 SCK 的空闲状态
# phase=0 表示在 SCK 的第一个边沿采样，phase=1 表示在第二个边沿采样
spi = SoftSPI(baudrate=100000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(4), miso=Pin(6))

spi.init(baudrate=200000) # 设置波特率

spi.read(10)            # 读取 10 字节的数据通过 MISO
spi.read(10, 0xff)      # 读取 10 字节，同时在 MOSI 上输出 0xff

buf = bytearray(50)     # 创建一个缓冲区
spi.readinto(buf)       # 读取到给定的缓冲区（在此例中读取 50 字节）
spi.readinto(buf, 0xff) # 读取到给定的缓冲区，并在 MOSI 上输出 0xff

spi.write(b'12345')     # 在 MOSI 上写入 5 字节

buf = bytearray(4)      # 创建一个缓冲区
spi.write_readinto(b'1234', buf) # 写入 MOSI 并从 MISO 读取到缓冲区
spi.write_readinto(buf, buf) # 写入 buf 到 MOSI 并将 MISO 的数据读取回 buf
```
#### 硬件 SPI 总线
硬件 SPI 通过机器访问。SPI 类，方法与上述软件 SPI 相同：
```python
from machine import Pin, SPI

hspi = SPI(1, 10000000)
hspi = SPI(1, 10000000, sck=Pin(7), mosi=Pin(9), miso=Pin(8))
```
| SPI  | 引脚 |
|------|-----|
| SCK  | D7  |
| MOSI | D9 |
| MISO | D8  |



### I2C

#### 软件 I2C 总线
软件 I2C（使用位操作）可以在所有具有输出能力的引脚上工作，并通过 *machine.SoftI2C* 类访问：
```python
from machine import Pin, SoftI2C

i2c = SoftI2C(scl=Pin(6), sda=Pin(5), freq=100000)

i2c.scan()              # 扫描设备

i2c.readfrom(0x3a, 4)   # 从地址为 0x3a 的设备读取 4 字节
i2c.writeto(0x3a, '12') # 向地址为 0x3a 的设备写入 '12'

buf = bytearray(10)     # 创建一个包含 10 字节的缓冲区
i2c.writeto(0x3a, buf)  # 将给定的缓冲区写入外设
```

#### 硬件 I2C 总线
驱动程序通过 *machine.I2C* 类访问，具有与上述软件 I2C 相同的方法：
```python
from machine import Pin, I2C
i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)
```

### XIAO 扩展板基础
*先决条件*：
*
<table align="center">
  <tbody><tr>
      <th>XIAO ESP32S3<br /> 带焊接引脚</th>
      <th>扩展板基础</th>
      <th>Grove 光传感器</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/15.png" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/16.png" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Light-Sensor-v1-2-LS06-S-phototransistor.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

#### 读取光传感器数据
<div align="center"><img width={300} src="https://files.seeedstudio.com/wiki/esp32c3_circuitpython/9.png" /></div>

```python
import time
from machine import Pin, ADC

# 初始化模拟输入引脚 2（对应 A0）
analog_in = ADC(Pin(1))
analog_in.atten(ADC.ATTN_11DB)  # 配置输入范围（0-3.6V）

def get_voltage(pin):
    # 将原始 ADC 值转换为电压
    return (pin.read() / 4095) * 3.3

while True:
    # 读取原始模拟值
    raw_value = analog_in.read()
    # 将原始值转换为电压
    voltage = get_voltage(analog_in)
    
    # 打印原始值和电压到串口控制台
    print("[光传感器] 原始值: {:5d} 电压: {:.2f}V".format(raw_value, voltage))
    
    # 延迟一段时间后再读取
    time.sleep(1)
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/13.png" /></div>

#### 点亮 OLED 屏幕
插入您的 XIAO ESP32S3，打开 Thonny，点击右下角配置解释器
选择解释器 - Micropython (ESP32)，并选择 **端口 >>> 点击 OK**
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/14.png" /></div>

如果一切顺利，您将看到 Shell 中的输出
从 MicroPython 安装 ssd1306 库。
点击 "工具" >>> 点击 "管理库" >>> 输入库的名称 >>> 点击 "搜索 micropython-lib 和 PyPl"

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32s3_micropython/15.png" /></div>

运行脚本并将其刷入板子。
完成编码后，点击绿色按钮运行脚本。
```python
import time
from machine import Pin, SoftI2C
import ssd1306
import math

# ESP8266 引脚分配
i2c = SoftI2C(scl=Pin(6), sda=Pin(5))  # 根据您的连接调整引脚号
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

oled.fill(0)  # 清除屏幕
oled.text("Hello, Seeder!", 10, 15)
oled.text("/////", 30, 40)
oled.text("(`3`)y", 30, 55)
oled.show()  # 显示文本
```
<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/esp32c3_micropython/17.png" /></div>

#### 摄像头流媒体测试

ESP32S3 Sense（带摄像头）也提供了摄像头流媒体测试，请参考 [这里](https://wiki.seeedstudio.com/XIAO_ESP32S3_Micropython/#step-3-testing-the-streaming-example)。

感谢阅读本文！欢迎在评论区分享您的想法。

## 资源
* [适用于 XIAO ESP32S3 的 MicroPython 固件二进制文件](https://micropython.org/download/ESP32_GENERIC_S3/)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在此为您提供各种支持，确保您与我们产品的使用体验尽可能顺畅。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
