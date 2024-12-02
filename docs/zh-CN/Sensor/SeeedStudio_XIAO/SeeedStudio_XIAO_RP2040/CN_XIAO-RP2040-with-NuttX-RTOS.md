---
description: Seeed Studio XIAO RP2040 与 NuttX
title: NuttX
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/xiao-rp2040-with-nuttx
last_update:
    date: 11/13/2024
    author: Agnes
---

# Seeed Studio XIAO RP2040 与 NuttX (RTOS)

## 介绍

[NuttX](https://nuttx.apache.org/) 是一个成熟的实时操作系统（RTOS），以其符合标准和小巧的体积广泛受到认可。NuttX 的主要特点之一是其可扩展性，能够在从 8 位微控制器到 64 位系统的不同环境中使用。这种灵活性通过遵循 POSIX 和 ANSI 标准实现，使你能够在不同架构、系列和半导体供应商的多种芯片上尝试类似的 NuttX 特性。

<div align="center"><img width ="{200}" src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/NuttX/nuttx.svg"/></div>

此外，NuttX 提供了许多先进且实用的功能，如 USB、以太网、音频和图形子系统。这些特点使 NuttX 成为开发人员的理想选择，特别适合那些希望在各种硬件上运行的多功能、稳健的 RTOS。

NuttX 支持大量且不断扩展的开发板。 [官方文档](https://nuttx.apache.org/docs/latest/platforms/) 提供了按架构和系统芯片（SoC）系列组织的受支持开发板的全面列表。

例如，NuttX 文档中的 [Seeed Studio Xiao RP2040](https://nuttx.apache.org/docs/latest/platforms/arm/rp2040/boards/seeed-xiao-rp2040/index.html) 页面提供了有关每个受支持功能的详细说明，并提供了如何使用它们的说明。

## 工具设置

开始使用 NuttX 的第一步是安装一系列所需的工具，包括你将要使用的架构的工具链，最后下载 NuttX 的源代码。NuttX 提供了针对不同平台的 [安装指南](https://nuttx.apache.org/docs/latest/quickstart/install.html) 。

安装并设置环境后，按照以下步骤操作：

1. 下载 Raspberry Pi Pico SDK:

```
git clone -b 1.1.2 https://github.com/raspberrypi/pico-sdk.git

```

3. 设置 PICO_SDK_PATH 环境变量：

```
export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>
```
2. 创建工作空间：

```
mkdir nuttxspace
```

3. 克隆仓库：

```
cd nuttxspace
git clone https://github.com/apache/nuttx.git nuttx
git clone https://github.com/apache/nuttx-apps apps
```
Apache Nuttx 分为两个项目：

- Nuttx: 包含实现了内核、驱动程序和子系统。
- Apps: 包含一组工具、shell、网络实用程序、库和解释器。

## 应用程序

要启动一个应用程序，需要在 NuttX 中加载一个配置，执行以下命令：

```
./tools/configurate.sh board_name:your_application
```
你也可以通过运行以下命令检查支持的板子列表：

```
./tools/configurate.sh -L
```

执行该脚本后，NuttX 需要进行编译。你可以使用 [Make](https://nuttx.apache.org/docs/latest/quickstart/compiling_make.html) 或 [CMake](https://nuttx.apache.org/docs/latest/quickstart/compiling_cmake.html)进行编译。

## 编程

编译成功后，接下来的目标是使用 BOOTSEL 编程。按照以下步骤操作：

**步骤 1**: 在按住 `B` 的同时，将 Seeed Studio XIAO RP2040 连接到 USB 端口。此时开发板将被识别为 USB 大容量存储设备 `RPI-RP2`。

**步骤 2**: 在工作空间中，进入 `nuttx` 文件夹，并将 `nuttx.uf2` 文件复制到 Seeed Studio XIAO RP2040 中。

**步骤 3**: 在计算机上查找新的 USB 设备。

**步骤 4**: 打开与 Seeed Studio XIAO RP2040 的串行通信。

## 实践操作

现在是时候实际操作 NuttX 了。在本次实践中，提供了四个应用程序：USB NSH、GPIO、USERLEDS 和 WS2812 驱动。

### USBNSH

NuttXShell (NSH) 是 NuttX 中的一个 shell 系统，类似于 bash 和其他类似的选项。它支持丰富的命令集、脚本和运行自定义应用程序作为“内建”（即作为同一 NuttX 二进制的一部分）。

我们可以通过清除先前的配置来开始构建过程：

```
$ cd ~/nuttxspace/nuttx
$ make distclean
```

接下来，选择 USBNSH 配置，适用于 seeed-xiao-rp2040 开发板：

```
$ ./tools/configurate.sh seeed-xiao-rp2040:usbnsh
```

编译源代码：

```
$  make -j
```

编译完成后，进行编程并打开串行通信：

```
picocom -b 115200 /dev/ttyACM0
```

你需要按 Enter 键三次，然后终端中将显示以下信息：

```
NuttShell (NSH) NuttX-12.5.1
nsh> 
```
输入 `?`，你将看到可用的命令和内建应用程序列表：

```
nsh> ?
help usage: [-v] [<cmd>]

    .           cp          exec        ls          reboot      truncate    
    [           cmp         exit        mkdir       rm          uname       
    ?           dirname     expr        mkrd        rmdir       umount      
    alias       date        false       mount       set         unset       
    unalias     dd          fdinfo      mv          sleep       uptime      
    basename    df          free        pidof       source      usleep      
    break       dmesg       help        printf      test        xd          
    cat         echo        hexdump     ps          time        
    cd          env         kill        pwd         true        

Builtin Apps:
    getprime    hello       nsh         ostest      sh 
``` 

让我们用 `hello` 向 NuttX 打个招呼，然后执行该命令：

```      
nsh> hello
Hello, World!!
```
恭喜你，完成了与 NuttX 的第一次互动。

### GPIO 驱动

通用输入输出（GPIO）是微控制器最基本的部分，使其能够与外部世界连接。通过这种方式，我们将使用 NSH 来访问和配置这些引脚。然而，在此之前，我们需要清除之前的配置。

```
$ cd ~/nuttxspace/nuttx
$ make distclean
```
选择适用于 seeed-xiao-rp2040 开发板的 GPIO 配置：

```
$ ./tools/configurate.sh seeed-xiao-rp2040:gpio
```

编译源代码：

```
$  make -j
```

编程后并打开串行通信：

```
picocom -b 115200 /dev/ttyACM0
```

你需要按 Enter 键三次，然后终端将显示以下信息：

```
NuttShell (NSH) NuttX-12.5.1
nsh>
```

要查看可与此应用程序交互的选项，请输入 `gpio -h`，它将返回一个参数列表。

```
NuttShell (NSH) NuttX-12.5.1
nsh> gpio -h
USAGE: gpio [-t <pintype>] [-w <signo>] [-o <value>] <driver-path>
       gpio -h
Where:
 <driver-path>: The full path to the GPIO pin driver.
 -t <pintype>:  Change the pin to this pintype (0-10):
 -w <signo>:    Wait for a signal if this is an interrupt pin.
 -o <value>:    Write this value (0 or 1) if this is an output pin.
mation and exit.
Pintypes:
  0: GPIO_INPUT_PIN
  1: GPIO_INPUT_PIN_PULLUP
IO_INPUT_PIN_PULLDOWN
  3: GPIO_OUTPUT_PIN
  4: GPIO_OUTPUT_PIN_OPENDRAIN
  5: GPIO_INTERRUPT_PIN
  6: GPIO_INTERRUPT_HIGH_PIN
  7: GPIO_INTERRUPT_LOW_PIN
  8: GPIO_INTERRUPT_RISING_PIN
  9: GPIO_INTERRUPT_FALLING_PIN
 10: GPIO_INTERRUPT_BOTH_PIN
```

要确认 GPIO 设备文件是否已创建，可以输入 `ls /dev`。执行后，你将看到一些已在 [seed-studio-gpio.c](https://github.com/apache/nuttx/blob/9d5b9b7c056e59c2fcc81e7029c95a995140063c/boards/arm/rp2040/seeed-xiao-rp2040/src/rp2040_gpio.c#L49-L61)中定义的 GPIO，它们分别代表：

- 3 个板载 LED（gpio 18、gpio 17 和 gpio 16）。
- 1 个输入（gpio 6）
- 1 个中断输入（gpio 7）

```
nsh> ls /dev
/dev:
 console
 gpio16
 gpio17
 gpio18
 gpio6
 gpio7
 null
 ttyACM0
```
根据 Seeed Studio RP2040 的原理图，当 GPIO 电平设置为零时，开发板的 LED 会亮起。

要控制 GPIO 输出，你必须传递参数 -o，值为 0 或 1，并指定路径（/dev/gpio）。

执行以下两个命令，你会看到黄色 LED 闪烁：
```
nsh> gpio -o 0 /dev/gpio17
Driver: /dev/gpio17
  Output pin:    Value=1
  Writing:       Value=0
  Verify:        Value=0

nsh> gpio -o 1 /dev/gpio17
Driver: /dev/gpio17
  Output pin:    Value=1
  Writing:       Value=1
  Verify:        Value=1
```
<div align="center"><img width ="{50}" src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/NuttX/seeed-studio-rp2040-gpio-turn-on-led-green.jpg"/></div>

同样的方式，你也可以对红色 LED 进行相同的操作：
```
nsh> gpio -o 0 /dev/gpio16
Driver: /dev/gpio16
  Output pin:    Value=1
  Writing:       Value=0
  Verify:        Value=0

nsh> gpio -o 1 /dev/gpio16
Driver: /dev/gpio16
  Output pin:    Value=1
  Writing:       Value=1
  Verify:        Value=1
```
<div align="center"><img width ="{50}" src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/NuttX/seeed-studio-rp2040-gpio-turn-on-led-red.jpg"/></div>


### USERLED

USERLED 是一个子系统，允许通过单一操作来控制 LED。你还可以像使用 printf 一样使用命令行。在这个演示中，我们将每隔 1 秒钟打开和关闭板载 LED。

首先，清除之前的配置。

```
$ cd ~/nuttxspace/nuttx
$ make distclean
```
使用以下命令为用户 LED 应用程序配置开发板：

```
$ ./tools/configurate.sh seeed-xiao-rp2040:userleds
```

编译源代码。

```
$  make -j
```
编程后，打开串行通信并按 Enter 键三次，遵循之前应用程序中显示的相同步骤。

如果输入：`ls /dev/`，将显示设备列表，观察到 `userleds` 文件已被创建。

```
nsh> ls /dev/
/dev:
 console
 userleds
 null
 ttyACM0
 ttyS0
```
输入 `leds`，你会看到 LED 同时开始闪烁。

```
NuttShell (NSH) NuttX-12.5.1
nsh> leds
leds_main: Starting the led_daemon
leds_main: led_daemon started

led_daemon (pid # 3): Running
led_daemon: Opening /dev/userled
led_daemon: Supported LEDs 0x07
led_daemon: LED set 0x01
led_daemon: LED set 0x00
led_daemon: LED set 0x01
led_daemon: LED set 0x00
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/NuttX/seeed-studio-userleds.gif" style={{width:300, height:'auto'}}/></div>

## WS2812 LED

WS2812 驱动程序允许控制任何使用 ws2812 协议的智能像素。这个应用程序通过 NSH 提供一个命令来执行示例。

首先，清除之前的配置。

```
$ cd ~/nuttxspace/nuttx
$ make distclean
```
选择适用于 seeed-xiao-rp2040 开发板的 ws2812 配置：
```
$ ./tools/configurate.sh seeed-xiao-rp2040:ws2812
```

编译源代码。

```
$  make -j
```

编译完成后，编程开发板，打开串行通信并按 Enter 键三次，遵循之前应用程序中显示的相同步骤。输入 `?`，可以看到 ws2812 应用程序可用。

```
NuttShell (NSH) NuttX-12.5.1
nsh> ?
help usage:  help [-v] [<cmd>]

    .           cp          exit        mkdir       rm          uname       
    [           cmp         expr        mkrd        rmdir       umount      
    ?           dirname     false       mount       set         unset       
    alias       dd          fdinfo      mv          sleep       uptime      
    unalias     df          free        pidof       source      usleep      
    basename    dmesg       help        printf      test        xd          
    break       echo        hexdump     ps          time        
    cat         env         kill        pwd         true        
    cd          exec        ls          reboot      truncate    

Builtin Apps:
    getprime    hello       nsh         ostest      sh          ws2812      
nsh> 

```

在运行应用程序之前，重要的是要确认文件夹 `ls /dev/` 中已创建 `leds0` 文件，这将用于 ws2812 驱动。

```
nsh> ls /dev/
/dev:
 console
 leds0
 null
 ttyACM0
 ttyS0

```
输入 `ws2812 -h` 将返回一列表格，显示可与此应用程序交互的参数。
```
nsh> ws2812 -h
Usage: ws2812 [OPTIONS]

Arguments are "sticky".  For example, once the device path is
specified, that path will be re-used until it is changed.
  [-p path] selects the ws2812 device.  Default: /dev/leds0 Current: /dev/leds0
  [-l leds] selects number of ws2812s in the chain.  Default: 1 Current: 1
  [-r repeat] selects the number change cycles.  Default: 4 Current: 4
  [-d delay] selects delay between updates.  Default: 20000 us Current: 20000 us

```
让我们启动应用程序，输入 `ws2812`，你将看到 LED 颜色变化。
```
nsh> ws2812
```
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/NuttX/seeed-studio-rp2040-rgb.gif" style={{width:400, height:'auto'}}/></div>

## 技术支持与产品讨论

感谢您选择我们的产品！我们提供不同的支持渠道，确保您的产品体验尽可能顺畅。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>