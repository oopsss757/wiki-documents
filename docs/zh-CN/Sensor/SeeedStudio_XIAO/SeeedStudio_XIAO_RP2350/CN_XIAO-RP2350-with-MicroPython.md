---
description: 如何将 MicroPython 用于 RP2350
title: 带有 MicroPython 的 Seeed Studio XIAO RP2350
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao-rp2350-micropython
sidebar_position: 1
last_update:
  date: 11/18/2024
  author: Agnes
draft: true
---

## 介绍

MicroPython 因其简洁和易用性而广受欢迎，RP2 系列设备完全支持 MicroPython 和 CircuitPython。  
欢迎使用 Seeed Studio XIAO RP2350 的 MicroPython 指南！本指南将帮助您设置 MicroPython，编写并运行第一个脚本，并通过 Thonny IDE 探索 XIAO RP2350 的功能。

## 您需要准备的工具

- Seeed Studio XIAO RP2350
- USB-C 数据线
- 具有互联网连接的电脑
- Thonny IDE（推荐给初学者）

## 在 XIAO RP2350 上设置 MicroPython

1. **下载并安装 Thonny IDE**

   Thonny IDE 是一款适合初学者的 Python 开发环境，非常适合用于编写和运行 MicroPython 脚本。您可以从 [这里](https://thonny.org/) 下载。

2. **安装 MicroPython 固件**

   在 XIAO RP2350 上使用 MicroPython 需要先将 MicroPython 固件烧录到开发板上。

   - 从 [官方站点](https://micropython.org/download/rp2-pico/) 下载最新的 RP2040 MicroPython 固件。
   - 按住 BOOTSEL 按钮，将 XIAO RP2350 连接到电脑以进入引导模式。
   - 将下载的 `.uf2` 文件拖放到出现的 RP2040 存储设备中。

3. **配置 Thonny IDE**

   - 打开 Thonny IDE。
   - 转到 `工具` > `选项` > `解释器`。
   - 选择 `MicroPython (Raspberry RP2350)` 作为解释器，并选择对应的 COM 端口。

## 编写您的第一个 MicroPython 脚本

1. **Hello, World!**

   我们从一个简单的脚本开始，在控制台打印 "Hello, World!"。

   ```python
   print("Hello, World!")
   ```

2. **运行脚本**

   - 在 Thonny 编辑器中输入脚本。
   - 点击 `运行` 按钮或按下 `F5`。
   - 您应该在控制台中看到打印的 "Hello, World!"。

## 探索 GPIO 控制

让我们探索如何使用 MicroPython 控制 XIAO RP2350 的 GPIO 引脚。

1. **让 LED 闪烁**

   将一个 LED 和适当的电阻连接到 GPIO 引脚（例如 GP25）。

   ```python
   import machine
   import time

   led = machine.Pin(25, machine.Pin.OUT)

   while True:
       led.value(1)  # 点亮 LED
       time.sleep(1)
       led.value(0)  # 熄灭 LED
       time.sleep(1)
   ```

2. **运行 LED 闪烁脚本**

   - 在 Thonny 编辑器中输入脚本。
   - 点击 `运行` 按钮或按下 `F5`。
   - LED 应该开始一闪一灭。

## 附加资源

- [Thonny IDE 设置指南](https://raspberrytips.com/thonny-ide-raspberry-pi/)
- [Raspberry Pi 官方文档](https://www.raspberrypi.com/documentation/microcontrollers/micropython.html)
- [Raspberry Pi Pico 系列 Python SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf)

## 技术支持与产品讨论

感谢您选择我们的产品！我们提供多种支持渠道，以确保您使用我们的产品时获得顺畅的体验。我们致力于满足您的不同需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
