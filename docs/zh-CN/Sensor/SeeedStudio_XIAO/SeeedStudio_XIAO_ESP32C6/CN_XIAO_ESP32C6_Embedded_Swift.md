---
description: 利用 Embedded swift 与 XIAO ESP32C6 配合使用
title: XIAO ESP32C6 与 （Embedded） Swift
image: https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/swift-xiao-esp32.png
slug: /cn/xiao-esp32-swift
last_update:
  date: 11/16/2024
  author: Agnes
---
# Guide to Embedded Swift with Seeed Studio XIAO-C6

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/swift-xiao-esp32.png" alt="pir" width={600} height="auto" /></p>

## 介绍

Seeed Studio XIAO ESP32C6 是一款强大且多功能的开发板，基于 ESP32-C6 SoC，搭载双 32 位 **RISC-V** 处理器。高性能的处理器最高运行速度为 160 MHz，低功耗处理器最高可达 20 MHz。XIAO ESP32C6 配备了 512KB SRAM 和 4MB Flash，提供了丰富的编程空间，非常适合各种物联网应用。该开发板在无线连接方面表现出色，支持 2.4 GHz Wi-Fi 6、Bluetooth® 5.3、Zigbee 和 Thread (802.15.4)，并原生支持 Matter 协议，确保智能家居的互操作性。

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

[Swift](https://www.swift.org/) 是由 Apple Inc. 开发的一种强大且直观的编程语言，主要用于开发 iOS、macOS、watchOS、tvOS 及其他平台的软件。在 [WWDC2024](https://developer.apple.com/wwdc24/)上，Apple 宣布了 Embedded Swift，这是一个开源项目，旨在将 Swift 带入嵌入式系统和物联网的世界。它允许在没有操作系统、资源有限的微控制器上使用 Swift。目前，官方的 swift-embedded-examples 支持 STM32、Raspberry Pi Pico、Nordic Semiconductor 以及 RISC-V ESP32（如 ESP32C6）等硬件平台。

<div style={{ textAlign: 'center' }}>
<img width={680} src="https://user-images.githubusercontent.com/1186214/264192528-ec5673ba-eec5-422a-b28c-8c0a1e814d1c.png" alt="Vision of Embedded Swfit " />

<div style={{ marginTop: '-8px' }}><em>Swift 语言</em></div>

</div>  

<br></br>

嵌入式 Swift 提供了多项优势：

- **性能与易用性**: 它提供与 C/C++ 相媲美的性能，同时保留了 Swift 语言友好的语法和特性 ([Swift Forums](https://forums.swift.org/t/accepted-a-vision-for-embedded-swift/68067)).
- **库和框架集成**: Swift 能够直接与 C/C++ 库链接，开发者可以无缝利用现有的框架 ([GitHub](https://github.com/swiftlang/swift-evolution/blob/main/visions/embedded-swift.md)).
- **开发者熟悉度**: 已熟练掌握 Swift 的开发者可以快速采用 Embedded Swift，利用熟悉的 API 和工具 ([Swift.org](https://www.swift.org/getting-started/embedded-swift/)).

:::tip 本指南目标

在本页面，我们将介绍以下内容：

- 如何设置所需的开发环境。
- 运行基于 GPIO 的闪烁示例。

:::

## 前提条件

要开始在 Seeed Studio XIAO-C6 上使用嵌入式 Swift，你需要以下技能和设置：

- **Seeed Studio XIAO-C6**: 本指南的核心硬件。
- **USB-C 数据线**: 用于将 XIAO-C6 连接到你的计算机。
- **一台计算机**: 运行 Linux、macOS 或 Windows（使用 **WSL2**）。
- **[ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32/get-started/index.html)**: 开发 ESP32-C6 所需的版本 5.2（推荐）。你可以从 [Espressif 网站](https://github.com/espressif/esp-idf)下载并安装。
- **[Swift 工具链](https://www.swift.org/getting-started/embedded-swift/)**: 确保你的计算机上安装了 Swift 6.0。你可以通过官方网站或操作系统的包管理器获取。

:::info 安装指南

**ESP-IDF 安装:**

- 按照 [Linux 和 macOS](https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32/get-started/linux-macos-setup.html)页面上的说明安装 ESP-IDF。
- 按照 [ XIAO 使用 Espressif ESP-IDF 开发指南](/xiao_idf) 获取详细步骤。

**Swift 工具链安装：**

- 从 [官方站点](https://www.swift.org/install/) 安装 Swift `release/6.0` 工具链。 

如果你偏好 Docker 解决方案，请参考 [Dockerfile](https://github.com/apple/swift-matter-examples/blob/main/Dockerfile) 来自 `apple/swift-matter-examples/`
:::

在继续之前，请确保你已准备好所有必要的前提条件，以确保顺利的设置和开发过程。

:::caution attention

本页面将介绍如何在 Ubuntu 22.04 Jammy Linux (WSL2) 上设置和使用嵌入式 Swift。

:::

### 安装 ESP-IDF

```shell
#!/bin/bash
# 更新并安装所需的软件包
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv python3.10-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# 克隆 ESP-IDF 仓库
cd ~
git clone --recursive https://github.com/espressif/esp-idf.git --jobs 10
cd esp-idf
git checkout v5.2.1; git submodule update --init --recursive

# 运行 ESP-IDF 安装脚本
cd ~/esp-idf && ./install.sh

# 设置 ESP-IDF 环境别名
echo 'alias get_idf=". ~/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc

echo "安装完成。使用 'get_idf' 来设置 ESP-IDF 环境。"
```

### 安装 Swift 6.0

你可以直接运行以下脚本来安装 Swift 6.0。

```shell
#!/bin/bash
# 下载 Swift 6.0 快照
wget https://download.swift.org/swift-6.0-branch/ubuntu2204/swift-6.0-DEVELOPMENT-SNAPSHOT-2024-07-03-a/swift-6.0-DEVELOPMENT-SNAPSHOT-2024-07-03-a-ubuntu22.04.tar.gz

# 解压 tar 包
tar -xvzf swift-6.0-DEVELOPMENT-SNAPSHOT-2024-07-03-a-ubuntu22.04.tar.gz

sudo mv swift-6.0-DEVELOPMENT-SNAPSHOT-2024-07-03-a-ubuntu22.04 $HOME/swift-6.0

# 更新 PATH
echo 'export PATH=$HOME/swift-6.0/usr/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# 验证安装
swift --version

echo "Swift 6.0 安装完成。"
```

## 使用 Swift 运行第一个 Blink 程序

### 步骤 1. 获取示例代码

```shell showLineNumbers
get_idf #get idf enviorment
cd ~ && git clone https://github.com/apple/swift-embedded-examples.git #get GitHub repo
cd ~/swift-embedded-examples/esp32-led-blink-sdk
```

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/apple/swift-embedded-examples">
    <strong><span><font color={'FFFFFF'} size={"4"}> GitHub Repository</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

### 步骤 2. 构建示例代码

由于用户 LED 连接到 GPIO15，您需要将代码中的 `let led = Led(gpioPin: 15)`修改为：

```swift
@_cdecl("app_main")
func app_main() {
    print("Hello from Swift on ESP32-C6!")

    var ledValue: Bool = false
    let blinkDelayMs: UInt32 = 500
    let led = Led(gpioPin: 15)

    while true {
        led.setLed(value: ledValue)
        ledValue.toggle() // 切换布尔值
        vTaskDelay(blinkDelayMs / (1000 / UInt32(configTICK_RATE_HZ)))
    }
}
```

Then you could build the firmware:

```shell
idf.py set-target esp32c6
idf.py build
```

:::info 如果遇到问题？
 如果遇到 `/bin/sh: 1: -target: not found`错误，可以通过编辑 `build.ninja` 文件来解决问题。

Find the keyword `-target` 关键词并在其前面添加 `swiftc` ，使其变为 `swiftc -target ...`此调整应有助于解决该问题。
:::

当您看到提示 `Project build complete.`后，表示固件已构建完成，可以继续将固件烧录到 XIAO C6 开发板中。

### 步骤 3. 烧录固件

<details><summary>如果使用 WSL2，需要安装 usbipd</summary>

如果您在 WSL2 环境中工作，需要安装 [usbipd](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/WSL.md#usbipd) installed.

安装方法：使用 Windows 包管理器运行以下命令：
```shell
winget install usbipd
```

安装完成后，通过以下命令共享 USB 设备：
```shell
usbipd list #Check BUSID
usbipd bind -b '<BUSID>'
```

在 WSL2 中连接设备：
```shell
usbipd attach --wsl --busid=<BUSID>
```

查看共享的端口：
```shell
dmesg | tail
# 或使用以下命令查看 tty 设备：dmesg | tail -n 100 | sed -n 's/.*\(ttyACM[0-9]*\).*/\1/p'
```
</details>

假设您已连接到 `ttyACM0`，则可以使用以下命令烧录固件：

```shell
idf.py -p /dev/ttyACM0 flash monitor
```

*此命令将：*
1. 把固件烧录到连接到 ttyACM0 的设备。
2. 开始监控设备的串口输出，显示任何打印语句或日志。

### 演示

<center>
<iframe width="407" height="724" src="https://www.youtube.com/embed/byqY2bYfVjA" title="Embedded Swift on XIAO ESP32C6 #Swift #Embedded #ESP32C6 #iot" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</center>

## 参考资料

- [Embedded Swift 博客](https://www.swift.org/blog/embedded-swift-examples/)
- [嵌入式 Swift 的愿景](https://github.com/swiftlang/swift-evolution/blob/main/visions/embedded-swift.md)
- [swift-embedded-examples](https://github.com/apple/swift-embedded-examples)
- [esp32-led-blink-sdk - GitHub](https://github.com/apple/swift-embedded-examples/tree/main/esp32-led-blink-sdk)

## 资源

- [WWDC24：利用嵌入式 Swift 缩小规模 - Apple YouTube](https://www.youtube.com/watch?v=LqxbsADqDI4&t)
- [在智能家居应用中添加对 Matter 的支持](https://developer.apple.com/videos/play/wwdc2021/10298)
- [Embedde Swift - Apple Forum](https://forums.swift.org/c/development/embedded/107)
- [在 Swift 项目中使用 C、C++ 和 Objective-C 库 - Cecilia Humlelu - YouTube](https://www.youtube.com/watch?v=jcNxtM_yTfk)
- [混合使用 Swift 和 C++](https://www.swift.org/documentation/cxx-interop/)
- [awesome-embedded-swift](https://github.com/Cosmo/awesome-embedded-swift)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供多种支持，确保您在使用产品过程中体验顺畅。我们提供了多种沟通渠道，以满足不同的偏好和需求。

<div class="table-center">
  <div class="button_tech_support_container">
  <a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
  <a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
  </div>

  <div class="button_tech_support_container">
  <a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
  </div>
</div>