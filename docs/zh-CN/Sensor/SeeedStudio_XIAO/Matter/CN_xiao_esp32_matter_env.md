---
description: 介绍如何安装和配置 ESP-Matter 环境。
title: Matter 开发与 XIAO ESP32 系列
keywords:
- ESP-IDF
- matter
- XIAO
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32_matter_env
last_update:
  date: 11/27/2024
  author: Agnes
---

# Matter 开发与 XIAO ESP32 系列

:::tip
本文是 Seeed Studio XIAO ESP32 开发 Matter 系列的第三篇教程。如果你尚未阅读前面的教程，建议先阅读它们，确保你的设备已按要求配置。

- **[在 XIAO 上使用 Espressif ESP-IDF 开发](https://wiki.seeedstudio.com/xiao_idf)**
- **[快速入门 Matter 与 XIAO ESP32 系列](https://wiki.seeedstudio.com/getting_started_with_matter)**
:::

在快速发展的物联网（IoT）领域，一种新的协议应运而生，旨在彻底改变智能家居设备之间的通信与交互方式。介绍一下 Matter，一个统一的协议，承诺在不同智能家居生态系统之间架起桥梁，为全球用户创造无缝、互操作的体验。

那么，Matter 到底是什么？为什么它在物联网社区中引发如此大的关注？Matter 是一个开源、标准化的协议，使得不同厂商的智能家居设备能够无缝协作。它通过提供一个共同的通信语言和框架，旨在简化物联网设备的开发和部署。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/Matter-stack.png" style={{width:700, height:'auto'}}/></div>

- 智能家居设备的通信协议。
- 1.0 版本于 2022 年 10 月 4 日发布，经过两次推迟。
- 标准化的命令集，允许不同厂商的设备相互通信。
- 运行在 IP 网络之上，使用 Thread、Wi-Fi 或以太网。
- 使用设计安全性与零信任模型。
- 本地运行，通常通过 Matter 集线器连接到云端。
- 与 Zigbee、Z-Wave、433MHz 等其他智能家居标准并存。
- 电池寿命和通信范围依赖于无线网络技术。
- 由 Matter 集线器协调。

Matter 的价值主张非常明确：它为更具连接性、更友好的用户体验和更安全的智能家居体验提供了路径。通过采用 Matter，设备制造商可以确保他们的产品与广泛的智能家居平台和助手（如 Amazon Alexa、Google Home 和 Apple HomeKit）兼容。这种互操作性不仅惠及消费者，还为物联网领域的开发者和企业打开了新的机会。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/Matter-network.jpg" style={{width:800, height:'auto'}}/></div>

作为开发者，拥抱 Matter 意味着能够接入一个庞大的设备和服务生态系统，从而创建可以无缝集成到现有智能家居设置中的创新解决方案。通过利用 Matter 的力量，你可以专注于构建吸引人的用户体验和功能，而不是担心设备通信和兼容性的复杂问题。

要开始你的 Matter 开发之旅，你需要合适的工具和环境。在本教程中，我们将引导你通过使用 Seeed Studio XIAO ESP32C6 设置 Matter 开发环境的过程。XIAO ESP32C6 是一款专为物联网应用设计的紧凑而强大的开发板，配备 ESP32-C6 微控制器和丰富的外设接口，非常适合开发支持 Matter 的设备。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/Thread-matter-smart-homes.png" style={{width:800, height:'auto'}}/></div>

接下来的部分，我们将逐步指导你如何配置 Matter 开发环境，包括安装必要的软件、设置 Seeed Studio XIAO ESP32C6 开发板，并运行你的第一个 Matter 示例程序。在本教程结束时，你将打下坚实的基础，开始构建你自己的 Matter 设备，并为日益增长的互操作智能家居解决方案生态系统做出贡献。

让我们深入了解，通过 Seeed Studio XIAO ESP32C6 开发 Matter，释放其潜力！

## 准备软件

以下是本文所使用的系统版本、ESP-IDF 版本和 ESP-Matter 版本，供参考。这是一个已被测试并能正常工作的稳定版本。

- **主机**： [Ubuntu 22.04 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)。
- **ESP-IDF**：标签 [v5.2.1](https://github.com/espressif/esp-idf/tree/v5.2.1)。
- **ESP-Matter**：主分支，截至 2024 年 5 月 10 日，提交 [bf56832](https://github.com/espressif/esp-matter/commit/bf568327d41ca29167fcf2743ace1941432e4aa5)。
- **connectedhomeip**：目前与提交 [13ab158f10](https://github.com/project-chip/connectedhomeip/tree/13ab158f10) 兼容，截至 2024 年 5 月 10 日。
- **[Git](https://git-scm.com/)**
- **[Visual Studio Code](https://code.visualstudio.com/)**

## 准备硬件

在本节中，我们将详细介绍如何在 Ubuntu 环境中配置 ESP-IDF，并执行 ESP-IDF 提供的灯光示例。因此，在本教程中，你只需要准备以下任意一款 XIAO ESP32 系列开发板。

<div class="table-center">
	<table align="center">
		<tr>
			<th>XIAO ESP32C3</th>
			<th>XIAO ESP32S3</th>
			<th>XIAO ESP32C6</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:110, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

除了 XIAO，我们还需要 WS281x 型号的灯带或灯珠。目前，Espressif 提供的灯光示例仅支持单个灯珠，因此无论你使用的是灯带还是单颗灯珠，都会只点亮一个灯。我们还建议你购买 XIAO 的 Grove 基础板，以便更方便地接线。

<div class="table-center">
	<table align="center">
		<tr>
			<th>Grove 基础板 for XIAO</th>
			<th>Grove - RGB LED 环 (20 - WS2813 Mini)</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Shield-for-Seeeduino-XIAO/img/xiao_-Preview-25.png" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-RGB_LED_Ring-20-WS2813Mini/img/main.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-RGB-LED-Ring-20-WS2813-Mini.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a></div></td>
		</tr>
	</table>
</div>

为了统一接口，本教程中我们将使用 **D9** 引脚作为示例，请将你的 LED 灯带或灯珠连接到 XIAO 的 **D9** 引脚。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/17.png" style={{width:400, height:'auto'}}/></div>

## 视频教程

<div class="table-center">
<iframe width="800" height="400" src="https://www.youtube.com/embed/g9hBp84xs1E?si=fzE--HA7v8H8R090?si=iH-oouOl_ItkG7vF?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

## 安装 ESP-Matter 的逐步教程

:::tip
在开始 Matter 环境安装之前，确保你已[安装并访问了 ESP-IDF 编程环境](https://wiki.seeedstudio.com/xiao_idf/#step-5-set-up-environment-variables)。
:::

### 第 1 步：安装依赖

首先，你需要使用 `apt-get` 安装所需的软件包。打开终端并执行以下命令：

```bash
sudo apt-get install git gcc g++ pkg-config libssl-dev libdbus-1-dev \
	 libglib2.0-dev libavahi-client-dev ninja-build python3-venv python3-dev \
	 python3-pip unzip libgirepository1.0-dev libcairo2-dev libreadline-dev
```

此命令安装了 `git`、编译器（`gcc`、`g++`）以及构建和运行 Matter SDK 所需的库。

### 第 2 步：克隆 ESP-Matter 仓库

使用 `git clone` 命令从 GitHub 克隆 `esp-matter` 仓库，指定深度为 1，仅获取最新快照：

```bash
cd ~/esp
git clone --depth 1 https://github.com/espressif/esp-matter.git
```

进入 `esp-matter` 目录并初始化所需的 Git 子模块：

```bash
cd esp-matter
git submodule update --init --depth 1
```

进入 `connectedhomeip` 目录并运行一个 Python 脚本来管理特定平台的子模块：

```bash
cd ./connectedhomeip/connectedhomeip
./scripts/checkout_submodules.py --platform esp32 linux --shallow
```

此脚本会以浅克隆的方式（仅最新提交）更新 ESP32 和 Linux 平台的子模块。

### 第 3 步：安装 ESP-Matter

返回 `esp-matter` 根目录，然后运行安装脚本：

```bash
cd ../..
./install.sh
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/1.png" style={{width:1000, height:'auto'}}/></div>

此脚本将安装与 ESP-Matter SDK 相关的额外依赖。

### 第 4 步：设置环境变量

通过执行 `export.sh` 脚本来设置开发所需的环境变量：

```bash
source ./export.sh
```

此命令会为你的 shell 配置必要的环境路径和变量。

### 第 5 步（可选）：快速访问 ESP-Matter 开发环境

要将提供的别名和环境变量设置添加到 `.bashrc` 文件中，请按照以下步骤操作。这将配置你的 shell 环境，以便轻松切换 IDF 和 Matter 开发设置，并启用 ccache 以加快构建速度。

打开终端并使用文本编辑器打开位于主目录中的 `.bashrc` 文件。你可以使用 `nano` 或任何你喜欢的编辑器。例如：

```bash
nano ~/.bashrc
```

滚动到 `.bashrc` 文件的底部，并添加以下行：

```bash
# 设置 ESP-Matter 环境的别名
alias get_matter='. ~/esp/esp-matter/export.sh'

# 启用 ccache 加速编译
alias set_cache='export IDF_CCACHE_ENABLE=1'
```

添加完这些行后，保存文件并退出文本编辑器。如果你使用的是 `nano`，可以按 `Ctrl+O` 保存，按 `Enter` 确认，然后按 `Ctrl+X` 退出。

为了使更改生效，你需要重新加载 `.bashrc` 文件。你可以通过重新加载 `.bashrc` 文件或关闭并重新打开终端来实现。要重新加载 `.bashrc` 文件，使用以下命令：

```bash
source ~/.bashrc
```

现在你可以在任何终端会话中运行 `get_matter` 和 `set_cache` 来设置或刷新 esp-matter 环境。

```bash
get_matter
set_cache
```

## 运行灯光示例

一旦 Matter 环境配置完成，我们可以编译并上传示例应用程序 light 进行测试。

### 第 1 步：配置项目参数

进入 `examples/light` 目录。

```bash
cd examples/light                # 进入灯光示例目录
```

执行清理操作，移除先前的构建文件。

```bash
rm -rf build/                     # 清理先前的构建文件
```

将目标设置为 ESP32-C6。

```bash
idf.py set-target esp32c6        # 设置构建目标为 ESP32-C6
```

进入配置菜单并进行必要的配置。

```bash
idf.py menuconfig                # 进入配置菜单
```

在 `menuconfig` 中，你需要找到并启用 `Channel for console output` 选项。通常，该选项可以在 **Component config** -> **ESP System Settings** 下找到。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/7.png" style={{width:1000, height:'auto'}}/></div>

1. 使用箭头键导航到该选项。
2. 按下空格键或回车键选择选项：`USB Serial/JTAG Controller`。

对于不同的 XIAO，我们还需要更新其 GPIO 引脚号。该选项可以在 **Component config -> Board Support Package (generic) -> LEDs** 下找到。

- 对于 XIAO ESP32C3，D9 的 GPIO 是 9。
- 对于 XIAO ESP32S3，D9 的 GPIO 是 8。
- 对于 XIAO ESP32C6，D9 的 GPIO 是 20。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/33.png" style={{width:1000, height:'auto'}}/></div>

1. 使用箭头键导航到该选项。
2. 按下空格键或回车键进入 GPIO 编号。
3. 启用必要的选项后，按 `q` 退出 `menuconfig`，然后按 `y` 确认保存。

### 第 2 步：编译并上传示例应用程序

继续构建并闪存程序：

```bash
idf.py build                      # 构建项目
```

如果编译顺利，你将看到如下结果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/2.png" style={{width:1000, height:'auto'}}/></div>

然后你可以上传程序。

```bash
idf.py -p /dev/ttyACM0 flash monitor  # 刷写固件并监控输出
```

请根据你的 XIAO ESP32 的实际 USB 设备文件，替换 `/dev/ttyACM0`。

记得仔细按照所有指示操作，确保每个步骤都成功完成后再进行下一步。如果遇到任何错误，需要先解决错误才能继续。

:::tip
在刷写 Matter 固件的过程中，你可能会遇到没有权限的情况，此时你可以使用以下命令为设备的端口授予权限，并重新上传程序。由于设备插入或重启后，权限可能需要重新授予。

```
sudo chmod 666 /dev/ttyACM0       # 为 USB 设备文件授予权限
```

如果你的 XIAO ESP32 使用的 USB 设备文件不同，请将 `/dev/ttyACM0` 替换为实际的设备文件。
:::

然后恭喜你，如果你成功地刷入了固件，那么在终端的监视器中你将看到调试日志的输出。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/3.png" style={{width:1000, height:'auto'}}/></div>

接下来，我们将学习如何使用 matter 命令和 chip-tool 配置 Matter 设备，以完成对 Matter 设备的调试和检查。

## 主机控制与设备调试

我们在 `menuconfig` 中设置了 `console output to USB Serial`，目的是让我们能够使用 USB 接口控制 XIAO，配置加入网络或进行其他调试。此步骤至关重要，它决定了我们是否能够使用串口工具向设备发送命令。

这些命令是通过电缆直接连接设备来控制设备的，通常以 `matter` 开头。

### 常用命令

- BLE 命令：启动和停止 BLE 广播：

	```
	matter ble [start|stop|state]
	```

- Wi-Fi 命令：设置和获取 Wi-Fi 模式：

	```
	matter wifi mode [disable|ap|sta]
	```

- 设备配置：转储设备的静态配置：

	```
	matter config
	```

- 恢复出厂设置：

	```
	matter device factoryreset
	```

- 入网配对码：转储入网配对码负载：

	```
	matter onboardingcodes
	```

- 获取属性：（ID 使用十六进制）：

	```
	matter esp attribute get <endpoint_id> <cluster_id> <attribute_id>
	```

	- 示例：on_off::on_off:

		```
		matter esp attribute get 0x1 0x6 0x0
		```

- 设置属性：（ID 使用十六进制）：

	```
	matter esp attribute set <endpoint_id> <cluster_id> <attribute_id> <attribute value>
	```

	- 示例：on_off::on_off:

		```
		matter esp attribute set 0x1 0x6 0x0 1
		```

- 诊断：

	```
	matter esp diagnostics mem-dump
	```

- Wi-Fi

	```
	matter esp wifi connect <ssid> <password>
	```

### 使用方法

#### 第 1 步：安装 Minicom

Minicom 是一个基于文本的调制解调器控制和终端仿真程序，适用于类 Unix 操作系统。通过安装 Minicom，我们可以轻松地将 Matter 控制命令发送到 XIAO。要在 Ubuntu 上安装 Minicom，打开终端并输入以下命令：

```bash
sudo apt update
sudo apt install minicom
```

该命令将更新你的软件包列表并安装 Minicom。

#### 第 2 步：配置用户权限

为了允许非 root 用户访问串口（如 `ttyACM0`），你需要将用户添加到 `dialout` 组。你可以使用以下命令来实现：

```bash
sudo usermod -a -G dialout $USER
```

将 `$USER` 替换为你的用户名，或者省略它以应用于当前登录的用户。运行此命令后，你 **必须注销并重新登录**，以使组更改生效。

#### 第 3 步：设置 Minicom

现在你需要配置 Minicom 使用 `ttyACM0` 端口。使用以下命令以设置模式运行 Minicom：

```bash
sudo minicom -s
```

在设置菜单中，按以下步骤操作：

1. 选择 **Serial port setup**（串口设置）。
2. 按 'A' 键将串口设备更改为 `/dev/ttyACM0`。
3. 根据需要调整其他设置。默认设置通常是 9600 8N1（9600 波特率，无奇偶校验，8 数据位，1 停止位）。我们只需要将波特率更改为 **115200**。
4. 按 'Enter' 键退出此屏幕。

#### 第 4 步：保存配置

设置串口后：

1. 选择 **Save setup as dfl**（将设置保存为默认配置）。
2. 通过选择 **Exit from Minicom**（退出 Minicom）退出 Minicom 设置。

#### 第 5 步：运行 Minicom

要使用默认设置启动 Minicom，只需输入：

```bash
minicom
```

如果你需要使用 sudo 权限运行它（例如，如果遇到权限问题），可以使用：

```bash
sudo minicom
```

要退出 Minicom，按 `Ctrl-A` 然后按 `Z` 打开帮助菜单，接着按 `X` 退出程序。

#### 第 6 步：设置 XIAO 的分发网络

使用以下命令将 XIAO 连接到你的网络。选择网络时，请注意 XIAO 仅支持 2.4G 网络，不支持 5G 网络。

```
matter esp wifi connect <ssid> <password>
```

配对成功后，你可以使用以下命令查询有关 Matter 设备的非常重要的信息：**VendorID**、**ProductId**、**Discriminator** 和 **PinCode**。这些信息有助于你在使用 Chip-tool 工具调试时为设备配对。

```
matter config
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/8.png" style={{width:1000, height:'auto'}}/></div>

最后，使用以下命令转储入网配对码负载。

```
matter onboardingcodes onnetwork
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/9.png" style={{width:1000, height:'auto'}}/></div>

最后显示的是设备配对二维码的链接。使用二维码，你可以像 [入门指南](https://wiki.seeedstudio.com/getting_started_with_matter/#step-2-add-a-device-by-scanning-the-code-using-the-iphone-home-app) 中的步骤一样，在手机上扫描二维码来绑定 Matter 设备。

## 使用 Chip-tool 远程调试 Matter 设备

Matter 设备是智能家居的重要组成部分，始终使用数据线进行调试和设置是不切实际的。在 Matter 调试工具中，最常用的是 Chip-tool，它可以帮助我们在设备连接时进行远程调试。

Chip-tool 命令通常需要一个 Chip-tool 脚本，因此它们通常以 `chip-tool` 开头。

### 通过 IP 配对设备

下面的命令将发现设备，并尝试使用提供的设置代码与它发现的第一个设备配对：

```
chip-tool pairing onnetwork ${NODE_ID_TO_ASSIGN} 20202021
```

下面的命令将发现具有长鉴别符 3840 的设备，并尝试使用提供的设置代码与它发现的第一个设备配对：

```
chip-tool pairing onnetwork-long ${NODE_ID_TO_ASSIGN} 20202021 3840
```

下面的命令将根据给定的二维码（设备启动时记录的二维码）发现设备，并尝试与它发现的第一个设备配对：

```
chip-tool pairing code ${NODE_ID_TO_ASSIGN} MT:#######
```

在所有这些情况下，设备将被分配节点 ID `${NODE_ID_TO_ASSIGN}`（必须是十进制数字或以 0x 开头的十六进制数字）。

### 忘记当前已调试的设备

```
chip-tool pairing unpair
```

### 使用客户端发送 Matter 命令

要使用客户端发送 Matter 命令，运行构建好的可执行文件，并传递目标集群名称、目标命令名称以及端点 ID。

端点 ID 必须在 1 和 240 之间。

```
chip-tool onoff on 1
```

客户端将发送一个命令包，然后退出。

### 使用方法

当你准备好使用 Chip-tool 进行调试时，可以断开 XIAO 与计算机的连接，并将其连接到一个已供电的源。

作为第一步，我们需要配对设备，这可以通过上述 [通过 IP 配对设备](#pair-a-device-over-ip) 部分中的任何方法完成。

例如，我使用以下命令。

```
chip-tool pairing onnetwork-long 0x12 20202021 3840
```

在这种情况下，设备将被分配节点 ID `0x12`（必须是十进制数字或以 0x 开头的十六进制数字）。20202021 是 PinCode，3840 是 Discriminator。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/5.png" style={{width:1000, height:'auto'}}/></div>

最后，使用以下命令验证你是否可以控制灯光的开关。

打开灯：

```
chip-tool onoff on 0x12 0x1
```

关闭灯：

```
chip-tool onoff off 0x12 0x1
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/6.png" style={{width:1000, height:'auto'}}/></div>

`0x12` 是我们在配对时分配给设备的节点 ID。

恭喜大家，经过这里的教程步骤，我相信你已经对 ESP-Matter 开发框架的一般步骤和调试工具的使用有了初步的了解。如果仍有不理解或不熟悉的地方，我们将在接下来的教程中继续使用并引导你，因此请继续关注！

## 故障排除

### Q1：为什么在安装环境时会遇到各种错误？

ESP-Matter 的环境要求较高，如果你使用的是常用于开发的 Ubuntu 主机，可能会因为某些 Python 依赖项的版本不同而出现错误。由于 Matter 框架并非 Seeed 开发，因此我们可能无法解决这一部分问题。如果在安装过程中遇到问题，建议你提交 issue 给官方 **[ESP-Matter 仓库](https://github.com/espressif/esp-matter)** 获取帮助。

### Q2：如何卸载 Matter 的环境？

如果你在运行 `./install.sh` 脚本时卡在配置 Python 环境的步骤，可能需要检查你使用的 Matter 版本是否与 connectedhomeip 版本匹配。

重置环境的简单方法是执行以下命令：

```
rm -r connectedhomeip/connectedhomeip/.environment
```

然后重新拉取适当版本的 connectedhomeip 分支：

```
git submodule update --init --depth 1
```

如果仍然无法解决问题，删除整个 esp-matter 文件夹，并按照 Wiki 内容重新运行。

## 资源

- **[ESPRESSIF Matter - 使用 SDK 开发](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html#)**

## 技术支持与产品讨论

感谢选择我们的产品！我们将为您提供不同的支持，确保您使用我们的产品时体验尽可能顺畅。我们提供了多种沟通渠道，以满足不同的需求和偏好。

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