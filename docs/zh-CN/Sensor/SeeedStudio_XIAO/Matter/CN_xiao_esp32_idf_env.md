---
description: 介绍如何安装和配置 IDF 环境。
title: 在 XIAO 上使用 Espressif ESP-IDF 开发
keywords:
- ESP-IDF
- XIAO
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_idf
last_update:
  date: 11/27/2024
  author: Agnes
---

# 在 XIAO 上使用 Espressif ESP-IDF 开发

:::tip
本文是 Seeed Studio XIAO ESP32 开发 Matter 系列的第一篇教程。如果您想使用 XIAO ESP32 系列开始 Matter 设备开发之旅，请从这里开始。

本教程适用于 XIAO ESP32C3、XIAO ESP32S3 和 XIAO ESP32C6。
:::

在物联网开发的世界中，ESP-IDF（Espressif IoT 开发框架）因其强大的功能和对 ESP32 系列微控制器的广泛支持而受到广泛欢迎。随着 Matter 协议的出现，它旨在为智能家居设备提供统一的标准，开发者们迫切希望探索这个新的前沿。然而，Matter 开发目前不支持 Arduino 框架，因此 ESP-IDF 成为主要的开发选择。在本教程中，我们将引导您如何在 Ubuntu 系统上安装 ESP-IDF，让您能够开始在 XIAO ESP32 系列上进行 Matter 开发。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/matter-sdk.png" style={{width:800, height:'auto'}}/></div>

## 基于 Ubuntu 22.04

在 Matter 开发中，操作系统的选择是一个重要的考虑因素。虽然 Windows 不提供对 Matter 开发的原生支持，但开发者仍然可以使用 Windows 子系统 Linux（WSL）**[创建兼容环境](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html#windows-10)**。**[WSL](https://learn.microsoft.com/en-us/windows/wsl/install)** 允许在 Windows 上直接运行 Linux 发行版（如 Ubuntu），从而可以访问 Matter 开发所需的工具和库。

另一方面，macOS 因其基于 Unix 的基础和全面的开发工具，也在开发者中非常流行。

最终，操作系统的选择取决于个人喜好和 Matter 项目的具体要求。作为 Linux 发行版，Ubuntu 为 Matter 开发提供了原生且流畅的体验。然而，如果您更习惯于 Windows 或 macOS，仍然可以使用诸如 WSL 或 macOS 内置终端等工具，设置适合的开发环境。

:::caution
我们不推荐在 Windows 上开发 ESP-Matter，即使使用 WSL。Windows 不支持将 COM 端口暴露给 WSL 发行版。您可能需要频繁切换终端或安装 [usbipd-win](https://github.com/dorssel/usbipd-win) 来解决端口问题，包括 [chip-tool 的使用也是一个瓶颈](https://github.com/espressif/esp-matter/blob/main/docs/en/using_chip_tool.rst)。
:::

**总结一下，我们将使用 Ubuntu 作为接下来的开发步骤，并使用 Matter 当前支持的最新 Ubuntu 22.04。如果您希望使用本教程以外的操作系统进行 Matter 开发，可以参考 Lexin 官方文档，本教程不再重复相关内容。**

- **[ESPRESSIF IDF - 开始使用](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)**

## 准备软件

以下是本文使用的系统版本和 ESP-IDF 版本，供参考。这是一个经过测试的稳定版本。

- **主机**: [Ubuntu 22.04 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)。
- **ESP-IDF**: 标签 [v5.2.1](https://github.com/espressif/esp-idf/tree/v5.2.1)。
- **[Git](https://git-scm.com/)**
- **[Visual Studio Code](https://code.visualstudio.com/)**

## 准备硬件

在本节中，我们将详细介绍如何在 Ubuntu 环境中配置 ESP-IDF 并执行 ESP-IDF 提供的灯光示例。因此，您只需要准备以下任意一款 XIAO ESP32 系列产品。

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

:::caution
XIAO ESP32C3 没有板载 LED，因此如果您使用 XIAO ESP32C3，可能需要额外准备一个 LED。
:::

## 视频教程

由于 ESP-IDF 开发框架面向的是更专业的软件开发人员，其使用门槛高于 Arduino，因此相关文档和信息较少。为了让 XIAO 用户尽快开始使用 ESP-IDF，并减少在配置开发环境时可能遇到的问题，我们将以视频和图文的形式展示环境配置。本节为视频部分，如果您想跟随视频进行操作，可以参考该视频来安装和配置 ESP-IDF 环境，并最终点亮 XIAO ESP32C6 的板载 LED。

<div class="table-center">
<iframe width="900" height="450" src="https://www.youtube.com/embed/QdPmsGDd7zs?si=5r_OO2EwZMX8D_HM?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

## 步骤安装 ESP-IDF

本节是 ESP-IDF 环境配置的图文教程部分。

### 步骤 1. 安装必要的软件

首先通过更新和升级您的 Ubuntu 系统，确保您拥有最新的软件包和依赖项。打开终端并运行以下命令：

```
sudo apt update
sudo apt upgrade
```

通过运行以下命令安装 ESP-IDF 所需的必要软件包，如果您确信这些软件包已安装，可以跳过此步骤，但我们仍然建议您运行此命令进行检查：

```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

:::note
ESP-IDF 需要使用 CMake 版本 3.16 或更高版本。如果您的操作系统版本没有此版本，可以运行 "[tools/idf_tools.py](https://github.com/espressif/esp-idf/blob/v5.2.1/tools/idf_tools.py) install cmake" 来安装适合的版本。
:::

### 步骤 2. 安装 Python3

通常，完整版本的 Ubuntu 22.04 LTS 会默认安装 Python 3.12。如果不确定，可以通过以下方法检查。如果没有安装，或者您曾经卸载过 Python 3，也可以按照下面的步骤重新安装。如果您确信已经安装了 Python 3，可以跳过此步骤。

检查您当前安装的 Python 版本：

```
python --version
```

如果输出类似 `Python 2.7.17`，则默认解释器是 `Python 2.7`。此时，您需要检查是否已安装 Python 3：

```
python3 --version
```

如果上述命令返回错误，则说明 Python 3 尚未安装。

下面是安装 Python 3 的步骤概述：

- 使用 HomeBrew 安装 Python 3，可以按照以下方式进行：
    ```
    brew install python3
    ```

- 如果您使用的是 MacPorts，可以运行：
    ```
    sudo port install python38
    ```

### 步骤 3. 获取 ESP-IDF

要获取 ESP-IDF，请导航到您的安装目录，并使用 `git clone` 克隆仓库，以下是适用于不同操作系统的具体指令。打开终端，并运行以下命令：

```
mkdir -p ~/esp
cd ~/esp
git clone -b v5.2.1 --recursive https://github.com/espressif/esp-idf.git
```

通过执行上述命令，ESP-IDF 将被下载到 `~/esp/esp-idf` 目录下。

### 步骤 4. 设置工具

除了 ESP-IDF 之外，您还需要安装 ESP-IDF 使用的工具，如编译器、调试器、Python 包等，来支持 ESP32 项目的开发。

```
cd esp-idf/
./install.sh
```

### 步骤 5. 设置环境变量

安装的工具尚未添加到 PATH 环境变量中。为了使工具可以从命令行使用，必须设置一些环境变量。ESP-IDF 提供了一个脚本来完成这一操作。

在您将要使用 ESP-IDF 的终端中，运行：

```
source ./export.sh
cd ..
```

此时，整个 ESP-IDF 环境已实际配置完成。为了验证 ESP-IDF 是否正确安装，运行以下命令：

```
idf.py --version
```

如果安装成功，您应该会看到 ESP-IDF 的版本信息。

### 步骤 6（可选）。快速访问 ESP-IDF 开发环境

如上所述，每次我们启动终端或重启计算机时，都需要在新终端中导入 ESP-IDF 的环境变量，这给我们带来了很大的不便，尤其是当我们需要频繁开发 ESP32 时。我们可以通过以下步骤修改 Shell 配置文件，使用 `get_idf` 命令启动 ESP-IDF 环境。

在终端中输入以下命令打开 `.bashrc` 文件：

```
nano ~/.bashrc
```

在 `.bashrc` 文件末尾添加以下内容：

```
alias get_idf='. ~/esp/esp-idf/export.sh'
```

通过重新启动终端会话或运行 `source [配置文件路径]` 来刷新配置，例如：`source ~/.bashrc`。

现在，您可以在任何终端会话中运行 `get_idf` 来设置或刷新 ESP-IDF 环境。

:::caution
从技术上讲，您可以将 `export.sh` 直接添加到 Shell 的配置文件中；但是不推荐这样做。这样做会在每次终端会话中激活 IDF 虚拟环境（包括那些不需要 IDF 的会话），这会破坏虚拟环境的目的，并可能影响其他软件的运行。
:::

## 运行 LED 示例程序

为了确保您的 ESP-IDF 环境正确配置，我们将运行一个简单的 LED 示例程序。

### 步骤 1. 将 XIAO ESP32C6 连接到 PC

现在，将 XIAO ESP32 系列开发板连接到计算机，并检查该板在什么串口下可见。我们将以 XIAO ESP32C6 为例。

串口的命名模式通常是 `/dev/tty`。一般情况下，您的计算机上可能有多个以 `tty` 开头的端口。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/11.png" style={{width:800, height:'auto'}}/></div>

确定端口也很简单，您可以使用查询命令查看未连接 XIAO 时默认存在的端口：

```
ls /dev/tty*
```

然后，在将 XIAO 连接到计算机并再次查询后，新增的串口名称即为 XIAO 的端口号。

记下该端口名称，因为接下来的步骤需要使用它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/12.png" style={{width:800, height:'auto'}}/></div>

如上所示，我的 XIAO 端口号应为 **ttyACM0**。

### 步骤 2. 启动一个项目

导航到示例目录：

```
cd ~/esp/esp-idf/examples/get-started/blink
```

设置目标设备：

```
idf.py set-target esp32c6
```

:::tip
- 如果您使用的是 **XIAO ESP32C3**，则需要使用命令 `idf.py set-target esp32c3`。
- 如果您使用的是 **XIAO ESP32S3**，则需要使用命令 `idf.py set-target esp32s3`。
:::

由于这个示例项目的主要效果是让板载 LED 闪烁，我们需要配置 LED 所在的 GPIO 引脚、闪烁时间和其他参数。ESP-IDF 提供了 `menuconfig` 命令来设置项目的一些可调参数。

```
idf.py menuconfig
```

该命令会打开一个配置菜单，您可以在其中修改各种设置。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/14.png" style={{width:800, height:'auto'}}/></div>

以 blink 项目为例，有三个可以调整的参数。

1. **Blink LED 类型**：设置使用的 LED 类型，在此我们设置为 **GPIO**。
2. **Blink GPIO 引脚编号**：设置 LED 所在的 GPIO 引脚编号，这里设置为 **15**。XIAO ESP32C6 的 LED 连接到 GPIO15。
3. **Blink 闪烁周期（毫秒）**：设置 LED 闪烁的时间间隔。默认是 **1000** 毫秒，即 1 秒。

设置完成后，按 **q** 退出设置菜单，再按 **y** 确认更改。

### 步骤 3. 构建并烧录示例程序

通过运行以下命令来构建项目：

```
idf.py build
```

该命令会编译应用程序和所有 ESP-IDF 组件，然后生成引导加载程序、分区表和应用程序二进制文件。如果没有错误，构建完成后会生成固件的 .bin 文件。

要将您刚才构建的二进制文件烧录到 ESP32 上，您需要运行以下命令：

```
idf.py -p PORT flash
```

将 `PORT` 替换为您的 XIAO ESP32 开发板的 USB 端口名称。如果 **PORT** 未定义，`idf.py` 会尝试使用可用的 USB 端口自动连接。根据我们在第一步查询的设备端口号，假设我的端口号是 **ttyACM0**，那么我应该输入以下命令来烧录程序：

```
idf.py -p /dev/ttyACM0 flash
```

如果烧录过程没有问题，XIAO 会重启并启动 "blink" 示例程序。

要查看 LED 示例程序的输出，运行以下命令：

```
idf.py monitor
```

您应该会看到 XIAO 上的 LED 闪烁，表明示例程序正在成功运行。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/13.png" style={{width:800, height:'auto'}}/></div>

如果您希望退出串口监控，可以使用快捷键 `Ctrl+]`。

:::tip
以下是 ESP-IDF 环境中常用的一些命令：

- `idf.py menuconfig`：打开项目配置菜单。
- `idf.py build`：构建项目。
- `idf.py flash`：将构建的固件烧录到连接的设备上。
- `idf.py monitor`：启动串口监控，查看设备的输出。
- `idf.py clean`：清理构建目录。
- `idf.py fullclean`：执行完全清理，包括下载的依赖项。
- `idf.py set-target`：设置项目的目标芯片。
- `idf.py size`：显示构建的固件的大小信息。
- `idf.py app`：管理项目中的应用程序。
- `idf.py component`：管理项目中的组件。
:::

恭喜！您已经成功在 Ubuntu 系统上安装了 ESP-IDF，为您的 Matter 开发之旅奠定了基础。Seeed Studio 将继续在 Matter 的背景下完善和扩展 XIAO ESP32 系列的开发文档。完成了 ESP-IDF 的安装和配置后，您现在可以继续进行 Matter 开发部分的工作。

Seeed Studio 致力于提供全面的资源和支持，帮助您顺利开展 Matter 开发工作。请关注即将发布的专门针对 XIAO ESP32 系列的 Matter 文档和教程。这些资源将指导您通过 ESP-IDF 框架，构建符合 Matter 协议的智能家居设备。

随着您深入探讨 Matter 开发，确保参考官方的 ESP-IDF 文档，获取框架功能和最佳实践的详细信息。与活跃的开发者和爱好者社区互动，交流知识、寻求指导，并在创新项目上进行合作。

借助 ESP-IDF 的强大功能以及 Seeed Studio 即将发布的 Matter 开发资源，您将能够创建出色的智能家居解决方案，突破互操作性和用户体验的边界。迎接未来的激动人心的可能性，自信地踏上您的 Matter 开发之旅。编程愉快！

## 故障排除

### 为什么我在安装环境时会遇到各种错误？

ESP-IDF 的环境要求较高，如果您使用的是经常用于开发的 Ubuntu 主机，可能会由于某些 Python 依赖库的版本不同而出现错误。由于 Matter 框架并非由 Seeed 开发，因此我们可能无法解决这部分问题。如果您在安装过程中遇到问题，建议您向官方 **[ESP-IDF 仓库](https://github.com/espressif/esp-idf)** 提交问题，寻求帮助。

## 资源

- **[ESPRESSIF IDF - 入门指南](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)**

## 技术支持与产品讨论

感谢您选择我们的产品！我们将为您提供各种支持，确保您在使用我们产品时的体验尽可能顺畅。我们提供了多种沟通渠道，以满足不同的需求和偏好。


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


