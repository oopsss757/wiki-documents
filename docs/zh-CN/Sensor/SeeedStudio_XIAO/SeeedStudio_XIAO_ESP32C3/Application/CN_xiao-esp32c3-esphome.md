---
description: 使用XIAO ESP32C3访问ESPHome
title: 应用 Home Assistant 实现屋内无摄像头监控
keywords:
- ESPHome
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao-esp32c3-esphome
last_update:
  date: 11/11/2024
  author: Agnes
---

# 将 XIAO ESP32C3 应用 Home Assistant 实现屋内无摄像头监控

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/78.jpg" style={{width:700, height:'auto'}}/></div>


本文将指导您在自己的Home Assistant环境中安装ESPHome服务。通过使用XIAO ESP32C3的WiFi功能，您将能够以非常平滑的方式连接您的XIAO与Home Assistant作为您的家庭终端的一部分。


此外，我们将结合目前最流行的24GHz毫米波人体静态存在模块Lite，构建一个具有人体存在检测的Home Assistant。


## 开始

:::tip
截至2023年7月31日，之前导致雷达完全死亡的问题现在已经修复，所以请更新这个教程物种的库文件和配置器以正常工作。
:::

如果您想跟随本教程完成所有内容，则需要准备以下内容。
<table align="center">
  <tbody><tr>
      <th>XIAO ESP32C3</th>
      <th>24GHz毫米波人体静态检测<br />存在感模块 Lite</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:100, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Radar_MR24HPCB1/0.jpg" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 点击购买 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/24GHz-mmWave-Sensor-Human-Static-Presence-Module-Lite-p-5524.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 点击购买  🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

该项目的最终目标是在Home Assistant中部署24GHz毫米波人体静态存在模块Lite。

我们为24GHz毫米波人体静态存在模块Lite编写了完整的配置文件和库，以方便您在此项目中快速部署传感器到Home Assistant。


本教程的内容大致包括以下步骤。

1. [选择您的HomeAssistant环境](#select-your-home-assistant-environment)
2. [在HomeAssistant中安装和配置]ESPHome(#install-and-configure-esphome-in-home-assistant)
3. [配置XIAO ESP32C3和ESPHome连接](#configure-the-xiao-esp32c3-and-esphome-connection)
4. [配置家庭辅助面板](#configure-home-assistant-panel)

当然，如果你对XIAO ESP32C3如何在Home Assistant中使用Grove感兴趣，可以直接阅读本章。


- [使用XIAO ESP32C3连接Grove到Home Assistant](#connect-grove-to-home-assistant-using-xiao-esp32c3)

本教程的内容大致包括以下步骤。

## 选择您的Home Assistant环境

在这个例程中，我们不会详细说明如何安装Home Assistant环境，我们将假设您已经有一个可用的Home Assistant设备。


如果你想学习如何安装Home Assistant，那么你可以参考[官方教程](https://www.home-assistant.io/installation/)。我们强烈建议您使用x86设备安装Home Assistant，因为这是使用Supervised安装Home Assistant最用户友好的方式。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/77.png" /></div>

根据上表，安装**Home Assistant OS**和**Home Assistant Supervised**是最合适的，这将为你省去很多麻烦。如果您在Docker上使用OpenWRT运行Home Assistant(例如使用LinkStar H68K)，请不要担心，我们还将为您提供如何做到这一点的详细参考。


我们也写了如何安装Home Assistant的一些Seeed工作室的产品，请参阅他们。

- [在ODYSSEY-X86上开始使用Home Assistant](https://wiki.seeedstudio.com/ODYSSEY-X86-Home-Assistant/)

- [在reTerminal上使用Home Assistant入门](https://wiki.seeedstudio.com/reTerminal_Home_Assistant/)

- [在LinkStar H68K/reRouter CM4上开始使用Home Assistant](https://wiki.seeedstudio.com/h68k-ha-esphome/)

## 在Home Assistant中安装ESPHome


### 第一步.安装ESPHome


- **场景1:在Home Assistant OS下安装ESPHome(带有插件商店)**


如果你安装了Home Assistant操作系统，它有一个插件商店，这使得安装ESPHome容易得多。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/79.png" /></div>


在插件商店中，你可以搜索并安装ESPHome。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/80.png" /></div>

- **场景2:ESPHome安装在OpenWRT Docker/Docker下的Home Assistant下(没有插件存储)**

由于我们正在安装家庭助手容器，我们不能简单地通过插件商店下载ESPHome服务，因此需要妥协。
我们需要下载ESPHome图像。

```
esphome / esphome:latest
```

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/17.png" /></div>


在创建容器的页面上，我们需要做一些简单的设置。

- 容器名称:容器名称

- Docker镜像:选择刚下载的**esphome**镜像

- 网络:选择**主机**模式

- 环境变量(-e):环境变量

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/18.png" /></div>


一旦您填写了以上内容，保存并申请。您将看到容器已经创建。你还需要启动它。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/19.png" /></div>


为了达到在Home Assistant中下载ESPHome的效果，我们需要修改Home Assistant下的配置文件。

到 Home Assistant处理器.

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/55.png" /></div>


我们去Home Assistant的终点站

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/56.png" /></div>


在终端输入以下命令。

```sh
vi configuration.yaml
```

将以下内容添加到`configuration.yaml`的末尾。

```
# Example configuration.yaml entry
panel_iframe:
  esphome:
    title: "ESPHome"
    url: "http://192.168.100.1:6052"
    icon: mdi:chip
```

在Home Assistant容器shell中输入 ```exit``` 退出docker容器。完成后，我们重新启动Home Assistant容器。

创建一个新的浏览器页面，输入地址`http://homeassistant:8123/` ，并输入您的Home Assistant帐户，您将看到ESPHome出现在左侧的工具栏中。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/57.png" /></div>

## 配置XIAO ESP32C3与ESPHome连接

### 步骤 2.硬件准备

我们教程的目标是能够在Home Assistant仪表板中查看24GHz毫米波人体静态存在模块Lite的数据信息。


通过主板连接设备和计算机。接线图如下表所示。

<div class="table-center">
<table align="center">
  <tbody>
    <tr>
      <td colspan="3"><div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/esphome-pinconnect.png" /></div></td>
    </tr>
    <tr>
      <td align="center">XIAO ESP32C3</td>
      <td align="center" />
      <td align="center">24GHz毫米波人体静态检测<br />存在感模块 Lite</td>
    </tr>
    <tr>
      <td align="center">5V</td>
      <td align="center">--&gt;</td>
      <td align="center">5V</td>
    </tr>
    <tr>
      <td align="center">GND</td>
      <td align="center">--&gt;</td>
      <td align="center">GND</td>
    </tr>
    <tr>
      <td align="center">D2</td>
      <td align="center">--&gt;</td>
      <td align="center">RX</td>
    </tr>
    <tr>
      <td align="center">D3</td>
      <td align="center">--&gt;</td>
      <td align="center">TX</td>
    </tr>
  </tbody></table>
</div>

### 步骤 3：将 XIAO ESP32C3 和 Home Assistant 保持在同一局域网中

我相信您的 Home Assistant 已经完成了联网工作，比如通过网线连接您的设备。接下来，您只需要启用一个本地网络（例如 WiFi），以便 XIAO ESP32C3 也能连接到该网络。

下面我将以 LinkStar H68K 为例，目标是让 XIAO 连接到 LinkStar H68K 的热点。

在 OpenWRT 的 **Network** 标签页中，选择 **Wireless** --> **ADD**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/58.png" /></div>


在 **Device Configuration** 中的 **Transmit Power** 设置为 **auto**。

对于 **Interface Configuration** 的设置，请按照以下说明填写。

- 一般设置
    - 模式：根据 LinkStar 访问互联网的方式选择。如果是通过有线连接，则选择 **Client**，如果是连接到 WiFi，则选择 **Access Point**。
    - ESSID: 输入您的 WiFi 名称，请尽量避免使用空格或特殊字符。
    - 网络: 勾选  **lan**.

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/23.png" /></div>


- 无线安全
    - 加密：WPA2-PSK
    - 密钥：输入您想设置的 WiFi 密码。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/24.png" /></div>



填写完以上信息后，点击右下角的 **Save and Apply** 并等待片刻，LinkStar 会启动一个热点。

当没有设备连接到这个热点时，热点会显示为没有信号。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/60.png" /></div>


一切就绪后，让我们返回到 Home Assistant 页面。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/61.png" /></div>


点击 **NEW DEVICE**。然后点击 **Continue**。 

在弹出的新窗口中，请输入您希望设置的应用名称，以及在 LinkStar 中设置的热点名称和密码（或者您自己的 WiFi）。确保 XIAO ESP32C3 和 Home Assistant 在 **同一局域网** 下。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/25.png" /></div>


然后点击 **Next**。

在设备类型中，选择 **ESP32-C3**。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/26.png" /></div>

然后点击 **Next**。

<span id="jump1">Click on the <strong>Encryption key</strong> and save it in a secure location, we will use this key in a later step.</span>

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/27.png" /></div>

然后点击 **SKIP**。

### 步骤 4：修改 XIAO ESP32C3 配置文件（yaml）

接下来，我们点击刚刚创建的设备标签，在左下角找到 **EDIT** 按钮。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/28.png" /></div>


请注意，我们需要修改这个 yaml 文件。我们将要修改的内容分为两部分，对应于下图中的 **①** 和 **②** 两个部分。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/62.png" /></div>


- 在 **①** 部分，除了您已经配置的设备名称外，请不要更改其他内容，其他配置请参考以下代码。

```css
# part 1:
substitutions:
  name: "xiao-esp32c3"
  friendly_name: "XIAO ESP32C3"

esphome:
  name: "${name}"
  friendly_name: "${friendly_name}"
  name_add_mac_suffix: true
  project:
    name: "seeedstudio.mmwave_kit"
    version: "2.0"
  platformio_options:
    board_build.flash_mode: dio
    board_build.mcu: esp32c3

external_components:
  - source: github://limengdu/mmwave-kit-external-components@main
    refresh: 0s

esp32:
  board: esp32-c3-devkitm-1
  variant: esp32c3
  framework:
    type: esp-idf

# Enable logging
logger:
  hardware_uart: USB_SERIAL_JTAG
  level: DEBUG
```

- 在 **②** 部分，复制以下代码并粘贴到 `captive_portal:` 后面。

<details>

<summary>Click here to preview the full code</summary>

```css
# 设置蓝牙 LE（仅适用于 ESP32），允许用户
# 向设备提供 Wi-Fi 凭据。
esp32_improv:
  authorizer: none

# 通过串行客户端设置 improv 进行 Wi-Fi 配置。
# 如果您的设备有 USB 端口，方便用户在首次使用时添加凭据。
# improv_serial: # 注释掉，直到 improv 与 usb-jtag 在 idf 上兼容

uart:
  id: uart_bus
  baud_rate: 115200
  rx_pin: 4
  tx_pin: 5
  parity: NONE
  stop_bits: 1

seeed_mr24hpc1:
  id: my_seeed_mr24hpc1

text_sensor:
  - platform: seeed_mr24hpc1
    heart_beat:
      name: "心跳"
    product_model:
      name: "产品型号"
    product_id:
      name: "产品 ID"
    hardware_model:
      name: "硬件型号"
    hardware_version:
      name: "硬件版本"
    keep_away:
      name: "主动报告接近"
    motion_status:
      name: "运动信息"
    custom_mode_end:
      name: "自定义模式状态"

binary_sensor:
  - platform: seeed_mr24hpc1
    someone_exist:
      name: "存在信息"

sensor:
  - platform: seeed_mr24hpc1
    custom_presence_of_detection:
      name: "静态距离"
    movement_signs:
      name: "人体运动参数"
    custom_motion_distance:
      name: "运动距离"
    custom_spatial_static_value:
      name: "存在能量"
    custom_spatial_motion_value:
      name: "运动能量"
    custom_motion_speed:
      name: "运动速度"
    custom_mode_num:
      name: "当前自定义模式"

switch:
  - platform: seeed_mr24hpc1
    underly_open_function:
      name: "底层开启功能信息输出开关"

button:
  - platform: seeed_mr24hpc1
    reset:
      name: "模块重置"
    custom_set_end:
      name: "自定义模式设置结束"

select:
  - platform: seeed_mr24hpc1
    scene_mode:
      name: "场景模式"
    unman_time:
      name: "进入无人状态的时间（标准功能）"
    existence_boundary:
      name: "存在边界"
    motion_boundary:
      name: "运动边界"

number:
  - platform: seeed_mr24hpc1
    sensitivity:
      name: "灵敏度"
    custom_mode:
      name: "自定义模式"
    existence_threshold:
      name: "存在能量阈值"
    motion_threshold:
      name: "运动能量阈值"
    motion_trigger:
      name: "运动触发时间"
    motion_to_rest:
      name: "运动到静止时间"
    custom_unman_time:
      name: "进入无人状态时间（底层开启功能）"
```

</details>


然后，请点击右上角的 **Save** 按钮。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/63.png" /></div>

### 第五步：上传固件到 XIAO ESP32C3

- **方法一：直接编译并上传**

如果您使用的是 x86 设备并且能够看到 XIAO 出现在设备端口中，那么您可以直接编译并将程序上传到 XIAO。

将 XIAO 连接到您的设备。

<div align="center"><img src="https://files.seeedstudio.com/wiki/ESPHome/49.png" style={{width:700, height:'auto'}}/></div>


点击设备栏右下角的三个点，选择 **安装**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/84.png" /></div>


点击 **插入到运行 ESPHome 仪表盘的计算机**。

<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/85.png" /></div>


选择连接的端口。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/ESPHome/18.png" /></div>

现在，它将下载所有必要的板卡包并将 ESPHome 固件刷入 XIAO ESP32C3。如果刷写成功，您将看到以下输出。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/ESPHome/19.png" /></div>

如果在连接 XIAO 到设备后无法找到端口，您可以尝试使用第二种方法。

- **方法二：使用主机上传已编译的固件**

像 LinkStar H68K 这样的软路由不支持识别外部 MCU 设备，我们需要先下载已编译的固件，然后通过另一台 PC 上传固件。

点击右上角的 **安装** 按钮。然后选择最后一个项 **手动下载**。

<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/30.png" /></div>


选择 **现代格式**。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/31.png" /></div>

然后，它将花费较长时间进行下载和编译，请耐心等待。准备好后，固件将自动下载到您的计算机。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/33.png" /></div>

要将固件上传到 XIAO ESP32C3，有几种选择，我们展示了两种方法：

- 选项 1：使用 [ESPhome Web 工具](https://web.esphome.io/?dashboard_install) 工具 上传。

确保已安装正确的驱动程序。以下是 ESP 设备常用芯片的驱动程序。

1. CP2102 驱动程序： [Windows & Mac](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)

2. CH342, CH343, CH9102 驱动程序： [Windowns](https://www.wch.cn/downloads/CH343SER_ZIP.html), [Mac](https://www.wch.cn/downloads/CH34XSER_MAC_ZIP.html)

3. CH340, CH341 驱动程序： [Windowns](https://www.wch.cn/downloads/CH341SER_ZIP.html), [Mac](https://www.wch.cn/downloads/CH341SER_MAC_ZIP.html)

在 Chrome 或 Edge 浏览器中打开 [ESPhome Web 工具](https://web.esphome.io/?dashboard_install)。

点击 **连接**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/34.png" /></div>

在弹出的窗口中选择 XIAO ESP32 的串口。

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/64.png" /></div>

点击 **安装**，然后选择从上述步骤下载的 `.bin` 文件。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/35.png" /></div>

<div align="center"><img width={400} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/38.png" /></div>


- 选项 2：使用 [esphome-flasher 工具](https://github.com/esphome/esphome-flasher) 工具。 

如果在安装驱动程序并更换浏览器后仍然无法使用方法一上传固件，您可以尝试使用方法二。请参考官方教程，了解具体的安装方法和操作说明。

:::tip
如果您希望查看 XIAO ESP32C3 的日志消息，您也可以通过该软件的 View Logs 按钮来查看。

<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/41.png" /></div>
:::

上传完成后，您可以断开 XIAO ESP32C3 与 PC 的连接（除非您需要查看日志），然后单独为 XIAO 提供电源。

如果一切顺利，XIAO ESP32C3 将搜索并连接到您为其设置的 Wi-Fi。

就像我一样，我使用 LinkStar H68K 的网络。您可以在网络选项中找到它，并看到 LinkStar H68K 分配给它的 IP 地址。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/42.png" /></div>

通常，此时在 Home Assistant 中，设备的状态也会从离线变为在线。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/65.png" /></div>

## 配置 Home Assistant 面板

### 第六步：连接 XIAO ESP32C3

如果您的局域网中没有太多 Home Assistant 设备，Home Assistant 可以自动搜索并将您的 ESP 设备添加到设备标签中。您可以在 **设置** 中的 **设备与服务** 标签页中看到此设备。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/66.png" /></div>

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/67.png" /></div>

如果它没有自动搜索，您也可以通过 IP 地址连接到 XIAO ESP32C3。

点击 **添加集成** 并搜索 **esphome**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/43.png" /></div>

然后输入 XIAO ESP32C3 的 IP 地址，端口号为 **6053**，然后点击 **提交**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/44.png" /></div>

如果输入的 IP 地址和端口号正确，您将看到提示输入加密密钥，这个密钥是我们在 [第4步](#jump1) 中保存的。

然后点击 **提交**。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/68.png" /></div>

此时，设备添加的步骤已经成功完成。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/51.png" /></div>

### 第七步：24GHz mmWave 模块 Lite 功能概述

为了帮助您快速了解该套件的完整功能和如何使用这些功能，您需要仔细阅读这一部分。如果您想要更详细的信息，我们建议您花时间阅读 [ 产品用户手册](https://files.seeedstudio.com/wiki/mmWave-radar/MR24HPC1_User_Manual-V1.5.pdf).

关于仪表板的配置和参数的详细信息，我们在 ESPHome 文档中整理了详细的介绍，请点击链接阅读完整的文档和详细信息。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://deploy-preview-3383--esphome.netlify.app/components/sensor/seeed_mr24hpc1">
            <strong><span><font color={'FFFFFF'} size={"4"}>ESPHome 文档 📕</font></span></strong>
    </a>
</div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave_kit/49.png" style={{width:700, height:'auto'}}/></div>

### 第八步：配置 Home Assistant 面板

如果您觉得默认的卡片展示数据非常枯燥且不友好，Home Assistant 提供了多种现成的仪表板供您选择。

您可以创建一个适合您个人偏好的仪表板。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/73.png" /></div>

例如，将信息输出的控制选项变成一个漂亮的开关。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/74.png" /></div>

将人体运动的速度转换成可视化的仪表板展示。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/75.png" /></div>

这就是我设计的内容，看起来像是一个智能家居控制中心。

<div align="center"><img width={800} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/76.png" /></div>

到目前为止，我们已经成功完成了教程的内容。

## 使用 XIAO ESP32C3 连接 Grove 到 Home Assistant

当然，XIAO ESP32C3 不仅仅支持将 24GHz mmWave 人体静态存在模块 Lite 集成到 Home Assistant，您可以在本文件中找到更多适用于您自己的教程。

- [使用 XIAO ESP32C3 连接 Grove 到 Home Assistant](https://wiki.seeedstudio.com/Connect-Grove-to-Home-Assistant-ESPHome/)

让您的创意发挥出来吧！

## 故障排除

### 常见问题 1：在使用 ESPhome Web 工具上传固件时出现以下错误，我该如何修复？

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/37.png" /></div>

> 回答: 如果上传时出现此提示，断开 XIAO ESP32C3 与 PC 的连接。然后，在按住 BOOT 按钮的同时，将板卡连接到 PC，接着释放 BOOT 按钮，进入引导加载程序模式。此时，只需重新连接并再次上传固件即可。

### 常见问题 2：按照 esphome flasher 的教程，我在 Linux 下无法安装 esphome flasher？

> 回答:在执行以下命令时，您需要选择适合您的系统版本，否则会出现错误。例如，如果我的计算机是 Ubuntu 22.04，那么应该执行如下命令。

```
sudo apt install python3

pip3 install -U \
    -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-22.04/ \
    wxPython

pip3 install esphomeflasher
```

### 常见问题 3：我填写了正确的 WiFi 和密码，为什么看不到 XIAO ESP32C3 的 IP 地址？

> 回答: 当遇到此问题时，请检查 XIAO ESP32C3 的天线是否连接到位。如果天线已连接，请确保 XIAO 与 LinkStar 的距离不超过 3 米（除非您已更换了更强大的天线）。如果仍然无法看到 XIAO，可以使用 esphome flasher 软件查看 XIAO 的日志信息，并通过日志检查 XIAO 的连接状态。您可以重新插拔 XIAO 尝试重新搜索 WiFi 并连接。

<!-- ### 常见问题 4：我的 XIAO ESP32C3 已连接到网络，但为什么看不到传感器数据刷新？

:::caution
截至 2023 年 6 月 1 日，故障排除表明，如果您在 ESPHome 仪表板中设置任何值或更改任何场景，可能会导致雷达停止工作。 
截至 2023 年 7 月 31 日，之前会导致雷达完全停止工作的漏洞已经修复，因此请更新库文件和配置器，以确保本教程中的内容正常运行。
:::

> A: 在之前的 Wiki 内容中，我们使用了默认的 UART 引脚（D6、D7）来接收和发送雷达数据，但许多用户反馈，需要重新给雷达供电才能使其正常工作。对此，我们已 **更新了 Wiki** 内容和步骤，将雷达的串口更换为 **D2 和 D3**，经过测试，这解决了问题。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/esphome-pinconnect.png" style={{width:600, height:'auto'}}/></div>

> **如果您没有注意到 Wiki 的更新，建议您重新接线雷达，并按照本文教程的 [第 2 步和第 5 步](#configure-the-xiao-esp32c3-and-esphome-connection) 重新编译和上传固件。**

> 然而，也有部分用户反馈，即使更换了串口引脚，雷达仍然无法正常工作。在这种情况下，我们提出以下方法和步骤来检查问题所在，如果您仍然无法解决雷达工作问题， **请提供您的操作步骤给技术支持邮箱**, 这样可以加速售后问题的处理。

**请按照以下排查顺序检查。**

> **排查 1：确保 XIAO ESP32C3 与 ESPHome 部署设备处于同一局域网。**

> 如果 XIAO ESP32C3 不与 ESPHome 设备处于同一局域网，您在 Home Assistant 中看到的日志将不完整，无法作为数据收集的依据。请再次检查您的路由器，看看 XIAO 的 IP 地址是否出现。

> **排查 2：检查是否开启数据实时传输按钮。**

> 当 XIAO 连接到网络并成功添加设备后，您将在仪表板中看到雷达组件。请注意，默认情况下，实时数据传输按钮是关闭的，您需要开启它才能看到雷达数据的持续报告。

<div align="center"><img width={500} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/69.png" /></div>

> **排查 3：检查雷达是否正常工作。**

> 我们需要首先确保雷达与 XIAO ESP32C3 正常工作，这将帮助我们快速判断问题是出在 ESPHome 还是产品上。请在 Arduino IDE 中将以下代码上传至 XIAO ESP32C3，请注意 **雷达的 RX/TX 引脚应连接到 XIAO 的 D2/D3**。

```cpp
#include "Arduino.h"
#include <humanstaticLite.h>
#include <HardwareSerial.h>

// 也可以尝试使用硬件串口
HumanStaticLite radar = HumanStaticLite(&Serial1);

void setup() {
  // 在此放置设置代码，仅运行一次：
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 4, 5);
  while(!Serial);   // 当串口打开时，程序开始执行
  Serial.println("Ready");
}

void loop() {
  // 在此放置主循环代码，重复运行：
  radar.recvRadarBytes();           // 接收雷达数据并开始处理
  radar.showData();                 // 串口打印接收到的数据帧
  delay(200);                       // 添加时间延迟以避免程序卡住
}
```

> 打开串口监视器并将波特率设置为115200，如果雷达正常工作，则应该看到大量数字输出。

>如果您在执行此步骤时没有看到任何数据输出，请根据 Wiki 中的说明重新刷写雷达的固件。我们为您提供了两种更新固件的方式： [固件版本更新](https://wiki.seeedstudio.com/Radar_MR24HPC1/#firmware-version-updates).

> 如果您更新了固件后仍然没有任何输出，请不要犹豫，直接联系技术支持团队，并告知他们您已经执行的所有操作。

> **排除 4：XIAO 和雷达在上述检查点正常工作，但更换串口引脚后，仍然无法获取雷达实时数据。**

> 如果您已经将雷达的 RX 和 TX 引脚更改为 D2/D3，并且按照上述步骤仔细排查，仍然无法获取实时数据，请联系技术支持团队。在此之前，**请告诉我们雷达在 Arduino 环境中是否正常工作**，这样我们可以帮助分析和解决问题。 -->

<!-- > 回答: 遇到此问题时，我们需要通过日志来了解传感器未返回数据的确切原因。到目前为止，发现可能遇到的情况是传感器没有响应，此时其日志通常如下所示：

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/71.png" /></div>

> 如果您看到类似的日志，请再次检查以下三个地方：
> 1. 是否给传感器提供了 5V 电源。
> 2. 传感器的 RX 和 TX 引脚是否正确连接。
> 3. 断开传感器与 XIAO 之间的 5V 线，并重新连接，以确保传感器重新供电。

> 通常情况下，第三点可以解决问题。正常的数据传输日志应该如下所示：

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/homs-xiaoc3-linkstar/72.png" /></div> -->

### 常见问题 4：我使用 Jlink 刷写固件时，出现了“编程范围 @address 0x08000000 失败（块验证错误）程序失败，无法编程和验证目标”错误，怎么办？

当您使用 Jlink 刷写固件并出现此错误时，可能是以下情况之一。

1. 您的传感器完全无法正常工作，无法接收到任何消息。
2. 您正在尝试使用无效或不正确的固件。

:::caution
如果您的雷达最初是正常工作的，请再次检查您使用的是正确的固件！不同的雷达和传感器型号使用不同的固件！通过 UART 升级固件与通过 Jlink 升级固件的方式不同！请停止执行以下步骤。
:::

<details>

<summary><strong>我已经确认我的产品在异常情况下显示此错误消息</strong></summary>

如果您的雷达完全无法工作，那么出现此错误消息可能是正常的。

因为雷达的异常操作已经触发了读写保护机制，通常情况下不允许用户再对产品进行刷写程序，所以我们需要解锁雷达的读写保护机制。

由于解锁读写保护存在较高风险，我们不会在此公开解锁方法，而是将该方法放在 [此处的ZIP 文件](https://files.seeedstudio.com/wiki/Radar_MR24HPCB1/ArteryICPProgrammer_V2.4.23.zip)，供需要的用户使用。一旦异常的雷达被解保护，固件就可以重新更新，以恢复正常工作。

</details>

## 技术支持与产品讨论



感谢您选择我们的产品！我们致力于为您提供不同的支持，确保您使用我们的产品体验顺利。我们提供多种沟通渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>


