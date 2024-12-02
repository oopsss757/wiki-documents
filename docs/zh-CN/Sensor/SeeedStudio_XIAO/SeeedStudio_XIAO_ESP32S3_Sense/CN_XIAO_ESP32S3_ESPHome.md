---
description: ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3
title: Seeed Studio 上的 ESPHome 支持 XIAO ESP32S3
keywords:
- ESPHome
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/XIAO_ESP32S3_esphome
last_update:
  date: 11/27/2024
  author: Agnes
---

# XIAO ESP32S3 感应器通过 ESPHome 连接到 Home Assistant（支持所有端口）

本 Wiki 将逐步介绍如何将 [Seeed Studio XIAO ESP32S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) 与运行在 Home Assistant 上的 ESPHome 连接，并在连接 Grove 模块到 XIAO ESP32S3 后，发送传感器数据或控制设备。那么，开始吧！

## 什么是 ESPHome 和 Home Assistant？

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/2.png" style={{width:600, height:'auto'}}/></div>
<br />

[ESPHome](https://esphome.io/) 是一款旨在使管理 ESP 开发板变得尽可能简单的工具。它读取 YAML 配置文件并创建自定义固件，安装到 ESP 设备上。ESPHome 配置中添加的设备或传感器会自动出现在 Home Assistant 的用户界面中。ESPHome 可以帮助您连接并将数据发送到 Home Assistant 设备。

## 硬件准备

如果您想跟随本教程完成所有步骤，您需要准备以下硬件。

<table align="center">
  <tbody><tr>
      <th>Seeed Studio XIAO ESP32S3 Sense</th>
      <th>Seeed Studio 扩展板</th>
      <th>Home Assistant 设备</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3sense.jpg" style={{width:300, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Home-Assistant/1.png" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://wiki.seeedstudio.com/home_assistant_topic/#-devices-for-home-assistant-">
            <strong><span><font color={'FFFFFF'} size={"4"}> 查看更多 🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

#### 使用的传感器

- [Grove - 温湿度传感器 (BME680)](https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-for-Arduino-BME680.html)<br />
- [Grove - 智能空气质量传感器 (SGP41)](https://www.seeedstudio.com/Grove-Air-Quality-Sensor-SGP41-p-5687.html)<br />
- [6x10 RGB MATRIX for XIAO](https://www.seeedstudio.com/6x10-RGB-MATRIX-for-XIAO-p-5771.html#)

## 软件准备

### 安装 Home Assistant

确保您已经安装并运行了 Home Assistant。您可以按照 [此 Wiki](https://wiki.seeedstudio.com/ODYSSEY-X86-Home-Assistant) 上的步骤，在 ODYSSEY-X86 SBC 上安装 Home Assistant，或者参照 [这个链接](https://www.mbreviews.com/how-to-home-assistant-seeed-mini-router/) 获取使用 Seeed Mini Router 配置 Home Assistant 的详细说明。

### 在 Home Assistant 上安装 ESPHome

ESPHome 可以作为 **Home Assistant 的附加组件** 安装，可以通过附加组件商店轻松安装。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/3.png" width="700" />  -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/1.png" style={{width:900, height:'auto'}}/></div>

<!--  
- **第1步。** 快速设置 ESPHome，点击下方按钮

<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/4.png" width="300">

- **第2步。** 当您看到以下弹出窗口时，点击 **OPEN LINK**

<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/5.png" width="300">

-->

- **第1步。** 点击 **安装**

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/2.png" style={{width:900, height:'auto'}}/></div>

- **第2步。** 启用所有选项并点击 **启动**

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/3.png" style={{width:900, height:'auto'}}/></div>

<!-- 
- **第3步。** 点击 **安装**

<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/6.png" width="700">

- **第4步。** 启用所有选项并点击 **启动**

<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/7.png" width="700">

- **第5步。** 点击 **打开 Web UI** 或 **侧边栏中的 ESPHOME**

<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/8.png" width="700">

-->
<br />
如果 ESPHome 成功加载，您将看到以下窗口：

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/9.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/4.png" style={{width:900, height:'auto'}}/></div>


## 开始使用

一旦所有的软件和硬件准备就绪，我们就可以开始了。

### 1. 将 Seeed Studio XIAO ESP32S3 (Sense) 添加到 ESPHome

- **第1步。** 点击 **+ 新设备**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/10.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/5.png" style={{width:900, height:'auto'}}/></div>

- **第2步。** 点击 **继续**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/11.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/6.png" style={{width:900, height:'auto'}}/></div>

- **第3步。** 输入设备的 **名称** 并输入 WiFi 凭据，例如 **网络名称** 和 **密码**。然后点击 **下一步**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/12.png" width="300"> 1.png-->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/1.png" style={{width:400, height:'auto'}}/></div>

- **第4步。** 选择 **ESP32-S3** 并点击

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/13.png" width="300"> 2.png-->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/2.png" style={{width:400, height:'auto'}}/></div>

- **第5步。** 点击 **跳过**，因为我们将手动配置此开发板

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/14.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/14.png" style={{width:400, height:'auto'}}/></div>

- **第6步。** 点击新创建的开发板下的 **编辑**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/15.png" width="300"> 3.png-->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/3.png" style={{width:400, height:'auto'}}/></div>

- **第7步。** 这将打开一个 **YAML** 文件，使用该文件设置所有开发板的配置。按如下方式编辑 **esp32** 下的内容：

```
esphome:
  name: esp32s3
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino


# 启用日志
logger:

# 启用 Home Assistant API
api:

ota:

wifi:
  ssid: "你的WiFi名称"
  password: "你的WiFi密码"

  # 在 WiFi 连接失败时启用回退热点（强制门户）
  ap:
    ssid: "Xiao-Esp32s3 回退热点"
    password: "MoLTqZUvHwWI"

```

**注意：** 这里我们使用的是 [最新版本的 Arduino 核心](https://github.com/espressif/arduino-esp32/releases) 和 [ESP32 对 PlatformIO 的支持](https://github.com/platformio/platform-espressif32/releases)

- **第8步。** 点击 **保存**，然后点击 **安装**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/16.png" width="700"> 4.png-->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/4.png" style={{width:900, height:'auto'}}/></div>

- **第9步。** 将一端的 USB Type-C 电缆连接到 Seeed Studio XIAO ESP32S3，另一端连接到 reRouter CM4 1432 的 USB 端口之一

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/17.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/17.png" style={{width:900, height:'auto'}}/></div>

- **第10步。** 点击 **连接到运行 ESPHome 仪表板的计算机**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/18.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/5.png" style={{width:900, height:'auto'}}/></div>

- **第11步。** 选择连接的端口。它可能是 ```/dev/ttyACM1```，因为 ```/dev/ttyACM0``` 已连接到 reRouter CM4 1432

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/19.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/6.png" style={{width:900, height:'auto'}}/></div>

:::tip 
最好使用 2.4GHz Wi-Fi
:::

现在，它将下载所有必要的开发板包并将 ESPHome 固件闪存到 XIAO ESP32S3。如果闪存成功，你将看到以下输出。如果看到错误信息，请尝试重新启动 XIAO ESP32S3 或通过按住 BOOT 按钮进入引导加载程序模式并连接 XIAO ESP32S3。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/20.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/7.png" style={{width:900, height:'auto'}}/></div>

- **第12步。** 上述窗口显示了来自连接开发板的实时日志。点击 **停止** 关闭它。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/21.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/8.png" style={{width:900, height:'auto'}}/></div>

- **第13步。** 如果你看到开发板状态为 **在线**，这意味着开发板已成功连接到 Wi-Fi

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/9.png" style={{width:400, height:'auto'}}/></div>

:::tip

现在，你可以将 XIAO ESP32S3 从 reRouter CM4 1432 上断开，只通过 USB 电缆供电。因为从现在开始，如果你想给 XIAO ESP32S3 刷写固件，只需通过 OTA 进行，而无需通过 USB 电缆连接到 X86 开发板。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/22.png" width="300"> -->

- 1. 点击 **三个点** 并选择 **安装**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/23.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/10.png" style={{width:400, height:'auto'}}/></div>

- 2. 选择 **无线**，它将通过无线将更改推送到开发板

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/24.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/11.png" style={{width:400, height:'auto'}}/></div>

:::

- **第14步。** 进入 **设置** 并选择 **设备与服务**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/25.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-img/19.png" style={{width:900, height:'auto'}}/></div>

- **第15步。** 你将看到 **ESPHome** 作为已发现的集成。点击 **配置**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/26.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/12.png" style={{width:900, height:'auto'}}/></div>

- **第16步。** 点击 **提交**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/27.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/13.png" style={{width:900, height:'auto'}}/></div>

- **第17步。** 点击 **完成**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/28.png" width="300"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/14.png" style={{width:400, height:'auto'}}/></div>

### 2. 将 Grove 模块与 ESPHome 和 Home Assistant 连接

现在我们将把 Grove 模块连接到 Seeed Studio XIAO ESP32S3 (Sense)，这样我们就可以使用 Home Assistant 显示传感器数据或控制设备！

### 开发知识

#### XIAO 扩展板

为了使用 Grove 模块与 Seeed Studio XIAO ESP32S3，我们将使用 [Seeed Studio XIAO 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) 并将 XIAO ESP32S3 连接到它上面。

之后，板上的 Grove 连接器可以用来连接 Grove 模块。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/29.png"style={{width:700, height:'auto'}}/></div>

#### 引脚定义

你需要按照下图来使用正确的内部引脚编号，当你将 Grove 模块连接到 Seeed Studio XIAO 的 Grove 扩展板连接器时。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/Figures/pinout.png" width="1000"> -->

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/2.jpg"style={{width:900, height:'auto'}}/></div>
<br />

例如，如果你想将一个 Grove 模块连接到 D0 端口，你需要在 ESPHome 中将引脚定义为 GPIO1。你可以通过 [点击这里](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#resources) 找到更多详细信息。

#### Grove 模块与 ESPHome 的兼容性列表

目前以下 Grove 模块已被 ESPHome 支持：

查看 [这里](https://esphome.io/components/sensor/index.html#see-also)

现在，我们将从上表中选择 6 个 Grove 模块，并解释它们如何与 ESPHome 和 Home Assistant 连接。

### 3. Grove 连接与数据传输

现在我们将选择几个 Grove 模块，并解释它们如何与 ESPHome 和 Home Assistant 连接。

#### Grove - 温湿度传感器 (BME680)

##### 配置设置

- **第1步。** 将 Grove - [温湿度、气压和气体传感器 (BME680)](https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-for-Arduino-BME680.html) 连接到 Seeed Studio XIAO 扩展板上的一个 I2C 连接器。

- **第2步。** 在之前创建的 **xiao-esp32s3-bme680.yaml** 文件中，修改文件内容并通过 OTA 将其推送到 XIAO ESP32S3。

```
# ESPHome 配置
esphome:
  # ESP32-S3 设备名称
  name: esp32s3
  
  # PlatformIO 构建选项
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

# ESP32 配置
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

# 启用日志
logger:

# 启用 Home Assistant API
api:

# 配置 OTA 更新
ota:

# Wi-Fi 配置
wifi:
  ssid: "你的 Wi-Fi 名称"
  password: "你的密码"

  # 如果 Wi-Fi 连接失败，启用备用热点 (强制门户)
  ap:
    ssid: "Xiao-Esp32s3 备用热点"
    password: "MoLTqZUvHwWI"

# 强制门户配置
captive_portal:

# BME680 传感器的 I2C 配置
i2c:
  sda: GPIO6
  scl: GPIO7

# BME680 传感器配置
sensor:
  - platform: bme680
    temperature:
      name: "BME680 温度"
      oversampling: 16x
    pressure:
      name: "BME680 气压"
    humidity:
      name: "BME680 湿度"
    gas_resistance:
      name: "BME680 气体阻力"
    address: 0x76
    update_interval: 60s
```

你可以在这里了解更多关于 [BME680 组件](https://esphome.io/components/sensor/bme680) 的信息。它允许你使用基于 BME280、BME680、BMP085、BMP280、AHT10、AHT20 和 AHT21 的传感器。由于 AHT20 使用 I2C 协议通信，我们在此添加了 I²C 总线组件。

##### 在仪表盘上可视化

- **第1步。** 在 Home Assistant 的概览页面，点击 3 个点并选择 **编辑仪表盘**。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/31.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/15.png" style={{width:900, height:'auto'}}/></div>

- **第2步。** 点击 **+ 添加卡片**。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/32.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/16.png" style={{width:900, height:'auto'}}/></div>

- **第3步。** 选择 **按实体**，输入 **temperature** 并选择 **温度** 旁边的 **复选框**。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/33.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/17.png" style={{width:900, height:'auto'}}/></div>

- **第4步。** 对 **湿度**、**气体阻力** 和 **气压** 重复相同的操作。

- **第5步。** 点击 **继续**。

- **第6步。** 点击 **添加到仪表盘**。

现在，你的 Home Assistant 仪表盘应该如下所示：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/18.png" style={{width:900, height:'auto'}}/></div>

- **第7步。** 你还可以将传感器数据可视化为仪表盘。点击 **仪表** 选项，位于 **按卡片** 下。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/35.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/19.png" style={{width:900, height:'auto'}}/></div>

- **第8步。** 从下拉菜单中选择 **温度**。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/36.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/20.png" style={{width:900, height:'auto'}}/></div>

- **第9步。** 点击 **保存**。

- **第10步。** 对 **湿度**、**气体阻力** 和 **气压** 重复相同的操作。

现在，你的仪表盘应该如下所示：

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/37.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/21.png" style={{width:900, height:'auto'}}/></div>

<br />

#### Grove - 智能空气质量传感器 (SGP41)

- **第1步。** 将 Grove - [智能空气质量传感器 (SGP41)](https://www.seeedstudio.com/Grove-Air-Quality-Sensor-SGP41-p-5687.html?queryID=3ac9c3a1ed9e1a56a66b142e8282868a&objectID=5687&indexName=bazaar_retailer_products) 连接到 Seeed Studio XIAO 扩展板上的一个 I2C 连接器。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/38.jpg" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/22.jpg" style={{width:900, height:'auto'}}/></div>

- **第2步。** 在之前创建的 **xiao-esp32S3.yaml** 文件中，修改文件内容并通过 OTA 将其推送到 XIAO ESP32S3。

```
# ESPHome 配置
esphome:
  # ESP32-S3 设备名称
  name: esp32s3
  
  # PlatformIO 构建选项
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

# ESP32 配置
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

# 启用日志
logger:

# 启用 Home Assistant API
api:

# 配置 OTA 更新
ota:

# Wi-Fi 配置
wifi:
  ssid: "你的 Wi-Fi 名称"
  password: "你的密码"

  # 如果 Wi-Fi 连接失败，启用备用热点 (强制门户)
  ap:
    ssid: "Xiao-Esp32s3 备用热点"
    password: "MoLTqZUvHwWI"

# 强制门户配置
captive_portal:

# SPI 配置
spi:
  clk_pin: GPIO8
  mosi_pin: GPIO10
  miso_pin: GPIO9

# BME680 传感器的 I2C 配置
i2c:
  sda: GPIO6
  scl: GPIO7
  scan: True
  id: bus_a
  frequency: 1MHz

# SGP4X 传感器配置
sensor:
  - platform: sgp4x
    voc:
      id: sgp41_voc
      name: "VOC 指数"
    nox:
      id: sgp41_nox
      name: "NOx 指数"
```

- **第3步。** 补偿示例  
补偿 (可选)：包含用于补偿的传感器的块。如果未设置，将使用默认值。  
我们将使用温湿度传感器 (BME680) 来补偿智能空气质量传感器 (SGP41)。  
这是更新后的 **xiao-esp32S3.yaml** 文件：

```
# ESPHome 配置
esphome:
  # ESP32-S3 设备名称
  name: esp32s3
  
  # PlatformIO 构建选项
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

# ESP32 配置
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

# 启用日志
logger:

# 启用 Home Assistant API
api:

# 配置 OTA 更新
ota:

# Wi-Fi 配置
wifi:
  ssid: "你的 Wi-Fi 名称"
  password: "你的密码"

  # 如果 Wi-Fi 连接失败，启用备用热点 (强制门户)
  ap:
    ssid: "Xiao-Esp32s3 备用热点"
    password: "MoLTqZUvHwWI"

# 强制门户配置
captive_portal:

# SPI 配置
spi:
  clk_pin: GPIO8
  mosi_pin: GPIO10
  miso_pin: GPIO9

# BME680 传感器的 I2C 配置
i2c:
  sda: GPIO6
  scl: GPIO7
  scan: True
  id: bus_a
  frequency: 1MHz

# BME680 传感器配置
sensor:
  - platform: bme680
    temperature:
      id: bme680_temp
      name: "BME680 温度"
      oversampling: 16x
    pressure:
      name: "BME680 压力"
    humidity:
      id: bme680_hum
      name: "BME680 湿度"
    gas_resistance:
      name: "BME680 气体阻力"
    address: 0x76

# SGP4X 传感器配置
  - platform: sgp4x
    voc:
      name: "VOC 指数"
    nox:
      name: "NOx 指数"
    compensation:
      humidity_source: bme680_hum
      temperature_source: bme680_temp
```

**注意：** 这个传感器需要 90 个周期来收集足够的数据样本，目前无法避免警告。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/38.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/23.png" style={{width:900, height:'auto'}}/></div>

##### 在仪表盘上可视化

和之前一样。
<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/43.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/24.png" style={{width:900, height:'auto'}}/></div>
<br />

#### OV2640 摄像头 (XIAO ESP32S3 Sense)

##### 配置步骤

- **第1步：** 将 OV2640 摄像头外部板连接到 XIAO ESP32S3 Sense

- **第2步：** 在之前创建的 **xiao-esp32s3-camera.yaml** 文件中，修改文件并通过 OTA 推送到 XIAO ESP32S3 Sense
```
# ESPHome 配置
esphome:
  # ESP32-S3 设备名称
  name: esp32s3
  
  # PlatformIO 构建选项
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

# ESP32 配置
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

# 启用日志
logger:

# 启用 Home Assistant API
api:

# 配置 OTA 更新
ota:

# Wi-Fi 配置
wifi:
  ssid: "你的 Wi-Fi 名称"
  password: "你的密码"

  # 如果 Wi-Fi 连接失败，启用备用热点 (强制门户)
  ap:
    ssid: "Xiao-Esp32s3 备用热点"
    password: "MoLTqZUvHwWI"

# 强制门户配置
captive_portal:

# ESP32 摄像头配置
esp32_camera:
  id: espcam
  name: 我的摄像头
  external_clock:
    pin: GPIO10
    frequency: 20MHz
  i2c_pins:
    sda: GPIO40
    scl: GPIO39
  data_pins: [GPIO15, GPIO17, GPIO18, GPIO16, GPIO14, GPIO12, GPIO11, GPIO48]
  vsync_pin: GPIO38
  href_pin: GPIO47
  pixel_clock_pin: GPIO13
  resolution: 800x600
  
# ESP32 摄像头 Web 服务器配置
esp32_camera_web_server:
  - port: 8080
    mode: stream
  - port: 8081
    mode: snapshot
```

**注意**：有关更多信息，请阅读 [这里](https://esphome.io/components/esp32_camera.html?highlight=camera)

##### 在仪表盘上可视化

- **第1步：** 在 Home Assistant 的概览页面，点击 3 个点，选择 **编辑仪表盘**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/31.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/15.png" style={{width:900, height:'auto'}}/></div>

- **第2步：** 点击 **+ 添加卡片**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/32.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/16.png" style={{width:900, height:'auto'}}/></div>

- **第3步：** 选择 **通过实体 (By ENTITY)**，输入 **Camera**，然后选择 **我的摄像头**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/cameravisulization.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/25.png" style={{width:900, height:'auto'}}/></div>

- **第4步：** 点击 **添加到仪表盘**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/cameravisulization2.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/26.png" style={{width:400, height:'auto'}}/></div>

- **第5步：** 当在仪表盘上查看网页流时，它处于空闲模式，每分钟只刷新几帧。点击卡片后，它会转为活动模式，通常刷新率为每秒 1 到 10 帧。在本教程中，刷新率大约为每秒 4 帧。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/camera.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/27.png" style={{width:400, height:'auto'}}/></div>

#### 用于语音助手的 PDM 麦克风

##### 配置步骤

- **第1步：** 在之前创建的 **xiao-esp32s3-microphone.yaml** 文件中，修改文件并通过 OTA 推送到 XIAO ESP32S3 Sense
```
# ESPHome 配置
esphome:
  name: esp32s3
  platformio_options:
    build_flags: -DBOARD_HAS_PSRAM
    board_build.arduino.memory_type: qio_opi
    board_build.f_flash: 80000000L
    board_build.flash_mode: qio 

# ESP32 配置
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

# 启用日志
logger:

# 启用 Home Assistant API
api:

# 配置 OTA 更新
ota:

# Wi-Fi 配置
wifi:
  ssid: "你的 Wi-Fi 名称"
  password: "你的密码"

  # 如果 Wi-Fi 连接失败，启用备用热点 (强制门户)
  ap:
    ssid: "Xiao-Esp32s3 备用热点"
    password: "MoLTqZUvHwWI"

# 强制门户配置
captive_portal:

# 状态 LED 灯配置
light:
  - platform: status_led
    id: light0
    name: "语音助手状态"
    pin:
      number: GPIO21
      inverted: true

# I2S 音频配置
i2s_audio:
  i2s_lrclk_pin: GPIO46 # 注意：标记为 "无用"
  i2s_bclk_pin: GPIO42

# 使用 I2S 音频的麦克风配置
microphone:
  - platform: i2s_audio
    id: echo_microphone
    i2s_din_pin: GPIO41
    adc_type: external
    pdm: true

# 语音助手配置
voice_assistant:
  microphone: echo_microphone

# 二进制传感器配置 (启动开关)
binary_sensor:    
  - platform: gpio
    pin: 
      number: GPIO2
      mode:
        input: true
        pullup: true
    name: 启动开关
    internal: true
    on_press:
      - voice_assistant.start:
      - light.turn_off: light0
    on_release:
      - voice_assistant.stop:
      - light.turn_on: light0
```

**注意**：有关更多信息，请阅读 [这里](https://esphome.io/components/microphone/i2s_audio)

##### 在仪表盘上可视化

- **第1步：** 在 Home Assistant 的概览页面，点击 3 个点，选择 **编辑仪表盘**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/31.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/15.png" style={{width:900, height:'auto'}}/></div>

- **第2步：** 点击 **+ 添加卡片**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/32.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/16.png" style={{width:900, height:'auto'}}/></div>

- **第3步：** 选择 **通过实体 (By ENTITY)**，然后选择 **Esp32S3 助手进行中**，**Esp32S3 助手进行中**，**Esp32S3 语音完成检测**，**状态** 和 **语音助手状态**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/va1.png" width="700">
<img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/va2.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/28.png" style={{width:900, height:'auto'}}/></div>
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/29.png" style={{width:900, height:'auto'}}/></div>

- **第4步：** 点击 **添加到仪表盘**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/va3.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/30.png" style={{width:400, height:'auto'}}/></div>

- **第5步：** 当你按下 Seeed Studio 扩展基板上的 **按钮 (D1)** 时，用户定义的 LED（GPIO2）将亮起，ESP32-S3 上的 LED 灯也会亮起，你可以通过 **语音助手** 与 ESPHome 进行交互。

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32S3/blob/main/Figures/va4.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/31.png" style={{width:600, height:'auto'}}/></div>

**注意**：有关更多信息，请 [阅读此文](https://esphome.io/components/voice_assistant.html)

#### 6x10 RGB MATRIX for XIAO

##### 配置步骤

- **第1步：** 首先需要将 6x10 RGB MATRIX 连接到 XIAO，请参阅 [此 Wiki 获取详细信息](https://wiki.seeedstudio.com/rgb_matrix_for_xiao/#hardware-preparation)。

- **第2步：** 复制下面的 .yaml 配置信息，并通过 OTA 推送到 XIAO ESP32S3。

```
esphome:
  name: sixtyled
  friendly_name: sixtyled

esp32:
  board: seeed_xiao_esp32s3
  variant: esp32s3
  framework:
    type: arduino
    version: latest
    platform_version: 6.4.0

# 启用日志
logger:

# 启用 Home Assistant API
api:

ota:

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # 如果 Wi-Fi 连接失败，启用备用热点（强制门户）
  ap:
    ssid: "Sixtyled 备用热点"
    password: "MoLTqZUvHwWI"

captive_portal:

light:
  - platform: esp32_rmt_led_strip
    rgb_order: GRB
    pin: GPIO1
    num_leds: 60
    rmt_channel: 0
    chipset: ws2812
    name: "XIAO LEDS"
```

##### 在仪表盘上可视化

- **第1步：** 打开路径 `设置 - 设备与服务 - ESPHome - sixtyled（你设置的名称）`，将卡片添加到仪表盘。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/sixty_dashboard_add.png" style={{width:900, height:'auto'}}/></div>

- **第2步：** 在 Home Assistant 的概览页面，点击 3 个点，选择 **编辑仪表盘**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/31.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/15.png" style={{width:900, height:'auto'}}/></div>

- **第3步：** 点击 **+ 添加卡片**

<!-- <img src="https://github.com/Zachay-NAU/ESPHome-Support-on-Seeed-Studio-XIAO-ESP32C3/blob/main/pictures/32.png" width="700"> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/16.png" style={{width:900, height:'auto'}}/></div>

- **第4步：** 选择 **通过实体（By ENTITY）**，输入 **xiao**，并勾选 **sixtyled XIAO LEDS** 旁边的复选框

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/add_card.png" style={{width:900, height:'auto'}}/></div>

- **第5步：** 点击 **继续**，然后点击 **添加到仪表盘**

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/sixty_dashboard_add2.png" style={{width:900, height:'auto'}}/></div>

- **第6步：** 随后，你可以在“概览”部分找到一张卡片，通过它来控制 XIAO 的 6x10 RGB 矩阵。你可以切换其开/关状态，并自定义其颜色和亮度。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/S3-ESPHome-full_function/LEDdemo.gif" style={{width:600, height:'auto'}}/></div>

## ✨ 贡献者项目

- 本项目由 Seeed Studio [贡献者项目](https://github.com/orgs/Seeed-Studio/projects/6/views/1?pane=issue&itemId=30957479) 支持。
- 感谢 **Zachary 的努力**，你的作品将被 [展示](https://wiki.seeedstudio.com/Honorary-Contributors/)。
- 感谢 **python 的努力**，该项目的源代码可在 [这里](https://community.home-assistant.io/t/seeed-studio-6x10-rgb-matrix-on-xiao-esp32s3/629867) 共享。

## 技术支持与产品讨论

感谢你选择我们的产品！我们在这里提供各种支持，确保你使用我们产品的体验尽可能顺畅。我们提供了多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>

















  