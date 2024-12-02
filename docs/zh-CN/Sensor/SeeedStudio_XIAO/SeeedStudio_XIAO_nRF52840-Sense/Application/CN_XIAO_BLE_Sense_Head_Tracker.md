---
description: 使用 XIAO BLE SENSE 制作头部追踪设备
title: 使用 XIAO BLE SENSE 制作头部追踪设备
keywords:
  - XIAO BLE
image: https://files.seeedstudio.com/wiki/wiki-platform/S.webp
slug: /cn/xiao_ble_sense_head_tracker
last_update:
  date: 11/18/2024
  author: Albert Abdilim
---
# 使用 XIAO BLE SENSE 制作头部追踪设备（Head Tracker）

## 目录
- [简介](#简介)
- [功能特点](#功能特点)
- [硬件需求](#硬件需求)
- [固件烧录及基础配置](#固件烧录及基础配置)
- [焊接PCB板](#焊接PCB板)
- [应用示例](#应用示例)
- [故障排除](#故障排除)
- [常见问题](#常见问题)

---

## 简介
[HeadTracker](https://headtracker.gitbook.io) 是一款开源头部追踪项目，可用于无人机控制、虚拟现实、游戏等领域。该设备通过感应头部的方向和运动，为用户提供实时的视角变化体验。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/1.png" alt="PCB" width="1000" /></div>

## 功能特点
- **实时头部追踪**：支持全向追踪和快速响应
- **多种通信接口**：兼容蓝牙、串口、PPM、SBUS等多种通信方式
- **多平台**：多平台GUI客户端，支持Windows、MacOS、Linux等操作系统
- **硬件简单**：基于Arduino平台，易于制作和维护，硬件成本低廉，本教程使用 XIAO BLE SENSE nRF52840 开发板来作为主控制器

## 硬件需求
- XIAO BLE SENSE nRF52840 开发板
- USB-C 数据线（用于连接和供电）
- 项目 PCB板（可选），用于方便的连接本项目到控制设备

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/2.png" alt="PCB" width="600" /></div>

- 3D 打印外壳（可选），用于保护设备和固定传感器

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/3.png" alt="PCB" width="600" /></div>

- 电池（可选），用于无线使用，由于 XIAO BLE SENSE 开发板板载电池管理芯片，可以轻松的实现锂电池的充放电管理，方便无线使用

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/4.png" alt="PCB" width="600" /></div>

## 固件烧录及基础配置

### 1. 烧录固件
1. 前往 [HeadTracker](https://github.com/dlktdr/HeadTracker) 的GitHub仓库主页，下载最新的固件。由于目前 XIAO BLE SENSE 目前在 beta 版本中被支持，所以需要前往 [GitHub Action](https://github.com/dlktdr/HeadTracker/actions/workflows/build-firmware.yml) 页面中下载最新的固件。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/5.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/6.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/7.png" alt="PCB" width="1000" /></div>

2. 解压下载的固件zip文件，在其中找到适用于 XIAO BLE SENSE 的固件，固件以 `.uf2` 为后缀，可能类似这个名称 `xiao_ble_nrf52840_sense-2_40pre-7ae7247.uf2`。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/8.png" alt="PCB" width="1000" /></div>

3. 将 XIAO BLE SENSE 开发板连接到电脑，双击开发板上的 `RST` 按键，此时开发板会进入烧录模式，此时电脑上会出现一个名为 `XIAO-SENSE` 的U盘，将固件拖拽到该U盘中，固件将自动烧录。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/9.png" alt="PCB" width="1000" /></div>

<br></br>

![Firmware](https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/10.png)

### 2. 下载 GUI 客户端
1. 同样的前往 [HeadTracker](https://github.com/dlktdr/HeadTracker) 的GitHub仓库主页，下载最新的GUI客户端。由于目前 XIAO BLE SENSE 目前在 beta 版本中被支持，所以需要前往 [GitHub Action](https://github.com/dlktdr/HeadTracker/actions) 页面中下载最新的支持 XIAO BLE SENSE 的 GUI 客户端。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/5.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/11.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/12.png" alt="PCB" width="1000" /></div>

2. 解压下载的 GUI 客户端 zip 文件，双击运行其中的可执行文件，即可使用。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/13.png" alt="PCB" width="1000" /></div>

### 3. 使用 GUI 客户端配置头部追踪设备
1. 连接 XIAO BLE SENSE 开发板到电脑，打开 GUI 客户端。
2. 在 GUI 客户端中选择连接的端口，点击 Connect 按钮，GUI 客户端会自动识别设备，如果没有出现设备信息，请检查连接是否正常，并且点击刷新按钮。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/14.png" alt="PCB" width="1000" /></div>

3. 在 GUI 客户端中选择校准选项，根据提示进行校准，校准完成后保存设置。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/15.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/16.png" alt="PCB" width="1000" /></div>

4. 在 GUI 客户端中需要输出的通道，注意通道数不能重复。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/17.png" alt="PCB" width="1000" /></div>

5. 在 GUI 客户端中选择需要输出的同性协议，例如串口、蓝牙等，根据需要进行配置。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/18.png" alt="PCB" width="1000" /></div>

6. 最后一定不要忘了点击 Save Settings 按钮保存设置。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/19.png" alt="PCB" width="1000" /></div>

## 焊接PCB板

### 1. 制作 PCB 板
1.相应的 PCB 设计文件可以在 [立创开源硬件平台](https://oshwhub.com/l1cardo/fpv-tou-zhui) 的开源硬件页面中找到，下载对应的 Gerber 文件。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/20.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/21.png" alt="PCB" width="1000" /></div>

2. 将 Gerber 文件上传到 [嘉立创](https://www.jlc.com/) 或者其他 PCB 制造商的网站，下单制作 PCB 板。

### 2. 焊接 PCB 板
1. 准备好焊接工具，加热台、锡膏等，因为 XIAO BLE SENSE 开发板的电池焊盘在开发版的底部，所以我没无法通过电烙铁焊接，所以需要使用加热台。
2. 将焊锡膏涂抹在 PCB 板焊盘上，将 XIAO BLE SENSE 开发板放置在 PCB 板上，加热台加热焊盘，焊接完成
3. 焊接完成后一定要检查焊接是否正确，是否有短路、断路等问题。

### 3. 焊接其他器件
1. 将其他器件焊接到 PCB 板上，3.5mm插头，开关，电池等。

## 3D 打印外壳（可选）
1. 在[立创开源硬件平台](https://oshwhub.com/l1cardo/fpv-tou-zhui) 的开源硬件页面中找到对应的 3D 外壳打印文件。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/20.png" alt="PCB" width="1000" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/22.png" alt="PCB" width="1000" /></div>

2. 焊接电池后，需要将突出的焊盘剪平，防止插入电池造成电池损坏，并且可以贴上绝缘胶带。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/23.png" alt="PCB" width="800" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/24.png" alt="PCB" width="800" /></div>

3. 将 XIAO BLE SENSE 开发板放置在外壳中，先放入电池，再放入 PCB 板，最后盖上外壳，外壳使用卡扣设计，不需要使用螺丝固定。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/25.png" alt="PCB" width="800" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/26.png" alt="PCB" width="800" /></div>

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/27.png" alt="PCB" width="800" /></div>

## 应用示例

### 示例 1：无人机操控
使用 HeadTracker 控制无人机摄像头视角和角度，实现沉浸式飞行体验。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/28.png" alt="PCB" width="600" /></div>

### 示例 2：电脑游戏
HeadTracker 可作为电脑游戏的输入设备，实现头部追踪效果，提升游戏体验，或仅仅用作体感输入设备。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/29.png" alt="PCB" width="600" /></div>

### 示例 3：与VR头盔的集成
HeadTracker 可作为VR头盔的补充设备，实现高精度头部追踪效果。

## 故障排除

### 设备无法连接
- 检查 USB 线缆连接是否正常。
- 尝试更换数据线或 USB 端口。
- 确保固件是否已正确烧录。
- 确保 GUI 客户端版本与固件版本匹配。

## 常见问题

### 1. HeadTracker的电池无法充电？
由于 XIAO BLE SENSE 开发板板载的电池管理芯片直接与底部的电池焊盘连接，中间没有添加开关，我们添加的开关是单纯的连接电池与开发版，属于物理切断，所以在充电时需要将开关打开，否则无法充电。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/30.png" alt="PCB" width="600" /></div>

### 2. HeadTracker的电池续航时间？
测试下来，XIAO BLE SENSE 作为本项目的主控制器，使用的耗电量很小，大概是0.006A左右，所以使用3.7V 500mAh的电池，续航时间大概在80小时左右，具体续航时间还需要根据具体的使用情况来判断。

<div style={{ textAlign: 'center' }}><img src="https://files.seeedstudio.com/wiki/XIAO-BLE-Sense-Head-Tracker/31.png" alt="PCB" width="600" /></div>

### 3. 可以与哪些设备兼容？
HeadTracker支持多种设备，具体请参考[官方GitBook文档](https://headtracker.gitbook.io)。

## ✨ 项目贡献者

这个项目是由Seeed Studio[贡献者项目](https://github.com/orgs/Seeed-Studio/projects/6/views/1?pane=issue&itemId=30957479)支持的。

-感谢[ Albert Abdilim 的努力](https://github.com/orgs/Seeed-Studio/projects/6/views/1?filterQuery=Head&pane=issue&itemId=87592250&issue=Seeed-Studio%7Cwiki-documents%7C1900)，您的作品将被[展出](https://wiki.seeedstudio.com/contributors/)。


## 技术支持和产品讨论

感谢您选择我们的产品！我们随时为您提供不同的支持，以确保您尽可能顺利地使用我们的产品。我们提供多种沟通渠道来满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
