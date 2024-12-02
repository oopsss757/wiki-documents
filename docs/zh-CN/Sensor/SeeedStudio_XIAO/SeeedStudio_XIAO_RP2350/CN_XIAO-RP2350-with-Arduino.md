---
description: 在 XIAO RP2350 板上使用 Arduino
title: Seeed Studio XIAO RP2350 使用 Arduino
image: https://files.seeedstudio.com/wiki/XIAO-RP2350/img/2-102010550_XIAO_RP2350-45font_1.webp
slug: /cn/xiao_rp2350_arduino
sidebar_position: 2
last_update:
  date: 11/18/2024
  author: Agnes
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Seeed Studio XIAO RP2350 使用 Arduino

得益于 [arduino-pico 核心](https://github.com/earlephilhower/arduino-pico)，Seeed Studio XIAO RP2350 板现在支持通过 Arduino 进行编程。本指南将帮助您设置并开始在 RP2350 板上使用 Arduino。

## 准备工作

开始之前，请确保您已具备以下条件：

- 一块 RP2350 开发板
- Arduino IDE 软件
- 一根 USB 数据线

## 软件设置

### 1. 安装 Arduino IDE

从官方网站下载并安装最新版 Arduino IDE：[Arduino 软件](https://www.arduino.cc/en/software)。

### 2. 添加 RP2350 板支持

1. 打开 Arduino IDE，导航到 **文件** > **首选项**。
2. 在 **附加开发板管理器 URLs** 字段中，添加以下 URL：

    ```shell
    https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    ```

    <div style={{ textAlign: 'center' }}>
    <img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-url.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
    </div>

3. 点击 **OK** 保存设置。
4. 转到 **工具** > **开发板** > **开发板管理器**。
5. 在开发板管理器中，搜索 **pico** 并点击 **安装**。
6. 安装完成后，转到 **工具** > **开发板** 并选择下图所示的开发板作为您的开发板。

:::note
请确保安装版本为 4.2.0 或更高，以完整支持 XIAO RP2350 开发板。
:::

<div style={{ textAlign: 'center' }}>
<img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-board-option.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
</div>

### 3. 上传代码示例

在上传代码前，需要将 XIAO RP2350 置于 BOOT 模式。使用以下方法之一：

<Tabs>
<TabItem value="method1" label="方法一：连接电脑前" default>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-no-charge.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/><div style={{ marginTop: '-8px' }}><em>按住 Boot-> 插入数据线-> 松开 Boot</em></div></div>

</TabItem>

<TabItem value="method2" label="方法二：已连接电脑">

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-charged.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/><div style={{ marginTop: '-8px' }}><em>按住 Boot-> 点击 Reset-> 松开 Boot</em></div></div>

</TabItem>
</Tabs>

1. 打开 Arduino IDE 并新建一个草图（Sketch）。
2. 编写代码，例如使用 `Blink` 示例代码。
3. 转到 **工具** > **端口**，选择 RP2350 连接的端口。

<div style={{ textAlign: 'center' }}>
<img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-firmware-upload.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
</div>

## 附加资源

- [arduino-pico GitHub](https://github.com/earlephilhower/arduino-pico)
- [Arduino-Pico 核心文档](https://arduino-pico.readthedocs.io/en/latest/install.html)

## 支持与讨论

感谢您选择 Seeed 产品！我们提供多种支持渠道和社区讨论平台：

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
