---
description: 介绍如何使用乐鑫的 Matter 部署工具快速体验 Matter Lighting。
title: 使用 XIAO ESP32 快速开始使用 Matter
keywords:
- ESP-IDF
- XIAO
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/getting_started_with_matter
last_update:
  date: 11/27/2024
  author: Agnes
---

# 快速开始使用 Matter 和 XIAO ESP32 系列

:::tip
本文是 Seeed Studio XIAO ESP32 开发 Matter 系列的第二篇教程。如果您还没有配置 ESP-IDF 环境，请先阅读第一篇教程：

- **[在 XIAO 上使用 Espressif ESP-IDF 开发](https://wiki.seeedstudio.com/xiao_idf)**

本教程适用于 XIAO ESP32C3、XIAO ESP32S3。由于 GPIO8 引脚未引出，Espressif 提供的 Matter 示例暂时不支持 XIAO ESP32C6。
:::

在智能家居技术领域，Matter 的出现被认为是一场革命，它承诺彻底改变设备之间的通信和互动方式。Matter 是一个开源的、标准化的协议，旨在实现不同厂商的智能家居设备之间的无缝互操作。通过提供统一的语言和框架，Matter 旨在简化物联网设备的开发和部署，创造更互联、更友好的智能家居体验。

作为开发者或爱好者，您可能急于探索 Matter 的潜力，并开始构建自己的 Matter 兼容设备。这时，XIAO ESP32 系列和 ESPLaunchPad 便能派上用场。XIAO ESP32 系列，特别是 XIAO ESP32C3、XIAO ESP32S3 和 XIAO ESP32C6，是紧凑而强大的开发板，提供了构建 Matter 设备的理想平台。凭借其强大的功能和丰富的外设接口，这些开发板为您的 Matter 项目提供了所需的硬件基础。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/15.png" style={{width:800, height:'auto'}}/></div>

为了简化开发过程并使其更易访问，Espressif 系统推出了 ESPLaunchPad，这是一个全面的平台，简化了 ESP32 设备的固件刷写和配置过程。ESPLaunchPad 利用 ESP RainMaker 生态系统的强大功能，允许您通过智能手机应用轻松配置和控制设备。通过将 XIAO ESP32 系列与 ESPLaunchPad 相结合，您可以快速开始 Matter 开发，体验这一变革性协议的好处。

在本教程中，我们将引导您使用 ESPLaunchPad 快速将固件刷写到 XIAO ESP32 开发板上，并通过二维码将其与 iPhone 配对。按照这些步骤，您将能够建立手机与 XIAO ESP32 设备之间的连接，实现无缝控制和互动。这一实践体验将为您提供关于 Matter 及其在智能家居生态系统中的潜力的基础理解。

在本教程中，您将学习如何：
1. 设置您的 XIAO ESP32 开发板并准备好刷写固件。
2. 使用 ESPLaunchPad 轻松将 Matter 固件刷写到 XIAO ESP32 设备上。
3. 使用二维码将您的 XIAO ESP32 开发板与 iPhone 配对。
4. 使用 iPhone 应用控制和与您的 Matter 设备互动。
5. 在实际环境中探索 Matter 的基本功能和能力。

通过本教程，您将对 Matter 及其在 XIAO ESP32 系列和 ESPLaunchPad 上的实现有一个坚实的理解。您将具备继续推进 Matter 开发的知识和技能，创造出创新的智能家居解决方案，充分利用这一开创性的协议。

那么，让我们开始吧，踏上与 XIAO ESP32 系列和 ESPLaunchPad 一起探索 Matter 世界的激动人心的旅程！

## 准备软件

以下列出了本教程支持的系统和版本。

- **主机**： [Ubuntu 22.04 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) 或 macOS 10.15 或更高版本。

<!-- Matter 的 ESPLaunchPad **不支持 Windows**。 -->

## 准备硬件

在本教程结束时，我们将向您展示如何将 XIAO ESP32 系列作为 Matter 终端设备添加到 Apple Home 中，以通过苹果生态系统控制 LED 灯带。到目前为止，本教程支持以下 XIAO 开发板，您可以直接选择它们作为本课程的内容。

<div class="table-center">
	<table align="center">
		<tr>
			<th>XIAO ESP32C3</th>
			<th>XIAO ESP32S3</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:110, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:250, height:'auto'}}/></div></td>
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
				</a></div></td>
		</tr>
	</table>
</div>

除了 XIAO 开发板外，我们还需要 WS281x 模型的灯带或灯珠。目前，Espressif 提供的灯光示例仅支持单颗灯珠，因此无论使用灯带还是灯珠，它都会只点亮一个灯。我们还建议您选择 Grove Base for XIAO，以便更轻松地进行接线。

<div class="table-center">
	<table align="center">
		<tr>
			<th>Grove Base for XIAO</th>
			<th>Grove - RGB LED Ring (20 - WS2813 Mini)</th>
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

如果您使用的是 XIAO ESP32C3，请将 LED 灯带连接到 **D8** 引脚。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/16.png" style={{width:400, height:'auto'}}/></div>

如果您使用的是 XIAO ESP32S3，请将 LED 灯带连接到 **D9** 引脚。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/17.png" style={{width:400, height:'auto'}}/></div>

本教程将以 **XIAO ESP32C3** 为例，概述如何刷写固件、添加设备等操作。

## 视频教程

<div class="table-center">
<iframe width="750" height="450" src="https://www.youtube.com/embed/bhHVbRe_Gtw?si=iH-oouOl_ItkG7vF?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

## 步骤 1. 为 XIAO ESP32 刷写 Matter 固件

使用 USB 数据线将 XIAO ESP32C3 开发板连接到您的计算机。确保计算机正确识别该开发板。

<!-- :::caution
请不要使用 Windows 计算机，您会发现 Flash 按钮在 Windows 计算机上始终是灰色的。
::: -->

在计算机上打开 ESPLaunchPad 网站。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-matter/launchpad.toml">
            <strong><span><font color={'FFFFFF'} size={"4"}>前往 ESPLaunchPAD</font></span></strong>
    </a>
</div>

<br />

在 **选择应用程序** 中选择 **light**，在 **ESP 芯片类型** 中选择 **ESP32C3**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/18.png" style={{width:800, height:'auto'}}/></div>

然后点击右上角的 **连接** 按钮，选择您已经连接到计算机的 XIAO 设备。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/19.png" style={{width:800, height:'auto'}}/></div>

接着点击下面的 **flash** 按钮，等待固件上传。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/20.png" style={{width:800, height:'auto'}}/></div>

固件上传完成后，您将看到一些操作警告框和一个二维码，我们只需点击 **完成**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/21.png" style={{width:800, height:'auto'}}/></div>

然后点击屏幕右侧的 **重置设备** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/22.png" style={{width:800, height:'auto'}}/></div>

当您看到调试信息时，意味着一切顺利。现在我们可以继续进行设备绑定。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/23.png" style={{width:800, height:'auto'}}/></div>

## 步骤 2. 使用 iPhone Home 应用扫描二维码添加设备

在添加设备的部分，我们以 Apple 的设备为例（因为我手头只有 Apple 设备）。如果您手头有 [Google 设备](https://support.google.com/googlenest/answer/12391458?hl=en&co=GENIE.Platform%3DAndroid) 或 [Amazon 设备](https://developer.amazon.com/en-US/alexa/matter)，也许您也可以使用它们。

如果像本文一样，您希望使用 iPhone 的 Home 应用添加设备，您需要有一个 Apple 设备作为家庭中心。目前，Home Hub 支持的设备有 HomePod 和 Apple TV，详情请阅读 [Apple 官网](https://support.apple.com/en-hk/102557)。本教程假设您已经在 Home 应用中添加了 Home Hub。

Home 应用默认已安装在 iOS 设备上。如果您曾删除过它，可以 [在 App Store 中重新下载](https://apps.apple.com/cn/app/home/id1110145103?l=en-GB)。

打开 iPhone 上的 Home 应用。点击屏幕右上角的 **+** 按钮。从菜单中选择 **添加或扫描配件**。使用 iPhone 的相机扫描与 XIAO ESP32C3 一起提供的二维码。Home 应用将识别 Matter 配件，并将其显示为新配件。点击 **添加到家庭** 将 XIAO ESP32C3 添加到您的 Home 应用。请参阅下面的详细操作截图。

<div class="table-center">
  <table align="center">
    <tr>
      <th>第 1 页</th>
      <th>第 2 页</th>
      <th>第 3 页</th>
    </tr>
    <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/24.png" style={{width:600, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/25.png" style={{width:600, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/26.png" style={{width:600, height:'auto'}}/></div></td>
    </tr>
    <tr>
      <th>第 4 页</th>
      <th>第 5 页</th>
      <th>第 6 页</th>
    </tr>
    <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/27.png" style={{width:600, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/28.png" style={{width:600, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/32.png" style={{width:600, height:'auto'}}/></div></td>
    </tr>
    <tr>
      <th>第 7 页</th>
      <th>第 8 页</th>
      <th>第 9 页</th>
    </tr>
    <tr>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/29.png" style={{width:600, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/31.png" style={{width:600, height:'auto'}}/></div></td>
    </tr>
  </table>
</div>

## 步骤 3. 在 Home 应用中使用 XIAO

在 Home 应用中，找到新添加的 XIAO ESP32C3 配件。点击该配件以访问其控制选项。使用亮度滑块调整连接灯光的亮度。点击颜色图标可以更改灯光的颜色。您可以从多种预设颜色中选择，或者使用颜色选择器创建自定义颜色。连接到 XIAO ESP32C3 的灯光将实时响应您在 Home 应用中进行的调整。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/30.png" style={{width:300, height:'auto'}}/></div>

恭喜！您已经成功使用 ESPLaunchPad 为 XIAO ESP32C3 开发板刷写了灯光固件，并将其与 iPhone 的 Home 应用配对。现在，您可以直接从 iPhone 控制连接灯光的亮度和颜色，创造个性化且方便的照明体验。

欢迎继续探索 Home 应用中的更多高级功能和自定义选项，尝试与 XIAO ESP32C3 开发板兼容的不同灯光配件。通过这一基础，您可以进一步扩展智能家居设置，打造真正互联和自动化的生活空间。

## 故障排除

### Q1: 在 Home 应用中长时间无法连接到设备。

如果长时间无法连接到设备，请确保在上传固件后看到日志消息。如果此时设备长时间没有配对，XIAO 可能进入了待机模式，此时您需要按下 XIAO 上的重置按钮，或者使用 ESPLaunchPad 上的 **重置设备** 功能来重启设备。然后再尝试添加设备。

除此之外，可能是网络问题。请确保 XIAO 和您的 iPhone 在同一个局域网 (LAN) 下，并且它们都需要连接到 2.4GHz 网络，而不是 5GHz 网络。配置完成后，手机可以使用其他网络，但 XIAO 只支持 2.4GHz 网络。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们产品的体验尽可能顺畅。我们提供了多个沟通渠道，以满足不同的偏好和需求。


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



