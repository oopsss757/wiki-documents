---
description: 使用 ESP32C3 的 XIAO 棱镜显示器
title: XIAO ESP32C3棱镜显示器
keywords:
- Xiao
- Prism
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao-esp32c3-prism-display
last_update:
  date: 11/12/2024
  author: Agnes
---

# XIAO ESP32C3棱镜显示器

<iframe class="youtube-video" src="https://www.youtube.com/embed/wSJa8HP0BkM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


## 开始

如果您想按照本教程完成所有操作，则需要准备以下内容。

<table align="center">
  <tbody><tr>
      <th>XIAO ESP32C3</th>
      <th>带电池管理芯片的 <br />Grove Shield for XIAO</th>
      <th>Grove 智能红外手势 <br />传感器(PAJ7660)</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:100, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Grove-Shield-for-Seeeduino-XIAO/img/xiao_-Preview-25.png" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/grove-gesture-paj7620/main.jpg" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Smart-IR-Gesture-Sensor-p-5721.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

<div align="center">
    <img width={400} src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/assembled.jpg" />
</div>

本项目使用 ESP32C3 通过分光棱镜创建三维显示。此指南解释了构建外壳、安装相关电子组件和安装固件所需的步骤。还提供了更新跟踪的加密货币列表以及缩小和处理动画图像以用于图像浏览器的其他文档。

主要步骤

1. [下载并 3D 打印外壳](#download-and-3d-print-enclosure)
2. [连接相关电子组件](#attach-associated-electronics)
3. [设置开发环境](#setup-environment)
4. [配置 XIAO ESP32C3](#configure-the-xiao-esp32c3)

附加文档

1. [添加新加密货币](#adding-new-cryptocurrencies)
2. [缩小和处理动画图像](#downsizing-and-processing-animated-images)

## 下载并 3D 打印外壳

<div align="center">
    <img width={400} src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/3d-render-bottom.png" />
</div>

3D 外壳由两个部分组成，设计为在内部组件组装后可以按压合并。打印零件时应使用支撑结构，以确保正确组装。

这些零件可以在 GitHub 仓库中找到，并可以通过其 STL 查看器预览：<br />
https://github.com/Timo614/xiao-prism-buddy/blob/main/docs/enclosure/xiao-prism-top.stl<br />
https://github.com/Timo614/xiao-prism-buddy/blob/main/docs/enclosure/xiao-prism-bottom.stl<br />

## 连接相关电子组件

<div align="center">
    <img width={400} src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/xiao_screwed_in.jpg" />
</div>

Grove Shield 简化了棱镜显示的设置，因为它允许使用简单的面包板连接线进行组装，而无需焊接。Grove Shield 应该将第二组女性插针焊接到靠近 Xiao 引脚的空插针处。这个 Shield 应该被固定到外壳上的两个螺丝孔里。这个过程有点复杂，可以先把螺丝放进去，再把 Shield 放下并拧紧每个螺丝。

从这里开始有两个主要连接：
- i2c 连接到手势传感器
- spi 连接到显示屏

对于 i2c 连接，使用从 Shield 到手势传感器的 Grove 4 针连接器。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/inward_bend_install.jpg" style={{width:400, height:'auto'}}/></div>

外壳设计允许手势传感器与外壳紧密贴合，传感器用螺丝固定在外壳外部。要正确安装手势传感器，首先应将 USB 面朝外，然后当传感器与外壳对齐时，安装螺丝。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/sensor_screwed_in.jpg" style={{width:400, height:'auto'}}/></div>

spi 连接稍微复杂一些，因为它需要更多的引脚。

```
Xiao 的 3V3 引脚连接到显示屏的 VCC 引脚
Xiao 的 GND 引脚连接到显示屏的 GND 引脚
Xiao 的 D0 引脚连接到显示屏的 CS 引脚
Xiao 的 D1 引脚连接到显示屏的 BL 引脚
Xiao 的 D2 引脚连接到显示屏的 DC 引脚
Xiao 的 D3 引脚连接到显示屏的 RST 引脚
Xiao 的 D8 (SCK) 引脚连接到显示屏的 SCL 引脚
Xiao 的 D10 (MOSI) 引脚连接到显示屏的 SDA 引脚
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pin_map-2.png" style={{width:700, height:'auto'}}/></div>

连接好所有组件后，可以完成设备的组装。如果打印外壳时使用类似的公差，外壳设计允许按压合并。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/enclosure/press_fit_close.jpg" style={{width:400, height:'auto'}}/></div>

完成组装后，将棱镜放置在显示屏上方。

## 设置开发环境

设置本地开发环境的最简单方法是使用 Visual Studio Code，因为它简化了机器配置。

- 从 GitHub 克隆仓库： https://github.com/Timo614/xiao-prism-buddy
- 在 Visual Studio Code 中打开该仓库
- 安装 ESP-IDF 扩展
- 安装 ESP-IDF 5.0.4
- 将 ESP-IDF Espressif 设备目标设置为 esp32c3
- 配置您的 WiFi 网络的环境变量（见下文步骤）
- 构建、刷写并监控设备

## 配置 XIAO ESP32C3

准备应用程序的下一步是配置您的 WiFi 网络凭证。

按 `Ctrl` + `Shift` + `P` 打开快速菜单，输入 `menuconfig`。这将过滤列表，显示 `ESP-IDF: SDK 配置编辑器 (Menuconfig)` 选项。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/menuconfig.png" style={{width:400, height:'auto'}}/></div>

在此菜单中，请设置 WiFi 网络名称和密码以进行连接。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/config.png" style={{width:400, height:'auto'}}/></div>

## 添加新加密货币

Xiao Prism Buddy 可以进行修改，以显示不同的加密货币，因为它是由 CoinGecko 提供支持的。CoinGecko 提供了一个免费的 API 级别，允许用户访问并有限制地使用该 API。

应用程序每 15 分钟请求一次 `/simple/price` 接口，更新配置的加密货币集合。通过 `config.h` [仓库中的文件](https://github.com/Timo614/xiao-prism-buddy/blob/main/main/config.h#L26) 可以配置所使用的货币以及应用中显示的货币符号。 [API 文档](https://www.coingecko.com/api/documentation) 可供参考，提供更多关于接口调用的信息。

添加新加密货币的过程目前稍微复杂，但可以按照以下步骤完成：

1. 更新加密货币总数，以反映正确的数量（此硬编码值用于在页面上循环显示）
https://github.com/Timo614/xiao-prism-buddy/blob/main/main/controller/prism_controller.cpp#L29
2. 在加密货币模型文件中搜索现有的加密货币（例如：比特币），并在代码中的每个相关部分添加一个新条目（数据初始化、CoinGecko 响应解析、CoinGecko 请求字符串等）
https://github.com/Timo614/xiao-prism-buddy/blob/main/main/model/prism_cryptocurrency.c
3. 更新加密货币视图数据结构，以反映新的加密货币 https://github.com/Timo614/xiao-prism-buddy/blob/main/main/view_data.h#L54
4. 在控制器逻辑中，搜索现有的加密货币（例如：比特币），并复制相关逻辑以处理事件数据和渲染 https://github.com/Timo614/xiao-prism-buddy/blob/main/main/controller/prism_controller.cpp
5. 提供适当大小的 PNG 文件用于新加密货币，使用 LVGL 在线图像转换器将 PNG 转换为 C 文件，并在控制器中引用 https://lvgl.io/tools/imageconverter

## 缩小和处理动画图像

使用微控制器时的一个限制是可用的闪存空间有限。GIF 文件占用相当大的闪存，因此将其包含进来变得更加困难。为此，本文档介绍了一个用于转换应用程序中使用的图像的处理过程，供其他人参考，以便他们能根据自己的需要替换图像浏览器中使用的图像。 

1. 选择合适的动画图像。理想情况下，图像应该是透明的、设置为循环播放，并且具有合适的尺寸。为了简化这个过程，我发现寻找 Lottie 文件特别有帮助。对于本应用程序，我通过 https://lottiefiles.com/
2. 使用 Lottie 到 GIF 的转换器将 Lottie 文件转换为透明的 GIF 文件（例如： https://lottiefiles.com/lottie-to-gif）

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/convert-lottie.png" style={{width:400, height:'auto'}}/></div>

3. 使用你选择的 GIF 编辑器进行以下修改（我发现在线工具 https://ezgif.com 提供了优化 GIF 的功能，适用于本应用程序）：

3.1. 减少帧之间的帧数，直到只剩下大约 20 帧

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/remove-frames.png" style={{width:400, height:'auto'}}/></div>

3.2. 将图像调整为大约 100x100 或更小

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/resize.png" style={{width:400, height:'auto'}}/></div>

3.3. 放慢图像速度，因为没有额外帧时它会非常快 

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/slow-speed.png" style={{width:400, height:'auto'}}/></div>

3.4. 减少 GIF 的颜色组成，以减小文件大小

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/reduce-colors.png" style={{width:400, height:'auto'}}/></div>

3.5. 根据需要进一步压缩 GIF

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Timo614/xiao-prism-buddy/main/docs/photos/gif-compression/compress.png" style={{width:400, height:'auto'}}/></div>

4. 使用 [LVGL 在线图像转换器](https://lvgl.io/tools/imageconverter) 将 GIF 文件转换为 C 数组，按照  [LVGL 文档](https://docs.lvgl.io/8.3/libs/gif.html#convert-gif-files-to-c-array) `注意：选择“Raw”颜色格式和“C 数组”输出格式。` 

通过执行这些步骤，你将得到一个文件大小减小但仍然有效的 GIF 图像，可以在 Prism 设备上显示。最简单的方法是替换现有的火焰或西瓜图像进行测试。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时体验顺畅。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>


