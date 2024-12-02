---
description: Getting Started XIAO eInk Expansion Board
title: 基于 XIAO 的 墨水屏接口扩展板
keywords:
- XIAO eInk Expansion
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/XIAO-eInk-Expansion-Board
last_update:
  date: 11/27/2024
  author: Agnes
---

# 入门 XIAO eInk 扩展板

<div style={{textAlign:'center'}}><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/4/-/4-105990172-epaper-breakout-board-45back.jpg" style={{width:700, height:'auto'}}/></div>

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://www.seeedstudio.com/ePaper-Breakout-Board-p-5804.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    </a>
</div><br />

认识 XIAO eInk 扩展板，它是与 XIAO 系列产品轻松驱动 eInk 显示器的理想解决方案。该板设计有一个 24 针 FPC 连接器，提供与我们 eInk 产品系列的无缝连接。如果你想扩展选项，8 针 2.54 毫米针脚可以让你轻松集成任何你选择的微控制器。把这个板看作是你 eInk 显示器的“永远的好朋友”，它能增强显示器的功能，让你的生活更加轻松。

:::note
此扩展板不包含 eInk 显示器；显示器需要单独购买。
:::

## 介绍

### 特性

- **24 针 FPC 连接器**：提供与 eInk 显示器的稳固和可靠的连接。
- **XIAO 插座**：允许你使用 XIAO 作为处理器，提供紧凑而强大的控制解决方案。
- **8 针 2.54 毫米针脚**：通过让你连接任何微控制器，提供灵活性，开启无限可能。
- **即插即用**：设计简单易用，适合初学者和专家。
- **多功能应用**：适用于各种大小的 eInk 显示器。

### 应用

- **数字标牌**：使用 eInk 扩展板创建动态且节能的数字标牌。
- **电子书阅读器**：构建自己的定制电子书阅读器，提供符合你需求的功能。
- **智能家居控制面板**：将该板集成到智能家居系统中，打造简洁现代的控制面板。
- **零售价格标签**：创建电子价格标签，提升零售体验的效率和环保性。
- **教育工具**：开发可以轻松更新且节能的互动教育材料。

## 硬件概述

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/xiao-expansion.png" style={{width:700, height:'auto'}}/></div>

1. **24 针 FPC 连接器**：用于 2.13 英寸 ePaper。
2. **XIAO 插座**：用于连接 Seeed Studio XIAO 开发板。
3. **IO 扩展**：用于连接其他控制器，如 Arduino UNO 或 Raspberry Pi。

### 引脚定义

<div class="table-center">

|  eInk SPI 引脚  |  XIAO  | 
|       ---      |  ---   |
|      RST       |   D0   |
|      CS        |   D1   |
|      DC        |   D3   |
|      BUSY      |   D5   |
|      SCK       |   D8   |
|      MOSI      |   D10  |

</div>

### 支持的 eInk 显示器

1. [1.54 英寸 E-paper - Dotmatix 200x200](https://www.seeedstudio.com/1-54-Monochrome-ePaper-Display-with-200x200-Pixels-p-5776.html)
2. [2.13 英寸 E-Paper - 可弯曲单色 212x104](https://www.seeedstudio.com/2-13-Flexible-Monochrome-ePaper-Display-with-212x104-Pixels-p-5781.html)
3. [2.13 英寸 E-Paper - 四色 212x104](https://www.seeedstudio.com/2-13-Quadruple-Color-ePaper-Display-with-122x250-Pixels-p-5779.html)
4. [2.9 英寸 E-paper - 单色 128x296](https://www.seeedstudio.com/2-9-Monochrome-ePaper-Display-with-296x128-Pixels-p-5782.html)
5. [2.9 英寸 e-paper - 四色 128x296](https://www.seeedstudio.com/2-9-Quadruple-Color-ePaper-Display-with-128x296-Pixels-p-5783.html)
6. [4.2 英寸 E-Paper - 单色 400x300](https://www.seeedstudio.com/4-2-Monochrome-ePaper-Display-with-400x300-Pixels-p-5784.html)
7. [5.65 英寸 E-paper - 七色 600x480](https://www.seeedstudio.com/5-65-Seven-Color-ePaper-Display-with-600x480-Pixels-p-5786.html)
8. [5.83 英寸 E-paper - 单色 648x480](https://www.seeedstudio.com/5-83-Monochrome-ePaper-Display-with-648x480-Pixels-p-5785.html)
9. [7.5 英寸 E-paper - 单色 800x480](https://www.seeedstudio.com/7-5-Monochrome-ePaper-Display-with-800x480-Pixels-p-5788.html)

## 使用 Image2lcd 软件

### 如何制作图片

使用 Windows 自带的 **画图** 软件创建一个与显示屏分辨率相同的图片，并将其保存为 `BMP` 或 `JPG` 文件；

:::note
你的图片分辨率必须与显示器的分辨率相同，例如，4.2 英寸 E-paper 的分辨率是 400 x 300 像素，那么你不能使用 300 x 400 的尺寸，这会导致 image2lcd 输出的 `.h` 文件多出 200 字节。
:::

图片的颜色应与 Windows 自带的画板标准颜色一致。画板的颜色如下：

<div class="table-center">

|      E-paper      | 颜色 |
|       ---      |  ---   |
|1.54 英寸 E-paper - Dotmatix 200x200           | 纯黑白          |
|2.13 英寸 E-Paper - 可弯曲单色 212x104 | 纯黑白          |
|2.13 英寸 E-Paper - 四色 212x104          | 黑、白、红、黄 |
|2.9 英寸 E-paper - 单色 128x296           | 纯黑白          |
|2.9 英寸 e-paper - 四色 128x296     | 黑、白、红、黄 |
|4.2 英寸 E-Paper - 单色 400x300           | 纯黑白          |
|5.65 英寸 E-paper - 七色 600x480        | 黑、白、红、黄、蓝、绿、橙 |
|5.83 英寸 E-paper - 单色 648x480          | 纯黑白          |
|7.5 英寸 E-paper - 单色 800x480           | 纯黑白          |

</div>

### 位图转换

**步骤 1.** 打开 [Image2lcd.7z](https://files.seeedstudio.com/wiki/eInk/xiao-expansion/Image2Lcd.7z)，解压并打开应用程序。
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/1.png" style={{width:700, height:'auto'}}/></div>

**步骤 2.** 打开图片，选择“输出文件类型”为“C 数组 (*.c)”，选择“扫描模式”为“水平扫描”，其他参数设置如下：

<div class="table-center">
  <table align="center">
    <tr>
        <th>电子纸</th>
        <th>位像素</th>
        <th>最大宽度和高度</th>
        <th>反色</th>
        <th>显示模式</th>    
    </tr>
    <tr>
        <th>1.54 英寸 E-paper - Dotmatix 200x200</th>
        <td align="center">单色</td>
        <td align="center">200x200</td>
        <td align="center">✅</td>
        <td align="center">镜像左右</td>
    </tr>
    <tr>
        <th>2.13 英寸 E-Paper - 可弯曲单色 212x104</th>
        <td align="center">单色</td>
        <td align="center">104x212</td>
        <td align="center">✅</td>
        <td align="center">正常</td>
    </tr>
    <tr>
        <th>2.13 英寸 E-Paper - 四色 212x104 </th>
        <td align="center">4 灰度</td>
        <td align="center">104x212</td>
        <td align="center">/</td>
        <td align="center">正常</td>     
    </tr>
    <tr>
        <th>2.9 英寸 E-paper - 单色 128x296</th>
         <td align="center">单色</td>
        <td align="center">128x296</td>
        <td align="center">✅</td>
        <td align="center">正常</td>   
    </tr>
    <tr>
        <th>2.9 英寸 e-paper - 四色 128x296 </th>
        <td align="center">4 灰度</td>
        <td align="center">128x296</td>
        <td align="center">/</td>
        <td align="center">正常</td>       
    </tr>
      <tr>
        <th>4.2 英寸 E-Paper - 单色 400x300</th>
        <td align="center">单色</td>
        <td align="center">400x300</td>
        <td align="center">✅</td>
        <td align="center">镜像左右</td>        
    </tr>
      <tr>
        <th>5.65 英寸 E-paper - 七色 600x480</th>
        <td align="center">256 色</td>
        <td align="center">600x448</td>
        <td align="center">/</td>
        <td align="center">正常</td>        
    </tr>
      <tr>
        <th>5.83 英寸 E-paper - 单色 648x480</th>
        <td align="center">单色</td>
        <td align="center">600x480</td>
        <td align="center">✅</td>
        <td align="center">镜像左右</td>        
    </tr>
        <tr>
        <th>7.5 英寸 E-paper - 单色 800x480</th>
        <td align="center">单色</td>
        <td align="center">800x480</td>
        <td align="center">✅</td>
        <td align="center">镜像左右</td>     
    </tr>
  </table>
</div>

:::tip
- 设置最大宽度和高度后，需要点击箭头确认。
- 不包括头部数据。
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/2.png" style={{width:700, height:'auto'}}/></div>
:::

**步骤 3.** 点击“保存”将 lcd 输出数组保存为 `.h` 文件。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/7.png" style={{width:500, height:'auto'}}/></div>

## 入门

要使用 **XIAO eInk 扩展板**，我们需要对 XIAO 系列进行编程。XIAO 支持的不同尺寸的 E-paper 如下表所示：

<div class="table-center">

|      电子纸 / XIAO     | XIAO SAMD21 | XIAO RP2040|  XIAO nRF52840 | XIAO ESP32-C3 | XIAO ESP32-S3 |
|       ---      |  ---  | --- | --- | --- | --- |
|1.54 英寸 E-paper - Dotmatix 200x200           | ✅ | ✅ | ✅ | ✅ | ✅ |
|2.13 英寸 E-Paper - 可弯曲单色 212x104 | ✅ | ✅ | ✅ | ✅ | ✅ |
|2.13 英寸 E-Paper - 四色 212x104          | ✅ | ✅ | ✅ | ✅ | ✅ |
|2.9 英寸 E-paper - 单色 128x296           | ✅ | ✅ | ✅ | ✅ | ✅ |
|2.9 英寸 E-paper - 四色 128x296     | ✅ | ✅ | ✅ | ✅ | ✅ |
|4.2 英寸 E-Paper - 单色 400x300           | ✅ | ✅ | ✅ | ✅ | ✅ |
|5.65 英寸 E-paper - 七色 600x480        | FLASH 溢出 | ✅ | ✅ | ✅ | ✅ |
|5.83 英寸 E-paper - 单色 648x480          | ✅ | ✅ | ✅ | ✅ | ✅ |
|7.5 英寸 E-paper - 单色 800x480           | RAM 溢出 | ✅ | ✅ | ✅ | ✅ |

</div>

### 硬件准备

**步骤 1.** 材料准备

<table align="center">
	<tr>
		<th>Seeed Studio XIAO SAMD21</th>
		<th>Seeed Studio XIAO RP2040</th>
		<th>Seeed Studio XIAO nRF52840 (Sense)</th>
		<th>Seeed Studio XIAO ESP32C3</th>
	    <th>Seeed Studio XIAO ESP32S3 (Sense)</th>
	</tr>
	<tr>
		<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO/img/Seeeduino-XIAO-preview-1.jpg" style={{width:400, height:'auto'}}/></div></td>
		<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/102010428_Preview-07.jpg" style={{width:500, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/round_display_for_xiao/xiaoblesense.jpg" style={{width:500, height:'auto'}}/></div></td>
		<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/round_display_for_xiao/xiaoesp32c3.jpg" style={{width:450, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:500, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
		<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
		<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
		<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/seeed-xiao-esp32c3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

**步骤 2.** 将 XIAO 插入 XIAO 插座：对齐引脚并轻轻将 XIAO 插入扩展板上的 XIAO 插座。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/connect_xiao.gif" style={{width:700, height:'auto'}}/></div>

**步骤 3.** 将电子纸插入 FPC 连接器：小心地将您的电子纸滑入 ePaper Breakout Board 上的 24 引脚 FPC 连接器。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/connect_eink.gif" style={{width:700, height:'auto'}}/></div>

### 软件准备

推荐的编程工具是 Arduino IDE，您需要为 XIAO 配置 Arduino 环境并添加相关开发板包。

:::tip
如果这是您第一次使用 Arduino，强烈建议您参考 [Arduino 入门指南](https://wiki.seeedstudio.com/Getting_Started_with_Arduino/)。
:::

**步骤 1.** 启动 Arduino 应用程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/seeed_logo/arduino.jpg" style={{width:800, height:'auto'}}/></div>

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载 Arduino IDE</font></span></strong>
    </a>
</div>

**步骤 2.** 选择您的开发板型号并将其添加到 Arduino IDE 中。

- 如果您想使用 **Seeed Studio XIAO SAMD21** 进行后续编程，请参考 **[此教程](https://wiki.seeedstudio.com/Seeeduino-XIAO/#software)** 完成添加。

- 如果您想使用 **Seeed Studio XIAO RP2040** 进行后续编程，请参考 **[此教程](https://wiki.seeedstudio.com/XIAO-RP2040-with-Arduino/#software-setup)** 完成添加。

- 如果您想使用 **Seeed Studio XIAO nRF52840** 进行后续编程，请参考 **[此教程](https://wiki.seeedstudio.com/XIAO_BLE/#software-setup)** 完成添加。

- 如果您想使用 **Seeed Studio XIAO ESP32C3** 进行后续编程，请参考 **[此教程](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started#software-setup)** 完成添加。

- 如果您想使用 **Seeed Studio XIAO ESP32S3** 进行后续编程，请参考 **[此教程](http://wiki.seeedstudio.com/xiao_esp32s3_getting_started#software-preparation)** 完成添加。


### 功能概述

在开始开发示例程序之前，我们先来看看可用的函数。

- `void EPD_HW_Init()`—— 全屏刷新初始化。
- `void EPD_HW_Init_180()` ——显示旋转180度初始化。
- `void EPD_WhiteScreen_ALL(const unsigned char *datas)` —— 全屏刷新显示功能。
  - **输入参数：**
    - **datas：** 需要刷新的图像数据。
- `void EPD_WhiteScreen_White()` —— 清屏显示。
- `void EPD_WhiteScreen_Black()` —— 显示全黑屏。
- `void EPD_DeepSleep()` —— 深度睡眠功能，进入睡眠模式，请勿删除此函数，否则会缩短屏幕的使用寿命。
- `void EPD_Init_Part()` —— 部分刷新显示。
- `void EPD_SetRAMValue_BaseMap(const unsigned char *datas)` —— 背景部分刷新显示。
  - **输入参数：**
    - **datas：** 需要刷新的图像数据。
- `void EPD_Dis_PartAll(const unsigned char *datas)`—— 全屏部分刷新显示。
- `void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char *datas,unsigned int PART_COLUMN,unsigned int PART_LINE)` —— 部分刷新显示。
  - **输入参数：**
    - **unsigned int x_start:** 起始点的 x 值。
    - **unsigned int y_start:** 起始点的 y 值。
    - **const unsigned char *datas:** 需要改变的数据。
    - **unsigned int PART_COLUMN:** 需要改变区域的长度。
    - **unsigned int PART_LINE:** 需要改变区域的宽度。
- `void EPD_HW_Init_Fast()` —— 快速刷新显示。
- `void EPD_WhiteScreen_ALL_Fast(const unsigned char *datas)` —— 快速刷新显示功能。
  - **输入参数：**
    - **datas：** 需要刷新的图像数据。
- `void EPD_HW_Init_Fast2()` —— 快速刷新显示功能。

### 示例演示

**步骤 1.** 下载示例代码，并打开适用于您的电子墨水屏的代码。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Allen-Kuang/e-ink_Demo">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载代码</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div><br />

**步骤 2.** 以使用 4.2 英寸电子墨水屏和 XIAO ESP32S3 为例，下载并解压示例代码后，打开 "4.2 inch E-paper - Monocolor 400x300" 文件夹，然后进入 "example" 文件夹，使用 Arduino 打开 "example.ino" 文件：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/3.png" style={{width:700, height:'auto'}}/></div>

**步骤 3.** 将通过 image2lcd 输出的 `.h` 文件放入与示例代码相同的文件夹，并将其包含在内：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/9.png" style={{width:500, height:'auto'}}/></div>

**步骤 4.** 点击板卡选择下拉菜单，然后点击 "选择其他板卡和端口..."，

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/4.png" style={{width:500, height:'auto'}}/></div>

**步骤 5.** 选择您的 XIAO 通信端口，然后搜索并选择 "XIAO_ESP32S3"。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/5.png" style={{width:700, height:'auto'}}/></div>

**步骤 6.** 请点击 "上传" 按钮将固件刷写到 XIAO 上。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/6.png" style={{width:500, height:'auto'}}/></div>

**步骤 7.** 您应该看到电子墨水屏刷新演示图像，如下所示：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/eInk/xiao-expansion/8.png" style={{width:500, height:'auto'}}/></div>

## 资源

- **[ZIP]** [PCB&SCH Eagle 文件](https://files.seeedstudio.com/wiki/eInk/xiao-expansion/epaperIO.zip)

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供不同的支持渠道，以确保您与我们产品的使用体验尽可能顺利。我们提供了多种沟通方式，以满足不同的需求和偏好。

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








