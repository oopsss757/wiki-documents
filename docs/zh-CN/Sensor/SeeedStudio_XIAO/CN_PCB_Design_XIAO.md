---
description: XIAO 的 PCB 设计
title: XIAO 的 PCB 设计
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/PCB_Design_XIAO
last_update:
  date: 11/27/2024
  author: Agnes
---

# 在 Flux.ai 上创建 XIAO 组件 中文版

Flux 是一个基于浏览器的 PCB 设计工具，旨在实现电子团队之间的无缝协作。该工具具有直观的界面，使用户能够快速轻松地创建原理图和布局，同时其内置的仿真功能有助于确保设计准确无误。

在本节中，我们将介绍如何在 Flux.ai 上创建 Seeed Studio XIAO 系列组件。

## 浏览器上的 Seeed Studio XIAO 系列 PCB 设计

### Seeed Studio XIAO SAMD21

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/cnaville89/seeed-xiao-samd21?editor=pcb_3d&embed=1">
</iframe>

### Seeed Studio XIAO RP2040

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-rp2040?editor=schematic&embed=1">
</iframe>

### Seeed Studio XIAO nRF52840

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-nrf52840?editor=schematic&embed=1">
</iframe>

### Seeed Studio XIAO nRF52840 Sense

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-nrf52840-sense?editor=pcb_3d&embed=1">
</iframe>

### Seeed Studio XIAO ESP32C3

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-esp32c3?editor=schematic&embed=1">
</iframe>

### Seeed Studio XIAO ESP32S3

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-esp32s3?editor=schematic&embed=1">
</iframe>

### Seeed Studio XIAO ESP32S3 Sense

<iframe height={450} width={800} allowFullScreen src="https://www.flux.ai/gokux/seeed-studio-xiao-esp32s3-sense?editor=schematic&embed=1">
</iframe>

## Flux.ai 知识 - 创建部件

Flux 中的部件由 5 个主要组件构成。所有这些组件都是可选的，但缺少某个组件的部件将无法提供完整的功能：

| 概念 | 描述 |
| --- | --- |
| 原理图 | 部件的“内部”视图，仅由端子表示。 |
| 符号 | 当部件拖入另一个项目时的表示，通常为用户熟悉的其他工具中的符号。 |
| 封装 | 表示部件如何在 PCB 上放置。 |
| 3D 模型 | 显示部件的 3D 形状和尺寸。 |
| 仿真模型 | 描述部件在仿真过程中应如何表现。 |

## 入门指南

### 步骤 1 - 创建新的部件原理图

第一步是创建一个新的空白项目，可以在 Flux 主菜单的右上角完成。端子是每个部件的基础，它们使部件能够与电路的其余部分进行交互。要向新部件添加端子，进入库，搜索“端子”，然后拖动所需数量的端子。

在本示例中，我们将添加 Seeed Studio XIAO ESP32S3，因此我添加了 14 个端子引脚，并为它们命名和编号。

您可以在部件属性中提供更多关于部件的信息，如制造商部件号（MPN）、制造商名称、数据表 URL 等。输入部件的 MPN 号将帮助您查看部件的当前库存和价格。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO.png" /></div>

### 步骤 2 - 创建符号

Flux 的工作方式与您可能使用的其他工具略有不同。在 Flux 中，部件有两种不同的视图：原理图视图和符号视图。步骤 1 中的原理图视图只包含端子，符号则仅在部件放置到项目中时可见。现在，让我们为 XIAO 创建一个符号，但我们需要使用一些外部工具，如 Illustrator 或 Inkscape。设计的符号格式需要是 .svg。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO2.png" /></div>

设计符号时需要注意的事项：
- 每个形状和线条应为白色，线条宽度为 1px，且没有填充。
- 引脚通常长 10 至 18 像素。

现在将符号导出为 SVG 文件。

### 步骤 3 - 将 SVG 作为资产添加

一旦您拥有了 SVG 文件，就可以将其作为资产添加。在添加外部文件作为资产之前，请确保没有选中任何对象（点击空白画布）。在右侧面板中，滚动到底部找到“资产”面板，点击展开并点击“添加”（或“管理”）。这将打开资产对话框。然后点击“添加项目”并从您的本地驱动器中选择文件。

**匹配引脚位置与自定义符号。**
默认情况下，所有端子都位于符号的中心。要将端子放置到所需位置，您需要执行以下几个步骤：

1. 将部件发布到库中。
2. 创建一个新的空白项目，并拖入您导入的部件。
3. 您会注意到，两个端子都位于符号的中心。现在回到导入的部件。
4. 您需要为部件中的每个端子执行此过程：
   a) 选择端子并在右侧面板中找到“属性”菜单。
   b) 在“符号引脚位置”字段中，输入端子应放置的 x 和 y 坐标。
   c) 发布部件并返回到新项目。您会看到底部左侧有“您的部件有更新”提示。点击“查看”并接受更改。
   d) 您会注意到端子已经移动。您可能需要重复此过程几次，直到找到完美的位置。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO3.png" /></div>

### 步骤 4 - 创建封装

在 Flux 中创建封装非常简单。封装由焊盘、线条、形状和文本节点组成，这些元素可以直接在 Flux PCB 编辑器中添加。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO4.png" /></div>

当你第一次在 Flux 中创建封装时，所有焊盘会集中在一个位置，显示为小点。
- 要改变焊盘位置：
  - 选择要移动的焊盘，在右侧面板的“对象特定规则”中找到“位置规则”，
  - 输入所需的 x 和 y 坐标，单位为毫米。

### 步骤 5 - 修改焊盘大小和形状

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO5.png" /></div>

点击焊盘后，您可以修改其形状、位置、孔径和其他属性。对于 XIAO，我使用了 3mm x 2mm 的焊盘，孔径为 1.1mm。通过调整 x 和 y 坐标，确保每个引脚间隔 2.54mm。

**添加 3D 模型**

现在，我们需要添加 XIAO 的 3D 模型，Flux 支持 .step 格式的 3D 模型，您可以从官方 Wiki 页面下载该文件。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO6.png" /></div>

您可以从“辅助”部分上传 3D 模型。关于如何添加 3D 模型的详细信息，您可以参考视频教程。您可以通过“对象特定规则”来调整模型的 x、y、z 位置和旋转角度。使用这些设置，您可以将 3D 模型定位在焊盘的顶部。

**发布到库中**

创建完部件后，就可以将其发布了。

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/PCB_Design_Flux_XIAO/PCB_Design_XIAO7.png" /></div>

选择左上角的 Flux 图标，然后选择“发布更改”。现在，我们的部件将出现在我们的个人资料中，同时也会显示在公共库的搜索结果中。

## 更多内容 - 教程视频

<iframe width={560} height={315} src="https://www.youtube.com/embed/5cGg5n6sXJE?si=nSYvVSl-q3axb4Ss" title="YouTube 视频播放器" frameBorder={0} allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen />

## ✨ 贡献者项目

- 本项目得到了 [Seeed Studio 贡献者项目](https://github.com/orgs/Seeed-Studio/projects/6) 的支持。
- 感谢 [Gokul](https://github.com/orgs/Seeed-Studio/projects/6/views/1?pane=issue&itemId=42323283) 的努力，您的作品将被 [展示](https://wiki.seeedstudio.com/Honorary-Contributors/)。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时能够享受到顺畅的体验。我们提供多种沟通渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
