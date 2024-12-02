---
description: 有关如何从拥有的数据集、标记、训练和部署到 XIAO ESP32S3的指南。
title: 将 Datasets 中的模型部署到 XIAO ESP32S3
keywords:
- SSCMA
- xiao
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32s3_sscma
last_update:
  date: 11/27/2024
  author: Agnes
---


# 从数据集到 XIAO ESP32S3 的模型部署

欢迎来到这篇全面的教程，我们将在这里将你的数据集转化为一个完整的模型，并将其部署到 XIAO ESP32S3 上。在本指南中，我们将通过 Roboflow 的直观工具标注数据集，并在 Google Colab 的协作环境中进行模型训练。

接下来，我们将使用 SenseCraft Model Assistant 部署训练好的模型，这是一个连接训练与实际应用之间的桥梁。通过本教程的学习，不仅能够让你的自定义模型在 XIAO ESP32S3 上运行，你还将掌握如何解读和利用模型的预测结果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/0.png" style={{width:1000, height:'auto'}}/></div>

从数据集到模型落地，我们将进行以下主要步骤：

1. [标注数据集](#labelled-datasets) —— 本章节介绍如何获取可以用于训练的标注数据集。主要有两种方式，一种是使用 Roboflow 社区提供的标注数据集，另一种是使用你自己的场景图片作为数据集，但需要你手动进行标注。

2. [训练数据集导出模型](#training-dataset-exported-model) —— 本章节将介绍如何使用 Google Colab 平台进行训练，生成一个可以部署到 XIAO ESP32S3 的模型。

3. [通过 SenseCraft Model Assistant 上传模型](#upload-models-via-sensecraft-model-assistant) —— 本部分介绍如何使用导出的模型文件，借助 SenseCraft Model Assistant 上传模型到 XIAO ESP32S3。

4. [模型的常见协议与应用](#common-protocols-and-applications-of-the-model) —— 最后，我们将介绍 SenseCraft AI 的统一数据通信格式，帮助你最大限度地利用设备与模型的潜力，开发符合自己场景的应用。

让我们一起深入，开始将你的数据转化为现实的激动人心的过程吧。

## 所需材料

在开始之前，你需要准备以下设备：

<div class="table-center">
	<table align="center">
		<tr>
			<th>Seeed Studio XIAO ESP32S3</th>
			<th>Seeed Studio XIAO ESP32S3 Sense</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3sense.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html">
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
</div>

无论是 XIAO ESP32S3 版本还是 Sense 版本都可以作为本教程的内容使用，但由于标准版产品无法使用摄像头扩展板，因此我们建议使用 Sense 版本。

## 标注数据集

在本部分内容中，我们允许用户自由选择他们已有的数据集。这包括社区的或者是自己拍摄的场景图片。本教程将介绍两种主流的场景。第一种是使用 Roboflow [社区](https://roboflow.com/about) 提供的现成标注数据集，第二种是使用你自己拍摄的高清图像并标注数据集。请根据你的需求，阅读下面不同的教程。

### 第1步：创建一个免费的 Roboflow 账户

Roboflow 提供了标注、训练和部署计算机视觉解决方案所需的一切。要开始，请创建一个 [免费的 Roboflow 账户](https://app.roboflow.com/?ref=blog.roboflow.com)。

在审核并接受服务条款后，你将被要求在公共计划和入门计划之间选择其一。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/1.png" style={{width:800, height:'auto'}}/></div>

接下来，你将被要求邀请合作者加入你的工作区。这些合作者可以帮助你标注图像或管理工作区中的视觉项目。邀请完合作者后（如果你希望这样做），你就可以创建项目了。

### 选择如何获取数据集

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="Download Labelled datasets using Roboflow" label="使用 Roboflow 下载标注数据集">

选择一个适合的 Roboflow 数据集来直接使用，涉及确定最符合你项目要求的数据集，考虑数据集的大小、质量、相关性和许可等方面。

**第2步：浏览 Roboflow Universe**

Roboflow Universe 是一个你可以找到各种数据集的平台。访问 Roboflow Universe 网站，探索可用的数据集。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/2.png" style={{width:1000, height:'auto'}}/></div>

Roboflow 提供了筛选器和搜索功能，帮助你找到符合条件的数据集。你可以按领域、类别数、注释类型等进行筛选，使用这些筛选器来缩小符合标准的数据集范围。

**第3步：评估单个数据集**

一旦你有了候选数据集，逐一评估它们，查看：

**标注质量**：检查标注是否准确、一致。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/3.png" style={{width:1000, height:'auto'}}/></div>

**数据集大小**：确保数据集足够大，以便模型能够有效学习，但又不至于太大，无法处理。

**类别平衡**：数据集中的各类别应尽量平衡。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/4.png" style={{width:1000, height:'auto'}}/></div>

**许可证**：审查数据集的许可证，确保你能按预期使用它。

**文档**：审查任何与数据集一起提供的文档或元数据，以更好地理解其内容和已应用的任何预处理步骤。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/5.png" style={{width:1000, height:'auto'}}/></div>

:::tip
你可以通过 **[Roboflow 健康检查](https://docs.roboflow.com/datasets/dataset-health-check)** 查看模型的状态。
:::

**第4步：下载样本**

如果你找到了合适的数据集，你可以选择下载并使用它。Roboflow 通常允许你下载数据集的样本。测试样本，查看它是否与你的工作流程兼容，是否适合你的模型。

继续后续步骤时，我们建议将数据集导出为如下所示的格式。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/6.png" style={{width:1000, height:'auto'}}/></div>

你将获得该模型的 **原始 URL**，请将其保管好，我们稍后将在模型训练步骤中使用该链接。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/26.png" style={{width:1000, height:'auto'}}/></div>


:::caution
如果您第一次使用 Roboflow，并且对数据集的选择没有任何判断，使用一个数据集训练模型并进行初步测试以查看其性能可能是必要的。通过这个步骤，您可以评估该数据集是否符合您的要求。

如果数据集符合您的要求，并且在初步测试中表现良好，那么它很可能适合您的项目。否则，您可能需要继续寻找，或者考虑通过添加更多的图像来扩展数据集。
:::

</TabItem>

<TabItem value="Use your own images as a dataset" label="使用自己的图像作为数据集">

在这里，我将以石头-剪刀-布手势图像作为示例，带您完成在 Roboflow 上上传图像、标注和导出数据集的任务。

:::note
我们强烈推荐您使用 XIAO ESP32S3 拍摄数据集的图片，因为它对 XIAO ESP32S3 最为适配。关于如何使用 XIAO ESP32S3 Sense 拍照的示例程序，您可以在以下 Wiki 链接中找到。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/#taking-photos-with-the-camera">
            <strong><span><font color={'FFFFFF'} size={"4"}>访问 Wiki</font></span></strong>
    </a>
</div>
:::

**步骤 2. 创建新项目并上传图像**

登录 Roboflow 后，点击 **创建项目**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/7.png" style={{width:1000, height:'auto'}}/></div>

为您的项目命名（例如，“石头-剪刀-布”）。将项目类型定义为 **目标检测**。设置 **输出标签** 为 **分类**（因为石头、剪刀、布是不同的类别）。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/8.png" style={{width:1000, height:'auto'}}/></div>

现在是时候上传您的手势图像了。

收集石头、剪刀和布手势的图像。确保图像包含不同的背景和光照条件。在您的项目页面，点击“添加图像”。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/9.png" style={{width:1000, height:'auto'}}/></div>

您可以拖放图像，或从计算机中选择图像。上传至少 100 张每个手势的图像，以确保数据集的健壮性。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/10.png" style={{width:1000, height:'auto'}}/></div>

:::tip
**数据集大小如何确定？**

这通常取决于多个因素：任务模型、任务复杂性、数据的纯净度等等。例如，人体检测模型涉及大量人员，覆盖范围广，任务复杂，因此需要收集更多的数据。
另一个例子是手势检测模型，它只需要检测“石头”、“剪刀”和“布”这三种类型，类别较少，因此数据集大约为 500 张。
:::

**步骤 3: 图像标注**

上传完成后，您需要通过标注手势来标注这些图像。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/11.png" style={{width:1000, height:'auto'}}/></div>

Roboflow 提供三种不同的图像标注方式：自动标注、Roboflow 标注和手动标注。

- [**自动标注**](https://blog.roboflow.com/yolo-world-prompting-tips/): 使用大型通用模型自动标注图像。
- **Roboflow 标注**: 与专业的人工标注团队合作。无最低数量要求，无预付款承诺。边界框标注起价为 0.04 美元，多边形标注起价为 0.08 美元。
- **手动标注**: 您和您的团队自行标注图像。

以下是手动标注最常用方法的描述。

点击“手动标注”按钮，Roboflow 会加载标注界面。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/12.png" style={{width:1000, height:'auto'}}/></div>

点击“开始标注”按钮。在每张图像上绘制一个边界框，框住手势。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/13.gif" style={{width:1000, height:'auto'}}/></div>

将每个边界框标注为“石头”、“剪刀”或“布”。

使用“>”按钮在数据集中进行切换，为每张图像重复标注过程。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/14.gif" style={{width:1000, height:'auto'}}/></div>

**步骤 4: 审查和编辑标注**

确保标注准确非常重要。

查看每张图像，确保边界框正确绘制且标签正确。如果发现错误，选择该标注以调整边界框或更改标签。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/15.png" style={{width:1000, height:'auto'}}/></div>

:::tip
不正确的标注会影响训练的整体性能，如果某些数据集未达到标注要求，可以被丢弃。以下是一些不良标注的示例。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/16.png" style={{width:700, height:'auto'}}/></div>
:::

**步骤 5: 生成并导出数据集**

所有图像标注完成后，在“标注”页面点击右上角的 **将 x 张图像添加到数据集** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/17.png" style={{width:1000, height:'auto'}}/></div>

然后点击新弹出窗口底部的 **添加图像** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/18.png" style={{width:400, height:'auto'}}/></div>

点击左侧工具栏中的 **生成**，然后点击第三步 **预处理** 中的 **继续**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/19.png" style={{width:1000, height:'auto'}}/></div>

在第 4 步的 **增强** 部分，选择 **马赛克**，以增加泛化能力。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/20.png" style={{width:1000, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/21.png" style={{width:1000, height:'auto'}}/></div>

在最终的 **创建** 步骤中，请根据 Roboflow 的提升合理计算图像数量；通常来说，图像越多，训练模型所需时间越长。然而，更多的图像并不一定会让模型更准确，主要取决于数据集是否足够好。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/22.png" style={{width:1000, height:'auto'}}/></div>

点击 **创建** 创建您的数据集版本。Roboflow 将处理图像和标注，创建一个版本化的数据集。数据集生成后，点击 **导出数据集**。选择符合您要训练的模型要求的 **COCO** 格式。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/23.png" style={{width:1000, height:'auto'}}/></div>

点击 **继续**，您将获得该模型的原始 URL。请保存该链接，我们将在模型训练步骤中稍后使用它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/27.png" style={{width:1000, height:'auto'}}/></div>

恭喜您！您已经成功使用 Roboflow 上传、标注并导出了一个用于石头-剪刀-布手势检测模型的数据集。数据集准备好后，您可以使用 Google Colab 等平台继续训练机器学习模型。

记得保持数据集的多样性和良好的标注，以提高未来模型的准确性。祝您的模型训练顺利，享受利用 AI 分类手势的乐趣！
</TabItem>
</Tabs>


## 训练数据集导出模型

### 步骤 1. 访问 Colab Notebook

您可以在 [SenseCraft 模型助手的 Wiki](https://wiki.seeedstudio.com/ModelAssistant_Introduce_Quick_Start/#model-training) 中找到不同类型的 Google Colab 代码文件。如果您不确定选择哪个代码文件，可以根据您的模型类别（目标检测或图像分类）选择其中的任何一个。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/24.png" style={{width:1000, height:'auto'}}/></div>

如果您尚未登录 Google 账户，请先登录以访问 Google Colab 的全部功能。

点击“连接”以为您的 Colab 会话分配资源。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/25.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 2. 添加您的 Roboflow 数据集

在正式逐步运行代码块之前，我们需要修改代码内容，以便代码可以使用我们准备的数据集。我们需要提供一个 URL，将数据集直接下载到 Colab 文件系统中。

请在代码中找到 **下载数据集** 部分。您将看到如下示例程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/28.png" style={{width:1000, height:'auto'}}/></div>

```sh
%mkdir -p Gesture_Detection_Swift-YOLO_192/dataset 
!wget -c https://universe.roboflow.com/ds/xaMM3ZTeWy?key=5bznPZyI0t -O Gesture_Detection_Swift-YOLO_192/dataset.zip 
!unzip -q Gesture_Detection_Swift-YOLO_192/dataset.zip -d Gesture_Detection_Swift-YOLO_192/dataset
```

这段代码用于创建一个目录，从 Roboflow 下载数据集，并将其解压到 Google Colab 环境中的新创建目录中。下面是每行代码的解释：

1. `%mkdir -p Gesture_Detection_Swift-YOLO_192/dataset`:
   - 这一行创建一个名为 `Gesture_Detection_Swift-YOLO_192` 的新目录，并在其中创建一个子目录 `dataset`。`-p` 参数确保如果目录已存在，不会返回错误，并会创建必要的父目录。

2. `!wget -c https://universe.roboflow.com/ds/xaMM3ZTeWy?key=5bznPZyI0t -O Gesture_Detection_Swift-YOLO_192/dataset.zip`:
   - 这一行使用 `wget` 命令行工具从提供的 Roboflow URL 下载数据集。`-c` 参数允许在下载中断时恢复下载，`-O` 参数指定下载文件的输出位置和文件名，在这里是 `Gesture_Detection_Swift-YOLO_192/dataset.zip`。

3. `!unzip -q Gesture_Detection_Swift-YOLO_192/dataset.zip -d Gesture_Detection_Swift-YOLO_192/dataset`:
   - 这一行使用 `unzip` 命令将 `dataset.zip` 文件的内容解压到之前创建的 `dataset` 目录中。`-q` 参数让 `unzip` 命令在静默模式下运行，抑制大部分输出信息。

要将此代码定制为您自己的 Roboflow 模型链接：

1. 将 `Gesture_Detection_Swift-YOLO_192` 替换为您希望存储数据集的目标目录名称。

2. 将 Roboflow 数据集的 URL (`https://universe.roboflow.com/ds/xaMM3ZTeWy?key=5bznPZyI0t`) 替换为您导出数据集的链接（这是我们在 [标注数据集的最后一步](#choose-how-you-get-your-dataset) 中获得的原始 URL）。确保如果需要访问密钥，包含该参数。

3. 如果有必要，调整 `wget` 命令中的输出文件名（`-O your_directory/your_filename.zip`）。

4. 确保 `unzip` 命令中的输出目录与您创建的目录一致，且文件名与您在 `wget` 命令中设置的文件名相匹配。

:::caution
如果您更改了文件夹目录名 `Gesture_Detection_Swift-YOLO_192`，请注意，您还需要修改代码中其他使用该目录名的部分，否则可能会出现错误！
:::

### 步骤 3. 调整模型参数

接下来需要调整模型的输入参数。请跳转到 **使用 SSCMA 训练模型** 部分，您将看到以下代码片段。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/29.png" style={{width:1000, height:'auto'}}/></div>

```sh
!sscma.train configs/swift_yolo/swift_yolo_tiny_1xb16_300e_coco.py \
--cfg-options  \
    work_dir=Gesture_Detection_Swift-YOLO_192 \
    num_classes=3 \
    epochs=10  \
    height=192 \
    width=192 \
    data_root=Gesture_Detection_Swift-YOLO_192/dataset/ \
    load_from=Gesture_Detection_Swift-YOLO_192/pretrain.pth
```

此命令用于启动机器学习模型（特别是 YOLO（You Only Look Once）模型）的训练过程，使用的是 SSCMA（Seeed Studio SenseCraft Model Assistant）框架。该命令包含多个选项来配置训练过程。下面是每部分的解释：

- `!sscma.train` 是启动 SSCMA 框架中的训练命令。

- `configs/swift_yolo/swift_yolo_tiny_1xb16_300e_coco.py` 指定了训练的配置文件，其中包括模型架构、训练计划、数据增强策略等设置。

- `--cfg-options` 允许您通过命令行覆盖 `.py` 文件中指定的默认配置。

- `work_dir=Gesture_Detection_Swift-YOLO_192` 设置了训练输出（如日志和保存的模型检查点）将存储的目录。

- `num_classes=3` 指定模型应训练识别的类别数。它取决于您拥有的标签数量，例如石头、剪刀、布应该是三个标签。

- `epochs=10` 设置训练的周期数（epochs）。推荐值通常在 50 到 100 之间。

- `height=192` 和 `width=192` 设置模型期望的输入图像的高度和宽度。

:::caution
我们不建议您更改 Colab 代码中的图像大小，因为这个值是我们验证过的一个合适数据集大小，经过验证能在大小、准确性和推理速度之间取得平衡。如果您使用的不是这种尺寸的图像，且想确保准确性，可以考虑更改图像大小，但请不要超过 240x240。
:::

- `data_root=Gesture_Detection_Swift-YOLO_192/dataset/` 定义了训练数据所在目录的路径。

- `load_from=Gesture_Detection_Swift-YOLO_192/pretrain.pth` 提供了一个预训练模型检查点文件的路径，模型将从该文件恢复训练，或者作为迁移学习的起点。

要根据您的训练自定义此命令，您需要：

1. 如果有自定义的配置文件，将 `configs/swift_yolo/swift_yolo_tiny_1xb16_300e_coco.py` 替换为您的配置文件路径。

2. 将 `work_dir` 更改为您希望保存训练输出的目录。

3. 更新 `num_classes` 为您数据集中类别的数量。它取决于您拥有的标签数量，例如石头、剪刀、布应该是三个标签。

4. 调整 `epochs` 为您模型所需的训练周期数。推荐值通常在 50 到 100 之间。

5. 设置 `height` 和 `width` 为与您的模型输入图像尺寸匹配的值。

6. 将 `data_root` 更改为指向您的数据集根目录的路径。

7. 如果您有不同的预训练模型文件，请相应地更新 `load_from` 路径。


### 步骤 4. 运行 Google Colab 代码

运行代码块的方法是点击代码块左上角的播放按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/30.png" style={{width:1000, height:'auto'}}/></div>

点击按钮后，代码块将会执行，如果一切顺利，您将看到代码块执行完成的标志——一个勾号符号会出现在代码块左侧。如图所示，这是执行第一个代码块完成后的效果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/31.png" style={{width:1000, height:'auto'}}/></div>

如果您遇到与我上图相同的错误信息，请检查您是否正在使用 **T4 GPU**，请 **不要使用 CPU** 进行此项目。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/32.png" style={{width:400, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/33.png" style={{width:600, height:'auto'}}/></div>

然后，重新执行代码块。对于第一个代码块，如果一切顺利，您将看到如下结果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/34.png" style={{width:1000, height:'auto'}}/></div>

接下来，执行从 **下载预训练模型权重文件** 到 **导出模型** 的所有代码块。并请确保每个代码块都没有错误。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/36.png" style={{width:400, height:'auto'}}/></div>

:::note
代码中出现的警告可以忽略。
:::

### 步骤 5. 评估模型

当您到达 **评估模型** 部分时，您可以选择执行 **评估 TFLite INT8 模型** 代码块。

:::tip
评估 TFLite INT8 模型涉及将量化模型的预测与单独的测试数据集进行对比，以衡量其准确性和性能指标，评估量化对模型精度的影响，并分析其推理速度和资源使用情况，以确保它满足边缘设备的部署约束。
:::

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/35.png" style={{width:1000, height:'auto'}}/></div>

以下是我执行此代码块后有效的部分结果。

```
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.450
 Average Precision  (AP) @[ IoU=0.50      | area=   all | maxDets=100 ] = 0.929
 Average Precision  (AP) @[ IoU=0.75      | area=   all | maxDets=100 ] = 0.361
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = -1.000
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.474
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.456
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=  1 ] = 0.515
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets= 10 ] = 0.529
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.529
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = -1.000
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.536
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.537
03/19 01:38:43 - mmengine - INFO - bbox_mAP_copypaste: 0.450 0.929 0.361 -1.000 0.474 0.456
{'coco/bbox_mAP': 0.45, 'coco/bbox_mAP_50': 0.929, 'coco/bbox_mAP_75': 0.361, 'coco/bbox_mAP_s': -1.0, 'coco/bbox_mAP_m': 0.474, 'coco/bbox_mAP_l': 0.456}
FPS: 128.350449 fram/s
```

评估结果包括一系列平均精度（AP）和平均召回率（AR）指标，计算时考虑了不同的交并比（IoU）阈值和物体大小，这些是评估目标检测模型性能时常用的指标。

1. **AP@[IoU=0.50:0.95 | area=all | maxDets=100] = 0.450**
   - 这个得分是模型在 IoU 阈值从 0.50 到 0.95 范围内的平均精度，增量为 0.05。0.450 的 AP 表示模型在这个范围内有中等的准确度。这是 COCO 数据集中常用的关键指标。

2. **AP@[IoU=0.50 | area=all | maxDets=100] = 0.929**
   - 在 IoU 阈值为 0.50 时，模型的平均精度为 0.929，说明模型在较宽松的匹配标准下检测物体非常准确。

3. **AP@[IoU=0.75 | area=all | maxDets=100] = 0.361**
   - 在更严格的 IoU 阈值 0.75 下，模型的平均精度下降到 0.361，表明在严格匹配标准下性能有所下降。

4. **AP@[IoU=0.50:0.95 | area=small/medium/large | maxDets=100]**
   - 对于不同大小的物体，AP 分数有所不同。然而，小物体的 AP 为 -1.000，这可能表示没有足够的小物体评估数据，或者模型在小物体检测方面的表现较差。中等和大型物体的 AP 分别为 0.474 和 0.456，表明模型在中大型物体检测方面表现较好。

5. **AR@[IoU=0.50:0.95 | area=all | maxDets=1/10/100]**
   - 不同 `maxDets` 值下的平均召回率相对一致，范围为 0.515 到 0.529，表明模型能够可靠地检索大部分真正的正例。

6. **FPS: 128.350449 fram/s**
   - 模型在推理过程中以大约 128.35 帧每秒的速度处理图像，表明其具有实时或接近实时的处理能力。

总体而言，模型在 IoU 为 0.50 时表现非常优秀，在 IoU 为 0.75 时表现中等。它在中大型物体检测上表现更好，但可能在小物体检测上存在问题。此外，模型推理速度非常快，适用于需要快速处理的场景。如果在应用中小物体的检测至关重要，可能需要进一步优化模型或收集更多小物体数据以提高性能。

### 步骤 6. 下载导出的模型文件

在 **导出模型** 部分之后，您将获得不同格式的模型文件，这些文件默认存储在 ModelAssistant 文件夹中。在本教程中，存储目录是 **Gesture_Detection_Swift_YOLO_192**。

:::tip
有时 Google Colab 不会自动刷新文件夹的内容。在这种情况下，您可能需要通过点击左上角的刷新图标来刷新文件目录。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/39.png" style={{width:500, height:'auto'}}/></div>

:::

在上述目录中，**.tflite** 格式的模型文件可用于 XIAO ESP32S3 和 Grove Vision AI V2。对于 XIAO ESP32S3 Sense，请确保选择使用 **xxx

_int8.tflite** 格式的模型文件。其他格式的文件无法被 XIAO ESP32S3 Sense 使用。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/37.png" style={{width:400, height:'auto'}}/></div>

一旦找到模型文件，请尽快将它们下载到您的电脑上，因为如果您长时间闲置，Google Colab 可能会清空您的存储目录！

通过这里进行的步骤，我们已经成功导出了可以支持 XIAO ESP32S3 的模型文件，接下来让我们将模型部署到设备上。


## 通过 SenseCraft Model Assistant 上传模型

### 步骤 7. 上传自定义模型到 XIAO ESP32S3

接下来，我们来到了 Model Assistant 页面。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://seeed-studio.github.io/SenseCraft-Web-Toolkit/#/setup/process">
            <strong><span><font color={'FFFFFF'} size={"4"}>Model Assistant 🖱️</font></span></strong>
    </a>
</div>
<br></br>

请在选择 XIAO ESP32S3 后连接设备，然后选择页面底部的 **上传自定义 AI 模型**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaos3-sscma/1.png" style={{width:1000, height:'auto'}}/></div>

然后，您需要准备模型名称、模型文件和标签。我想在这里特别说明如何确定标签 ID 这一元素。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/41.png" style={{width:500, height:'auto'}}/></div>

**如果您是直接下载 Roboflow 的数据集**

如果您直接下载了 Roboflow 的数据集，那么您可以在健康检查页面查看不同的类别及其顺序。只需安装此处输入的顺序即可。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/42.png" style={{width:1000, height:'auto'}}/></div>

:::tip
您不需要在 **ID:Object** 中填写数字，只需直接填写类别名称，图像中类别前面的数字和冒号会自动添加。
:::

**如果您使用的是自定义数据集**

如果您使用的是自定义数据集，您可以在健康检查页面查看不同的类别及其顺序。只需安装此处输入的顺序即可。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/43.png" style={{width:1000, height:'auto'}}/></div>

:::tip
您不需要在 **ID:Object** 中填写数字，只需直接填写类别名称，图像中类别前面的数字和冒号会自动添加。
:::

然后点击右下角的发送模型按钮。这个过程可能需要约 3 到 5 分钟左右。如果一切顺利，您将能够在上方的模型名称和预览窗口中看到您的模型结果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/visionai_v2_train_model/44.png" style={{width:1000, height:'auto'}}/></div>

至此，恭喜您，您已经成功地训练并部署了自己的模型。

## 模型的常见协议和应用

在上传自定义模型的过程中，除了我们可以可视化上传的模型文件外，还有设备的固件需要传输到设备中。在设备的固件中，包含了一套已建立的通信协议，规定了模型结果输出的格式，以及用户可以对模型执行的操作。

由于篇幅问题，我们在此 Wiki 中不会展开这些协议的具体细节，我们将在 Github 上通过文档详细说明这一部分。如果您对更深入的开发感兴趣，请点击以下链接。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://github.com/Seeed-Studio/SSCMA-Micro/blob/dev/docs/protocol/at_protocol.md">
            <strong><span><font color={'FFFFFF'} size={"4"}>SenseCraft 协议</font></span></strong>
    </a>
</div>
<br></br>

## 故障排除

### 1. 如果我按照步骤操作，最终模型结果不尽如人意，该怎么办？

如果您的模型识别精度不理想，您可以通过以下几个方面进行诊断和改进：

1. **数据质量和数量**
   - **问题**：数据集可能太小或缺乏多样性，或者标注存在不准确的情况。
   - **解决方案**：增加训练数据的规模和多样性，进行数据清洗，修正标注错误。

2. **训练过程**
   - **问题**：训练时间可能不够，或者学习率设置不当，导致模型无法有效学习。
   - **解决方案**：增加训练周期，调整学习率和其他超参数，并实现早期停止以避免过拟合。

3. **类别不平衡**
   - **问题**：某些类别的样本远多于其他类别，导致模型偏向于多数类。
   - **解决方案**：使用类别权重、过采样少数类或欠采样多数类来平衡数据。

通过全面分析并实施有针对性的改进，您可以逐步提高模型的准确性。记得在每次修改后使用验证集测试模型的性能，以确保改进有效。

### 2. 为什么我按照 Wiki 中的步骤操作后，在 SenseCraft 部署中看到 **Invoke failed** 消息？

如果遇到 **Invoke failed**，说明您训练的模型不符合设备使用的要求。请关注以下几个方面：

1. 请检查是否修改了 Colab 中的图像大小。默认的压缩大小为 **192x192**，而 Grove Vision AI V2 要求图像大小为正方形，请不要使用非正方形尺寸进行压缩。并且不要使用过大的尺寸（推荐不要超过 240x240）。

<!-- 2. Grove Vision AI V2 的模型文件必须以 **int8_vela.tflite** 为后缀。请不要使用其他格式的模型文件。这包括 **int8.tflite**，该格式也不适用于 Grove Vision AI V2。 -->

## 技术支持与产品讨论

感谢您选择我们的产品！我们致力于为您提供多种支持，确保您的产品使用体验尽可能顺利。我们提供了多种沟通渠道，以满足不同的需求和偏好。


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

