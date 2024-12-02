---
description: 本教程介绍了如何使用 XIAO ESP32S3，录制语音，识别语音，然后向 ChatGPT 提问并获得答案并显示在屏幕上。
title: 基于 XIAO ESP32S3 Sense 的微型 ChatGPT 语音助手
keywords:
- xiao esp32s3 sense
- chatGPT
- speech to text
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32s3_speech2chatgpt
last_update:
  date: 11/27/2024
  author: Agnes
---

# 基于 XIAO ESP32S3 的微型 ChatGPT 语音助手

<iframe width="100%" height="400" src="https://www.youtube.com/embed/wPi-XjeJPNw?controls=0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

今天，我们很高兴为大家带来一个全新的项目，使用 XIAO ESP32S3 Sense 和 XIAO 圆形显示屏！该项目的目标是首先利用 XIAO ESP32S3 Sense 的麦克风和 Google Cloud 的语音转文字服务，构建一个语音识别系统。然后，将识别到的语音文本用来调用 OpenAI 的接口，向 ChatGPT 提问并返回答案。最后，我们将显示识别到的语音文本和 ChatGPT 的回答内容在屏幕上。

这就是我们的智能 "XIAO" 助手！

让我们来看看完成这个项目所需的一些基本步骤。

- [注册并启用 Google Cloud 语音转文本服务](#sign-up-and-enable-google-cloud-speech-to-text-service)
- [在本地部署语音转文本服务](#deploy-speech-to-text-services-on-local-hosts)
- [将 XIAO ESP32S3 Sense 录制的音频文件上传到 Google Cloud 进行识别](#upload-xiao-esp32s3-sense-recorded-sound-files-to-google-cloud-for-recognition)
- [在 XIAO ESP32S3 Sense 上部署 ChatGPT](#deploy-chatgpt-on-xiao-esp32s3-sense)
- [设计屏幕显示内容与程序集成](#design-of-screen-display-content--integration-of-programs)

下面的图示展示了整个项目的框架结构。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/17.png" style={{width:1000, height:'auto'}}/></div>

## 开始之前

在开始这个项目之前，您可能需要提前准备好硬件和软件，如下所述。

### 硬件准备

如果您想完整体验整个程序内容，您至少需要以下硬件设备。

<div class="table-center">
  <table align="center">
    <tr>
        <th>Seeed Studio XIAO ESP32S3 Sense</th>
        <th>Seeed Studio XIAO 圆形显示屏</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3sense.jpg" style={{width:250, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/round_display_for_xiao/rounddisplay.jpg" style={{width:250, height:'auto'}}/></div></td>
    </tr>
      <tr>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
          </a>
      </div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-Round-Display-for-XIAO-p-5638.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
          </a>
      </div></td>
    </tr>
  </table>
</div>

除此之外，我们还需要一张格式为 FAT32，大小不超过 32GB 的 microSD 卡来存储录音文件。

由于 XIAO ESP32S3 Sense 的设计中，SD 卡插槽连接了三个上拉电阻（R4~R6），而圆形显示屏也有上拉电阻，因此当同时使用两者时，SD 卡无法正常读取。为了解决这个问题，我们需要切断 XIAO ESP32S3 Sense 扩展板上的 J3 连接。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/33.png" style={{width:500, height:'auto'}}/></div>

切断 J3 后，XIAO ESP32S3 Sense 上的 SD 卡插槽将无法正常工作，因此需要将 microSD 卡插入圆形显示屏上的 SD 卡插槽。

接下来，请按顺序安装 microSD 卡、XIAO ESP32S3 Sense 和圆形显示屏。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/101.gif" style={{width:500, height:'auto'}}/></div>

:::tip
我们建议您先拆下摄像头模块，以免在切割 J3 连接时刮伤摄像头。
:::

### 软件准备

由于使用的是 XIAO ESP32S3，请按照 Wiki 上的说明安装 XIAO ESP32S3 开发板包。

- [Seeed Studio XIAO ESP32S3 入门指南](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#software-preparation)

此外，我们还使用了 XIAO 的圆形显示屏，因此您还需要按照 Wiki 上的说明准备扩展板的库。

- [Seeed Studio XIAO 圆形显示屏入门](https://wiki.seeedstudio.com/get_start_round_display/#getting-started)

在项目过程中，我们还可能使用一些第三方库，如 ChatGPT 的库和 ArduinoJSON，可以从以下链接下载并添加到 Arduino 开发环境中。

- [库](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/tree/main/libraries)

除了基础库外，我们还需要使用 Node 服务，因此您需要自行安装 Nodejs，您可以直接从 [官网](https://nodejs.org/en) 下载。

一切准备就绪，接下来我们开始今天的教程。

## 注册并启用 Google Cloud 语音转文本服务

:::tip
您也可以直接参考 [Google Cloud 官方教程](https://cloud.google.com/speech-to-text/docs/before-you-begin#setting_up_your_google_cloud_platform_project)，了解如何注册并启动 Google Cloud 语音转文本服务进行配置。
:::

语音转文本（Speech-to-Text）是由 Google 的人工智能（AI）技术支持的 API。您将音频数据发送到 Speech-to-Text，然后收到音频数据的文本转录结果。在开始向 Speech-to-Text 发送请求之前，您必须在 Google Cloud 控制台中启用该 API。

### 步骤 1. 登录 Google Cloud 控制台

您可以通过点击 [这里](https://console.cloud.google.com/?_ga=2.241031875.1758680688.1685496686-1606155345.1684977559) 跳转到 Google Cloud 控制台。如果您尚未注册 Google Cloud，您可以 [点击这里](https://console.cloud.google.com/?_ga=2.241031875.1758680688.1685496686-1606155345.1684977559) 注册。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/18.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 2. [进入项目选择页面](https://console.cloud.google.com/projectselector2/home/dashboard?_ga=2.5754355.1758680688.1685496686-1606155345.1684977559)

您可以选择一个现有的项目或创建一个新项目。有关创建项目的更多信息，请参阅 [创建和管理项目](https://cloud.google.com/resource-manager/docs/creating-managing-projects)。

如果您创建新项目，系统会提示您将计费账户关联到该项目。如果您使用的是已有项目，请确保该项目已启用计费。

:::note
注意：您必须启用计费才能使用 Speech-to-Text API，然而，除非超出免费配额，否则不会收取费用。有关详细信息，请参阅 [定价](https://cloud.google.com/speech-to-text/pricing) 页面。
:::

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/2.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 3. 启动语音转文本服务

一旦选择了项目并将其与计费账户关联，您可以启用 Speech-to-Text API。前往页面顶部的搜索产品和资源栏，输入 **speech**。从搜索结果中选择 **Cloud Speech-to-Text API**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/1.png" style={{width:600, height:'auto'}}/></div>

### 步骤 4. 创建服务账户

如果您的项目尚未有服务账户，您需要创建一个。使用 Speech-to-Text 服务必须有一个服务账户。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/3.png" style={{width:600, height:'auto'}}/></div>

在弹出的新页面中，选择 **CREATE CREDENTIALS** 下的 **Service account**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/4.png" style={{width:1000, height:'auto'}}/></div>

在 **服务账户名称** 框中，输入新服务账户的唯一名称。您的输入会自动填充到 **Service account ID** 框中。**Service account description** 框是可选的，但如果您打算将多个服务账户与项目关联，建议填写该描述。输入简短的描述后，点击 **CREATE AND CONTINUE**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/5.png" style={{width:500, height:'auto'}}/></div>

我们建议您为服务账户分配一个基础的 IAM 角色。如果需要，您还可以为单个服务账户分配多个角色。有关可用角色及其权限的详细信息，请参阅 [IAM 角色](https://cloud.google.com/iam/docs/understanding-roles)。点击下拉菜单中的 **Select a role**，然后向下滚动至 **Owner**，从右侧栏中选择角色。点击 **CONTINUE**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/6.png" style={{width:500, height:'auto'}}/></div>

最后一步，您可以选择是否允许其他实体（个人、Google 群组等）访问您的服务账户。如果不需要授予额外权限，可以点击 **DONE** 而不输入任何信息。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/7.png" style={{width:500, height:'auto'}}/></div>

现在，服务账户已列出在 **Service Accounts** 页面上。您可以随时更改服务账户的权限、添加或生成新的密钥，并授予访问权限。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/8.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 5. 为服务账户创建 JSON 密钥

在向 Speech-to-Text 发送请求时，您需要使用此私钥进行 [身份验证](https://cloud.google.com/speech-to-text/docs/before-you-begin#set_up_your_environment_variables)。

要创建密钥，点击服务账户并选择 **KEYS** 标签。点击 **ADD KEY -> Create new key**。我们建议您创建 JSON 格式的密钥。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/9.png" style={{width:800, height:'auto'}}/></div>

系统将自动下载您选择格式的新密钥。请将此文件保存在安全的地方，并记录文件路径。在每次开始新的 Speech-to-Text 会话时，您需要将 **GOOGLE_APPLICATION_CREDENTIALS** 环境变量指向此文件。这是进行 Speech-to-Text 请求身份验证的必要步骤。密钥的唯一 ID 会出现在服务账户名称旁边。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/10.png" style={{width:1000, height:'auto'}}/></div>

:::note
请保持 JSON 格式的密钥，因为我们将在后续步骤中使用它。
:::

## 在本地主机上部署语音转文本服务

### 步骤 6. 下载项目文件

我们已经将完成整个教程所需的项目文件打包，您可以直接从 Github 下载，或者使用 Git 命令将其下载到本地。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载项目</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

<br />

```
git clone https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT.git
```

同时，您可以将我们在 **步骤 5** 中准备的 JSON 文件复制到 **NodejsServer** 文件夹中，稍后我们会使用它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/19.png" style={{width:600, height:'auto'}}/></div>

### 步骤 7. 设置身份验证环境变量

为了设置 **GOOGLE_APPLICATION_CREDENTIALS**，您必须拥有与项目关联的服务账户，并访问该服务账户的 JSON 密钥。

通过设置环境变量 **GOOGLE_APPLICATION_CREDENTIALS**，为您的应用程序代码提供身份验证凭据。

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="Windows" label="Windows">

对于 PowerShell：

```
$env:GOOGLE_APPLICATION_CREDENTIALS="KEY_PATH"
```

将 **KEY_PATH** 替换为包含您的服务账户密钥的 JSON 文件路径。

例如：

```
$env:GOOGLE_APPLICATION_CREDENTIALS="C:\Users\username\Downloads\service-account-file.json"
```

对于命令提示符：

```
set GOOGLE_APPLICATION_CREDENTIALS=KEY_PATH
```

将 **KEY_PATH** 替换为包含您的服务账户密钥的 JSON 文件路径。

</TabItem>


<TabItem value="MacOS or Linux" label="MacOS 或 Linux">

```
export GOOGLE_APPLICATION_CREDENTIALS="KEY_PATH"
```

将 **KEY_PATH** 替换为包含您的服务账户密钥的 JSON 文件路径。

例如：

```
export GOOGLE_APPLICATION_CREDENTIALS="/home/user/Downloads/service-account-file.json"
```

</TabItem>
</Tabs>

在前一步中，我们已经将 JSON 文件放置在 **NodejsServer** 文件夹中，因此可以直接进入该文件夹，右键点击并选择 **Open in Powershell** 打开 Windows 终端。

然后只需输入以下命令。

```
$env:GOOGLE_APPLICATION_CREDENTIALS="tensile-yen-3xxxxx-fdxxxxxxxxxx.json"
```

:::tip
执行上述命令时，请使用您的 JSON 文件名。
:::

:::caution
如果您重启了计算机或关闭了 Powershell，可能需要重新配置环境变量来添加密钥。
:::

### 步骤 8. 测试本地 Google Cloud 语音转文本服务的部署

一切就绪后，我们可以使用一段录制的音频，结合 JSON 程序，检查我们的部署是否成功地将录音转换为文本。

请在项目文件夹中的 **NodejsServer** 中打开一个 Powershell 窗口。

然后输入以下命令。此命令将执行 `speechAPItest.js` 文件，并使用项目资源文件夹中的录音文件作为音频输入源，发送到 Google Cloud 进行分析，并返回识别出的语音内容。

```
node ./speechAPItest.js
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/13.png" style={{width:800, height:'auto'}}/></div>

如果您的实现如上所示，则表示您已成功在本地主机上部署了 Google Cloud 服务，并准备继续执行下一步。

如果遇到问题，您可以查阅 [官方 Google Cloud 指南](https://cloud.google.com/speech-to-text/docs/) 以检查部署过程中是否存在任何错误或遗漏的步骤。

## 上传 XIAO ESP32S3 Sense 录制的音频文件到 Google Cloud 进行识别

接下来，我们将修改上传音频文件的路径。由本地上传变为通过 XIAO ESP32S3 Sense 录音上传。XIAO ESP32S3 Sense 录制的音频文件首先保存在 microSD 卡上，然后通过本地端口传输到 Google Cloud。

### 步骤 9. 启动 Google Cloud 语音识别服务的端口监听

同样，在 NodejsServer 文件夹中，使用 PowerShell 执行以下命令。

```
node ./speechAPIServer.js
```

执行后，**speechAPIServer.js** 程序将开始执行，并持续监听 `localhost:8888`。一旦有文件传输到该端口，Google Cloud 服务将被调用。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/20.png" style={{width:800, height:'auto'}}/></div>

一旦监听开始，只需保持该窗口打开，服务将持续运行。

### 步骤 10. 检查主机 IP 地址

由于 XIAO 录制的文件需要通过主机的端口号上传到 Google Cloud 服务，我们需要知道您计算机主机的 IP 地址。

<Tabs>
<TabItem value="Windows" label="Windows">

在 PowerShell 中执行以下命令以获取计算机的 IP 地址信息。

```
ipcofig
```

</TabItem>

<TabItem value="MacOS or Linux" label="MacOS 或 Linux">

在 shell 中执行以下命令以获取计算机的 IP 地址信息。

```
ifconfig
```

</TabItem>
</Tabs>

请记下您的 IP 地址，因为稍后我们将需要使用它。

### 步骤 11. 上传 XIAO ESP32S3 Sense 的程序

在项目文件夹 **[XIAOESP32S3-RECORD-UPLOAD](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/main/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino)** 中，我们已经为本节的示例准备了程序。

<details>

<summary>如果您的 ESP32 版本是 2.0.x，请点击这里查看完整程序</summary>

```cpp
#include <I2S.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// 用于录音程序的变量，最好不要更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 10      // 秒，最大值为240

// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;

File file;
const char filename[] = "/recording.wav";

bool isWIFIConnected;

void setup() {
  // 在此编写您的初始化代码，仅运行一次：
  Serial.begin(115200);
  while (!Serial) ;
  
  I2S.setAllPins(-1, 42, 41, -1, -1);
  
  // 传输模式为 PDM_MONO_MODE，即使用 PDM（脉冲密度调制）单声道模式进行传输
  if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
    Serial.println("初始化 I2S 失败！");
    while (1) ;
  }

  if(!SD.begin(D2)){
    Serial.println("SD 卡挂载失败！");
    while (1) ;
  }
  
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 1, NULL);
  delay(500);
  xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
}

void loop() {
  // 主循环代码，在此重复运行：
}

void i2s_adc(void *arg)
{
  uint32_t sample_size = 0;

  // 该变量用于指向实际的录音缓冲区
  uint8_t *rec_buffer = NULL;
  Serial.printf("准备开始录音 ...\n");

  File file = SD.open(filename, FILE_WRITE);

  // 将头信息写入 WAV 文件
  uint8_t wav_header[WAV_HEADER_SIZE];

  // 将 WAV 文件头信息写入 wav_header 数组
  generate_wav_header(wav_header, record_size, SAMPLE_RATE);

  // 调用 file.write() 函数将 wav_header 数组中的数据写入新创建的 WAV 文件
  file.write(wav_header, WAV_HEADER_SIZE);

  // 该代码使用 ESP32 的 PSRAM（外部缓存内存）动态分配一块内存来存储录音数据
  rec_buffer = (uint8_t *)ps_malloc(record_size);
  if (rec_buffer == NULL) {
    Serial.printf("malloc 失败！\n");
    while(1) ;
  }
  Serial.printf("缓冲区：%d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());

  // 开始录音
  // I2S 端口号（在此为 I2S_NUM_0），
  // 指向要写入数据的缓冲区的指针（即 rec_buffer），
  // 读取数据的大小（即 record_size），
  // 指向变量的指针，用于指示读取数据的实际大小（即 &sample_size），
  // 最大等待时间（此处为 portMAX_DELAY，表示无限等待时间）
  esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
  if (sample_size == 0) {
    Serial.printf("录音失败！\n");
  } else {
    Serial.printf("录音 %d 字节\n", sample_size);
  }

  // 提升音量
  for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
    (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
  }

  // 将数据写入 WAV 文件
  Serial.printf("正在写入文件 ...\n");
  if (file.write(rec_buffer, record_size) != record_size)
    Serial.printf("写入文件失败！\n");

  free(rec_buffer);
  rec_buffer = NULL;
  file.close();
  Serial.printf("录音结束。\n");
    
  listDir(SD, "/", 0);

  if(isWIFIConnected){
    uploadFile();
  }
  
  vTaskDelete(NULL);
}


void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // 参考此链接： http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void wifiConnect(void *pvParameters){
  isWIFIConnected = false;
  char* ssid = "wifi-ssid";
  char* password = "wifi-password";
  Serial.print("Try to connect to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi Connected!");
  isWIFIConnected = true;
  while(true){
    vTaskDelay(1000);
  }
}

void uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    Serial.println("====================      End      ====================");
  }else{
    Serial.println("Error");
  }
  file.close();
  client.end();
}
```

</details>

<details>

<summary>If your ESP32 version is 3.0.x. Click here to preview the full program</summary>

```cpp
#include <ESP_I2S.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// 用于录音程序的变量，最好不要更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 10      // 秒，最大值为240

// 定义 I2S
I2SClass I2S;

// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;

File file;
const char filename[] = "/recording.wav";

bool isWIFIConnected;

void setup() {
  // 在此编写您的初始化代码，仅运行一次：
  Serial.begin(115200);
  while (!Serial) ;
  
  // 设置42为PDM时钟，41为PDM数据引脚
  I2S.setPinsPdmRx(42, 41);

  // 传输模式为PDM_MONO_MODE，即使用PDM（脉冲密度调制）单声道模式进行传输
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("初始化 I2S 失败！");
    while (1) ;
  }

  if(!SD.begin(D2)){
    Serial.println("SD卡挂载失败！");
    while (1) ;
  }
  
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 1, NULL);
  delay(500);
  xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
}

void loop() {
  // 主循环代码，在此重复运行：
}

void i2s_adc(void *arg)
{
  uint32_t sample_size = 0;

  // 该变量用于指向实际的录音缓冲区
  uint8_t *rec_buffer = NULL;
  Serial.printf("准备开始录音 ...\n");

  File file = SD.open(filename, FILE_WRITE);

  // 将头信息写入WAV文件
  uint8_t wav_header[WAV_HEADER_SIZE];

  // 将WAV文件头信息写入wav_header数组
  generate_wav_header(wav_header, record_size, SAMPLE_RATE);

  // 调用file.write()函数将wav_header数组中的数据写入新创建的WAV文件
  file.write(wav_header, WAV_HEADER_SIZE);

  // 该代码使用ESP32的PSRAM（外部缓存内存）动态分配一块内存来存储录音数据
  rec_buffer = (uint8_t *)ps_malloc(record_size);
  if (rec_buffer == NULL) {
    Serial.printf("malloc失败！\n");
    while(1) ;
  }
  Serial.printf("缓冲区：%d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());

  // 开始录音
  // I2S端口号（在此为I2S_NUM_0），
  // 指向要写入数据的缓冲区的指针（即rec_buffer），
  // 读取数据的大小（即record_size），
  // 指向变量的指针，用于指示读取数据的实际大小（即&sample_size），
  // 最大等待时间（此处为portMAX_DELAY，表示无限等待时间）
  esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
  if (sample_size == 0) {
    Serial.printf("录音失败！\n");
  } else {
    Serial.printf("录音 %d 字节\n", sample_size);
  }

  // 提升音量
  for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
    (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
  }

  // 将数据写入WAV文件
  Serial.printf("正在写入文件 ...\n");
  if (file.write(rec_buffer, record_size) != record_size)
    Serial.printf("写入文件失败！\n");

  free(rec_buffer);
  rec_buffer = NULL;
  file.close();
  Serial.printf("录音结束。\n");
    
  listDir(SD, "/", 0);

  if(isWIFIConnected){
    uploadFile();
  }
  
  vTaskDelete(NULL);
}


void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // 参考资料： http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // 格式
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (PCM为16)
    0x01, 0x00, // AudioFormat (PCM为1)
    0x01, 0x00, // NumChannels (1通道)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16位)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void wifiConnect(void *pvParameters){
  isWIFIConnected = false;
  char* ssid = "wifi-ssid";
  char* password = "wifi-password";
  Serial.print("Try to connect to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi Connected!");
  isWIFIConnected = true;
  while(true){
    vTaskDelay(1000);
  }
}

void uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    Serial.println("====================      End      ====================");
  }else{
    Serial.println("Error");
  }
  file.close();
  client.end();
}
```

</details>

在编译并上传示例程序之前，您需要根据您的情况进行一些修改。

1. **录音时间** - 在代码的 [第13行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino#LL13C2-L13C2)，默认的录音时间设置为10秒，您可以根据需要调整这个录音时间，最多可设置为240秒。
2. **保存录音文件的文件名** - 在代码的 [第19行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino#L19)，您可以更改录音文件的名称。
3. **WiFi网络名称** - 在代码的 [第172行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino#L172)，将网络名称更改为与您部署Google Cloud服务的主机在同一局域网中的网络名称。
4. **WiFi网络密码** - 在代码的 [第172行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino#LL173C5-L173C5)，更改与网络对应的密码。
5. **主机IP地址** - 在代码的 [第198行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-RECORD-UPLOAD/XIAOESP32S3-RECORD-UPLOAD.ino#LL198C7-L198C7)，您需要将此处的IP地址更改为您的主机IP地址，并保持端口号为8888。

一旦您根据需要更改了程序并上传，就可以打开串口监视器，开始准备录制您想说的内容。录音完成后，Google Cloud将分析您的录音文件，并将识别结果返回给您。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/15.png" style={{width:1000, height:'auto'}}/></div>

## 在XIAO ESP32S3 Sense上部署ChatGPT

接下来，我们增加难度，继续在代码中添加ChatGPT的调用。

### 第12步. 使用识别出的文本作为问题向ChatGPT提问

在项目文件夹 **[XIAOESP32S3-SPEECH-TO-CHATGPT](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/main/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino)** 中，我们为本节中的示例准备了程序。

<details>

<summary>如果您的ESP32版本是2.0.x，请点击这里预览完整程序</summary>

```cpp
#include <I2S.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ChatGPT.hpp>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// 录音程序中要使用的变量，最好不要更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 5  // 秒，最大值为240

const char* ssid = "wifi-ssid";
const char* password = "wifi-password";

// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;

File file;
const char filename[] = "/recording.wav";
bool isWIFIConnected;

String chatgpt_Q;

TaskHandle_t chatgpt_handle;
WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", "OpenAI-TOKEN");

//*****************************************Arduino 基础设置******************************************//

void setup() {
  // 设置代码，运行一次：
  Serial.begin(115200);
  while (!Serial) ;
  
  I2S.setAllPins(-1, 42, 41, -1, -1);
  
  // 传输模式是 PDM_MONO_MODE，意味着使用 PDM（脉冲密度调制）单声道模式进行传输
  if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
    Serial.println("初始化 I2S 失败！");
    while (1) ;
  }

  if(!SD.begin(D2)){
    Serial.println("SD 卡挂载失败！");
    while (1) ;
  }

  xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
  delay(500);
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 1, NULL);
  xTaskCreate(chatgpt, "chatgpt", 1024 * 8, NULL, 2, &chatgpt_handle);
}

void loop() {
  // 主循环代码，重复运行：
}

//*****************************************RTOS任务******************************************//

void i2s_adc(void *arg)
{
  while(1){
    uint32_t sample_size = 0;
  
    // 这个变量将用于指向实际的录音缓冲区
    uint8_t *rec_buffer = NULL;
    Serial.printf("准备开始录音 ...\n");
  
    File file = SD.open(filename, FILE_WRITE);
  
    // 写入 WAV 文件头
    uint8_t wav_header[WAV_HEADER_SIZE];
  
    // 将 WAV 文件头信息写入 wav_header 数组
    generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  
    // 调用 file.write() 函数将 wav_header 数组中的数据写入新创建的 WAV 文件
    file.write(wav_header, WAV_HEADER_SIZE);
  
    // 这段代码使用 ESP32 的 PSRAM（外部缓存内存）动态分配一块内存来存储录音数据
    rec_buffer = (uint8_t *)ps_malloc(record_size);
    if (rec_buffer == NULL) {
      Serial.printf("内存分配失败！\n");
      while(1) ;
    }
    Serial.printf("缓冲区: %d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());
  
    // 开始录音
    // I2S 端口号（此处为 I2S_NUM_0），
    // 指向缓冲区的指针（即 rec_buffer），
    // 要读取的数据大小（即 record_size），
    // 指向实际读取数据大小的变量的指针（即 &sample_size），
    // 读取数据的最大等待时间（此处为 portMAX_DELAY，表示无限等待）。
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
    if (sample_size == 0) {
      Serial.printf("录音失败！\n");
    } else {
      Serial.printf("录音 %d 字节\n", sample_size);
    }
  
    // 增加音量
    for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
      (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
    }
  
    // 写入数据到 WAV 文件
    Serial.printf("写入文件 ...\n");
    if (file.write(rec_buffer, record_size) != record_size)
      Serial.printf("写入文件失败！\n");
  
    free(rec_buffer);
    rec_buffer = NULL;
    file.close();
    Serial.printf("录音结束。\n");
      
    listDir(SD, "/", 0);

    bool uploadStatus = false;
  
    if(isWIFIConnected){
      uploadStatus = uploadFile();
    }
    
    if(uploadStatus)
      xTaskNotifyGive(chatgpt_handle);
    vTaskDelay(10000);       // 每次录音间隔10秒
  }
//  vTaskDelete(NULL);
}

void wifiConnect(void *pvParameters){
  isWIFIConnected = false;
  Serial.print("尝试连接到 ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi 已连接！");
  isWIFIConnected = true;
  // 忽略 SSL 证书验证
  client.setInsecure();
  while(true){
    vTaskDelay(1000);
  }
}

void chatgpt(void *pvParameters){
  while(1){
    // 等待来自任务1的通知信号
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    String result;
    if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", chatgpt_Q, result)) {
      Serial.println("===成功===");
      Serial.println(result);
    } else {
      Serial.println("===错误===");
      Serial.println(result);
    }

  }
}

//*****************************************音频处理******************************************//

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // 参考文档：http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

//*****************************************文件处理******************************************//

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

bool uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return false;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    chatgpt_Q = response;
    Serial.println("====================      End      ====================");
    file.close();
    client.end();
    return true;
  }else{
    Serial.println("Error");
    return false;
  }
  
}
```

</details>

<details>

<summary>如果您的 ESP32 版本是 3.0.x，请点击这里预览完整程序</summary>

```cpp
#include <ESP_I2S.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ChatGPT.hpp>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// 用于录音程序的变量，最好不要更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 5  // 秒，最大值为 240

const char* ssid = "wifi-ssid";
const char* password = "wifi-password";

// 定义 I2S
I2SClass I2S;

// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;

File file;
const char filename[] = "/recording.wav";
bool isWIFIConnected;

String chatgpt_Q;

TaskHandle_t chatgpt_handle;
WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", "OpenAI-TOKEN");

//*****************************************Arduino 基础设置******************************************//

void setup() {
  // 在此处放置您的设置代码，只会运行一次：
  Serial.begin(115200);
  while (!Serial) ;
  
  // 设置 42 引脚为 PDM 时钟，41 引脚为 PDM 数据
  I2S.setPinsPdmRx(42, 41);

  // 传输模式为 PDM_MONO_MODE，表示使用 PDM（脉冲密度调制）单声道模式进行传输
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("初始化 I2S 失败！");
    while (1) ;
  }

  if(!SD.begin(D2)){
    Serial.println("SD 卡挂载失败！");
    while (1) ;
  }

  xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
  delay(500);
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 1, NULL);
  xTaskCreate(chatgpt, "chatgpt", 1024 * 8, NULL, 2, &chatgpt_handle);
}

void loop() {
  // 在此处放置您的主循环代码，将重复运行：
}

//*****************************************RTOS 任务******************************************//

void i2s_adc(void *arg)
{
  while(1){
    uint32_t sample_size = 0;
  
    // 此变量将指向实际的录音缓冲区
    uint8_t *rec_buffer = NULL;
    Serial.printf("准备开始录音 ...\n");
  
    File file = SD.open(filename, FILE_WRITE);
  
    // 将头部写入 WAV 文件
    uint8_t wav_header[WAV_HEADER_SIZE];
  
    // 将 WAV 文件头信息写入 wav_header 数组
    generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  
    // 调用 file.write() 函数，将 wav_header 数组中的数据写入新创建的 WAV 文件
    file.write(wav_header, WAV_HEADER_SIZE);
  
    // 这段代码使用 ESP32 的 PSRAM（外部缓存内存）动态分配一块内存来存储录音数据
    rec_buffer = (uint8_t *)ps_malloc(record_size);
    if (rec_buffer == NULL) {
      Serial.printf("内存分配失败！\n");
      while(1) ;
    }
    Serial.printf("缓冲区: %d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());
  
    // 开始录音
    // I2S 端口号（在此案例中为 I2S_NUM_0），
    // 指向要写入数据的缓冲区的指针（即 rec_buffer），
    // 要读取的数据大小（即 record_size），
    // 指向变量的指针，该变量指向实际读取的数据大小（即 &sample_size），
    // 最大等待时间（在此案例中为 portMAX_DELAY，表示无限等待）。
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
    if (sample_size == 0) {
      Serial.printf("录音失败！\n");
    } else {
      Serial.printf("录音 %d 字节\n", sample_size);
    }

  
    // 增加音量
    for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
      (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
    }
  
    // 写入数据到 WAV 文件
    Serial.printf("Writing to the file ...\n");
    if (file.write(rec_buffer, record_size) != record_size)
      Serial.printf("Write file Failed!\n");
  
    free(rec_buffer);
    rec_buffer = NULL;
    file.close();
    Serial.printf("The recording is over.\n");
      
    listDir(SD, "/", 0);

    bool uploadStatus = false;
  
    if(isWIFIConnected){
      uploadStatus = uploadFile();
    }
    
    if(uploadStatus)
      xTaskNotifyGive(chatgpt_handle);
    vTaskDelay(10000);       // 每次录音间隔 10 秒
  }
//  vTaskDelete(NULL);
}

void wifiConnect(void *pvParameters){
isWIFIConnected = false;
Serial.print("尝试连接到 ");
Serial.println(ssid);
WiFi.begin(ssid, password);
while(WiFi.status() != WL_CONNECTED){
  vTaskDelay(500);
  Serial.print(".");
}
Serial.println("Wi-Fi 连接成功！");
isWIFIConnected = true;
// 忽略 SSL 证书验证
client.setInsecure();
while(true){
  vTaskDelay(1000);
}
}

void chatgpt(void *pvParameters){
while(1){
// 等待来自任务 1 的通知信号
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

String result;
if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", chatgpt_Q, result)) {
  Serial.println("===成功===");
  Serial.println(result);
} else {
  Serial.println("===错误===");
  Serial.println(result);
}

}
}

//*****************************************音频处理******************************************//

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
// 参考资料： http://soundfile.sapp.org/doc/WaveFormat/
uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
const uint8_t set_wav_header[] = {
  'R', 'I', 'F', 'F', // ChunkID
  file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
  'W', 'A', 'V', 'E', // Format
  'f', 'm', 't', ' ', // Subchunk1ID
  0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
  0x01, 0x00, // AudioFormat (1 for PCM)
  0x01, 0x00, // NumChannels (1 channel)
  sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
  byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
  0x02, 0x00, // BlockAlign
  0x10, 0x00, // BitsPerSample (16 bits)
  'd', 'a', 't', 'a', // Subchunk2ID
  wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
};
memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

//*****************************************文件处理******************************************//

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

bool uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return false;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    chatgpt_Q = response;
    Serial.println("====================      End      ====================");
    file.close();
    client.end();
    return true;
  }else{
    Serial.println("Error");
    return false;
  }
  
}
```

</details>

再次强调，在使用此程序之前，您需要根据需要对代码进行以下更改：

1. **WiFi 网络名称** - 更改代码中的网络名称，位于 [第18行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#L18)，将其设置为与您部署 Google 云服务的主机在同一局域网中的网络名称。
2. **WiFi 网络密码** - 在代码的 [第19行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#LL19C40-L19C40)，更改对应网络的密码。
3. **主机 IP 地址** - 在代码的 [第241行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#LL241C7-L241C7)，您需要将这里的 IP 地址更改为您的主机 IP 地址，并将端口号保持为 8888。
4. **OpenAI API Token** - 由于需要调用 ChatGPT 接口，您需要准备 OpenAI Token，并将其填入代码中的 [第33行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#L33)。如果您是第一次使用 Token，您可以阅读 [此 Wiki 内容](https://wiki.seeedstudio.com/xiaoesp32c3-chatgpt/#submit-questions-via-the-built-in-web-page) 来了解如何获取它们。

修改完成后，上传程序并打开串口监视器。录音完成后，您将看到 ChatGPT 返回的答案。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/16.png" style={{width:1000, height:'auto'}}/></div>

## 屏幕显示内容设计 & 程序集成

最后，我们加了一些炫酷的效果。我们不再使用串口监视器，这种界面对于显示效果不太合适，而是使用触摸屏来实现触摸和点击功能。

### 步骤13. 使用 SquareLine Studio 绘制显示屏

SquareLine Studio 是由 LVGL 开发的图形用户界面设计工具，专为嵌入式系统设计。SquareLine Studio 旨在帮助开发人员快速高效地创建和设计嵌入式系统的用户界面。它提供了一个拖放界面来设计 UI，并且支持多种小部件和主题。

因此，我们推荐您使用该工具来设计这样简单的界面。如果您想了解更多关于在 SquareLine Studio 中使用圆形显示的内容，可以访问我们的使用 [Wiki](https://wiki.seeedstudio.com/using_lvgl_and_tft_on_round_display/#drawing-complex-ui-interfaces-with-squareline-studio)。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/21.png" style={{width:1000, height:'auto'}}/></div>

由于篇幅限制，本文不再详细介绍如何设计显示页面，但我们会提供导出的程序代码，您可以直接使用。它目前位于该 [项目文件夹](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/tree/main/ui) 下的 **ui** 文件夹中。

:::caution
我们建议您使用 **v1.2.3** 版本的 SquareLine Studio。经过测试，v1.3.0 版本与 tft_eSPI 库可能存在兼容性问题。
:::

### 步骤14. 程序集成

最终的完整项目代码位于 **[XIAOESP32S3-SPEECH-CHATGPT-COMPLETE](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/main/XIAOESP32S3-SPEECH-CHATGPT-COMPLETE/XIAOESP32S3-SPEECH-CHATGPT-COMPLETE.ino)** 文件夹中。

<details>

<summary>如果您的 ESP32 版本是 2.0.x，请点击此处预览完整程序</summary>

```cpp
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ChatGPT.hpp>
#include <I2S.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"


// 导入圆形显示的库，并定义作为 TFT 显示框架的框架
#define USE_TFT_ESPI_LIBRARY
#include "lv_xiao_round_screen.h"


/* 请根据您的屏幕分辨率进行修改 */
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;


// 用于录音程序的变量，请不要更改，以保证最佳效果
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 5  // 秒，最大值为 240


// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;


// 保存录音的文件名称
File file;
const char filename[] = "/recording.wav";


// 网络连接状态标志
bool isWIFIConnected;


// ChatGPT 回复问题的答案
String response;


// 不同任务启动的标志
bool recordTask = false;
bool chatgptTask = false;

WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", "OpenAI-TOKEN");   // 请填写您的 OpenAI 密钥


// 请更改为您的网络信息
const char* ssid = "wifi-ssid";
const char* password = "wifi-password";

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];


/****************************************LVGL****************************************************//

#if LV_USE_LOG != 0
/* 串口调试 */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* 显示刷新 */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/* 读取触摸板 */
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    // uint16_t touchX = 0, touchY = 0;
    // bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    lv_coord_t touchX, touchY;
    chsc6x_get_xy(&touchX, &touchY);

    // if( !touched )
    if(!chsc6x_is_pressed())
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /* 设置坐标 */
        data->point.x = touchX;
        data->point.y = touchY;

        //        Serial.print( "数据 x " );
        //        Serial.println( touchX );
        //
        //        Serial.print( "数据 y " );
        //        Serial.println( touchY );

        // 您还可以通过取消注释并在点击徽标时配置来开始录音
//        if((touchX < 240 && touchX > 230) && (touchY < 120 && touchY > 100)){
          recordTask = true;
//        }
    }
}

//****************************************Arduino 基础设置****************************************************//

void setup()
{
    Serial.begin( 115200 ); /* 为可能的串口调试做准备 */
    //    while(!Serial);

    pinMode(TOUCH_INT, INPUT_PULLUP);
    Wire.begin();

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "我是 LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* 注册调试用的打印函数 */
#endif

    tft.begin();          /* TFT 初始化 */
    tft.setRotation( 0 ); /* 横屏方向，翻转 */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /* 初始化显示 */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /* 更改以下行以匹配您的显示分辨率 */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /* 初始化（虚拟）输入设备驱动 */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    I2S.setAllPins(-1, 42, 41, -1, -1);
  
    // 传输模式是 PDM_MONO_MODE，这意味着使用 PDM（脉冲密度调制）单声道模式进行传输
    if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
        Serial.println("I2S 初始化失败！");
        while (1) ;
    }

    if(!SD.begin(D2)){
        Serial.println("SD 卡挂载失败！");
        while (1) ;
    }

    Serial.println( "设置完成" );

    // 创建一个 FreeRTOS 任务，定期检查网络的连接状态。
    xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
}

// void loop()
{
    lv_timer_handler(); /* 让GUI做它的工作 */
    record();
    chatgpt();
    delay(5);
}

//*****************************************音频处理******************************************//

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // 参考资料：http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16位)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

//*****************************************文件处理******************************************//

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

bool uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return false;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    const char* chatgpt_Q = response.c_str();
    lv_label_set_text(ui_question, chatgpt_Q);
    Serial.println("====================      End      ====================");
    file.close();
    client.end();
    recordTask = false;
    chatgptTask = true;
    return true;
  }else{
    Serial.println("Error");
    lv_label_set_text(ui_question, "Error");
    recordTask = false;
    chatgptTask = false;
    return false;
  }
}


//*****************************************主功能******************************************//

void record(){
  if(recordTask){
    Serial.println("录音任务开始!!!");
    lv_label_set_text(ui_question, "正在录音 ...");
    lv_timer_handler();
    uint32_t sample_size = 0;
    
    // 这个变量将用于指向实际的录音缓冲区
    uint8_t *rec_buffer = NULL;
    Serial.printf("准备开始录音 ...\n");
  
    File file = SD.open(filename, FILE_WRITE);
  
    // 将头信息写入WAV文件
    uint8_t wav_header[WAV_HEADER_SIZE];
  
    // 将WAV文件头信息写入wav_header数组
    generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  
    // 调用file.write()函数将wav_header数组中的数据写入新创建的WAV文件
    file.write(wav_header, WAV_HEADER_SIZE);
  
    // 这段代码使用ESP32的PSRAM（外部缓存内存）动态分配一块内存，用于存储录音数据
    rec_buffer = (uint8_t *)ps_malloc(record_size);
    if (rec_buffer == NULL) {
      Serial.printf("malloc失败!\n");
      while(1) ;
    }
    Serial.printf("缓冲区: %d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());
  
    // 开始录音
    // I2S端口号（在这个例子中是I2S_NUM_0），
    // 一个指向存储数据的缓冲区的指针（即rec_buffer），
    // 要读取的数据大小（即record_size），
    // 一个指向实际读取数据大小的变量的指针（即&sample_size），
    // 以及等待数据读取的最大时间（在这里portMAX_DELAY表示无限等待）
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
    if (sample_size == 0) {
      Serial.printf("录音失败!\n");
    } else {
      Serial.printf("录音 %d 字节\n", sample_size);
    }
  
    // 提升音量
    for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
      (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
    }
  
    // 将数据写入WAV文件
    Serial.printf("写入文件中 ...\n");
    if (file.write(rec_buffer, record_size) != record_size)
      Serial.printf("写入文件失败!\n");
  
    free(rec_buffer);
    rec_buffer = NULL;
    file.close();
    Serial.printf("录音结束.\n");
    lv_label_set_text(ui_question, "正在识别 ...");
    lv_timer_handler();
    listDir(SD, "/", 0);
  
    bool uploadStatus = false;
  
    if(isWIFIConnected){
      uploadStatus = uploadFile();
    }
  }
}

void chatgpt(){
  if(chatgptTask){
    Serial.println("ChatGPT Task Begin!!!");
    lv_label_set_text(ui_answer,"Answering ...");
    lv_timer_handler();
    String result;
    if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", response, result)) {
      Serial.println("===OK===");
      Serial.println(result);
      const char* chatgpt_A = result.c_str();
      lv_label_set_text(ui_answer, chatgpt_A);
    } else {
      Serial.println("===ERROR===");
      Serial.println(result);
      lv_label_set_text(ui_answer, "ERROR");
      lv_timer_handler();
    }
    recordTask = false;
    chatgptTask = false;
  }
}

//*****************************************RTOS******************************************// 

void wifiConnect(void *pvParameters){
  isWIFIConnected = false;
  Serial.print("尝试连接到 ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi 已连接!");
  isWIFIConnected = true;
  // 忽略SSL证书验证
  client.setInsecure();
  while(true){
    vTaskDelay(1000);
  }
}
```

</details>

<details>

<summary>如果你的 ESP32 版本是 3.0.x，点击这里预览完整程序</summary>

```cpp
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ChatGPT.hpp>
#include <ESP_I2S.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"


// 导入圆形显示屏的库，并定义用作TFT显示框架的框架
#define USE_TFT_ESPI_LIBRARY
#include "lv_xiao_round_screen.h"


/* 修改为你的屏幕分辨率 */
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;


// 用于录音程序的变量，最好不要更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2
#define RECORD_TIME 5  // 秒，最大值为240


// 录音缓冲区所需的字节数
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;

// 定义I2S
I2SClass I2S;

// 录音保存的文件名
File file;
const char filename[] = "/recording.wav";


// 网络连接状态标志
bool isWIFIConnected;


// ChatGPT回复的问题答案
String response;


// 不同任务开始的标志
bool recordTask = false;
bool chatgptTask = false;

WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", "OpenAI-TOKEN");   // 请填写你的 OpenAI 密钥


// 请修改为你的网络信息
const char* ssid = "wifi-ssid";
const char* password = "wifi-password";

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];


 //****************************************LVGL****************************************************//

#if LV_USE_LOG != 0
/* 串口调试 */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* 显示刷新 */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/* 读取触摸板 */
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    // uint16_t touchX = 0, touchY = 0;
    // bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    lv_coord_t touchX, touchY;
    chsc6x_get_xy(&touchX, &touchY);

    // 如果没有触摸
    if(!chsc6x_is_pressed())
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /* 设置坐标 */
        data->point.x = touchX;
        data->point.y = touchY;

        // 你也可以通过取消注释并在点击 logo 时配置来启动录音
//        if((touchX < 240 && touchX > 230) && (touchY < 120 && touchY > 100)){
          recordTask = true;
//        }
    }
}

//**************************************** Arduino 基础 ****************************************************//

void setup()
{
    Serial.begin( 115200 ); /* 为可能的串口调试做准备 */
    // while(!Serial);

    pinMode(TOUCH_INT, INPUT_PULLUP);
    Wire.begin();

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "我是 LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* 注册打印函数以进行调试 */
#endif

    tft.begin();          /* TFT 初始化 */
    tft.setRotation( 0 ); /* 横屏模式，翻转显示 */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /* 初始化显示 */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /* 修改为你的屏幕分辨率 */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /* 初始化（虚拟）输入设备驱动 */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    // 设置 42 号 PDM 时钟和 41 号 PDM 数据引脚
    I2S.setPinsPdmRx(42, 41);  

    // 传输模式为 PDM_MONO_MODE，意味着使用 PDM（脉冲密度调制）单声道模式进行传输
    if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("初始化 I2S 失败！");
        while (1) ;
    }

    if(!SD.begin(D2)){
        Serial.println("SD 卡挂载失败！");
        while (1) ;
    }

    Serial.println( "设置完成" );

    // 创建一个 FreeRTOS 任务，用于定期检查网络连接状态
    xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
}

void loop()
{
    lv_timer_handler(); /* 让 GUI 执行它的工作 */
    record();
    chatgpt();
    delay(5);
}

//*****************************************音频处理******************************************//

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // 参考文档：http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // 格式
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size（PCM格式为16）
    0x01, 0x00, // AudioFormat（PCM为1）
    0x01, 0x00, // NumChannels（1个声道）
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample（16位）
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

//*****************************************文件处理******************************************//

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

bool uploadFile(){
  file = SD.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return false;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.208:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    const char* chatgpt_Q = response.c_str();
    lv_label_set_text(ui_question, chatgpt_Q);
    Serial.println("====================      End      ====================");
    file.close();
    client.end();
    recordTask = false;
    chatgptTask = true;
    return true;
  }else{
    Serial.println("Error");
    lv_label_set_text(ui_question, "Error");
    recordTask = false;
    chatgptTask = false;
    return false;
  }
}


//*****************************************主功能******************************************//

void record(){
  if(recordTask){
    Serial.println("录音任务开始!!!");
    lv_label_set_text(ui_question, "正在录音 ...");
    lv_timer_handler();
    uint32_t sample_size = 0;
    
    // 这个变量将用于指向实际的录音缓冲区
    uint8_t *rec_buffer = NULL;
    Serial.printf("准备开始录音 ...\n");
  
    File file = SD.open(filename, FILE_WRITE);
  
    // 将头信息写入WAV文件
    uint8_t wav_header[WAV_HEADER_SIZE];
  
    // 将WAV文件头信息写入wav_header数组
    generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  
    // 调用file.write()函数将wav_header数组中的数据写入新创建的WAV文件
    file.write(wav_header, WAV_HEADER_SIZE);
  
    // 这段代码使用ESP32的PSRAM（外部缓存内存）动态分配一块内存用于存储录音数据。
    rec_buffer = (uint8_t *)ps_malloc(record_size);
    if (rec_buffer == NULL) {
      Serial.printf("malloc失败!\n");
      while(1) ;
    }
    Serial.printf("缓冲区: %d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());
  
    // 开始录音
    // I2S端口号（在此为I2S_NUM_0），
    // 一个指向数据将要写入的缓冲区的指针（即rec_buffer），
    // 要读取的数据大小（即record_size），
    // 一个指向指向实际读取数据大小的变量的指针（即&sample_size），
    // 以及等待数据读取的最大时间（在此为portMAX_DELAY，表示无限等待时间）。
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, rec_buffer, record_size, &sample_size, portMAX_DELAY);
    if (sample_size == 0) {
      Serial.printf("录音失败!\n");
    } else {
      Serial.printf("录音 %d 字节\n", sample_size);
    }
  
    // 增加音量
    for (uint32_t i = 0; i < sample_size; i += SAMPLE_BITS/8) {
      (*(uint16_t *)(rec_buffer+i)) <<= VOLUME_GAIN;
    }
  
    // 将数据写入WAV文件
    Serial.printf("正在写入文件 ...\n");
    if (file.write(rec_buffer, record_size) != record_size)
      Serial.printf("写入文件失败!\n");
  
    free(rec_buffer);
    rec_buffer = NULL;
    file.close();
    Serial.printf("录音结束。\n");
    lv_label_set_text(ui_question, "正在识别 ...");
    lv_timer_handler();
    listDir(SD, "/", 0);
  
    bool uploadStatus = false;
  
    if(isWIFIConnected){
      uploadStatus = uploadFile();
    }
  }
}

void chatgpt(){
  if(chatgptTask){
    Serial.println("ChatGPT Task Begin!!!");
    lv_label_set_text(ui_answer,"Answering ...");
    lv_timer_handler();
    String result;
    if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", response, result)) {
      Serial.println("===OK===");
      Serial.println(result);
      const char* chatgpt_A = result.c_str();
      lv_label_set_text(ui_answer, chatgpt_A);
    } else {
      Serial.println("===ERROR===");
      Serial.println(result);
      lv_label_set_text(ui_answer, "ERROR");
      lv_timer_handler();
    }
    recordTask = false;
    chatgptTask = false;
  }
}

//*****************************************RTOS******************************************//

void wifiConnect(void *pvParameters){
  isWIFIConnected = false;
  Serial.print("尝试连接到 ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi 连接成功!");
  isWIFIConnected = true;
  // 忽略SSL证书验证
  client.setInsecure();
  while(true){
    vTaskDelay(1000);
  }
}
```

</details>

在编译和上传示例程序之前，您需要更改以下几个设置以适应您的情况。

1. **WiFi网络名称** - 在代码的[第18行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#L18)，将网络名称更改为与您部署Google Cloud服务的主机位于同一局域网中的网络名称。
2. **WiFi网络密码** - 在代码的[第19行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#LL19C40-L19C40)，将密码更改为与网络对应的密码。
3. **主机IP地址** - 在代码的[第241行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#LL241C7-L241C7)，您需要将此处的IP地址更改为您的主机IP地址，并将端口号保持为8888。
4. **OpenAI API Token** - 由于您需要调用ChatGPT接口，您需要准备OpenAI Token并将其填入代码中的[第33行](https://github.com/limengdu/XIAO-ESP32S3Sense-Speech2ChatGPT/blob/404007a16f42495576d729848d00c6bb6a8149fc/XIAOESP32S3-SPEECH-TO-CHATGPT/XIAOESP32S3-SPEECH-TO-CHATGPT.ino#L33)。如果这是您第一次使用Token，您可以阅读[这个Wiki内容](https://wiki.seeedstudio.com/xiaoesp32c3-chatgpt/#submit-questions-via-the-built-in-web-page)来了解如何获取它们。

一旦您上传了程序并点击屏幕，录音任务将开始，此时您可以对着麦克风说出您想要提问的问题。一旦识别出结果，问题将显示在屏幕的上半部分。随即，我们将获得ChatGPT的答案，并将其显示在屏幕的下半部分。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3sense-speech2chatgpt/22.jpg" style={{width:600, height:'auto'}}/></div>

## 技术支持与产品讨论

感谢您选择我们的产品！我们将为您提供各种支持，以确保您使用我们的产品时体验顺畅。我们提供了多个沟通渠道，以适应不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>





