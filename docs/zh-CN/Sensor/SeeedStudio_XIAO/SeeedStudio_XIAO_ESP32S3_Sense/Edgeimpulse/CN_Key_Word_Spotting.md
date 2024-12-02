---
description:  关键词识别
title: 关键词识别
keywords:
- tinyml course
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/tinyml_course_Key_Word_Spotting
last_update:
  date: 11/27/2024
  author: Agnes
---

### 在 XIAO ESP32S3 Sense 上实现 Edge Impulse KWS 应用

本教程将指导您如何使用 TinyML 在 XIAO ESP32S3 Sense 微控制器板上实现关键字检测（KWS）系统，并通过 Edge Impulse 完成数据收集和模型训练。KWS 对于语音识别系统至关重要，借助 TinyML 的强大功能，我们可以在更小、更低功耗的设备上实现这一功能。让我们使用 Edge Impulse 和 XIAO ESP32S3 Sense 构建我们自己的 KWS 系统！

#### 2.3.1 入门

在开始这个项目之前，请按照以下步骤准备所需的软件和硬件。

##### 硬件

为了顺利进行此项目，您需要准备以下硬件：

- XIAO ESP32S3 Sense
- microSD 卡（不超过 32GB）
- microSD 卡读卡器
- USB-C 数据线

将 microSD 卡插入 microSD 卡槽。请注意插入方向，带有金手指的一面应朝内。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/66.jpg" style={{width:500, height:'auto'}}/></div>

##### 软件

如果这是您第一次使用 XIAO ESP32S3 Sense，我们建议您在开始之前阅读以下两个 Wiki 页面，了解如何使用它：

- [Seeed Studio XIAO ESP32S3 入门](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Seeed Studio XIAO ESP32S3 麦克风使用说明](https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/)

#### 2.3.2 捕获（离线）音频数据

##### 步骤 1. 将录制的声音样本保存为 .wav 音频文件到 microSD 卡。

我们将使用板载 SD 卡读卡器保存 .wav 音频文件，首先需要启用 XIAO 的 PSRAM。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/94.png" style={{width:600, height:'auto'}}/></div>

然后编译并上传以下程序到 XIAO ESP32S3。

:::tip
此代码使用 Seeed XIAO ESP32S3 Sense 板的 I2S 接口录制音频，将录音保存为 .wav 文件到 SD 卡，并允许通过串口监视器发送命令控制录音过程。音频文件的名称是可以自定义的（应该是训练时使用的类标签），并且可以进行多次录制，每次录制都会保存在新文件中。代码还包括增加录音音量的功能。
:::

```cpp
/* 
 * Seeed XIAO ESP32S3 Sense WAV 录音器 
 * 
 * 注意：要执行此代码，我们需要使用 ESP-32 芯片的 PSRAM 功能，
 * 所以在上传代码前，请先启用它。
 * 工具 > PSRAM: "OPI PSRAM"
 * 
 * 由 M.Rovai @May23 修改自原始 Seeed 代码
*/

#include <I2S.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// 根据需要修改
#define RECORD_TIME   10  // 秒，最大值为 240
#define WAV_FILE_NAME "data"

// 最佳配置，勿更改
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 2

int fileNumber = 1;
String baseFileName;
bool isRecording = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  
  I2S.setAllPins(-1, 42, 41, -1, -1);
  if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
    Serial.println("初始化 I2S 失败！");
    while (1) ;
  }
  if(!SD.begin(21)){
    Serial.println("挂载 SD 卡失败！");
    while (1) ;
  }
  Serial.printf("请输入标签名称\n");
  //record_wav();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "rec") {
      isRecording = true;
    } else {
      baseFileName = command;
      fileNumber = 1; // 每次设置新标签名称时重置文件编号
      Serial.printf("发送 'rec' 开始录制标签\n");
    }
  }
  if (isRecording && baseFileName != "") {
    String fileName = "/" + baseFileName + "." + String(fileNumber) + ".wav";
    fileNumber++;
    record_wav(fileName);
    delay(1000); // 延迟以避免同时录制多个文件
    isRecording = false;
  }
}

void record_wav(String fileName)
{
  uint32_t sample_size = 0;
  uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS / 8) * RECORD_TIME;
  uint8_t *rec_buffer = NULL;
  Serial.printf("开始录音...\n");
   
  File file = SD.open(fileName.c_str(), FILE_WRITE);
  // 写入 WAV 文件头
  uint8_t wav_header[WAV_HEADER_SIZE];
  generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  file.write(wav_header, WAV_HEADER_SIZE);

  // 使用 PSRAM 为录音分配内存
  rec_buffer = (uint8_t *)ps_malloc(record_size);
  if (rec_buffer == NULL) {
    Serial.printf("内存分配失败！\n");
    while(1) ;
  }
  Serial.printf("缓冲区: %d 字节\n", ESP.getPsramSize() - ESP.getFreePsram());

  // 开始录音
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
  Serial.printf("写入文件...\n");
  if (file.write(rec_buffer, record_size) != record_size)
    Serial.printf("写入文件失败！\n");

  free(rec_buffer);
  file.close();
  Serial.printf("录音完成：\n");
  Serial.printf("发送 'rec' 获取新样本，或输入新标签\n\n");
}

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate) 
{
  // 参考资料：https://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (PCM时为16)
    0x01, 0x00, // AudioFormat (PCM时为1)
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
```

现在，将代码上传到XIAO，并从关键词（如hello和stop）中获取样本。您还可以捕获噪声和其他词汇。串口监视器将提示您输入要录制的标签。

发送标签（例如，**hello**）。程序将等待另一个命令：**rec**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/2.png" style={{width:600, height:'auto'}}/></div>

每次发送命令**rec**时，程序将开始录制新的样本。文件将保存为hello.1.wav, hello.2.wav, hello.3.wav等，直到发送新的标签（例如，**stop**）。在这种情况下，您应为每个新样本发送**rec**命令，这些样本将保存为stop.1.wav, stop.2.wav, stop.3.wav等。

最终，我们将获得保存在SD卡上的文件。

:::note
我们建议您为每个标签准备足够的声音样本。您可以在每次十秒的录音过程中多次重复关键词，后续我们将在步骤中对样本进行切分。但关键词之间需要保持一定间隔。
:::

#### 2.3.3 训练数据采集

##### 步骤2. 上传收集的声音数据

当原始数据集被定义和收集后，我们应该在[Edge Impulse](https://edgeimpulse.com/)上启动一个新项目。创建项目后，在**数据采集**部分选择**上传现有数据**工具，选择要上传的文件。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/3.png" style={{width:1000, height:'auto'}}/></div>

然后将它们上传到Studio（您可以自动将数据划分为训练/测试数据）。为所有类别和原始数据重复此操作。

数据集中的所有数据都是1秒长的，但在上一节中录制的样本是10秒的，必须将其切分为1秒的样本以兼容。点击样本名称后的三个点，选择**切分样本**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/5.png" style={{width:600, height:'auto'}}/></div>

进入工具后，将数据切分为1秒的记录。如果需要，添加或删除段。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/4.png" style={{width:1000, height:'auto'}}/></div>

此过程应对所有样本重复进行。

##### 步骤3. 创建Impulse（预处理 / 模型定义）

Impulse处理原始数据，使用信号处理提取特征，然后使用学习模块来分类新数据。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/6.png" style={{width:1000, height:'auto'}}/></div>

首先，我们将使用1秒窗口的数据点，增强数据，并将该窗口每500ms滑动一次。请注意，**零填充数据**选项已设置。这对于填充小于1秒的样本（在某些情况下，我将1000毫秒窗口缩短到**切分工具**中的值以避免噪声和尖峰）非常重要。

每个1秒的音频样本都应进行预处理并转换为图像（例如，13 x 49 x 1）。我们将使用MFCC，它通过Mel频率倒谱系数从音频信号中提取特征，非常适合人声。

接下来，我们选择**KERAS**进行分类，它通过使用卷积神经网络从头开始构建我们的模型进行图像分类。

##### 步骤 4. 预处理（MFCC）

下一步是创建用于在下一阶段进行训练的图像。我们可以保留默认的参数值，或者利用DSP的**Autotuneparameters选项**，我们将选择后者。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/7.png" style={{width:1000, height:'auto'}}/></div>

#### 2.3.4 构建机器学习模型

##### 步骤 5. 模型设计与训练

我们将使用卷积神经网络（CNN）模型。基本架构定义为两个卷积块Conv1D + 最大池化（分别使用8和16个神经元），并设置0.25的Dropout。在最后一层，经过Flatten处理后有四个神经元，每个神经元对应一个类别。

作为超参数，我们将学习率设置为0.005，并通过100个epochs进行训练。我们还将加入数据增强，例如添加噪声。结果似乎不错。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/8.png" style={{width:600, height:'auto'}}/></div>

#### 2.3.5 部署到XIAO ESP32S3 Sense

##### 步骤 6. 部署到XIAO ESP32S3 Sense

Edge Impulse将打包所有所需的库、预处理函数和训练模型，并将其下载到您的计算机。您应该选择Arduino库选项，并在底部选择量化（Int8），然后点击“Build”按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/9.png" style={{width:600, height:'auto'}}/></div>

尽管Edge Impulse尚未发布其用于ESP32S3的SDK来支持ESP NN加速器，但得益于Dmitry Maslov的贡献，我们可以恢复其汇编优化，并将其修复为ESP32-S3的兼容版本。该解决方案尚未正式发布，一旦解决与其他开发板的冲突，EI将会将其包含在EI SDK中。

:::caution
目前，这只适用于非EON版本。因此，您还应该确保未选中**启用EON编译器**选项。
:::

当选择“Build”按钮后，会创建并下载一个Zip文件到您的计算机。

在使用下载的库之前，我们需要启用**ESP NN**加速器。为此，您可以从[项目GitHub](https://github.com/Mjrovai/XIAO-ESP32S3-Sense/blob/main/ESP-NN.zip)下载一个初步版本，解压后将ESP NN文件夹替换为您的Arduino库文件夹中的`src/edge-impulse-sdk/porting/espressif/ESP-NN`。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/10.png" style={{width:800, height:'auto'}}/></div>

在您的Arduino IDE中，转到**Sketch**标签，选择**添加.ZIP库**选项，然后选择Edge Impulse下载的.zip文件。

您可以在[项目的GitHub](https://github.com/Mjrovai/XIAO-ESP32S3-Sense/tree/main/xiao_esp32s3_microphone_led)上找到完整代码。将草图上传到您的开发板，并进行一些实际推理测试。

:::tip
代码中导入的库需要更新为您实际使用的库名称。点亮LED的逻辑也需要根据您实际训练的标签顺序进行修改。
:::

```cpp
/* Edge Impulse Arduino 示例
 * 版权所有 (c) 2022 EdgeImpulse Inc.
 *
 * 特此免费授权任何获得此软件及相关文档文件（“软件”）的人，无限制地使用、复制、修改、合并、发布、分发、再授权和/或销售软件副本，并允许向其提供软件的人在以下条件下进行这些操作：
 *
 * 上述版权声明和本许可声明应包括在所有副本或软件的实质性部分中。
 *
 * 本软件按“原样”提供，不提供任何形式的明示或暗示的担保，包括但不限于适销性、适用于特定目的和非侵权的担保。在任何情况下，作者或版权持有者均不对因使用本软件或与本软件的使用或其他交易有关的任何索赔、损害或其他责任负责，无论是在合同诉讼、侵权行为或其他方面。
 */

/* 如果您的目标内存有限，可以去掉此宏以节省10K RAM */
#define EIDSP_QUANTIZE_FILTERBANK   0

/*
 ** 注意：如果遇到TFLite内存分配问题。
 **
 ** 这可能是由于动态内存碎片化引起的。
 ** 尝试在boards.local.txt中定义"-DEI_CLASSIFIER_ALLOCATION_STATIC"（如果该文件不存在，请创建它），并将该文件复制到
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`。
 **
 ** 参见
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** 查找Arduino安装核心的位置。
 **
 ** 如果问题仍然存在，则表示该模型和应用程序所需的内存不足。
 */

/* 包含文件 ---------------------------------------------------------------- */
#include <XIAO-ESP32S3-KWS_inferencing.h>

#include <I2S.h>
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16

#define LED_BUILT_IN 21 

/** 音频缓冲区、指针和选择器 */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false; // 设置为true时，可以查看从原始信号生成的特征
static bool record_status = true;

/** 
 * @brief      Arduino 设置函数
 */
void setup()
{
    // 在此处放置您的设置代码，只会运行一次：
    Serial.begin(115200);
    // 注释掉下面一行以取消等待 USB 连接（对于原生 USB 必须需要）
    while (!Serial);
    Serial.println("Edge Impulse 推理演示");

    pinMode(LED_BUILT_IN, OUTPUT); // 设置引脚为输出
    digitalWrite(LED_BUILT_IN, HIGH); // 关闭 LED
    
    I2S.setAllPins(-1, 42, 41, -1, -1);
    if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
      Serial.println("初始化 I2S 失败!");
    while (1) ;
  }
    
    // 推理设置摘要（来自 model_metadata.h）
    ei_printf("推理设置:\n");
    ei_printf("\t间隔: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" 毫秒。\n");
    ei_printf("\t帧大小: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\t样本长度: %d 毫秒。\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\t类别数: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    ei_printf("\n将在2秒后开始连续推理...\n");
    ei_sleep(2000);

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("错误: 无法分配音频缓冲区（大小 %d），这可能是由于模型的窗口长度设置导致的\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    ei_printf("开始录音...\n");
}

/**
 * @brief      Arduino 主函数。运行推理循环。
 */
void loop()
{
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("错误: 录音失败...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("错误: 推理分类器失败 (%d)\n", r);
        return;
    }

    int pred_index = 0;     // 初始化 pred_index
    float pred_value = 0;   // 初始化 pred_value

    // 打印预测结果
    ei_printf("预测结果 ");
    ei_printf("(DSP: %d 毫秒, 分类: %d 毫秒, 异常: %d 毫秒)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");

        if (result.classification[ix].value > pred_value){
           pred_index = ix;
           pred_value = result.classification[ix].value;
      }
    }
    // 显示推理结果
    if (pred_index == 3){
      digitalWrite(LED_BUILT_IN, LOW); // 打开 LED
    }
    else{
      digitalWrite(LED_BUILT_IN, HIGH); // 关闭 LED
    }

    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    异常分数: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif
}

static void audio_inference_callback(uint32_t n_bytes) 
{
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
          inference.buf_count = 0;
          inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {

  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {

    /* 从 i2s 一次性读取数据 - 针对 XIAO ESP32S3 Sense 和 I2S.h 库的修改 */
    // i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read <= 0) {
      ei_printf("I2S 读取错误: %d", bytes_read);
    }
    else {
        if (bytes_read < i2s_bytes_to_read) {
        ei_printf("部分 I2S 读取");
        }

        // 调整数据大小（否则声音会太小）
        for (int x = 0; x < i2s_bytes_to_read/2; x++) {
            sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        }

        if (record_status) {
            audio_inference_callback(i2s_bytes_to_read);
        }
        else {
            break;
        }
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief      初始化推理结构并设置/启动 PDM
 *
 * @param[in]  n_samples  样本数量
 *
 * @return     { 返回值描述 }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    //    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
    //        ei_printf("I2S 启动失败!");
    //    }

    ei_sleep(100);

    record_status = true;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);

    return true;
}

/**
 * @brief      等待新数据
 *
 * @return     完成时返回 True
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    while (inference.buf_ready == 0) {
        delay(10);
    }

    inference.buf_ready = 0;
    return ret;
}

/**
 * 获取原始音频信号数据
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      停止 PDM 并释放缓冲区
 */
static void microphone_inference_end(void)
{
    free(sampleBuffer);
    ei_free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "当前传感器的模型无效。"
#endif
```

这个想法是，当检测到关键词 **HELLO** 时，LED 会亮起。同样地，除了点亮 LED，这也可以作为触发外部设备的“信号”，正如我们在介绍中所看到的。

# 待办事项
- [ ] 构建你自己的 KWS 项目并在 XIAO ESP32S3 Sense 上运行。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/11.png" style={{width:700, height:'auto'}}/></div>

#### 特别感谢

特别感谢 **[MJRoBot (Marcelo Rovai)](https://mjrobot.org/)** 提供的 XIAO ESP32S3 Sense 访问 Edge Impulse 的教程内容。原始文章非常详细，包含了很多关于机器学习的知识。

如果您想阅读这篇文章的原始内容，可以直接通过滚动下方链接访问原文。

- [TinyML 简单入门：关键词检测 (KWS)](https://www.hackster.io/mjrobot/tinyml-made-easy-keyword-spotting-kws-5fa6e7#toc-capturing--offline--audio-data-with-the-xiao-esp32s3-sense-5)

MJRoBot 还拥有许多关于 XIAO ESP32S3 的有趣项目。

- [与全新的 XIAO ESP32S3 一起探索机器学习](https://www.hackster.io/mjrobot/exploring-machine-learning-with-the-new-xiao-esp32s3-6463e5)
- [TinyML 简单入门：图像分类](https://www.hackster.io/mjrobot/tinyml-made-easy-image-classification-cb42ae)
