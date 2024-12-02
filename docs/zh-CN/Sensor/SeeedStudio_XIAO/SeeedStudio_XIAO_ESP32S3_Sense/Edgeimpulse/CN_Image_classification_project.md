---
description: 对图像中的狗和猫进行分类。
title: 图像分类
keywords:
- tinyml course
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/tinyml_course_Image_classification_project
last_update:
  date: 11/27/2024
  author: Agnes
---

# 图像分类

在这里，我们将使用 XIAO ESP32-S3-Sense 构建一个 tinyML 项目，用于分类狗和猫的照片。让我们开始吧。

### 它是如何工作的
我们将在 XIAO 上运行机器学习模型，并将相机流输入到模型中。然后，XIAO 会推理结果并进行预测，借助我们实现的板载神经网络。让我们来构建一个。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/digram.png?raw=true" style={{width:1000, height:'auto'}}/></div> 

#### 所需物品
* XIAO ESP32-Sense
* <32GB MicroSD 卡
* Type-C 数据线
* Arduino IDE
* Edgeimpulse 账户

## 第 1 步：收集猫狗图像

机器学习项目的第一步是收集数据集，这里我们需要收集猫和狗的图像。我们可以通过两种方式收集图像：

1. 直接通过 XIAO-ESP32-S3-Sense 收集图像并保存到 SD 卡，之后上传到 EdgeImpulse。
2. 直接通过手机、互联网或公开数据集收集图像，然后上传到 EdgeImpulse。

### 1.1 方法 1：通过 XIAO-ESP32-S3-Sense 收集图像

在这里，我们使用 Sense 相机模块收集图像并保存到 SD 卡，然后稍后上传到 EdgeImpulse。

#### 1.1.1 连接相机

如果你正在购买 XIAO ESP32-S3-Sense，那么你应该还需要一个扩展板。这个扩展板配备了 1600*1200 分辨率的 OV2640 相机传感器，板载 SD 卡插槽和数字麦克风。

通过将扩展板安装到 XIAO ESP32-S3-Sense 上，你可以使用扩展板上的功能。

安装扩展板非常简单，只需要将扩展板上的连接器与 XIAO ESP32-S3 上的 B2B 连接器对齐，按下直到听到“咔哒”一声，安装完成。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/61.gif" style={{width:500, height:'auto'}}/></div>

现在，我们有了一款全新的、完全兼容 XIAO ESP32-S3-Sense 的强大相机——OV5640。如果你购买了它，可以将其替换为现有的相机。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/ov5640.gif" style={{width:500, height:'auto'}}/></div>

<div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/OV5640-Camera-for-XIAO-ESP32S3-Sense-With-Heat-Sink-p-5739.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
</div>

- **第 1 步**：根据你的操作系统下载并安装稳定版的 Arduino IDE。

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载 Arduino IDE</font></span></strong>
    </a>
</div>

#### 1.1.2 在 Arduino 中安装 XIAO-ESP32-S3-Sense

- **第 1 步**：将 ESP32 板包添加到 Arduino IDE 中。

    导航至 **文件 > 首选项**，在 **“附加开发板管理器网址”** 中填入以下网址：
    *<https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json>*

    <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/6.png" style={{width:800, height:'auto'}}/></div>

    导航至 **工具 > 开发板 > 开发板管理器...**，在搜索框中输入关键字 **esp32**，选择最新版本的 **esp32**，并安装。

    <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/9.png" style={{width:1000, height:'auto'}}/></div>

:::caution
XIAO ESP32-S3 的板载包至少需要 **2.0.8** 版本才能使用。
:::

- **第 2 步**：选择开发板和端口。

在 Arduino IDE 顶部，你可以直接选择端口，通常为 COM3 或更高版本（**COM1** 和 **COM2** 通常保留用于硬件串口）。

此外，在左侧的开发板列表中搜索 **xiao**，并选择 **XIAO_ESP32S3**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/10.png" style={{width:600, height:'auto'}}/></div>

完成这些准备工作后，你可以开始为 XIAO ESP32S3 编写程序并进行编译和上传。

#### 1.2 上传可以拍照并保存到 SD 卡的 Sketch

从 [这里](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/take_photos_command.ino.zip) 下载 Sketch 文件，解压到你的计算机并打开 "take_photos_command.ino" 文件。然后选择正确的端口和开发板，上传到 XIAO。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/img_CaptureSketch01.png?raw=true" style={{width:600, height:'auto'}}/></div>

在拍照时，确保对准我们需要收集的图像或物体作为数据集。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/cat_image_snap.png?raw=true" style={{width:600, height:'auto'}}/></div>

由于这是图像分类，我们需要收集尽可能多的图像，以使系统更加稳定。因此，收集更多可爱的狗狗和猫咪图像。

### 1.2 方法 2：直接将图像上传到 EdgeImpulse

对于这种方法，我们需要通过互联网或手机收集图像。幸运的是，我们有像 [Kaggle](https://www.kaggle.com/) 这样的服务提供猫狗图像数据集，请 [点击这里](https://www.kaggle.com/datasets/tongpython/cat-and-dog) 进行查看。

一旦收集到数据集，打开你的 EdgeImpulse 项目，进入 **数据采集**，然后选择 **+ 添加数据**，选择 **上传数据** 选项。

<div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/salmanfarisvp/TinyML/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/collectData1.png" style={{width:600, height:'auto'}}/></div>

在该页面中，选择上传模式为 **"选择文件夹"**，然后选择以下选项中的文件。确保选择 **“自动拆分为训练和测试”** 并将标签设置为 **“从文件名推断”**。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/DataCollection01.png?raw=true" style={{width:600, height:'auto'}}/></div>

上传完成后，你将看到如下窗口。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/DataCollection02.png?raw=true" style={{width:600, height:'auto'}}/></div>

## 第 2 步：设计 Impulse 并训练我们的神经网络

在收集完项目的数据后，你现在可以创建你的 Impulse。一个完整的 Impulse 将由 3 个主要模块组成：输入模块、处理模块和学习模块。

1. 要设计 Impulse，选择 **创建 Impulse**，进入 **Impulse 设计**。
2. **输入模块**：输入模块表示你用来

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/train2.png?raw=true" style={{width:600, height:'auto'}}/></div>


一旦训练完成，我们就可以下载我们创建的TinyML库了。前往**部署选项**，选择**Arduino库**作为部署方式，然后点击**构建**来生成库文件。

<div style={{textAlign:'center'}}><img src="https://github.com/salmanfarisvp/TinyML/blob/main/XIAO-esp32-S3-Sense/Image%20Recognition/src/img/deployment1.png?raw=true" style={{width:600, height:'auto'}}/></div>

库文件将自动下载。下载完成后，打开Arduino IDE，点击**草图** > **包含库** > **添加.ZIP库**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Get_Started_With_Arduino/img/Add_Zip.png" style={{width:1000, height:'auto'}}/></div>

选择刚才下载的zip文件，如果库安装成功，您会看到库已添加到您的库列表中，说明库已经成功安装。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Get_Started_With_Arduino/img/upload_complete.png" style={{width:1000, height:'auto'}}/></div>

## 第3步：运行示例代码

尽管Edge Impulse尚未发布其用于ESP32S3并使用ESP NN加速器的SDK，但得益于Dmitry Maslov的工作，我们可以恢复并修复ESP32-S3的汇编优化。这个解决方案目前尚未官方发布，EI将在修复与其他开发板的冲突后将其纳入EI SDK中。

:::caution
目前，这只适用于非EON版本。因此，您还应确保未选中**启用EON编译器**选项。
:::

当点击**构建**按钮时，系统会创建一个Zip文件并将其下载到您的计算机。

在使用下载的库之前，我们需要启用**ESP NN**加速器。为此，您可以从[项目GitHub](https://github.com/Mjrovai/XIAO-ESP32S3-Sense/blob/main/ESP-NN.zip)下载预发布版本，解压后将ESP NN文件夹替换为`src/edge-impulse-sdk/porting/espressif/ESP-NN`，并将其放入您的Arduino库文件夹中。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoesp32s3_kws/10.png" style={{width:800, height:'auto'}}/></div>

复制下面的推理代码并粘贴到您的Arduino IDE中：

```
/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <XIAO_esp32S3_CatDog2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

// 选择相机模型 - 更多相机模型可以在 camera_pins.h 文件中找到
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

#define CAMERA_MODEL_XIAO_ESP32S3 // 支持PSRAM

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      21


/* 常量定义 -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* 私有变量 ------------------------------------------------------- */
static bool debug_nn = false; // 设置为true以查看例如从原始信号生成的特征
static bool is_initialised = false;
uint8_t *snapshot_buf; // 指向捕获结果的输出

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz或10MHz用于OV2640双倍帧率（实验性）
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // 支持YUV422, GRAYSCALE, RGB565, JPEG
    .frame_size = FRAMESIZE_QVGA,    // 支持QQVGA-UXGA，不建议使用大于QVG的分辨率，除非使用JPEG

    .jpeg_quality = 12, // 0-63，数值越低质量越高
    .fb_count = 1,       // 如果大于1，i2s将运行在连续模式下。仅在JPEG模式下使用
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* 函数定义 ------------------------------------------------------- */ 
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/**
* @brief      Arduino 设置函数
*/
void setup()
{
    // 在此处写下设置代码，只会运行一次：
    Serial.begin(115200);
    // 注释掉下面这一行，以便上传后立即开始推理
    while (!Serial);
    Serial.println("Edge Impulse 推理演示");
    if (ei_camera_init() == false) {
        ei_printf("相机初始化失败!\r\n");
    }
    else {
        ei_printf("相机初始化成功\r\n");
    }

    ei_printf("\n2秒后开始连续推理...\n");
    ei_sleep(2000);
}

/**
* @brief      获取数据并运行推理
*
* @param[in]  debug  如果为true，获取调试信息
*/
void loop()
{

    // 替代wait_ms，我们将等待信号，这样允许线程取消我们...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // 检查内存分配是否成功
    if(snapshot_buf == nullptr) {
        ei_printf("错误：内存分配快照缓冲区失败!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("获取图像失败\r\n");
        free(snapshot_buf);
        return;
    }

    // 运行分类器
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("错误：运行分类器失败 (%d)\n", err);
        return;
    }

    // 打印预测结果
    ei_printf("预测结果 (DSP: %d ms., 分类: %d ms., 异常: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("    %s (%f) [ x: %u, y: %u, 宽: %u, 高: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found) {
        ei_printf("    没有找到物体\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                    result.classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    异常分数: %.3f\n", result.anomaly);
#endif

    free(snapshot_buf);
}

/** 
 * @brief   设置图像传感器并开始流媒体传输
 *
 * @retval  如果初始化失败则返回 false
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    // 初始化相机
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("相机初始化失败，错误代码 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // 初始传感器上下翻转，颜色稍微饱和
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // 翻转回来
      s->set_brightness(s, 1); // 稍微提高亮度
      s->set_saturation(s, 0); // 降低饱和度
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      停止传感器数据流
 */
void ei_camera_deinit(void) {

    // 反初始化相机
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("相机反初始化失败\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      捕获、重缩放并裁剪图像
 *
 * @param[in]  img_width     输出图像的宽度
 * @param[in]  img_height    输出图像的高度
 * @param[in]  out_buf       用于存储输出图像的指针，如果要使用ei_camera_frame_buffer进行捕获和重缩放/裁剪，out_buf可以为NULL。
 *
 * @retval     如果未初始化，或捕获、重缩放或裁剪图像失败则返回 false
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("错误：相机未初始化\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("相机捕获失败\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("转换失败\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // 我们已经有一个 RGB888 缓冲区，因此重新计算偏移量到像素索引
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // 转到下一个像素
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // 完成！
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "当前传感器的模型无效"
#endif
```

确保将 **"#include <XIAO_esp32S3_CatDog2_inferencing.h>"** 替换为您生成的库。然后上传代码并指向猫或狗的图像或实际的猫狗，我们可以在串口监视器中看到结果。

恭喜您 🎉 完成了 tinyML 图像分类项目。

## 资源

[PDF] **[ESP32-S3 数据手册](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/esp32-s3_datasheet.pdf)**

### 适用于 Seeed Studio XIAO ESP32S3

- **[PDF]** [Seeed Studio XIAO ESP32S3 原理图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_SCH_v1.2.pdf)

- **[ZIP]** [Seeed Studio XIAO ESP32S3 Eagle 库](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_v1.1_SCH&PCB_230327.zip)

- **[DXF]** [Seeed Studio XIAO ESP32S3 DXF 尺寸图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_v1.1_Dimensioning.dxf)

- **[LBR]** [Seeed Studio XIAO ESP32S3 Eagle 封装库](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/Seeed-Studio-XIAO-ESP32S3-footprint-eagle.lbr)

- **[ZIP]** [Seeed Studio XIAO ESP32S3 工厂固件](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO-ESP32S3-firmware.zip)

- **[XLSX]** [Seeed Studio XIAO ESP32S3 引脚表](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_Sense_Pinout.xlsx)

<!-- - **[STEP]** [Seeed Studio XIAO ESP32S3 3D 模型]() -->

<!-- - **[ZIP]** [Seeed Studio XIAO ESP32S3 认证文件]() -->

### 适用于 Seeed Studio XIAO ESP32S3 Sense

- **[PDF]** [Seeed Studio XIAO ESP32S3 Sense 原理图](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf)

- **[ZIP]** [Seeed Studio XIAO ESP32S3 Sense KiCAD 库](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/Seeeduino-xiao-ESP32S3-KiCAD-Library.zip)

- **[ZIP]** [Seeed Studio XIAO ESP32S3 Sense Eagle 库](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_SCH&PCB_230324.zip)

- **[DXF]** [Seeed Studio XIAO ESP32S3 Sense DXF 尺寸图（顶部）](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_top.dxf)

- **[DXF]** [Seeed Studio XIAO ESP32S3 Sense DXF 尺寸图（底部）](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_bot.dxf)

- **[ZIP]** [Seeed Studio XIAO ESP32S3 Sense 工厂固件](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAOESP32S3-Sense-firmware.zip)

- **[XLSX]** [Seeed Studio XIAO ESP32S3 Sense 引脚表](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_Sense_Pinout.xlsx)

<!-- - **[STEP]** [Seeed Studio XIAO ESP32S3 Sense 3D 模型]() -->

<!-- - **[ZIP]** [Seeed Studio XIAO ESP32S3 Sense 认证文件]() -->

### 其他

- **[STP]** [XIAO ESP32S3 Sense 外壳设计（顶部）](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO-ESP32S3-Sense-housing-design(top).stp)

- **[STP]** [XIAO ESP32S3 Sense 外壳设计（底部）](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO-ESP32S3-Sense-housing-design(bottom).stp)

*剩余的开源材料正在整理中，敬请期待！*

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供各种支持，确保您使用我们产品的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的需求和偏好。


<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>








