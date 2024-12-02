---
description: XIAO ESP32 Sense 使用不同的睡眠模式消费
title: XIAO ESP32S3 Sense 睡眠模式
keywords:
- Sleep_Modes
image: https://files.seeedstudio.com//wiki/ESP32S3_Sense_SleepMode/1.png
slug: /cn/XIAO_ESP32S3_Consumption
last_update:
  date: 11/20/2024
  author: Agnes
---

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/ESP32S3_Sense_SleepMode/1.png" style={{width:700, height:'auto'}}/></div>


在这里，我将使用一些简单的示例来演示如何使用这些低功耗睡眠模式。所有 ESP32 都是通用的，而我在这里使用的开发板是 XIAO ESP32S3 Sense。

<div class="table-center">
  <table align="center">
    <tr>
        <th>Seeed Studio XIAO ESP32S3 Sense</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/xiaoesp32s3sense.jpg" style={{width:250, height:'auto'}}/></div></td>
    </tr>
      <tr>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
          </a>
      </div></td>
    </tr>
  </table>
</div>

## 软件准备

在开始本文之前，如果您还没有使用过 XIAO ESP32S3 Sense 上的所有硬件功能，您需要进行一些软件安装准备。

下面，我提供了相关的跳转链接：

- [麦克风使用](https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/)
- [MicroSD 使用](https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/)
- [摄像头使用](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/)

## 深度睡眠

### 深度睡眠介绍

在深度睡眠模式下，CPU、大部分 RAM 以及所有由 APB_CLK 时钟控制的数字外设都会关闭。只有以下几个芯片部分仍然保持通电：

- RTC 控制器
- ULP 协处理器
- RTC 快速内存
- RTC 慢速内存

### 唤醒方式

- 定时器唤醒
- 触摸板中断唤醒
- 外部唤醒
- ULP 协处理器活动唤醒
- GPIO 唤醒

### 使用深度睡眠代码

在这段代码中，我使用定时器作为唤醒源，每 7 秒唤醒一次。

``` cpp
/*需要包含头文件*/
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ESP_I2S.h>
#include "esp_camera.h"
#include "camera_pins.h"


/*设置唤醒时间为七秒*/
#define uS_TO_S_FACTOR 1000000ULL /* 微秒到秒的转换因子 */
#define TIME_TO_SLEEP  7          /* ESP32进入睡眠的时间（单位：秒） */
#define CAMERA_MODEL_XIAO_ESP32S3 // 启用PSRAM

RTC_DATA_ATTR int bootCount = 0;

I2SClass I2S;


/*函数声明*/
void print_wakeup_reason();    // 打印唤醒源 
void SDCard_enable();         // 启用SD卡 
void Microphone_enable();     // 启用麦克风 
void Camera_enable();         // 初始化相机 
void Deep_Sleep_enable();     // 启动深度睡眠
void close_SDCard();
void close_Camera();
void close_Microphone();
void startCameraServer();
void setupLedFlash(int pin);



void setup() {
  Serial.begin(115200);
  while(!Serial);        // 确保串口已启用
}

void loop() {
  while(!Serial);        
  /*启用或关闭您的外设*/
  Deep_Sleep_enable();
  delay(10000);
}


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


void SDCard_Function(){
    if(!SD.begin(21)){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.println("the sc card is connecting");
}

void Microphone_Function(){
    // 设置42号引脚为PDM时钟，41号引脚为PDM数据
  I2S.setPinsPdmRx(42, 41);
  // 以16 kHz的采样率和每个样本16位的数据宽度启动I2S
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("初始化I2S失败!");
    while (1); // 如果初始化失败，程序停在这里，不继续执行
  }
  
  Serial.println("麦克风已打开");
  
  // 5秒后停止读取
  delay(500);  // 暂停500毫秒
}

void Camera_Function(){
  Serial.setDebugOutput(true);
  Serial.println();

/*config设置*/
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // 用于流媒体传输
//config.pixel_format = PIXFORMAT_RGB565; // 用于人脸检测/识别
config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
config.fb_location = CAMERA_FB_IN_PSRAM;
config.jpeg_quality = 12;
config.fb_count = 1;

// 如果存在PSRAM IC，则以UXGA分辨率和更高的JPEG质量初始化
//                      用于更大的预分配帧缓冲区。
if (config.pixel_format == PIXFORMAT_JPEG) {
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // 如果没有PSRAM，限制帧大小
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
} else {
  // 人脸检测/识别的最佳选项
  config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
  config.fb_count = 2;
#endif
}

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // 相机初始化 
esp_err_t err = esp_camera_init(&config);
if (err != ESP_OK) {
  Serial.printf("相机初始化失败，错误码 0x%x", err);
  return;
}

sensor_t *s = esp_camera_sensor_get();
// 初始传感器垂直翻转且颜色有些过饱和
if (s->id.PID == OV3660_PID) {
  s->set_vflip(s, 1);        // 翻转回来
  s->set_brightness(s, 1);   // 稍微调高亮度
  s->set_saturation(s, -2);  // 降低饱和度
}
// 为了提高初始帧率，降低帧大小
if (config.pixel_format == PIXFORMAT_JPEG) {
  s->set_framesize(s, FRAMESIZE_QVGA);
}

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
// 如果在camera_pins.h中定义了LED引脚，则设置LED闪光灯
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

Serial.println("相机已开启");
}

void Deep_Sleep_Function(){
  ++bootCount;
  Serial.println("启动次数: " + String(bootCount));
  // 打印ESP32的唤醒原因
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // 设置唤醒源
  Serial.println("设置ESP32每 " + String(TIME_TO_SLEEP) + " 秒进入睡眠");
  Serial.println("现在进入深度睡眠");
  Serial.flush();
  esp_deep_sleep_start();
}

void close_SDCard(){
  SD.end();
  Serial.println("the sd card already closed");
}
void close_Camera(){
  esp_camera_deinit();
  Serial.println("the camera already closed");
}
void close_Microphone(){
  I2S.end();
  Serial.println("the microphone already closed");
}

```

:::tip
如果在 XIAO ESP32S3 Sense 中使用深度睡眠模式，你需要将 XIAO ESP32S3 Sense 重新进入下载模式，按下 boot 按钮，然后按下重置按钮。
:::

## 调制解调器睡眠模式 (Modem-Sleep)

### 介绍调制解调器睡眠模式

调制解调器睡眠模式是 ESP32 中的另一种重要低功耗模式，区别于深度睡眠模式。调制解调器睡眠模式主要针对 ESP32 的无线通信模块进行了优化。

在此模式下，ESP32 的 WiFi/Bluetooth 模块进入睡眠状态，而 CPU 核心保持活跃。这使得 ESP32 能够在保持一定无线连接性的同时，显著降低功耗。

### 唤醒方式

- 定时器唤醒
- 外部中断唤醒
- 任务唤醒
- 网络活动唤醒

### 使用调制解调器睡眠代码

```cpp
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP_I2S.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "esp_camera.h"

/*此处顺序不可更改，否则会出现错误：未选择相机模式*/
#define CAMERA_MODEL_XIAO_ESP32S3 // 带有 PSRAM
#include "camera_pins.h"
/**************************/
I2SClass I2S;

const char* ssid = "************";
const char* password = "***********";

/*函数声明*/
void print_wakeup_reason(); // 打印唤醒源
void SDCard_enable(); // 启用SD卡
void Microphone_enable(); // 启用麦克风
void Camera_enable(); // 初始化相机
void wifi_enable(); // 启用WiFi
void Deep_Sleep_enable(); // 启动深度睡眠
void close_SDCard(); // 关闭SD卡
void close_Camera(); // 关闭相机
void close_Microphone(); // 关闭麦克风
void startCameraServer(); // 启动相机服务器
void setupLedFlash(int pin); // 设置LED闪光灯



void setup() {
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  delay(500);
  Serial.println("ready into deep sleep");
  esp_deep_sleep_start();
  Serial.flush();
  delay(5000);
}


void Camera_enable(){
  Serial.setDebugOutput(true);
  Serial.println();

/*config设置*/
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // 用于流媒体传输
//config.pixel_format = PIXFORMAT_RGB565; // 用于人脸检测/识别
config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
config.fb_location = CAMERA_FB_IN_PSRAM;
config.jpeg_quality = 12;
config.fb_count = 1;

// 如果存在PSRAM IC，则初始化为UXGA分辨率和更高的JPEG质量
//                      用于更大的预分配帧缓冲区。
if (config.pixel_format == PIXFORMAT_JPEG) {
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // 如果没有PSRAM，限制帧大小
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
} else {
  // 人脸检测/识别的最佳选项
  config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
  config.fb_count = 2;
#endif
}

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

// 相机初始化
esp_err_t err = esp_camera_init(&config);
if (err != ESP_OK) {
  Serial.printf("相机初始化失败，错误码 0x%x", err);
  return;
}

sensor_t *s = esp_camera_sensor_get();
// 初始传感器垂直翻转且颜色有些过饱和
if (s->id.PID == OV3660_PID) {
  s->set_vflip(s, 1);        // 翻转回来
  s->set_brightness(s, 1);   // 稍微调高亮度
  s->set_saturation(s, -2);  // 降低饱和度
}
// 为了提高初始帧率，降低帧大小
if (config.pixel_format == PIXFORMAT_JPEG) {
  s->set_framesize(s, FRAMESIZE_QVGA);
}


#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
// 如果在camera_pins.h中定义了LED引脚，则设置LED闪光灯
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

Serial.println("相机已开启");
}

void SDCard_Function(){
    if(!SD.begin(21)){
      Serial.println("卡片挂载失败");
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println("没有插入SD卡");
      return;
  }
  Serial.print("SD卡类型: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("未知");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD卡大小: %lluMB\n", cardSize);
  Serial.println("SD卡已连接");
}

void Microphone_Function(){
    // 设置42号PDM时钟和41号PDM数据引脚
  I2S.setPinsPdmRx(42, 41);
  // 启动I2S，采样率16 kHz，每个样本16位
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("I2S初始化失败！");
    while (1); // 什么都不做
  }
  Serial.println("麦克风已开启");
  // 5秒后停止读取
  delay(500);
}

void wifi_enable(){
  WiFi.mode(WIFI_STA); // 设置WiFi为客户端模式
  esp_wifi_start(); // 启用WiFi外设
  Serial.println("WiFi已开启");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi已连接");
  Serial.println("IP地址为：");
  Serial.print(WiFi.localIP());
}

void Modem_Sleep_enable(){
  // 关闭WiFi
  WiFi.mode(WIFI_OFF);

  // 检查WiFi是否关闭
  if (WiFi.getMode() == WIFI_OFF) {
    Serial.println("WiFi已关闭");
  } else {
    Serial.println("WiFi仍然开启");
  }
}

void close_SDCard(){
  SD.end();
  Serial.println("the sd card already closed");
}
void close_Camera(){
  esp_camera_deinit();
  Serial.println("the camera already closed");
}
void close_Microphone(){
  I2S.end();
  Serial.println("the microphone already closed");
}

```

## 总结

### 为什么使用深度睡眠模式
最大化节省电力而不影响功能，以延长设备的电池寿命。  
适用场景：电池寿命至关重要的应用，如远程传感器节点、可穿戴设备和其他低功耗物联网设备。尽管唤醒时间相对较慢，但这个权衡是值得的。

### 为什么使用调制解调器睡眠模式
优化无线通信模块的功耗，同时仍然保持网络连接。  
适用场景：需要保持网络连接但又要求低功耗的应用，如间歇性工作物联网设备。调制解调器睡眠模式可以显著降低无线模块的功耗，同时仍能提供快速的唤醒响应。

### 总结
这三种睡眠模式为开发者提供了不同的电力/性能权衡选项，可以根据应用的具体需求灵活选择。对于有电池寿命要求的设备，深度睡眠模式是一个不错的选择；而对于需要保持网络连接的物联网设备，调制解调器睡眠模式是最佳选择。

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，确保您使用我们的产品时体验顺畅。我们提供了多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>