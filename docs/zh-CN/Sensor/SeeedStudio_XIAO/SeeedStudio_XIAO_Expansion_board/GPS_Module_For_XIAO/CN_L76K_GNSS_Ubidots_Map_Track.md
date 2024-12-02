---
title: Ubidots 上的 L76K 路径跟踪
description: Connecting L76K GNSS Module and SeeedStudio XIAO to Ubidots for Location Path Tracking on A Map
keywords: 
  - XIAO
  - Expansion Boards for XIAO
  - GPS Module for XIAO
  - L76K Path Tracking on Ubidots
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/L76K_Path_Tracking_on_Ubidots
last_update: 
  date: 11/20/2024
  author: Agnes
---


# 将 L76K GNSS 模块和 SeeedStudio XIAO 连接到 Ubidots，实现地图上的位置轨迹追踪

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic00_Track.png" alt="pir" width={600} height="auto"/>
</p>

## 简介
在 [开始使用 L76K GNSS 模块 for SeeedStudio XIAO](https://wiki.seeedstudio.com/get_start_l76k_gnss/) 之后，您可能想使用 L76K GNSS 模块来定位物体并在地图上显示轨迹。为此，我们可以通过结合 SeeedStudio XIAO 开发板和 Ubidots IoT 数据平台来实现。

[Ubidots](https://ubidots.com/) 是一个低代码的物联网开发平台，专为那些没有时间或精力自行构建完整、生产就绪的物联网应用的工程师和开发者设计。从面向设备的 API 到为最终用户设计的简洁 UI，Ubidots 提供了实现快速上市所需的基本构建模块，无需雇佣昂贵的工程团队来开发和维护定制的解决方案。

### 特性
- 当连接到 Wi-Fi 时，上传实时位置数据（纬度和经度）
- 在地图上显示由位置点连接的轨迹

## 开始使用
### 步骤 1：获取 Ubidots Token
首先，在浏览器中打开 https://ubidots.com，然后注册一个账户。确认您的电子邮件，并登录到 Ubidots 控制台。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic02_SignUp.png" alt="pir" width={600} height="auto"/>
</p>

点击右上角的头像 - "我的资料"，向下滚动并将纬度和经度的“小数位数”从 2 改为 6，以提高精度。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic04_Setting.png" alt="pir" width={600} height="auto"/>
</p>

然后，点击左侧的 "API 凭证"，复制 token（**不要复制 API 密钥**），以备后续使用。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic05_Token.png" alt="pir" width={600} height="auto"/>
</p>

### 步骤 2：将代码上传到 XIAO
将 SeeedStudio XIAO 开发板（本示例中使用的是 SeeedStudio XIAO ESP32S3）、L76K GNSS 模块、GNSS 天线、Wi-Fi 天线等连接到一起，并与计算机连接。

:::danger **警告**
请特别注意模块的安装方向，务必不要插反，否则可能会烧毁模块或 XIAO。
:::

接下来，启动 Arduino IDE。记得在库管理器中添加 `EspSoftwareSerial` 和 `TinyGPSPlus` 库，同时下载并添加 [Ubidots ESP32 库](https://github.com/ubidots/ubidots-esp32)。

选择对应的开发板和端口，然后粘贴以下代码：

```cpp
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <Ubidots.h>

static const int RXPin = D7, TXPin = D6;
static const uint32_t GPSBaud = 9600;
const char WIFI_SSID[]     = "INPUT YOUR WIFI NAME HERE";
const char WIFI_PASS[]     = "INPUT YOUR WIFI PASSWORD HERE";
const char UBIDOTS_TOKEN[] = "INPUT YOUR UBIDOTS TOKEN HERE";

SoftwareSerial MySerial(RXPin, TXPin);
TinyGPSPlus gps;
Ubidots ubidots(UBIDOTS_TOKEN, UBI_UDP);
double lat;
double lng;

void setup() {
  Serial.begin(115200);
  MySerial.begin(GPSBaud);
  ubidots.setDebug(true);    // 用于观察 Ubidots 上传日志。如果需要简化的串口监视器输出，可以将其改为 "false"。
  Serial.println("\nTinyGPSPlus 库版本: " + String(TinyGPSPlus::libraryVersion()));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  while (WiFi.status() != 3) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println(WiFi.status());
    delay(5000);
  }

  /*
    WL_NO_SHIELD        = 255,    // 与 WiFi Shield 库兼容
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
  */

  Serial.println("WiFi 已连接！");
}

void loop() {
  while (MySerial.available() > 0) {
    if (gps.encode(MySerial.read())) {
      getLocation();
      sendToUbidots();
      delay(10 * 1000);  // 更改此参数来修改获取和上传位置的时间间隔。
    }
  }
}

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected, please check wiring.");
  }
}

void getLocation() {
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();

    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  } else {
    Serial.println("Unable to get location currently");
  }
}

void sendToUbidots() {
  if (lat != 0 && lng != 0) {
    char charLat[20];
    char charLng[20];
    sprintf(charLat, "%.6lf", lat);
    sprintf(charLng, "%.6lf", lng);

    ubidots.addContext("lat", charLat);
    ubidots.addContext("lng", charLng);
    char* context = (char*)malloc(sizeof(char) * 60);
    ubidots.getContext(context);
    ubidots.add("position", 1, context);

    if (ubidots.send()) {
      Serial.println("Values sent");
    } else {
      Serial.println("Values not sent");
    }
    free(context);
  }
}
```

上传到开发板后，您很快会在串口监视器中看到类似以下的输出：

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic06_SerialMonitor.png" alt="pir" width={600} height="auto"/>
</p>

<!--硬件连接正常工作与屏幕截图放一起-->

正如上图所示，等待连接 Wi-Fi 网络并从卫星获取位置信息是正常的。如果这些错误输出持续几分钟，尝试按一下位于 USB-C 端口旁的小 "R" 按钮重启 XIAO 开发板。

:::tip
L76K GNSS 模块需要在户外使用，请将其放置在无遮挡的开阔区域，否则可能无法获取位置信息。
:::

### 步骤 3：在地图上显示数据
现在，L76K GNSS 模块和 SeeedStudio XIAO 正在从 GNSS 获取位置，并将纬度和经度信息发送到 Ubidots。让我们回到 Ubidots，查看数据。在 https://industrial.ubidots.com/app/devices 页面，您会看到一个新 "设备"，该设备已通过我们发送数据的 token 自动创建。点击设备名称，您可以看到该设备的位置信息已自动设置为我们上传的数据。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic08_DeviceInfo.png" alt="pir" width={600} height="auto"/>
</p>

接下来，让我们创建一个地图来显示轨迹。进入顶部的 "Data" - "Dashboards"，点击 "Demo Dashboard" 旁的汉堡菜单按钮，然后点击 "CREATE" 创建一个新仪表板。您可以按如下方式修改设置，或根据自己的需求进行定制。记得点击 "SAVE" 保存新仪表板。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic10_NewDashboard.png" alt="pir" width={600} height="auto"/>
</p>

在新仪表板中，点击 "Add new widget"，然后向下滚动找到 "Map"。点击 "ADD MARKER GROUP"，选择我们刚才检查的设备，地图就会出现。将鼠标移动到地图的右下角，可以调整地图的大小。

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic11_NewWidget.png" alt="pir" width={600} height="auto"/>
</p>

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic12_MapSetting.png" alt="pir" width={600} height="auto"/>
</p>

太棒了！由位置点连接的轨迹现在就在我们面前展示了！

<p style={{textAlign: 'center'}}>
  <img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/GPS_Module/Ubidots/Pic00_Track.png" alt="pir" width={600} height="auto"/>
</p>

:::tip
如果 L76K GNSS 模块长时间处于固定位置且没有移动，地图将仅显示一个点，而不是路径，这是显而易见的。
:::

## 参考链接
- [GPS Location | Ubidots API Reference](https://docs.ubidots.com/reference/gps-location)

- [在 Ubidots 中创建地图小部件 | Ubidots 帮助中心](https://help.ubidots.com/en/articles/1712418-create-map-widgets-in-ubidots)

- [如何创建实时地图？ | Ubidots 帮助中心](https://help.ubidots.com/en/articles/693652-how-to-create-a-real-time-map)

- [Ubidots ESP32 库在 GitHub 上](https://github.com/ubidots/ubidots-esp32)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供不同的支持，以确保您的使用体验尽可能顺利。我们提供多个沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
  <a href="https://forum.seeedstudio.com/" class="button_forum"></a>
  <a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>
<div class="button_tech_support_container">
  <a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a>
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
