---
description: 蓝牙在Seeed Studio XIAO ESP32C3上的使用
title: 蓝牙使用
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/XIAO_ESP32C3_Bluetooth_Usage
last_update:
  date: 11/13/2024
  author: Agnes
---

# 使用 XIAO ESP32C3 的蓝牙功能

ESP32C3支持蓝牙5 (LE)连接。本wiki将介绍在此板上使用蓝牙的基础知识。

## 硬件设置

- **步骤 1.** 将附带的**WiFi/蓝牙天线**连接到电路板上的**IPEX连接器**

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/wifi-6.png" alt="pir" width={130} height="auto" /></div>


- **步骤 2.** 通过USB Type-C数据线将XIAO ESP32C3连接到您的计算机

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/cable-connect.png" alt="pir" width={120} height="auto" /></div>



## 扫描蓝牙设备

在本例中，我们将使用XIAO ESP32C3扫描它周围可用的蓝牙设备。

- **步骤 1.** 将下面的代码复制粘贴到Arduino IDE中

```cpp
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; // 扫描时间，单位：秒
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("广告设备: %s \n", advertisedDevice.toString().c_str());
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("开始扫描...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // 创建新的扫描
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // 启用主动扫描，使用更多电力，但可以更快地获得结果
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // 设置窗口，值应小于或等于setInterval的值
}

void loop() {
  // 在这里写入你的主程序，重复运行：
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("发现设备数量: ");
  Serial.println(foundDevices.getCount());
  Serial.println("扫描完成！");
  pBLEScan->clearResults();   // 从BLEScan缓冲区清除结果以释放内存
  delay(2000);
}
```
:::tip
如果您已经将 ESP32 开发板升级到了 3.0.0 以上版本，则需要更改一些代码以兼容它。
1. ```BLEScanResults foundDevices = pBLEScan->start(scanTime, false);``` 改为 ```BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);```
2. ```Serial.println(foundDevices.getCount());``` 改为 ```Serial.println(foundDevices->getCount());```
:::

**步骤 2.** 上传代码并打开串行监视器开始扫描蓝牙设备

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-1.jpg" alt="pir" width={1000} height="auto" /></div>


## ESP32C3作为蓝牙服务器

在本例中，我们将使用XIAO ESP32C3作为蓝牙服务器。在这里，我们将使用智能手机搜索XIAO ESP32C3板，并发送字符串以显示在串行监视器上

- **步骤  1.** 将下面的代码复制粘贴到Arduino IDE中

```cpp
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// 有关生成 UUID 的信息，请参阅以下内容：
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {
  // 将你的主代码放在这里，以重复运行：
  delay(2000);
}
```

:::tip
如果您已经将 ESP32 开发板升级到了 3.0.0 以上版本，则需要更改一些代码以兼容它。
1.  ```std::string value = pCharacteristic->getValue();``` 改为 ```String value = pCharacteristic->getValue();```
:::

- **步骤 2.**上传代码并打开串行监视器

- **步骤 3.** 在您的智能手机上下载并安装LightBlue App


  - [LightBlue App (Android)](https://play.google.com/store/apps/details?id=com.punchthrough.lightblueexplorer&hl=en_US&gl=US)
  - [LightBlue App (Apple)](https://apps.apple.com/us/app/lightblue/id557428110)


- **步骤 4.** 打开手机上的蓝牙，将手机靠近XIAO ESP32C3，扫描设备并连接**MyESP32**设备

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-2.jpg" alt="pir" width={300} height="auto" /></div>


- **步骤 5.** 打开LightBlue应用程序，点击**Bonded**标签

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-3.jpg" alt="pir" width={350} height="auto" /></div>


- **步骤 6.** 点击**MyESP32**旁边的**CONNECT**

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-4.jpg" alt="pir" width={350} height="auto" /></div>


- **步骤 7.** 点击最底部说**可读，可写**的部分

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-5.jpg" alt="pir" width={300} height="auto" /></div>


- **步骤 8.** 在**数据格式**下拉菜单中选择**UTF-8字符串**

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-6.jpg" alt="pir" width={300} height="auto" /></div>


- **步骤  9.** **WRITE**
57/5000
翻译

在**WRITTEN VALUES**下键入“Hello”，然后点击**WRITE**
<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-7.jpg" alt="pir" width={300} height="auto" /></div>


您将在Arduino IDE的串行监视器上看到文本字符串“Hello”输出

<div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/ble-9.jpg" alt="pir" width={500} height="auto" /></div>


## 技术支持和产品讨论

感谢您选择我们的产品!我们在这里为您提供不同的支持，以确保您使用我们产品的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的喜好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>