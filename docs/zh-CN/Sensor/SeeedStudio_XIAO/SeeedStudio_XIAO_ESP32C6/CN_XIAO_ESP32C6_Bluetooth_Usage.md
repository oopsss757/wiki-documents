---
description: Seeed Studio XIAO ESP32C6 的蓝牙使用
title: 蓝牙使用
keywords:
- esp32c6
- xiao
- ble
- bluetooth
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32c6_bluetooth
last_update:
  date: 11/16/2024
  author: Agnes
---

# Seeed Studio XIAO ESP32C6 的蓝牙使用

<div class="table-center">
  <table align="center">
    <tr>
        <th>Seeed Studio XIAO ESP32C6</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
    </tr>
      <tr>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
          </a>
      </div></td>
    </tr>
  </table>
</div>

Seeed Studio XIAO ESP32C6 是一款强大的开发板，支持蓝牙5、BLE（蓝牙低能耗）以及Mesh网络，特别适用于需要无线连接的物联网应用。凭借其卓越的射频性能，XIAO ESP32C6可以在不同的距离范围内提供可靠且高速的无线通信，既适合短距离应用，也适合长距离应用。在本教程中，我们将重点介绍XIAO ESP32C6的蓝牙功能，包括如何扫描附近的蓝牙设备、如何建立蓝牙连接以及如何通过蓝牙连接进行数据传输和接收。

## 蓝牙低能耗（BLE）使用

蓝牙低能耗（BLE）是一种节能的蓝牙变种。BLE的主要应用是进行短距离、小数据量（低带宽）传输。与总是处于开启状态的传统蓝牙不同，BLE在没有连接时保持休眠模式，只有在发起连接时才会唤醒。

由于BLE的特点，它非常适用于需要定期交换少量数据的应用，且能够使用纽扣电池长时间运行。例如，BLE在医疗保健、健身、追踪、信标、安全以及家居自动化等行业中都有广泛应用。

与传统蓝牙相比，BLE的功耗低得多，约为蓝牙的1/100（具体取决于使用场景）。

关于XIAO ESP32C6上的BLE功能，我们将通过以下三个部分进行介绍：

- [一些基本概念 ](#some-fundamental-concepts) -- 介绍在BLE中常用的概念，帮助我们理解BLE程序的执行过程和思路。
- [BLE扫描器 ](#ble-scanner) -- 讲解如何搜索附近的蓝牙设备，并在串口监视器中显示扫描结果。
- [BLE服务器/客户端](#ble-serverclient) -- 讲解如何使用XIAO ESP32C6作为服务器和客户端发送和接收指定数据。
- [BLE传感器数据交换](#ble-sensor-data-exchange) -- 本教程的最后部分，演示如何通过BLE发送传感器数据。

### 一些基本概念

#### 服务器和客户端

在蓝牙低能耗（BLE）中，有两种设备类型：服务器和客户端。XIAO ESP32C6既可以作为客户端，也可以作为服务器。

服务器会广播其存在，以便其他设备可以找到它，并提供客户端可以读取的数据。客户端则扫描附近的设备，当它找到目标服务器后，会建立连接并监听接收的数据。这种方式称为点对点通信。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/ble.png" style={{width:800, height:'auto'}}/></div>

#### 属性

属性实际上是一条数据。每个蓝牙设备提供一种服务，服务由一组数据构成，这组数据可以称为数据库，每个数据库中的条目即为属性。所以，可以将属性理解为数据条目。你可以将蓝牙设备看作一张表格，每一行就是一个属性。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/52.png" style={{width:600, height:'auto'}}/></div>

#### GATT（通用属性配置文件）

当两台蓝牙设备建立连接时，它们需要一个协议来确定如何通信。GATT（通用属性配置文件）就是这样一个协议，定义了蓝牙设备间如何传输数据。

在GATT协议中，设备的功能和属性被组织成服务、特征和描述符。服务表示设备提供的一组相关功能和特性，每个服务可以包含多个特征，特征定义了服务的某个属性或行为，如传感器数据或控制命令。每个特征都有一个唯一的标识符和一个值，可以进行读取或写入操作。描述符用于描述特征的元数据，例如特征值的格式和访问权限。

通过使用GATT协议，蓝牙设备可以在不同的应用场景下进行通信，如传输传感器数据或控制远程设备。

#### BLE特征

ATT（属性协议）由一组ATT命令组成，包括请求和响应命令。ATT是蓝牙包的最上层，因此我们最常分析的蓝牙包就是ATT命令。

ATT命令正式称为ATT PDU（协议数据单元），它包括4类命令：读取、写入、通知和指示。这些命令分为两种类型：一种是需要响应的命令，另一种是只需要确认（ACK）但不需要响应的命令。

服务和特征在GATT层中定义。服务端提供服务，服务包含数据，数据为属性，服务和特征是数据的逻辑呈现，用户看到的数据最终会转化为服务和特征。

让我们从移动角度看一下服务和特征是什么样子的。nRF Connect 是一款非常直观的应用程序，它向我们展示了每个数据包应该是什么样子。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/62.png" style={{width:400, height:'auto'}}/></div>

如您所见，在蓝牙规范中，每个特定的蓝牙应用都由多个 Service 组成，每个 Service 又由多个 Characteristics 组成。特征由 UUID、Properties 和 Value 组成。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/50.png" style={{width:300, height:'auto'}}/></div>

属性用于描述对特征的操作类型和权限，例如是否支持读取、写入、通知等。这类似于 ATT PDU 中包含的四个类别。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/51.png" style={{width:800, height:'auto'}}/></div>

#### UUID（通用唯一标识符）

每个服务、特征和描述符都有一个UUID（通用唯一标识符）。UUID是一个唯一的128位（16字节）数字。例如：

```
ea094cbd-3695-4205-b32d-70c1dea93c35
```

所有类型、服务和配置文件都有缩短的 UUID，具体参见 [SIG (蓝牙特别兴趣小组)](https://www.bluetooth.com/specifications/gatt/services)。但是，如果你的应用需要自定义 UUID，可以使用这个 [UUID 生成网站](https://www.uuidgenerator.net/)来生成。

### BLE 扫描器

创建 XIAO ESP32C6 BLE 扫描器非常简单。以下是一个创建扫描器的示例程序。

```cpp
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; //扫描时间，单位为秒
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("广告设备: %s \n", advertisedDevice.toString().c_str());
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("正在扫描...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //创建新的扫描器
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //激活扫描，使用更多电力，但扫描速度更快
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // 设置窗口小于等于扫描间隔值
}

void loop() {
  // 主循环代码，重复执行：
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("扫描到的设备数量: ");
  Serial.println(foundDevices.getCount());
  Serial.println("扫描完成！");
  pBLEScan->clearResults();   // 删除扫描结果缓存以释放内存
  delay(10000);
}
```

现在你可以选择 XIAO ESP32C6 主板并上传该程序。如果程序运行正常，打开串口监视器并将波特率设置为 115200，你将看到以下结果。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/54.png" style={{width:700, height:'auto'}}/></div>

该程序将打印出扫描到的蓝牙设备的名称、MAC 地址、制造商数据和信号强度。

#### 程序注释

程序开始时导入了 BLE 功能所需的库文件。

然后定义了一个名为 `MyAdvertisedDeviceCallbacks` 的类，继承自 `BLEAdvertisedDeviceCallbacks` 类。它包含一个名为 `onResult` 的函数，在扫描到广告蓝牙设备时被调用。该函数使用 `Serial.printf` 函数将设备信息打印到串口。此类可用于在扫描过程中处理蓝牙设备信息。

```c
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};
```

在 `setup` 函数中，我们设置了 BLE 扫描器并指定了扫描间隔和窗口值。它还初始化了 BLE 设备并设置了一个回调函数，用于处理扫描到的广告设备。

```c
BLEDevice::init("");
pBLEScan = BLEDevice::getScan();
pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
pBLEScan->setActiveScan(true);
pBLEScan->setInterval(100);
pBLEScan->setWindow(99);
```

最后，`loop` 函数启动了 BLE 扫描，设置了指定的扫描时间和阻塞标志。然后，它将扫描到的设备数量打印到串口，并清除结果缓存以释放内存。

```c
BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
Serial.print("Devices found: ");
Serial.println(foundDevices.getCount());
Serial.println("Scan done!");
pBLEScan->clearResults();
```

### BLE 服务器/客户端

如前所述，XIAO ESP32C6 可以同时作为服务器和客户端。我们来看一个作为服务器的示例程序，并了解如何使用它。在上传以下程序到 XIAO 后，它将充当服务器，并向所有连接到 XIAO 的蓝牙设备发送一个 "Hello World" 消息。

```cpp
//服务器代码
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
  Serial.begin(115200);
  Serial.println("开始 BLE 工作！");

  BLEDevice::init("XIAO_ESP32C6");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // 为了向后兼容仍然可以工作
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // 帮助解决 iPhone 连接问题
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("特征已定义！现在可以在手机上读取！");
}

void loop() {
  // 主循环代码，重复执行：
  delay(2000);
}
```

同时，您可以在主流的移动应用商店中搜索并下载 **nRF Connect** 应用程序，该应用允许您的手机搜索并连接蓝牙设备。

- Android: [nRF Connect](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en)
- IOS: [nRF Connect](https://apps.apple.com/us/app/nrf-connect-for-mobile/id1054362403)

下载软件后，按照以下步骤搜索并连接 XIAO ESP32C6，您将看到广告中显示的 "Hello World"。

<table align="center">
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/55.jpg" style={{width:200, height:'auto'}}/></div></td>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/ble_app.png" style={{width:200, height:'auto'}}/></div></td>
		<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/58.jpg" style={{width:200, height:'auto'}}/></div></td>
		<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/59.jpg" style={{width:200, height:'auto'}}/></div></td>
	</tr>
</table>

如果您希望使用另一块 XIAO ESP32C6 作为客户端接收来自服务器的消息，您可以使用以下代码在客户端 XIAO 上实现。

```cpp
// 客户端代码
#include "BLEDevice.h"
//#include "BLEScan.h"

// 我们希望连接的远程服务。
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// 我们感兴趣的远程服务的特性。
static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.write(pData, length);
    Serial.println();
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("正在连接到 ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - 创建客户端");

    pClient->setClientCallbacks(new MyClientCallback());

    // 连接到远程 BLE 服务器。
    pClient->connect(myDevice);  // 如果传递 BLEAdvertisedDevice 而不是地址，它将识别对等设备地址（公共或私有）
    Serial.println(" - 连接到服务器");
    pClient->setMTU(517); //设置客户端请求最大 MTU（默认是 23）

    // 获取我们在远程 BLE 服务器中要查找的服务引用。
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("未找到我们的服务 UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - 找到我们的服务");

    // 获取服务中远程 BLE 服务器的特性引用。
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("未找到我们的特性 UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - 找到我们的特性");

    // 读取特性的值。
    if(pRemoteCharacteristic->canRead()) {
      String value = pRemoteCharacteristic->readValue();
      Serial.print("特性值为: ");
      Serial.println(value);
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}

/**
 * 扫描 BLE 服务器，并找到第一个广告其服务的设备。
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * 针对每个广告的 BLE 服务器调用此方法。
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("找到 BLE 广告设备: ");
    Serial.println(advertisedDevice.toString().c_str());

    // 我们找到了设备，看看它是否包含我们需要的服务。
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // 找到我们的服务器
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setup() {
  Serial.begin(115200);
  Serial.println("启动 Arduino BLE 客户端应用...");
  BLEDevice::init("");

  // 获取扫描器并设置回调函数，当我们发现新设备时会通知我们。
  // 设置为主动扫描并开始扫描，运行 5 秒。
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // setup 结束。

// 这是 Arduino 的主循环函数。
void loop() {
  // 如果 "doConnect" 标志为 true，表示我们已经扫描到并找到了想连接的 BLE 服务器。
  // 现在连接到它，一旦连接成功，我们将设置 connected 标志为 true。
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("我们现在已经连接到 BLE 服务器。");
    } else {
      Serial.println("连接服务器失败；我们将不再执行其他操作。");
    }
    doConnect = false;
  }

  // 如果已连接到 BLE 服务器，则每次到达时更新特性，输出从启动以来的当前时间。
  if (connected) {
    String newValue = "自启动以来的时间: " + String(millis()/1000);
    Serial.println("将新特性值设置为 \"" + newValue + "\"");
    
    // 将特性的值设置为实际是字符串的字节数组。
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  } else if (doScan) {
    BLEDevice::getScan()->start(0);  // 这是在断开连接后重新开始扫描的示例，实际上可能有更好的方法在 Arduino 中实现。
  }
  
  delay(1000); // 在每次循环之间延迟 1 秒。
} // loop 结束
```

上面的程序将使XIAO作为一个客户端，搜索附近的蓝牙设备。当蓝牙设备的UUID与您提供的UUID匹配时，它将连接到该设备并获取其特征值。


<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/60.png" style={{width:800, height:'auto'}}/></div>


#### 程序注释

让我们快速了解一下BLE服务器示例代码的工作原理。首先，它通过导入必要的BLE库来启用蓝牙功能。然后，您需要为服务和特征定义UUID。

```c
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
```

您可以保留默认的UUID，也可以访问 [uuidgenerator.net](https://www.uuidgenerator.net/) 为您的服务和特征生成随机UUID。 

接下来，您创建一个名为“XIAO_ESP32C6”的BLE设备。您可以根据需要更改此名称。在接下来的行中，您将BLE设备设置为服务器。然后，您创建一个使用之前定义的UUID的BLE服务器服务。

```c
BLEServer *pServer = BLEDevice::createServer();
BLEService *pService = pServer->createService(SERVICE_UUID);
```

然后，您为该服务设置特征。正如您所看到的，您还使用了之前定义的UUID，并且需要将特征的属性作为参数传递。在此示例中，属性为：READ 和 WRITE

```c
BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                     CHARACTERISTIC_UUID,
                                     BLECharacteristic::PROPERTY_READ |
                                     BLECharacteristic::PROPERTY_WRITE
                                     );
```

创建特征后，您可以使用`setValue()`方法设置其值。在此示例中，我们将值设置为文本“Hello World”。您可以将该文本更改为任何您喜欢的内容。在未来的项目中，这个文本可以是传感器读数，或者灯泡的状态，例如。

最后，您可以启动服务并开始广播，以便其他BLE设备能够扫描并找到该BLE设备。

```c
BLEAdvertising *pAdvertising = pServer->getAdvertising();
pAdvertising->start();
```

这只是一个创建BLE服务器的简单示例。在这段代码中，`loop()`中没有做任何事情，但您可以添加当新客户端连接时的处理逻辑（可以参考`BLE_notify`示例进行指导）。

### BLE传感器数据交换

接下来，我们将进入实际的案例。在这个案例中，我们将让XIAO ESP32C6连接到一个光强传感器，然后通过蓝牙将光传感器的值发送到另一个XIAO ESP32C6，并在扩展板的屏幕上显示该值。

为了方便布线，我们将使用两个XIAO扩展板，以下示例程序仅供参考，您可以根据实际项目需求选择产品。

<table align="center">
	<tr>
	    <th>Seeed Studio XIAO ESP32C6</th>
      <th>带有 Grove OLED 的 XIAO 的 Seeed Studio 扩展底座</th>
      <th>Grove - 数字光传感器 - TSL2561</th>
	</tr>
	<tr>
	    <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:200, height:'auto'}}/></div></td>
      <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Digital_Light_Sensor/img/hardware%20overview.jpg" style={{width:180, height:'auto'}}/></div></td>
	</tr>
    <tr>
	    <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
              <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
          </a>
      </div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
    		<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Digital-Light-Sensor-TSL2561.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
    		</a>
		</div></td>
	</tr>
</table>

除了上述硬件准备，您可能需要准备以下库，并将其下载并添加到Arduino IDE环境中。

- **OLED显示器库u8g2**:

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/olikraus/U8g2_Arduino">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

<br></br>

- **Grove - 数字光传感器 - TSL2561库：**:

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Seeed-Studio/Grove_Digital_Light_Sensor">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载库</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

<br></br>

我们需要准备两块XIAO，其中一块作为服务器，另一块作为客户端。以下是服务器端的示例程序。XIAO作为服务器有以下主要任务：
- 首先，获取光传感器的实时值；
- 其次，创建蓝牙服务器；
- 第三，通过蓝牙广播光强值；
- 最后，显示实时光强值及发送情况。

```c
//服务器端
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

// OLED显示屏设置，无复位功能
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

//BLE服务器名称（另一个运行服务器代码的ESP32设备名称）
#define bleServerName "XIAOESP32C6_BLE"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

int light_val = 0;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  
  //OLED显示器初始化
  u8x8.begin();
  u8x8.setFlipMode(1);

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 3, "Starting...");

  Wire.begin();
  TSL2561.init();
  
  BLEDevice::init(bleServerName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181A)); // 环境感应
  pCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2A59), // 模拟输出
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x1F);
  BLEDevice::startAdvertising();
  u8x8.clearDisplay();
}

void loop() {
  if (deviceConnected) {
    light_val = TSL2561.readVisibleLux();
    pCharacteristic->setValue(light_val);
    pCharacteristic->notify();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("Light Value:");
    u8x8.clearLine(2);
    u8x8.setCursor(0, 2);
    u8x8.print(light_val);
    u8x8.drawString(0, 4, "Sending...");
    delay(10); // 蓝牙堆栈会因为发送太多数据包而出现拥堵
  }
}
```

上传程序到一块 XIAO 开发板后，程序运行正常时，您可以拿出手机，使用 nRF Connect 应用搜索蓝牙设备 **XIAOESP32C6_BLE**，连接该设备并点击下方的按钮，您将收到来自传感器的数据。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/63.jpg" style={{width:300, height:'auto'}}/></div>

在此您会发现，与之前的示例相比，软件操作方式有所不同，因为通常我们发送的消息是传感器类型的，我们选择使用 **notify** 属性来确保消息传递的高效性。

接下来，我们需要取出另一块 XIAO 开发板，它作为客户端来收集并显示数据。

```c
//客户端
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

// OLED显示屏设置，无复位功能
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

BLEClient*  pClient;
bool doconnect = false;

//BLE服务器名称（另一个运行服务器代码的ESP32设备名称）
#define bleServerName "XIAOESP32C6_BLE"

// 外设设备地址。地址将在扫描过程中找到...
static BLEAddress *pServerAddress;

BLEUUID serviceUUID("181A"); // 环境感应
BLEUUID charUUID("2A59");    // 模拟输出

char light_val[1024];

// 扫描到另一个设备时调用的回调函数
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { // 检查广告设备的名称是否匹配
      advertisedDevice.getScan()->stop(); // 找到目标设备后停止扫描
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); // 获取广告设备的地址
      Serial.println("Device found. Connecting!");
    }
  }
};

// 打印最新传感器读数到OLED显示屏的函数
void printReadings(){
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  u8x8.print("Light Value:");
  u8x8.drawString(0, 2, light_val);
}

void setup() {
  Serial.begin(115200);
  //OLED显示器初始化
  u8x8.begin();
  u8x8.setFlipMode(1);

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 3, "Starting...");
  
  Serial.println("Starting BLE client...");

  BLEDevice::init("XIAOESP32C6_Client");

  // 获取扫描器并设置回调函数，以便在检测到新设备时通知
  // 设置为主动扫描，并启动扫描，持续时间为30秒。
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);

  pClient = BLEDevice::createClient();
  // 连接到远程BLE服务器
  pClient->connect(*pServerAddress);
  Serial.println(" - Connected to server");

  // 获取远程BLE服务器中的服务
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    u8x8.clearDisplay();
    u8x8.drawString(0, 3, "False UUID");
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return;
  }

  // 获取远程BLE服务器服务中的特征
  BLERemoteCharacteristic* pCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pCharacteristic == nullptr) {
    u8x8.clearDisplay();
    u8x8.drawString(0, 3, "False UUID");
    Serial.print("Failed to find our characteristic UUID");
    return;
  }
  Serial.println(" - Found light value characteristics");
  u8x8.clearDisplay();
  u8x8.drawString(0, 3, "Connected!");
  
  // 注册通知
  pCharacteristic->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Notify received");
    Serial.print("Value: ");
    Serial.println(*pData);
    snprintf(light_val, sizeof(light_val), "%d", *pData);
  });

  doconnect = true;
  u8x8.clearDisplay();
  u8x8.drawString(0, 4, "Receiving...");
}

void loop() {
  if (doconnect) {
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    BLERemoteCharacteristic* pCharacteristic = pRemoteService->getCharacteristic(charUUID);
    pCharacteristic->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
      Serial.println("Notify received");
      Serial.print("Value: ");
      Serial.println(*pData);
      snprintf(light_val, sizeof(light_val), "%d", *pData);
    });
  }
  printReadings();
  delay(1000);
  u8x8.clearLine(2);
}
```

在使用上述示例时，我们建议先上传 **Server** 程序，并确保其成功运行后，再使用 **Client** 程序。如果程序运行正常，你将看到如下效果：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/img/65.gif" style={{width:700, height:'auto'}}/></div>

#### 程序注解

对于上述程序，我们将挑选一些关键部分进行解释。我们先从 Server 程序开始。

程序开头，我们定义了蓝牙服务器的名称，这个名称可以是你设置的名称，但你需要记住这个名称，因为你需要依赖它来搜索这个蓝牙设备。

```c
#define bleServerName "XIAOESP32C6_BLE"
```

在前面的教程中，我们已经提到过，在服务器下会有 Characteristic，而在 Characteristic 下会有值和其他内容。所以，在我们创建广告时需要遵循这个规则。

```c
BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181A)); // 环境传感
  pCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2A59), // 模拟输出
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
```

I在上述程序中，你可以看到使用 `createService()` 来创建一个服务。参数是一个特定的 UUID：**0x181A**。在 GATT 协议中，**0x181A** 表示环境监测类型的数据，而同一个 `Characteristic` 的 UUID：**0x2A59** 也有特定的含义。在 GATT 协议中，它表示模拟输出。这正好符合我们光传感器值的情况，因此我们这样定义它。你可以阅读 [这里](https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/GATT.pdf) ，了解一些 GATT 协议为我们准备的具体 UUID 含义。

当然，你也可以设置不遵循 GATT 标准的 UUID，只需要确保这两个值是唯一的，并且不会影响客户端通过识别这些 UUID 来找到值。你可以去 [uuidgenerator.net](https://www.uuidgenerator.net/) 生成随机的 UUID 用于你的服务和特征。

`createCharacteristic()` 函数的第二个参数是设置特性属性。请注意，这里需要设置为 **PROPERTY_NOTIFY** ，以确保数据可以持续发送。

```c
pCharacteristic->setValue(light_val);
pCharacteristic->notify();
```

最后，在 `loop` 中，我们每隔 10 毫秒就将读取的光传感器值进行广告发布。

接下来是 Client 程序，看起来会复杂一些。

程序开头的内容还是非常熟悉的，你需要确保这部分内容与你在服务器端配置的内容一致。

```c
//BLE 服务器名称（运行服务器 sketch 的另一个 ESP32 名称）
#define bleServerName "XIAOESP32C6_BLE"

BLEUUID serviceUUID("181A"); // 环境传感
BLEUUID charUUID("2A59");    // 模拟输出
```

接下来我们编写一个回调函数，主要功能是搜索附近的蓝牙设备，并将其与你提供的蓝牙设备名称进行对比，捕获它并获取其 MAC 地址。

```c
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { // 检查广告设备的名称是否匹配
      advertisedDevice.getScan()->stop(); // 找到目标设备后停止扫描
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); // 获取目标设备的地址
      Serial.println("Device found. Connecting!");
    }
  }
};
```

接下来的程序是关键，它用于在服务器中找到光强度值。首先，我们需要找到服务器的 UUID，然后在该服务下查找特性的 UUID，最后获取光值。这个解析方法与蓝牙数据结构是一一对应的。

```c
// 获取远程 BLE 服务器中的服务引用
BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
if (pRemoteService == nullptr) {
  Serial.print("Failed to find our service UUID: ");
  Serial.println(serviceUUID.toString().c_str());
  return;
}

// 获取远程 BLE 服务器中服务的特征引用
BLERemoteCharacteristic* pCharacteristic = pRemoteService->getCharacteristic(charUUID);
if (pCharacteristic == nullptr) {
  Serial.print("Failed to find our characteristic UUID");
  return;
}
Serial.println(" - Found light value characteristics");

pCharacteristic->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Notify received");
    Serial.print("Value: ");
    Serial.println(*pData);
  });
```

最后，光值被存放在指针 pData 中。

:::tip
上述示例提供了一个单传感器单值的最简单示例。如果你想通过蓝牙发布多个传感器或多个传感器值，建议你阅读这里的教程示例。

- [ESP32 BLE 服务器与客户端 (低能耗蓝牙)](https://randomnerdtutorials.com/esp32-ble-server-client/)
:::



## 技术支持与产品讨论

感谢你选择我们的产品！我们在这里为你提供各种支持，确保你在使用我们产品时能获得顺利的体验。我们提供了多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>


