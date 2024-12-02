---
description: 将 XIAO ESP32C6 和传感器与 AWS IoT Core 结合使用，以实现可以执行异常检测的 AI 设备。
title: 用AWS IoT Core为XIAO ESP32C6赋能AI
keywords:
- xiao esp32c6
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32c6_aws_iot
last_update:
  date: 11/18/2024
  author: Agnes
---

# 用AWS IoT Core为XIAO ESP32C6赋能AI

本Wiki作为一个综合指南，介绍了如何部署一个先进的物联网系统，利用AWS服务和XIAO ESP32C6微控制器来监控和分析环境数据。从无缝收集传感器数据开始，本文档详细介绍了如何将这些信息分别传输并存储到AWS IoT Core和AWS Analytics中。它深入探讨了利用AWS Sagemaker在正常环境模式下训练机器学习模型，强调系统能够学习并适应其操作环境，从而提高效率。

此外，Wiki还概述了如何使用XIAO ESP32C6实现实时异常检测，这是一个关键组件，可以主动扫描与常规模式的偏差，并迅速触发警报。它总结了从设置警报机制到通知利益相关者异常条件的全过程，确保及时的关注和行动。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/0.jpg" style={{width:1000, height:'auto'}}/></div>

- [**将传感器数据上传到AWS IoT Core**](#capture-sensor-data-to-aws-iot-core)
- [**使用AWS Analytics存储数据**](#store-the-data-using-aws-analytics)
- [**使用AWS Sagemaker训练正常环境下的数据**](#use-aws-sagemaker-to-train-data-in-normal-environments)
- [**XIAO ESP32C6用于异常环境检测**](#xiao-esp32c6-for-abnormal-environment-detection)
- [**异常状态消息通知**](#abnormal-status-message-notification)

通过阅读本Wiki，用户将详细了解每个组件在创建智能、响应迅速且稳健的环境监控系统中的作用，并获得有关配置和维护的实用见解。

## 所需材料

本示例将介绍如何使用XIAO ESP32C6与Grove DHT20温湿度传感器，完成AWS IoT Core的SageMaker任务。以下是完成此任务所需的所有硬件设备。

<div class="table-center">
	<table align="center">
		<tr>
			<th>XIAO ESP32C6</th>
			<th>DHT20</th>
			<th>扩展板</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/Grove-Temperature-Humidity-Sensor/Tem-humidity-sensor1.jpg" style={{width:250, height:'auto'}}/></div></td><td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/extensionboard.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-V2-0-DHT20-p-4967.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
            <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

## 将传感器数据上传到AWS IoT Core

我们通过XIAO ESP32C6开发板连接多个传感器，实时收集环境数据并上传至AWS IoT Core。这为我们提供了一种可靠且安全的方式来处理来自各种传感器的大量数据流。

要注册AWS IoT Core并创建一个名为“XIAO_ESP32C6”的设备，按照以下步骤操作。请注意，此过程假设您已经拥有Amazon Web Services帐户。如果没有，您需要先[创建一个](https://aws.amazon.com/)。

### 步骤1. 创建一个设备

打开您的网页浏览器并访问[AWS管理控制台](https://aws.amazon.com/console/)。使用您的AWS帐户凭证登录。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/1.png" style={{width:1000, height:'auto'}}/></div>

进入AWS管理控制台后，找到页面顶部的**服务**下拉菜单。在**服务**菜单中，点击**IoT Core**。如果找不到它，可以使用顶部的搜索框搜索**IoT Core**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/2.png" style={{width:1000, height:'auto'}}/></div>

在AWS IoT Core仪表盘中，点击左侧导航面板中的**所有设备**以展开选项。点击**Things**，然后点击“Things”页面右上角的**创建设备**按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/3.png" style={{width:1000, height:'auto'}}/></div>

选择**创建单个设备**来继续注册一个设备。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/5.png" style={{width:800, height:'auto'}}/></div>

在**创建设备**页面，输入**XIAO_ESP32C6**作为设备名称。（可选）您还可以根据需要为设备添加类型、分组或属性。对于简单的设置，可以跳过这些选项。点击“下一步”。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/6.png" style={{width:900, height:'auto'}}/></div>

现在您将进入**配置设备证书**页面。AWS IoT Core要求设备使用证书进行安全通信。请选择**自动生成新证书（推荐）**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/7.png" style={{width:1000, height:'auto'}}/></div>

在**将策略附加到证书**页面，如果您没有策略，需要点击**创建策略**来创建一个。您将被带到一个新页面，在该页面上创建一个定义设备权限的策略。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/8.png" style={{width:1000, height:'auto'}}/></div>

创建并命名策略后，选中策略名称旁边的复选框，将其附加到您新创建的证书上，然后点击**创建**。

我们需要以下权限：
- **iot:Publish**
- **iot:Connect**
- **iot:Receive**
- **iot:Subscribe**

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/9.png" style={{width:1000, height:'auto'}}/></div>

设备注册完成后，您将被重定向到设备详细信息页面，在此页面中查看设备信息。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/10.png" style={{width:1000, height:'auto'}}/></div>

配置您的设备（在本例中为XIAO_ESP32C6），使用您在创建设备时下载的证书和私钥。您需要按照特定设备的说明设置AWS IoT SDK，并建立与AWS IoT Core的安全连接。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/11.png" style={{width:600, height:'auto'}}/></div>

完成设备设置并连接到AWS IoT Core后，您可以通过订阅主题、发布消息并使用AWS IoT Core规则引擎来处理物联网数据与之交互。

请记住，保密您的证书和密钥，并遵循安全最佳实践。AWS IoT Core文档提供了详细的指南和教程，帮助您设置和管理物联网设备。

### 步骤2. 基于证书准备头文件

创建一个新的头文件，命名为**secrets.h**，并将以下代码模板粘贴到头文件中。

```cpp
#include <pgmspace.h>

#define SECRET
#define THINGNAME "DHTsensor"

const char WIFI_SSID[] = "YOUR_SSID";              //更改为您的SSID
const char WIFI_PASSWORD[] = "YOUR_PASSWORD";           //更改为您的密码
const char AWS_IOT_ENDPOINT[] = "YOUR_AWS_IOT_ENDPOINT";       //更改为您的AWS IoT端点

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

// 设备证书                                               //更改为您的设备证书
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----


)KEY";

// 设备私钥                                               //更改为您的设备私钥
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----


)KEY";
```

此C++代码模板设计用于一个物联网设备，该设备连接到Wi-Fi网络并与AWS IoT服务通信。模板中包含了多个字符串占位符，您需要用实际的凭证和证书来替换它们。以下是每个部分的填写方式：

1. **Wi-Fi凭证**：
   - `WIFI_SSID`：将 `"YOUR_SSID"` 替换为您的Wi-Fi网络的SSID（名称）。
   - `WIFI_PASSWORD`：将 `"YOUR_PASSWORD"` 替换为您的Wi-Fi网络的密码。

2. **AWS IoT端点**：
   - `AWS_IOT_ENDPOINT`：将 `"YOUR_AWS_IOT_ENDPOINT"` 替换为您唯一的AWS IoT端点。此端点与您的AWS IoT账户和区域相关。您可以在AWS IoT控制台的设置中找到它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/12.png" style={{width:1000, height:'auto'}}/></div>

3. **Amazon Root CA 1（证书授权机构）**：
   - `AWS_CERT_CA`：在 `-----BEGIN CERTIFICATE-----` 和 `-----END CERTIFICATE-----` 标记之间，粘贴由AWS提供的完整Amazon Root CA 1证书。该证书允许您的设备信任服务器的身份。

4. **设备证书**：
   - `AWS_CERT_CRT`：将 `-----BEGIN CERTIFICATE-----` 和 `-----END CERTIFICATE-----` 标记之间的占位符替换为您的设备的PEM格式证书。此证书是设备唯一的，用于验证设备与AWS IoT的身份。

5. **设备私钥**：
   - `AWS_CERT_PRIVATE`：在 `-----BEGIN RSA PRIVATE KEY-----` 和 `-----END RSA PRIVATE KEY-----` 标记之间，粘贴您设备的PEM格式私钥。此私钥必须保密，绝不应共享，因为它用于在与AWS IoT通信时证明设备的身份。

:::caution
**Amazon Root CA 1** 对应于 **RSA 2048位密钥：Amazon Root CA 1** 下载的文件信息。
**设备证书** 对应于下载的 **设备证书** 文件信息。
**设备私钥** 对应于下载的 **私钥文件** 信息。

由于此代码包含敏感信息，如Wi-Fi凭证和私钥，因此务必确保其安全。请勿公开分享已修改的代码或将其提交到公开的代码库。
:::

### 步骤3. 上传数据采集程序到XIAO ESP32C6

请将Grove DHT20传感器连接到XIAO ESP32C6的IIC接口。如果您希望更加便捷，我们建议您购买[Grove Base for XIAO](https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html)。

接下来，在Arduino中创建一个新的项目并保存在本地。将我们在**步骤2**中创建的**secrets.h**文件复制到与.ino文件相同的目录中。然后，请将以下代码上传到XIAO ESP32C6，数据将根据您提供的AWS凭证通过MQTT发送到指定的主题。

<details>
<summary>点击此处预览完整代码</summary>

```cpp
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "Wire.h"

// DHT传感器设置
#include "DHT.h"
#define DHTTYPE DHT20   // DHT 20
/* 注意：DHT10和DHT20与其他DHT*传感器不同，使用I2C接口，而不是单线接口*/
/* 因此不需要定义引脚。*/
DHT dht(DHTTYPE);         //   DHT10 DHT20无需定义引脚

// MQTT设置
#define AWS_IOT_PUBLISH_TOPIC   "xiao_esp32c6/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "xiao_esp32c6/sub"

// 存储温湿度数据
float h;
float t;

// 网络设置
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

#if defined(ARDUINO_ARCH_AVR)
    #define debug  Serial

#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
    #define debug  SerialUSB
#else
    #define debug  Serial
#endif

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("正在连接Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // 配置WiFiClientSecure以使用AWS IoT设备凭证
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // 连接到我们之前定义的AWS IoT端点上的MQTT代理
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // 创建消息处理函数
  client.setCallback(messageHandler);
 
  Serial.println("正在连接AWS IoT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT 超时！");
    return;
  }
 
  // 订阅一个主题
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("已连接到AWS IoT！");
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["humidity"] = h;
  doc["temperature"] = t;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // 打印到客户端
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
 
void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("接收到消息：");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void setup() {

    debug.begin(115200);
    debug.println("DHTxx 测试！");
    Wire.begin();

    connectAWS();
    dht.begin();
}

void loop() {
    h = dht.readHumidity();
    t = dht.readTemperature();

    if (isnan(h) || isnan(t))  // 检查读取是否失败，若失败则跳过并重新尝试
    {
      Serial.println(F("读取DHT传感器失败！"));
      return;
    }
  
    Serial.print(F("湿度: "));
    Serial.print(h);
    Serial.print(F("%  温度: "));
    Serial.print(t);
    Serial.println(F("°C"));
  
    publishMessage();
    client.loop();
    delay(1000);
}
```
</details>

### 步骤 4. MQTT 测试客户端

在 AWS IoT Core 中，我们需要在 MQTT 测试客户端订阅 XIAO ESP32C6 发布的主题，以便确定传感器的数据是否成功上传到目标 AWS 账户。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/13.png" style={{width:1000, height:'auto'}}/></div>

如果程序正常工作，您应该每隔 1 秒看到一组温度和湿度数据消息。

```json
{
    "humidity": 58,
    "temperature": 23.6
}
```

## 使用 AWS Analytics 存储数据

收集到的数据将转发到 AWS Analytics 服务，AWS Analytics 不仅存储原始数据，还为我们提供强大的数据处理和分析工具。这些工具帮助我们从收集到的数据中提取有价值的见解。

### 步骤 5. 配置 AWS IoT Analytics

在 AWS 控制台中导航到 AWS IoT Analytics 服务。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/14.png" style={{width:1000, height:'auto'}}/></div>

在 **Get started with AWS IoT Analytics** 中，填写新创建资源的名称，并输入您的订阅主题。（例如，`xiao_esp32c6/pub`）。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/15.png" style={{width:1000, height:'auto'}}/></div>

等待片刻（大约十分钟），直到所有资源创建完成。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/16.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 6. 创建规则

返回到 AWS IoT Core，在左侧菜单栏中的 **Message routing** 下点击 **Rules**，然后点击 **Create rule**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/17.png" style={{width:1000, height:'auto'}}/></div>

为您的规则提供一个名称，并可选地填写描述，帮助识别其用途。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/18.png" style={{width:1000, height:'auto'}}/></div>

使用 AWS IoT SQL 语法定义规则查询语句。此语句指定了过滤和处理传入 MQTT 消息的标准。您可以使用通配符、函数和运算符来匹配特定主题、从消息有效载荷中提取数据并应用转换。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/19.png" style={{width:1000, height:'auto'}}/></div>

选择您创建的 IoT Analytics 渠道作为规则操作的目标。点击 **Create Role** 按钮。在控制台中，提供角色的名称，例如 **XIAO_ESP32C6_Role**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/20.png" style={{width:1000, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/21.png" style={{width:1000, height:'auto'}}/></div>

审核您的规则配置，并点击 **Create Rule** 按钮以保存并激活规则。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/22.png" style={{width:1000, height:'auto'}}/></div>

一旦规则创建完成，它将开始处理匹配定义的规则查询语句的传入 MQTT 消息。每当规则触发时，规则操作将被执行，允许您根据特定要求路由和处理数据。

您可以在 AWS IoT 中创建多个规则，以处理不同的场景和数据处理需求。规则提供了一种灵活且可扩展的方式，将您的 IoT 设备与各种 AWS 服务集成，并构建强大的 IoT 应用。

### 步骤 7. 存储传感器数据流

导航到 AWS IoT Analytics 服务。在 AWS IoT Analytics 仪表盘中，点击左侧边栏中的 **Datasets** 选项。找到包含您想要下载的数据的数据集，点击其名称以打开数据集详情页面。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/25.png" style={{width:1000, height:'auto'}}/></div>

在下载数据集内容之前，您需要手动触发数据集的生成。选择 **Run now**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/26.png" style={{width:1000, height:'auto'}}/></div>

AWS IoT Analytics 将处理数据并根据指定的时间范围准备数据集内容。由于每秒报告一次传感器数据，我们建议在正常环境中至少收集一个小时的数据，这样可以确保数据的准确性。

等待数据集生成完成。您可以在数据集详情页面中监控进度。一旦状态变为 "SUCCEEDED"，数据集内容就可以下载了。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/28.png" style={{width:1000, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/23.png" style={{width:1000, height:'auto'}}/></div>

:::tip
如果 XIAO 的程序运行正常，但您在数据集中没有看到任何数据，您可以右键点击数据集标签，并在新浏览器页面中打开它，这可能会解决您的问题。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/27.png" style={{width:500, height:'auto'}}/></div>

在数据集详情页面中，您将看到有关数据集的信息，包括其名称、状态和最后更新时间。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/24.png" style={{width:1000, height:'auto'}}/></div>
:::

## 使用 AWS Sagemaker 在正常环境中训练数据

通过 AWS Sagemaker，我们训练机器学习模型，以识别代表正常环境的模式。Sagemaker 提供了一个全面的平台，便于机器学习模型的开发、训练和部署，从而实现对环境数据的智能处理。

### 步骤 8. 创建新的笔记本实例

在 AWS 管理控制台中导航到 Amazon SageMaker 服务。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/29.png" style={{width:1000, height:'auto'}}/></div>

在 SageMaker 仪表盘中点击 **Notebook instances**，然后点击 **Create notebook instance** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/30.png" style={{width:1000, height:'auto'}}/></div>

填写必要的信息，如实例类型和 IAM 角色。确保该 IAM 角色具有 **访问存储数据的 S3 桶的必要权限**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/34.png" style={{width:600, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/31.png" style={{width:1000, height:'auto'}}/></div>

一旦实例状态变为 **InService**，点击 **Open Jupyter** 链接打开 Jupyter 笔记本界面。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/32.png" style={{width:1000, height:'auto'}}/></div>

打开后，我们选择 **conda_python3** 作为我们的代码环境。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/45.png" style={{width:1000, height:'auto'}}/></div>

接下来，我们需要上传我们收集的数据集。该数据集是我们在 **步骤 7** 中下载到本地计算机的。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/46.png" style={{width:1000, height:'auto'}}/></div>

然后，我们可以在 Jupyter Notebook 中输入我们准备好的程序，或者直接上传我们提供的程序。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Seeed-Projects/XIAO_ESP32C6_AWS_DHT20_Project/blob/main/TrainingModel/Jupyter_Notebook.ipynb">
    <strong><span><font color={'FFFFFF'} size={"4"}>下载代码</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div><br />

运行程序的第一部分，检查您导入的数据集是否正常。您可能需要将代码中的文件名更改为您自己的文件名。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/47.png" style={{width:600, height:'auto'}}/></div>

### 步骤 9: 运行整个 Jupyter Notebook

S3 用于存储训练数据集、测试数据集、模型工件等。在 SageMaker 中，数据源通常来自 S3 桶。
模型保存：训练后的模型也保存在 S3 中，以供后续部署和推理。

然后，请复制第二个代码块，并将字段 **bucket_name** 命名。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/48.png" style={{width:600, height:'auto'}}/></div>

然后，只需依次在 Jupyter Notebook 中运行各个代码块。

:::note
在运行 Jupyter Notebook 之前，您需要修改以下路径或名称：

1. **In[22]** 代码块中的 **ENDPOINT_NAME** 是运行 **In[19]** 代码块后得到的结果。
2. 请将 **In[3]** 和 **In[10]** 代码块中的 **bucket_name** 设置为相同的名称。
3. 最后一个代码块中的 **API_ENDPOINT**，请使用您自己的值。
:::

### 步骤 10. 配置 AWS Lambda

Lambda 可以作为 SageMaker 工作流的触发器。例如，当数据上传到 S3 时，它可以触发一个 Lambda 函数以启动 SageMaker 训练或处理任务。

登录到 AWS 管理控制台，并导航到 AWS Lambda 服务。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/37.png" style={{width:1000, height:'auto'}}/></div>

点击 **Create function** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/38.png" style={{width:1000, height:'auto'}}/></div>

选择 **Author from scratch** 选项。为您的 Lambda 函数提供一个名称，例如 **XIAO-ESP32C6-FUNCTION**。
选择所需的运行时 **Python3.9**。为您的 Lambda 函数选择一个执行角色。您可以选择创建一个新角色或使用现有的角色。如果创建新角色，选择 **Create a new role with basic Lambda permissions**。点击 **Create function** 按钮以创建您的 Lambda 函数。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/39.png" style={{width:1000, height:'auto'}}/></div>

导航到 IAM（身份和访问管理）控制台。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/40.png" style={{width:1000, height:'auto'}}/></div>

然后找到我们刚创建的 Lambda 函数的名称，并点击它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/41.png" style={{width:1000, height:'auto'}}/></div>

在 IAM 角色页面中，点击 **Attach policies** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/42.png" style={{width:1000, height:'auto'}}/></div>

为策略提供一个名称，例如 **AmazonSageMakerFullAccess**。点击 **Add permissions** 按钮以添加权限。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/43.png" style={{width:1000, height:'auto'}}/></div>

返回到 Lambda 函数配置页面。创建一个新的测试事件或使用现有的测试事件。使用测试事件调用 Lambda 函数，确保它能够成功运行。监控 Lambda 函数的执行日志和输出，以验证其行为。

```
{"data": [62.93016434, 24.31583405]}
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/44.png" style={{width:1000, height:'auto'}}/></div>

将 [以下代码片段](https://github.com/Seeed-Projects/XIAO_ESP32C6_AWS_DHT20_Project/blob/main/TrainingModel/Lambda.ipynb) 复制到 **Code** 中。

```python
#lambda function with sns
import boto3
import json

ENDPOINT_NAME = 'randomcutforest-2024-03-18-10-47-37-165'# 这里填入您的端点
runtime = boto3.client('runtime.sagemaker')
email_client = boto3.client('sns')

def lambda_handler(event, context):
    input = event['data']
    
    serialized_input = ','.join(map(str, input))

    response = runtime.invoke_endpoint(EndpointName=ENDPOINT_NAME, 
                                       ContentType='text/csv', 
                                       Body=serialized_input)

    result_str = response['Body'].read().decode()
    result_json = json.loads(result_str)
    inference = result_json['scores'][0]['score']
    
    try:
        if(inference>3):
            response_sns = email_client.publish(
                TopicArn='arn:aws:sns:us-east-1:381491976713:dhco2Topic2',# 这里填入您的 SNS 主题
                Message='环境数据异常',
                Subject='环境状态'
            )
    except Exception as e:
        print(f"error: {e}")

    return inference
```

:::caution
请确保修改代码中的内容为您自己的。
:::

然后点击 **Deploy** 按钮。

### 其他注意事项

- 确保您为 SageMaker 配置了正确的 IAM 角色和策略，以便其访问 S3 中的数据。
- 考虑使用 SageMaker 的自动模型调优来找到最佳的模型版本。
- 监控费用，因为在 SageMaker 上训练模型和部署端点可能会根据所使用的计算资源产生 significant 费用。

请始终查阅最新的 AWS 文档，以获取详细的说明和最佳实践，因为 AWS 服务的界面和功能经常更新。

## 异常状态消息通知

在检测到异常状态时，系统会通过消息通知机制立即向维护人员发送警报，确保及时干预并采取必要的措施。

### 步骤 11. 配置 Amazon SNS

导航到 Amazon SNS 服务。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/49.png" style={{width:1000, height:'auto'}}/></div>

点击 **Create topic** 按钮。为您的主题提供一个名称，例如 "XIAO_ESP32C6_Topic"。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/50.png" style={{width:1000, height:'auto'}}/></div>

在 SNS 主题仪表板中，点击新创建的主题。点击 **Create subscription** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/51.png" style={{width:1000, height:'auto'}}/></div>

选择订阅的协议，例如 "Email"、"SMS"、"HTTP/HTTPS"、"AWS Lambda" 或 "Amazon SQS"。

根据选择的协议提供端点详情。例如：

- 对于电子邮件订阅，输入电子邮件地址。
- 对于 SMS 订阅，输入电话号码。
- 对于 HTTP/HTTPS 订阅，输入 URL 端点。
- 对于 AWS Lambda 订阅，选择 Lambda 函数。
- 对于 Amazon SQS 订阅，选择 SQS 队列。

点击 **Create subscription** 按钮以创建订阅。如果需要，可以重复步骤 2-5 来向主题添加更多订阅。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/52.png" style={{width:1000, height:'auto'}}/></div>

然后我们返回到 Lambda 代码中，将代码中的 **TopicArn** 字段替换为 **SNS 中的 ARN 字段**。

### 步骤 12. 授予 SNS 权限给 Lambda

导航到 IAM（身份和访问管理）控制台。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/40.png" style={{width:1000, height:'auto'}}/></div>

然后找到我们刚创建的 Lambda 函数的名称并点击它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/41.png" style={{width:1000, height:'auto'}}/></div>

在 IAM 角色页面中，点击 **Attach policies** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/42.png" style={{width:1000, height:'auto'}}/></div>

为策略提供一个名称，例如 **AmazonSNSFullAccess**。点击 **Add permissions** 按钮以添加权限。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/53.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 13. 配置 API Gateway

在 AWS 管理控制台中导航到 Amazon API Gateway 服务。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/54.png" style={{width:1000, height:'auto'}}/></div>

点击 **Create API** 按钮。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/55.png" style={{width:1000, height:'auto'}}/></div>

选择 **REST API** 作为 API 类型，然后点击 **Build**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/56.png" style={{width:1000, height:'auto'}}/></div>

为您的 API 提供一个名称，例如 "XIAO_ESP32C6_API"。选择 **Regional** 作为 API 的端点类型。点击 **Create API** 按钮来创建您的 REST API。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/57.png" style={{width:1000, height:'auto'}}/></div>

在 API Gateway 仪表板中，选择您刚创建的 API。点击 **Create Resource**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/58.png" style={{width:1000, height:'auto'}}/></div>

为您的资源提供一个名称，例如 "XIAO_ESP32C6_Resource"。点击 **Create Resource** 按钮来创建该资源。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/59.png" style={{width:1000, height:'auto'}}/></div>

在选择新创建的资源后，点击 **Create Method**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/60.png" style={{width:1000, height:'auto'}}/></div>

从下拉菜单中选择 **POST** 作为 HTTP 方法。选择 **Lambda Function** 作为集成类型。选择 Lambda 函数所在的区域。输入您的 Lambda 函数名称，例如 "XIAO_ESP32C6_Function"。点击 **Create method** 按钮以保存集成设置。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/61.png" style={{width:1000, height:'auto'}}/></div>

点击 **Deploy API**。选择一个部署阶段（例如 "prod"、"dev"）或创建一个新的部署阶段。如果需要，可以为部署提供描述。点击 "Deploy" 按钮来部署您的 API。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/63.png" style={{width:500, height:'auto'}}/></div>


在 API Gateway 仪表板中，选择您的 API，并导航到 "Stages" 部分。展开部署阶段，点击您的资源的 POST 方法。在 **Invoke URL** 部分，复制提供的 URL。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao-esp32c6-aws/64.png" style={{width:1000, height:'auto'}}/></div>

最后，复制并粘贴 [api_gateway 代码](https://github.com/Seeed-Projects/XIAO_ESP32C6_AWS_DHT20_Project/blob/main/TrainingModel/api_gateway.ipynb) 到 SageMaker Jupyter Notebook 的末尾（创建一个新的代码片段），并将代码中的 **API_ENDPOINT** 字段替换为 **Invoke URL**。

## XIAO ESP32C6 用于异常环境检测

一旦建立了正常环境的数据模型，XIAO ESP32C6 将持续监控传感器数据，以检测任何潜在的异常。作为一款强大的微控制器，它能够在数据表明出现异常情况时迅速响应。

### 步骤 14. 上传 XIAO ESP32C6 的实时数据报告程序

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Seeed-Projects/XIAO_ESP32C6_AWS_DHT20_Project/blob/main/GetResult/GetResult.ino">
    <strong><span><font color={'FFFFFF'} size={"4"}>下载代码</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div><br />

从以下地址获取程序并将代码中的 api 字段替换为您自己的。然后编译并上传到 XIAO ESP32C6。恭喜您，到此为止，您已经成功完成了整个项目的步骤。一旦环境出现异常，AWS SNS 服务会立即向您发送警告邮件通知。

## 技术支持与产品讨论

感谢您选择我们的产品！我们将在此为您提供不同的支持，确保您使用我们产品的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的偏好和需求。

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


