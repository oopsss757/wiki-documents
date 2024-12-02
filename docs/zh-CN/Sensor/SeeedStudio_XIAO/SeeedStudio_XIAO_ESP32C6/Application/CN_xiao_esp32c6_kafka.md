---
description: 使用 XIAO ESP32C6 和 sensors 收集数据并发送到 Apache Kafka
title: 由 Apache Kafka 提供支持的实时 IoT 数据处理节点
keywords:
- xiao esp32c6
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao_esp32c6_kafka
last_update:
  date: 11/18/2024
  author: Agnes
---

<div class="table-center">
<iframe width="730" height="500" src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/kafka_xiao.mp4?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

我们的尖端处理节点 Kafka-ESP32，结合了 Apache Kafka 和 ESP32C6 微控制器的强大功能，为处理 IoT 数据流提供了一种高效的解决方案。通过使用 XIAO ESP32C6 与 DHT20 环境传感器，数据被收集并通过 ESP32C6 无缝发送到 Apache Kafka。Kafka 的高吞吐量、低延迟消息传递能力使得实时数据处理和分析成为可能，同时其分布式架构使得扩展变得轻松。Kafka-ESP32 使您能够开发定制应用和集成，彻底改变了您在数据驱动的环境中管理和利用 IoT 资产的方式。

## 所需材料

本示例将介绍如何使用 XIAO ESP32C6 和 Grove DHT20 温湿度传感器来完成 AWS IoT Core 的 SageMaker 任务。以下是完成此例程所需的所有硬件设备。

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

## Docker 安装

为什么使用 Docker？因为 Docker 可以在单台机器上模拟多个计算机的环境，并轻松部署应用程序。因此，在本项目中，我们将使用 Docker 来设置环境并提高效率。

### 步骤 1. 下载 Docker

根据您的计算机类型下载不同的安装程序。点击 [这里](https://www.docker.com/products/docker-desktop/) 进行跳转。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/1.png" style={{width:1000, height:'auto'}}/></div>

:::tip
如果您的计算机是 **Windows**，请在完成 **步骤 2** 后再安装 Docker。
:::

### 步骤 2. 安装 WSL（Windows 子系统 Linux）

:::tip
此步骤适用于 **Windows**。如果您的计算机是 Mac 或 Linux，可以跳过此步骤。
:::

1. 以管理员身份运行以下代码。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/3.png" style={{width:1000, height:'auto'}}/></div>

```bash
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

2. 从 [这里](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi) 下载此工具并双击进行安装。

3. 打开 **Microsoft Store**，搜索并下载您喜欢的 Linux 版本，这里我安装了 Ubuntu。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/4.png" style={{width:1000, height:'auto'}}/></div>

4. 安装 Linux 后，您需要打开它并设置用户名和密码，然后稍等片刻等待初始化完成。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/5.png" style={{width:1000, height:'auto'}}/></div>

5. 运行以下命令以使用 **WSL**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/6.png" style={{width:1000, height:'auto'}}/></div>

6. 安装 WSL 后，现在您可以双击 Docker 安装程序进行安装。当看到以下图像时，表示安装成功。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/2.png" style={{width:1000, height:'auto'}}/></div>

## 部署服务

在我们开始之前，我想介绍一下本项目中每个服务的功能。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/structure.png" style={{width:700, height:'auto'}}/></div>

这是本项目的目录结构，供您参考。在接下来的步骤中，我将逐一创建这些文件。每个文件的位置非常重要。强烈建议您参考此目录结构。创建一个 **kafka_xiao_project** 目录，并包含这些文件。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/30.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 3. 部署 Python 服务器

由于 MCU 设备的性能不足，无法直接作为 Kafka 客户端使用。因此，您需要构建一个服务器来进行数据传输。此步骤是用 Python 构建一个简单的服务器。XIAO ESP32C6 主要用于收集 DHT20 的环境数据并将其发送到服务器。

1. 首先，我们需要创建 **app.py** 文件，这是服务器的功能实现。

```python
from flask import Flask
from kafka import KafkaProducer, KafkaConsumer

app = Flask(__name__)

@app.route('/favicon.ico')
def favicon():
    return '', 204

@app.route('/<temperature>/<humidity>')
def send_data(temperature, humidity):
    producer = KafkaProducer(bootstrap_servers='kafka:9092')
    data = f'Temperature: {temperature}, Humidity: {humidity}'
    producer.send('my_topic', data.encode('utf-8'))
    return f'Temperature: {temperature}, Humidity: {humidity}'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
```

2. 创建 **requirements.txt**，该文件列出了依赖库。

```
flask
kafka-python
```

3. 创建 **Dockerfile**文件

```
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["python", "app.py"]
```

4. 创建完这三个文件后，我们可以通过运行以下代码来构建 Docker 镜像。

```
docker build -t pyserver .
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/9.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 4. 部署 Jupyter Notebook

Jupyter Notebook 主要用于调试，它是一个非常好用的工具。我们还可以使用 Python 操作 Kafka。

1. 首先创建 **Dockerfile** 文件。

```
FROM python:3.9

RUN pip install jupyter

WORKDIR /notebook

EXPOSE 8888

CMD ["jupyter", "notebook", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root"]
```

2. 构建 Jupyter 的 Docker 镜像。
```
docker build -t jupyter .
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/8.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 5. 启动 Docker 集群

我们可以使用 **docker-compose.yml** 来构建 Docker 集群。docker-compose 中的每个服务代表一个独立的计算机，我们使用 *kafka-net* 网络将它们连接起来。

1. 首先，我们需要创建 **docker-compose.yml** 文件。

```
services:
  zookeeper:
    container_name: zookeeper
    hostname: zookeeper
    image: docker.io/bitnami/zookeeper
    ports:
      - "2181:2181"
    environment:
      - ALLOW_ANONYMOUS_LOGIN=yes
    networks:
      - kafka-net

  kafka:
    container_name: kafka
    hostname: kafka
    image: docker.io/bitnami/kafka
    ports:
      - "9092:9092"
      - "9093:9093"
    environment:
      - KAFKA_CFG_ZOOKEEPER_CONNECT=zookeeper:2181
      - KAFKA_CFG_BROKER_ID=0
      - ALLOW_PLAINTEXT_LISTENER=yes
      - KAFKA_CFG_LISTENER_SECURITY_PROTOCOL_MAP=INTERNAL:PLAINTEXT,EXTERNAL:PLAINTEXT
      - KAFKA_CFG_LISTENERS=INTERNAL://kafka:9092,EXTERNAL://localhost:9093
      - KAFKA_CFG_ADVERTISED_LISTENERS=INTERNAL://kafka:9092,EXTERNAL://localhost:9093
      - KAFKA_CFG_INTER_BROKER_LISTENER_NAME=INTERNAL
    depends_on:
      - zookeeper
    networks:
      - kafka-net
      
  jupyter:
    image: jupyter:latest
    depends_on:
      - kafka
    volumes:
      - ./myjupyter:/notebook
    ports:
      - "8888:8888"
    environment:
      - JUPYTER_ENABLE_LAB=yes
    networks:
      - kafka-net
      
  pyserver:
    image: pyserver:latest
    depends_on:
      - kafka
    volumes:
      - ./myserver/app.py:/app/app.py
    ports:
      - "5001:5001"
    networks:
      - kafka-net

networks:
  kafka-net:
    driver: bridge
```

2. 然后，运行以下命令启动 Docker 集群。

```
docker-compose up -d
```

:::tip
可能会出现端口被占用的情况，您可以将端口从 5001 更改为 5002 等，或者关闭占用端口的应用程序。
:::

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/10.png" style={{width:1000, height:'auto'}}/></div>

3. 在接下来的操作中，我们将测试它是否正常工作。首先，打开 **docker desktop** 软件并点击进入 **pyserver**。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/11.png" style={{width:1000, height:'auto'}}/></div>

4. 现在，服务器正在 http://127.0.0.1:5001 上运行。点击此链接打开它。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/12.png" style={{width:800, height:'auto'}}/></div>

5. 然后，按照如下格式输入两个参数，测试 Docker 集群是否正常工作。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/13.png" style={{width:700, height:'auto'}}/></div>

6. 进入 Kafka 查看数据是否已发送到 Kafka。
```
docker exec -it kafka bash

cd opt/bitnami/kafka/bin/

kafka-console-consumer.sh --bootstrap-server localhost:9093 --topic my_topic --from-beginning
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/14.png" style={{width:1000, height:'auto'}}/></div>

7. 您可以尝试使用不同的参数进行测试，您会看到数据立即发送到 Kafka。恭喜！您的 Docker 集群正在完美运行。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/15.png" style={{width:1000, height:'auto'}}/></div>

### 步骤 7. 使用 Python 测试 Kafka

:::tip
此步骤主要介绍如何使用 Python 操作 Kafka。您也可以跳过此步骤，这不会影响整个项目的操作。
:::

1. 打开 Docker Desktop 并点击进入 jupyter。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/16.png" style={{width:1000, height:'auto'}}/></div>

2. 点击此链接访问 jupyter。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/17.png" style={{width:1000, height:'auto'}}/></div>

3. 成功访问 Jupyter 后，您将看到此页面。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/18.png" style={{width:1000, height:'auto'}}/></div>

4. 右键点击并创建 **New Notebook**，使用 Python3 (ipykernel)。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/19.png" style={{width:800, height:'auto'}}/></div>

5. 通过运行 ```pip install kafka-python``` 安装 kafka-python 库。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/21.png" style={{width:1000, height:'auto'}}/></div>

6. 安装该库后，您需要重启 Jupyter。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/22.png" style={{width:1000, height:'auto'}}/></div>

7. 现在运行以下代码，通过 Python 向 Kafka 发送一些数据。

```python
from kafka import KafkaProducer, KafkaConsumer

# 初始化生产者
producer = KafkaProducer(bootstrap_servers='localhost:9093')
# 发送消息
producer.send('my_topic', b'Hello, Kafka2')
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/23.png" style={{width:1000, height:'auto'}}/></div>

8. 您也可以在 Kafka 中查看这些数据。

```python
from kafka import KafkaConsumer

# 初始化消费者
consumer = KafkaConsumer(
    'my_topic',
    bootstrap_servers='localhost:9093',
    auto_offset_reset='earliest',
    enable_auto_commit=True,
    group_id='group1'
)

# 接收数据并打印
for message in consumer:
    print(f"Received message: {message.value.decode('utf-8')}")
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/25.png" style={{width:1000, height:'auto'}}/></div>

## XIAO ESP32C6 和 Apache Kafka

[Kafka](https://kafka.apache.org/) 是一个分布式流处理平台，可以进行大规模的数据流实时处理。它支持系统之间的数据发布-订阅消息传递，提供容错性、持久性和高吞吐量。Kafka 被广泛应用于构建实时数据管道和流处理应用，适用于多个领域。

现在，我们将使用 XIAO ESP32C6 和 DHT20 温湿度传感器来收集数据，并将数据实时发送到 Kafka。

### 步骤 8. 收集数据并发送到 Apache Kafka

1. 将以下代码复制到您的 Arduino IDE 中。

```cpp
#include <WiFi.h>
#include <HTTPClient.h>

// 请在此处更改为您的 WiFi 名称和密码。
const char* ssid = "Maker_2.4G";
const char* password = "15935700";

// 请在此处更改为您的计算机 IP 地址和服务器端口。
const char* serverUrl = "http://192.168.1.175:5001";

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("正在连接 WiFi...");
  }
  
  Serial.println("已连接到 WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // 创建访问链接
    String url = serverUrl;
    url += "/";
    url += "30.532";  // 温度
    url += "/";
    url += "60.342";  // 湿度
    
    http.begin(url);
    
    int httpResponseCode = http.GET();
    
    // 获取 HTTP 响应并打印
    if (httpResponseCode == 200) {
      String response = http.getString();
      Serial.println("服务器响应:");
      Serial.println(response);
    } else {
      Serial.print("HTTP 错误代码: ");
      Serial.println(httpResponseCode);
    }
    
    http.end();
  } else {
    Serial.println("WiFi 断开连接");
  }
  
  delay(5000);  // 每 5 秒访问一次服务器。
}
```

如果您不知道计算机的 IP 地址，可以运行 ```ipconfig```（Windows）或 ```ifconfig | grep net```（Mac 或 Linux）来查看。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/26.png" style={{width:600, height:'auto'}}/></div>

2. 使用 Type-C 电缆将计算机连接到 C6，并使用 Grove 电缆将 XIAO 扩展板的 **I2C 端口** 连接到 DHT20 传感器。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/hardware.jpeg" style={{width:600, height:'auto'}}/></div>

3. 选择您的开发板。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/27.png" style={{width:1000, height:'auto'}}/></div>

4. 上传代码并打开串口监视器。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/28.png" style={{width:1000, height:'auto'}}/></div>

5. 打开您运行 Kafka 的 Windows PowerShell。现在，您将看到环境数据正在发送到 Kafka。恭喜！您成功运行了这个项目！

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiao_esp32c6_kafka/29.png" style={{width:1000, height:'auto'}}/></div>

## 资源

- **[链接]** [Apache Kafka 介绍](https://kafka.apache.org/)

## 技术支持与产品讨论

感谢您选择我们的产品！我们将为您提供多种支持方式，确保您使用我们的产品体验顺畅。我们提供了多个沟通渠道，以满足不同的偏好和需求。

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