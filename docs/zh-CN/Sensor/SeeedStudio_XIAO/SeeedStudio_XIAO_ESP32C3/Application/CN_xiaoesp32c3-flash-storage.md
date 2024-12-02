---
description: XIAO ESP32C3闪存
title: 在 XIAO ESP32C3 上的 EEPROM 永久数据存储
keywords:
- XIAO ESP32C3
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/xiaoesp32c3-flash-storage
last_update:
  date: 11/12/2024
  author: Agnes
---


当我们使用开发板时，我们中的许多人都希望能够使用芯片上的闪存来存储一些重要的数据。这需要一种存储方法，确保即使在开发板异常的情况下也不会丢失数据。


本教程将介绍如何通过以下两种不同的存储方法将重要数据存储在XIAO ESP32C3的闪存上
1. 第一篇指南展示了如何使用`Preferences.h`库在ESP32闪存上**永久保存数据**。保存在闪存中的数据在重置或断电时仍然存在。使用`Preferences.h`库有助于保存网络凭据、API密钥、阈值甚至GPIO的最后状态等数据。你将学习如何从闪存中保存和读取数据。


2. 第二guid解释了什么是XIAO ESP32C3 **EEPROM**和它的用途。我们还将向您展示如何从EEPROM中读写，并构建一个项目示例来将学到的概念付诸实践。
本文的大部分内容来自[**RandomNerdTutorials.com**](https://randomnerdtutorials.com/)，其中一些程序和描述已经进行了轻微的修改，以适应XIAO ESP32C3。特别感谢[**RandomNerdTutorials.com**](https://randomnerdtutorials.com/)提供的教程和方法。这是原始源代码的直接链接。
- [ESP32闪存-存储永久数据(写入和读取)](https://randomnerdtutorials.com/esp32-flash-memory/)


- [Arduino EEPROM解释-记住上一次LED状态](https://randomnerdtutorials.com/arduino-eeprom-explained-remember-last-led-state/)


- [ESP32使用偏好库永久保存数据](https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/)
## 使用首选项库永久保存数据
### Preferences.h 库

当您在Arduino IDE中安装XIAO ESP32C3板时，此库将自动“安装”

最好使用`Preferences.h`库来通过键值对存储变量值。永久保存数据对以下方面很重要:


- 记住变量的最后一个状态;


- 保存设置;
 

- 保存设备被激活的次数;


- 或者你需要永久保存的任何其他数据类型。


如果您想使用XIAO ESP32C3来存储文件或非常长的字符串或数据，我们建议您使用扩展板和SD卡，我们不建议您使用本教程中的两种方法。


下面是**Preferences.h库中有用的函数**

**函数 1**.`begin()`方法用定义好的命名空间打开一个“存储空间”。false参数意味着我们将在读/写模式下使用它。使用true以只读模式打开或创建命名空间。
```c
preferences.begin("my-app", false);
```

在本例中，命名空间名称为my-app。命名空间名称限制为15个字符。



**函数 2**.使用`clear()`来清除打开的命名空间下的所有首选项(它不会删除命名空间):

```c
preferences.clear();
```

**函数 3**.从打开的命名空间中删除一个键:

```c
preferences.remove(key);
```

**函数 4**.使用 end() 方法关闭打开的命名空间下的首选项：

```c
preferences.end();
```

**函数 5**.根据要保存的变量类型，你应该使用不同的方法。

在使用`Preferences.h`库时，你应该定义要保存的数据类型。之后，如果你想读取这些数据，你必须知道保存的数据类型。换句话说，写入和读取的数据类型应该是相同的。

使用 `Preferences.h`你可以保存以下数据类型：char、Uchar、short、Ushort、int、Uint、long、Ulong、long64、Ulong64、float、double、bool、string 和 bytes。

<table align="center">
  <tbody><tr>
      <td align="center">Char</td>
      <td align="left"><code>putChar(const char*key, int8_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Unsigned Char</td>
      <td align="left"><code>putUChar(const char* key, int8_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Short</td>
      <td align="left"><code>putShort(const char*key, int16_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Unsigned Short</td>
      <td align="left"><code>putUShort(const char* key, uint16_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Int</td>
      <td align="left"><code>putInt(const char*key, int32_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Unsigned Int</td>
      <td align="left"><code>putUInt(const char* key, uint32_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Long</td>
      <td align="left"><code>putLong(const char*key, int32_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Unsigned Long</td>
      <td align="left"><code>putULong(const char* key, uint32_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Long64</td>
      <td align="left"><code>putLong64(const char*key, int64_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Unsigned Long64</td>
      <td align="left"><code>putULong64(const char* key, uint64_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Float</td>
      <td align="left"><code>putFloat(const char*key, const float_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Double</td>
      <td align="left"><code>putDouble(const char* key, const double_t value)</code></td>
    </tr>
    <tr>
      <td align="center">Bool</td>
      <td align="left"><code>putBool(const char*key, const bool value)</code></td>
    </tr>
    <tr>
      <td align="center">String</td>
      <td align="left"><code>putString(const char* key, const String value)</code></td>
    </tr>
    <tr>
      <td align="center">Bytes</td>
      <td align="left"><code>putBytes(const char*key, const void* value, size_t len)</code></td>
    </tr>
  </tbody></table>

**函数 6**. 类似地，你应该根据要获取的变量类型使用不同的方法。

<table align="center">
    <tr>
     <td align="center">Char</td>
     <td align="left"><code>getChar(const char*key, const int8_t defaultValue)</code></td>
 </tr>
 <tr>
     <td align="center">Unsigned Char</td>
     <td align="left"><code>getUChar(const char* key, const uint8_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Short</td>
     <td align="left"><code>getShort(const char*key, const int16_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Unsigned Short</td>
     <td align="left"><code>getUShort(const char* key, const uint16_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Int</td>
     <td align="left"><code>getInt(const char*key, const int32_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Unsigned Int</td>
     <td align="left"><code>getUInt(const char* key, const uint32_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Long</td>
     <td align="left"><code>getLong(const char*key, const int32_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Unsigned Long</td>
     <td align="left"><code>getULong(const char* key, const uint32_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Long64</td>
     <td align="left"><code>getLong64(const char*key, const int64_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Unsigned Long64</td>
     <td align="left"><code>gettULong64(const char* key, const uint64_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Float</td>
     <td align="left"><code>getFloat(const char*key, const float_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Double</td>
     <td align="left"><code>getDouble(const char* key, const double_t defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">Bool</td>
     <td align="left"><code>getBool(const char*key, const bool defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">String</td>
     <td align="left"><code>getString(const char* key, const String defaultValue)</code></td>
 </tr>
    <tr>
     <td align="center">String</td>
     <td align="left"><code>getString(const char*key, char* value, const size_t maxLen)</code></td>
 </tr>
    <tr>
     <td align="center">Bytes</td>
     <td align="left"><code>getBytes(const char*key, void* buf, size_t maxLen)</code></td>
 </tr>
</table>

**函数 7**. 删除命名空间

在偏好设置的Arduino实现中，没有完全删除命名空间的方法。因此，在几个项目的过程中，ESP32非易失性存储(nvs)首选项分区可能会满。要完全擦除并重新格式化偏好设置使用的NVS内存，请创建一个包含以下内容的草图:
```c
#include <nvs_flash.h>

void setup() {
  nvs_flash_erase(); // 擦除 NVS 分区并...
  nvs_flash_init(); // 初始化 NVS 分区。
  while(true);
}

void loop() {

}
```

在运行上述代码后，你应该立即下载一个新的sketch到你的板上，否则每次启动NVS分区时，它都会重新格式化。


有关更多信息，您可以访问Preferences.cpp文件[此处](https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp)。

### 使用Preferences.h库的一般方法

**步骤 1.** 要使用Preferences.h库存储数据，首先你需要在你的sketch中包含它:

```c
#include <Preferences.h>
```

**步骤 2.** 然后，您必须初始化Preferences库的一个实例。您可以将其称为preferences，例如:

```c
Preferences preferences;
```

**步骤 3.** 在`setup()`中，初始化波特率为115200的串行监视器。

```c
Serial.begin(115200);
```

**步骤 4.** 在闪存中创建一个名为“my-app”的读写模式的“存储空间”。你可以给它取别的名字。

```c
preferences.begin("my-app", false);
```

**步骤 5.** 使用get和put方法来获取/存储数据内容。

#### 存储/获取键：值对数据

使用首选项保存的数据结构如下:

```c
namespace {
  key:value
}
```

你可以在同一个命名空间中保存不同的键，例如:

```c
namespace {
  key1: value1
  key2: value2
}
```

你也可以用同一个键创建多个命名空间(但每个键对应一个值):

```c
namespace1{
  key:value1
}
namespace2{
  key:value2
}
```

例如，将新值存储在" counter "键上:

```c
preferences.putUInt("counter", counter);
```

然后，获取保存在首选项上的`counter`键的值。如果没有找到任何值，则默认返回0(这段代码第一次运行时会发生这种情况)。

```c
unsigned int counter = preferences.getUInt("counter", 0);
```

所以，你的数据是这样结构的:
```c
my-app{
  counter: counter
}
```

#### Store/get字符串数据

以下代码使用`Preferences.h`将您的网络凭据永久保存在ESP32闪存上。

创建一个名为ssid的键来保存你的ssid值(ssid变量)——使用`putString()`方法。

```c
preferences.putString("ssid", ssid);
```

添加另一个名为password的键来保存密码值(password变量):

```c
preferences.putString("password", password);
```

所以，你的数据是这样结构的:

```c
my-app{
  ssid: ssid
  password: password
}
```

使用`getString()`方法获取SSID和密码值。你需要使用保存变量时使用的密钥名，在本例中是ssid和password密钥:

```c
String ssid = preferences.getString("ssid", ""); 
String password = preferences.getString("password", "");
```

作为`getString()`函数的第二个参数，我们传递了一个空字符串。这是在首选项中没有保存`ssid`或`password`键的情况下的返回值。

**步骤 6.** 关闭首选项。

```c
preferences.end();
```

- “存储/获取键:值对”数据完成过程如下所示。

```c
#include <Preferences.h>

Preferences preferences;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println();

  // 使用 "my-app" 命名空间打开首选项。每个应用模块、库等
  // 都必须使用命名空间名称，以防止键名冲突。我们将以
  // RW（读写）模式打开存储（第二个参数必须为 false）。
  // 注意：命名空间名称的长度限制为 15 个字符。
  preferences.begin("my-app", false);

  // 删除打开命名空间下的所有首选项
  //preferences.clear();

  // 或者只删除 "counter" 键
  //preferences.remove("counter");

  // 获取计数器的值，如果键不存在，则返回默认值 0
  // 注意：键名的长度限制为 15 个字符。
  unsigned int counter = preferences.getUInt("counter", 0);

  // 将计数器加 1
  counter++;

  // 将计数器值打印到串口监视器
  Serial.printf("当前计数器值: %u\n", counter);

  // 将计数器值存储到首选项中
  preferences.putUInt("counter", counter);

  // 关闭首选项
  preferences.end();

  // 等待 10 秒
  Serial.println("10 秒后重启...");
  delay(10000);

  // 重启 ESP
  ESP.restart();
}

void loop() {

}
```

把代码上传到你的板上，这就是你应该在串行显示器上得到的结果:

<div align="center"><img width ="600" src="https://files.seeedstudio.com/wiki/xiaoesp32c3-permanently-data/1.png"/></div>

- 存储/获取字符串数据的完整过程如下所示。

使用`Preferences.h`保存网络凭据。

```c
#include <Preferences.h>

Preferences preferences;

const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println();

  preferences.begin("credentials", false);
  preferences.putString("ssid", ssid); 
  preferences.putString("password", password);

  Serial.println("Network Credentials Saved using Preferences");

  preferences.end();
}

void loop() {

}
```

把代码上传到你的板上，这就是你应该在串行显示器上得到的结果:

<div align="center"><img width ="600" src="https://files.seeedstudio.com/wiki/xiaoesp32c3-permanently-data/2.png"/></div>

连接Wi-Fi与网络凭据保存在首选项。

```c
#include <Preferences.h>
#include "WiFi.h"

Preferences preferences;

String ssid;
String password;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println();
  
  preferences.begin("credentials", false);
 
  ssid = preferences.getString("ssid", ""); 
  password = preferences.getString("password", "");

  if (ssid == "" || password == ""){
    Serial.println("No values saved for ssid or password");
  }
  else {
    // 连接到 Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi ");
    Serial.println(ssid);
    Serial.println(password);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.println(WiFi.localIP());  
  }
}

void loop() {
  // 将你的主代码放在这里，以重复运行：
}
```

在前一个代码之后，将这段代码上传到你的板上(以确保你保存了凭据)。如果一切按照预期进行，这就是您应该在串行显示器上得到的结果。

<div align="center"><img width ="600" src="https://files.seeedstudio.com/wiki/xiaoesp32c3-permanently-data/3.png"/></div>

## 使用EEPROM存储永久数据
### 什么是EEPROM?

EEPROM是ESP32微控制器的内部存储器，允许在重新启动板后保持在内存中数据。在使用微控制器时，将数据保存在内存中是很有趣的，特别是当卡关闭时，无论是否需要，就像在失去电力的情况下。


ESP32微控制器有一个闪存区，可以像Arduino的EEPROM一样接口，即使在电路板关闭后也可以将数据保存在内存中。


:::cautionAttention

需要注意的一件重要的事情是EEPROM的大小和寿命有限。内存单元可以被任意多次读取，但写入周期的数量被限制为**100,000**。建议密切关注存储数据的大小以及多久更新一次。EEPROM存储器可以存储从0到255或128个IP地址或RFID标签的512个值。

:::


ESP32上的微控制器有EEPROM(电可擦可编程只读存储器)。这是一个可以存储字节变量的小空间。存储在EEPROM中的变量保存在那里，当您重置或关闭ESP32时发生事件。简单地说，EEPROM是一种永久存储器，类似于计算机中的硬盘驱动器。


EEPROM可以通过电子方式读取、擦除和重写。在Arduino中，您可以使用EEPROM库轻松读取和写入EEPROM。


每个EEPROM位置可以保存1字节，这意味着只能存储8位数字，其中包括0到255之间的整数。

### 可用的EEPROM功能

要使用Arduino IDE从ESP32闪存读写，我们将使用EEPROM库。在ESP32中使用这个库与在Arduino中使用它非常相似。如果你以前用过Arduino EEPROM，这没什么不同。

因此，我们也建议看看我们关于[Arduino EEPROM](https://randomnerdtutorials.com/arduino-eeprom-explained-remember-last-led-state/)的文章。

**函数 1**.初始化内存大小

Before using the function, we have to initialize the size of the memory with `EEPROM.begin()`.

```c
EEPROM.begin(EEPROM_SIZE);
```

**函数 2**. 写入或者输出

要将数据写入EEPROM，需要使用`EEPROM.write()`函数，该函数接受两个参数。第一个是你想保存数据的EEPROM位置或地址，第二个是我们想保存的值:

```c
EEPROM.write(address, value);
```

`EEPROM.write()`等同于`EEPROM.put()`。

```c
EEPROM.put(address, value);
```

例如，要在地址0上写9，可以这样写:

```c
EEPROM.write(0, 9);
```

:::tip
如果我们想存储浮点数据，我们通常使用`EEPROM.put()`方法而不是`EEPROM.write()`方法。如果你想使用write()方法存储它，那么你需要使用`EEPROM.writeFloat()`。
:::

**函数 3**. 读取或者获取

要从EEPROM中读取一个字节，需要使用`EEPROM.read()`函数。这个函数接受一个字节的地址作为参数。

```c
EEPROM.read(address);
```

`EEPROM.read()`等同于`EEPROM.get()`。

```c
EEPROM.get(address);
```

例如，读取之前存储在地址0中的字节:

```c
EEPROM.read(0);
```

这将返回**9**，这是存储在该位置的值。

:::tip
如果我们想获取浮点数数据，我们通常使用`EEPROM.get()`方法而不是`EEPROM.read()`方法。如果你想使用read()方法获取它，那么你需要使用`EEPROM.readFloat()`。
:::

**函数 4**. 更新一个值

`EEPROM.update()`函数特别有用。如果写入的值与已经保存的值不同，它只写入EEPROM。

由于EEPROM的写入/擦除周期有限，因此使用`EEPROM.update()`函数而不是`EEPROM.write()`来节省周期。

像下面这样使用`EEPROM.update()`函数:

```c
EEPROM.update(address, value);
```

目前，我们将9存储在地址0中。因此，如果我们调用:

```c
EEPROM.update(0, 9);
```

它不会再次写入EEPROM，因为当前保存的值与我们想要写入的值相同。

:::note
要了解有关EEPROM操作的更多信息，您可以阅读[官方Arduino文档](https://docs.arduino.cc/learn/programming/eeprom-guide#eeprom-clear)。
:::

### EEPROM的一般使用方法

为了向您展示如何在XIAO ESP32C3闪存中保存数据，我们将保存输出的最后一个状态，在本例中是LED。

如图所示，将LED连接到XIAO ESP32C3。

<div align="center"><img width ="400" src="https://files.seeedstudio.com/wiki/XIAO_WiFi/connect-led-2.png"/></div>

首先，需要包含EEPROM库。

```c
#include <EEPROM.h>
```

然后，定义EEPROM的大小。这就是你想要访问闪存的字节数。在这种情况下，我们只保存LED状态，因此EEPROM大小设置为1。

```c
#define EEPROM_SIZE 1
```

我们还定义了其他变量，需要使这个草图工作。

```c
// 常量不会改变。它们在这里用于设置引脚号：
const int ledPin = D10;      // LED 引脚的编号

// 变量会发生变化：
int ledState = LOW;  // 用于设置 LED 的状态

// 通常，应该使用 "unsigned long" 类型的变量来存储时间
// 因为 int 类型的变量存储的值很快就会超过其上限
unsigned long previousMillis = 0;  // 存储上次 LED 更新的时间

// 常量不会改变：
const long interval = 10000;  // 间隔时间，用于控制 LED 闪烁的间隔（毫秒）
```

在`setup()`中，使用预定义的大小初始化EEPROM。

```c
EEPROM.begin(EEPROM_SIZE);
```

为了确保你的代码初始化时使用最新的LED状态，在`setup()`中，你应该从闪存中读取最新的LED状态。它存储在地址0上。


然后，你只需要根据从闪存中读取的值打开或关闭LED。

```c
digitalWrite (ledPin, ledState);
```

在`loop()`函数部分，我们需要做的就是在一段时间内翻转LED的状态。

```c
// 检查是否该闪烁 LED；即，如果当前时间与上次闪烁 LED 的时间差大于
// 设置的闪烁间隔时间，则执行 LED 闪烁。
unsigned long currentMillis = millis();

if (currentMillis - previousMillis >= interval) {
    // 保存上次闪烁 LED 的时间
    previousMillis = currentMillis;
    Serial.println("状态已改变");
    // 如果 LED 关闭，则打开它；反之亦然：
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // 使用 ledState 变量的值来设置 LED 状态：
    digitalWrite(ledPin, ledState);
}
```

接下来，我们需要判断倒计时是否结束，在倒计时结束后翻转LED的状态，并将其存储在闪存中。

```c
EEPROM.write(0, ledState);
```

最后，我们使用EEPROM.commit()让修改生效。

```c
EEPROM.commit();
```

以下是完成的过程。
:::cautionAttention
请注意，你**不应该**长时间运行这个例子。在这个例子中，我们每十秒钟写一次EEPROM，长时间运行这个例子会**大大减少** EEPROM的寿命。
:::

```c
// 包含用于读写闪存的库
#include <EEPROM.h>

// 定义要访问的字节数
#define EEPROM_SIZE 1

// 常量不会改变。它们在这里用于设置引脚号：
const int ledPin = D10;      // LED 引脚的编号

// 变量会发生变化：
int ledState = LOW;  // 用于设置 LED 的状态

// 通常，应该使用 "unsigned long" 类型的变量来存储时间
// 因为 int 类型的变量存储的值很快就会超过其上限
unsigned long previousMillis = 0;  // 存储上次 LED 更新的时间

// 常量不会改变：
const long interval = 10000;  // 间隔时间，用于控制 LED 闪烁的间隔（毫秒）

void setup() { 
  Serial.begin(115200);
  
  // 初始化 EEPROM，使用预定义的大小
  EEPROM.begin(EEPROM_SIZE);

  pinMode(ledPin, OUTPUT);

  // 从闪存中读取最后的 LED 状态
  ledState = EEPROM.read(0);
  // 设置 LED 为最后保存的状态
  digitalWrite(ledPin, ledState);
}

void loop() {
  // 这里是运行中需要持续执行的代码。

  // 检查是否该闪烁 LED；即，如果当前时间与上次闪烁 LED 的时间差大于
  // 设置的闪烁间隔时间，则执行 LED 闪烁。
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // 保存上次闪烁 LED 的时间
    previousMillis = currentMillis;
    Serial.println("状态已改变");
    // 如果 LED 关闭，则打开它；反之亦然：
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // 将 LED 状态保存到闪存中
    EEPROM.write(0, ledState);
    EEPROM.commit();
    Serial.println("状态已保存到闪存");

    // 使用 ledState 变量的值来设置 LED 状态：
    digitalWrite(ledPin, ledState);
  }
}
```

把代码上传到你的板上，这就是你应该在串行显示器上得到的结果:

<div align="center"><img width ="600" src="https://files.seeedstudio.com/wiki/xiaoesp32c3-permanently-data/4.png"/></div>

## 技术支持和产品讨论

感谢您选择我们的产品!我们在这里为您提供不同的支持，以确保您使用我们的产品的体验尽可能顺利。我们提供多种沟通渠道，以满足不同的喜好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
