---
description: XIAO ESP32S3(Sense) With FreeRTOS
title: XIAO ESP32S3(Sense) 与 FreeRTOS
keywords:
  - Software
  - FreeRtos
  - sd
  - camera
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/xiao-esp32s3-freertos
last_update:
  date: 11/19/2024
  author: Agnes
---

# XIAO ESP32S3(Sense) 与 FreeRTOS

本维基涵盖了 [FreeRTOS](https://freertos.org/) 对 [Seeed Studio XIAO ESP32S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) 的支持。通过本指南，您将能够利用该板所提供的功能集。

## 什么是 [FreeRTOS](https://www.freertos.org/index.html)

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/1.png" alt="pir" width={600} height="auto" /></p>

FreeRTOS 是一组 C 库，包含一个实时内核和一组实现互补功能的模块化库。FreeRTOS 内核是一个实时内核（或实时调度器），它使得基于 FreeRTOS 构建的应用程序能够满足硬实时要求。它使得应用程序能够组织成一组独立的执行线程。

_参考资料 : [**Mastering the FreeRTOS Real Time Kernel**](https://www.freertos.org/Documentation/02-Kernel/07-Books-and-manual/01-RTOS_book)_

## FreeRTOS 移植

FreeRTOS 是一个开源的实时操作系统（RTOS）内核，已作为组件集成到 ESP-IDF 中。因此，所有 ESP-IDF 应用程序和许多 ESP-IDF 组件都是基于 FreeRTOS 编写的。FreeRTOS 内核已经移植到 ESP 芯片的所有架构（即 Xtensa 和 RISC-V）上。

我们将使用 ESP IDF 版本的 FreeRTOS。

## 硬件准备

我使用的是 [Seeed Studio XIAO ESP32S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)，以及该板载的相机、麦克风、SD 卡读卡器和 ESP32S3 的 WiFi 功能。

<div class="table-center">
  <table align="center">
    <tr>
        <th>Seeed Studio XIAO ESP32S3(Sense)</th>
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

### 其他组件

- [Grove - 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - I2C 显示器、RTC & 按钮
- [空气质量传感器 v1.3](https://www.seeedstudio.com/Grove-Air-Quality-Sensor-v1-3-Arduino-Compatible.html)
- [Grove - 温湿度、气压和气体传感器 BME680](https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-for-Arduino-BME680.html)
- [Seeed Studio XIAO 扩展板的有机玻璃外壳](https://www.seeedstudio.com/XIAO-p-4812.html)

## 软件准备

我在 Windows 系统上使用 Visual Studio Code 和 ESP-IDF。

1. VSCode 安装
2. ESP-IDF 安装指南
3. Git 仓库

<div class="table-center">
  <table align="center">
    <tr>
        <th>VS Code</th>
        <th>适用于 VSCode 的 ESP-IDF</th>
    </tr>
      <tr>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://code.visualstudio.com/download">
              <strong><span><font color={'FFFFFF'} size={"4"}> 下载 VSCode⏬</font></span></strong>
          </a>
      </div></td>
        <td><div class="get_one_now_container" style={{textAlign: 'center'}}>
          <a class="get_one_now_item" href="https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md">
              <strong><span><font color={'FFFFFF'} size={"4"}> ESP-IDF 安装 ⏬</font></span></strong>
          </a>
      </div></td>
    </tr>
  </table>
</div>

## 开始使用

### 设置 ESP-IDF

在设置 [Visual Studio 扩展](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) 后，打开终端并粘贴以下命令，以便从普通终端环境（外部的 VScode）访问 ESP-IDF 命令行工具。

:::note
VS-Code 的 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) 扩展的正常安装将处理 90% 的使用案例，只有在需要在 VSCode 外部使用 ESP 命令行工具时，才需要执行以下步骤。
:::

PowerShell（Windows）

```shell
.$HOME\esp\v5.3\esp-idf\export.ps1
```

:::info
“.$HOMEespv5.3esp-idf” 可能因用户而异。这是默认安装路径。
将它替换为设备上的安装路径。
:::

:::tip
为避免重复设置，请在管理员模式下启动 PowerShell 并键入以下命令
```c
notepad $PSHOME\Profile.ps1
```
将打开一个记事本实例。将 export shell 命令粘贴到记事本中并保存。打开 PowerShell 的一个实例，它应该具有接近以下输出。
```
Done! You can now compile ESP-IDF projects.
```
::: 

如果一切设置正确，执行以下命令：

```shell
idf.py
```

应该会显示如下输出：

```shell
Usage: idf.py [OPTIONS] COMMAND1 [ARGS]... [COMMAND2 [ARGS]...]...

  ESP-IDF CLI build management tool. For commands that are not known to idf.py an attempt to execute it as a build
  system target will be made. Selected target: None
```

## 什么是任务？

任务是处理器被要求执行的小功能/工作，带有一组设置。任务的类型可以从小的功能到无限循环的功能不等。  
任务是 ESP-IDF 应用程序中的基本执行单元。它们本质上是与其他任务并发运行的功能。这使得高效的多任务处理和响应成为可能。

### 任务的属性是什么？

由于这个主题非常广泛，在此我将仅介绍一些我们在本指南中将使用的属性。

- **TaskFunction**：这是包含任务实际逻辑的函数。它是任务执行的入口点。
- **StackSize**：指定为任务的栈分配的内存量。栈用于存储局部变量、函数返回地址和临时数据。
- **TaskPriority**：决定任务相对于其他任务的重要性。优先级较高的任务更有可能先于优先级较低的任务执行。
- **TaskParameters**：这是在创建任务时可以传递给任务函数的可选参数。它们可以用于为任务提供额外的上下文或配置。
- **CoreAffinity**：指定任务应该分配到哪个 CPU 核心。在多核系统中，可以使用此属性来优化性能或平衡工作负载。

### 创建任务

要在 FreeRTOS 中创建任务，可以使用 `xTaskCreate` 函数。此函数需要多个参数，包括任务函数、任务名称、栈大小、参数、优先级和创建的任务句柄。

```c
TaskHandle_t task;
xTaskCreate(
        taskFunction,             /* 实现任务的函数。 */
        "taskName",               /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字或字节为单位）。 */
        NULL,                     /* 传递给任务的参数。 */
        tskIDLE_PRIORITY,         /* 任务创建时的优先级。 */
        &task                     /* 用于传出创建任务的句柄。 */
        );
```

### 创建并将任务绑定到一个核心

要创建任务并将其绑定到特定核心（仅当使用的芯片是双核时），可以使用 `xTaskCreatePinnedToCore` 函数。此函数与 `xTaskCreate` 类似，但增加了一个额外的参数，用于指定核心。

```c
TaskHandle_t task;
xTaskCreatePinnedToCore(
        taskFunction,             /* 实现任务的函数。 */
        "taskName",               /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字或字节为单位）。 */
        NULL,                     /* 传递给任务的参数。 */
        tskIDLE_PRIORITY,         /* 任务创建时的优先级。 */
        &task,                    /* 用于传出创建任务的句柄。 */
        0);                       /* 核心 ID */
```

### 任务函数调用

任务函数是任务将要执行的实际代码。

```c
void taskFunction(void * pvParameters) {
  /*
  函数定义在这里
  */
}
```

## 任务的可视化

我正在创建四个简单的任务来可视化 FreeRTOS 是如何工作的。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/2.png" alt="pir" width={700} height="auto" /></p>

### 可视化表示

```shell
CPU0
-----
taskFunction1 (1000ms 延迟)

CPU1
-----
taskFunction2 (500ms 延迟)
taskFunction3 (500ms 延迟)
taskFunction4 (500ms 延迟)
```

### 代码

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

TaskHandle_t task1, task2, task3, task4;

void taskFunction1(void * pvParameters) {
    while (true) {
        ESP_LOGI("Task1", "来自任务 1 的问候");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 添加延迟，避免输出过多
    }
}

void taskFunction2(void * pvParameters) {
    while (true) {
        ESP_LOGI("Task2", "来自任务 2 的问候");
        vTaskDelay(pdMS_TO_TICKS(500)); // 添加延迟，避免输出过多
    }
}

void taskFunction3(void * pvParameters) {
    while (true) {
        ESP_LOGI("Task3", "来自任务 3 的问候");
        vTaskDelay(pdMS_TO_TICKS(500)); // 添加延迟，避免输出过多
    }
}

void taskFunction4(void * pvParameters) {
    while (true) {
        ESP_LOGI("Task4", "来自任务 4 的问候");
        vTaskDelay(pdMS_TO_TICKS(500)); // 添加延迟，避免输出过多
    }
}

void app_main(void) {
    xTaskCreatePinnedToCore(
        taskFunction1, /* 实现任务的函数。 */
        "task_1",      /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字为单位，而非字节）。 */
        NULL,          /* 传递给任务的参数。 */
        tskIDLE_PRIORITY, /* 任务创建时的优先级。 */
        &task1,        /* 用于传出创建任务的句柄。 */
        0);            /* 核心 ID */

    xTaskCreatePinnedToCore(
        taskFunction2, /* 实现任务的函数。 */
        "task_2",      /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字为单位，而非字节）。 */
        NULL,          /* 传递给任务的参数。 */
        tskIDLE_PRIORITY, /* 任务创建时的优先级。 */
        &task2,        /* 用于传出创建任务的句柄。 */
        1);            /* 核心 ID */

    xTaskCreatePinnedToCore(
        taskFunction3, /* 实现任务的函数。 */
        "task_3",      /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字为单位，而非字节）。 */
        NULL,          /* 传递给任务的参数。 */
        tskIDLE_PRIORITY, /* 任务创建时的优先级。 */
        &task3,        /* 用于传出创建任务的句柄。 */
        1);            /* 核心 ID */

    xTaskCreatePinnedToCore(
        taskFunction4, /* 实现任务的函数。 */
        "task_4",      /* 任务的名称。 */
        configMINIMAL_STACK_SIZE, /* 栈大小（以字为单位，而非字节）。 */
        NULL,          /* 传递给任务的参数。 */
        tskIDLE_PRIORITY, /* 任务创建时的优先级。 */
        &task4,        /* 用于传出创建任务的句柄。 */
        1);            /* 核心 ID */
}
```

:::tip
configMINIMAL_STACK_SIZE 可以在 sdkconfig 中更改。
:::

1. 四个任务：代码定义了四个任务：taskFunction1、taskFunction2、taskFunction3 和 taskFunction4。
2. 任务优先级：所有任务都使用 tskIDLE_PRIORITY 创建。这意味着它们的优先级相同。
3. 任务绑定：taskFunction1 被绑定到 CPU0，而其他三个任务被绑定到 CPU1。
4. 任务延迟：taskFunction1 的延迟为 1000ms，而其他三个任务的延迟为 500ms。

### 创建 CPU0 和 CPU1 任务调度

我为 CPU0 和 CPU1 创建了一个基本的任务调度。

#### CPU0 任务调度

```shell
Task: taskFunction1
Priority: Idle (lowest)
Delay: 1000ms
Core: 0
```

#### CPU1 任务计划

```shell
Tasks: taskFunction2, taskFunction3, taskFunction4
Priorities: All Idle (same priority)
Delays: 500ms for all tasks
Core: 1
```

:::info
这是一个简化的时间表。实时系统中的实际任务调度将涉及更复杂的因素，例如任务优先级、截止日期和资源限制。
:::

<details>

<summary> 输出</summary>

```shell
I (11412) Task1: Hello from task 1
I (11522) Task3: Hello from task 3
I (11522) Task2: Hello from task 2
I (11532) Task4: Hello from task 4
I (12032) Task3: Hello from task 3
I (12032) Task2: Hello from task 2
I (12042) Task4: Hello from task 4
I (12422) Task1: Hello from task 1
I (12542) Task3: Hello from task 3
I (12542) Task2: Hello from task 2
I (12552) Task4: Hello from task 4
I (13052) Task3: Hello from task 3
I (13052) Task2: Hello from task 2
I (13062) Task4: Hello from task 4
I (13432) Task1: Hello from task 1
I (13562) Task3: Hello from task 3
I (13562) Task2: Hello from task 2
I (13572) Task4: Hello from task 4
I (14072) Task3: Hello from task 3
I (14072) Task2: Hello from task 2
I (14082) Task4: Hello from task 4
```

</details>

## 使用 FreeRTOS 进行传感器轮询

为此，我使用了一个模拟传感器 [空气质量传感器 v1.3](https://www.seeedstudio.com/Grove-Air-Quality-Sensor-v1-3-Arduino-Compatible.html)，以及 ESP_IDF_v5.3。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Priyanshu0901/Air_quality_Sensor_ESP-IDF.git">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载代码</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

### 硬件设置

将 Xiao-S3 连接到 [Grove 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html)，并将 [空气质量传感器 v1.3](https://www.seeedstudio.com/Grove-Air-Quality-Sensor-v1-3-Arduino-Compatible.html) 连接到模拟接口。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/3.jpg" alt="pir" width={600} height="auto" /></p>

### 软件设置

克隆 Git 仓库后，在 VSCode 中打开该文件夹。然后选择 **视图 -> 命令面板 -> ESP-IDF: 添加 vscode 配置文件夹**。  
从底部面板选择正确的 COM 端口、芯片（ESP-S3），并进行构建、烧录和监视。

### 代码概述

此代码设计用于从传感器收集空气质量数据，处理原始数据以确定空气质量水平，并定期将结果打印到控制台。

#### 主要组件：

- 传感器初始化：

```c
air_quality_sensor_t air_quality_sensor;

void sensor_setup()
{
    air_quality_sensor._io_num = ADC_CHANNEL_0;
    air_quality_sensor._adc_num = ADC_UNIT_1;
    printf("Starting Air Quality Sensor...\n");
    if(!initialize_air_quality_sensor(&air_quality_sensor))
    {
        printf("Sensor ready.\n");
    }
    else{
        printf("Sensor ERROR!\n");
    }
}
```

- sensor_setup() 函数配置传感器的 I/O 引脚和 ADC 单元。
- 它尝试通过 initialize_air_quality_sensor() 初始化传感器。
- 如果初始化成功，传感器就可以开始数据收集。

- 数据收集任务：

```c
void poll_read_air_quality_sensor(void *pvParameters)
{
    for (;;)
    {
        air_quality_sensor_slope(&air_quality_sensor);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
```

- poll_read_air_quality_sensor() 任务用于持续从传感器读取原始数据。
- 它调用 air_quality_sensor_slope() 处理原始数据并计算斜率，斜率是空气质量的一个指标。
- 任务在读取下一个数据点之前会延迟 500 毫秒。

- 数据打印任务：

```c

void print_read_air_quality_sensor(void *pvParameters)
{
    for (;;)
    {
        char buf[40];
        air_quality_error_to_message(air_quality_sensor._air_quality,buf);
        printf("Time : %lu\tSlope : %d\tRaw Value : %d\n%s\n", (uint32_t)esp_timer_get_time() / 1000, air_quality_sensor._air_quality, air_quality_sensor._sensor_raw_value,buf);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

- print_read_air_quality_sensor() 任务用于定期打印收集的数据和计算的空气质量。
- 它使用 air_quality_error_to_message() 获取当前时间、斜率、原始值和空气质量消息。
- 任务将数据以格式化的方式打印到控制台。
- 任务在打印下一个数据点之前会延迟 1000 毫秒。

```c

void app_main(void)
{
    sensor_setup();
    xTaskCreatePinnedToCore(
        poll_read_air_quality_sensor,   /* 实现任务的函数。 */
        "poll_read_air_quality_sensor", /* 任务的文本名称。 */
        configMINIMAL_STACK_SIZE * 2,   /* 堆栈大小（单位：字），不是字节。 */
        NULL,                           /* 传递给任务的参数。 */
        tskIDLE_PRIORITY,               /* 创建任务时的优先级。 */
        NULL,                           /* 用于传出创建的任务句柄。 */
        0);                             /* 核心 ID */

    xTaskCreatePinnedToCore(
        print_read_air_quality_sensor,   /* 实现任务的函数。 */
        "print_read_air_quality_sensor", /* 任务的文本名称。 */
        configMINIMAL_STACK_SIZE * 2,    /* 堆栈大小（单位：字），不是字节。 */
        NULL,                            /* 传递给任务的参数。 */
        tskIDLE_PRIORITY + 1,            /* 创建任务时的优先级。 */
        NULL,                            /* 用于传出创建的任务句柄。 */
        0);                              /* 核心 ID */
}
```

### 输出

```shell
Time : 37207    Slope : 3       Raw Value : 273
Fresh air.
Time : 38217    Slope : 3       Raw Value : 269
Fresh air.
Time : 39227    Slope : 3       Raw Value : 274
Fresh air.
Time : 40237    Slope : 3       Raw Value : 251
Fresh air.
Time : 41247    Slope : 3       Raw Value : 276
Fresh air.
Time : 42257    Slope : 3       Raw Value : 250
Fresh air.
Time : 43267    Slope : 3       Raw Value : 236
Fresh air.
Time : 44277    Slope : 3       Raw Value : 253
Fresh air.
Time : 45287    Slope : 3       Raw Value : 245
Fresh air.
Time : 46297    Slope : 3       Raw Value : 249
Fresh air.
Time : 47307    Slope : 3       Raw Value : 244
Fresh air.
Time : 48317    Slope : 3       Raw Value : 235
Fresh air.
Time : 49327    Slope : 3       Raw Value : 239
Fresh air.
Time : 50337    Slope : 3       Raw Value : 233
Fresh air.
Time : 51347    Slope : 3       Raw Value : 235
Fresh air.
```

## FreeRTOS 中的摄像头和 SdCard 使用

为此，我将 onBoard Camera 和 SdCard 与 ESP_IDF_v5.3 一起使用。

<div class="github_container" style={{textAlign: 'center'}}>
    <a class="github_item" href="https://github.com/Priyanshu0901/Camera-and-SdCard-FreeRTOS.git">
    <strong><span><font color={'FFFFFF'} size={"4"}> 下载代码</font></span></strong> <svg aria-hidden="true" focusable="false" role="img" className="mr-2" viewBox="-3 10 9 1" width={16} height={16} fill="currentColor" style={{textAlign: 'center', display: 'inline-block', userSelect: 'none', verticalAlign: 'text-bottom', overflow: 'visible'}}><path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" /></svg>
    </a>
</div>

### 硬件设置

按照[microSD卡指南](https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/)和[摄像头指南](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/)将摄像头和microSD卡扩展板连接到

- 格式化microSD卡（支持最大32Gb）
- 将microSD卡连接到扩展板

设置如下所示：

<div class="table-center">
  <table align="center">
    <tr>
        <th>正面</th>
        <th>背面</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/6.jpeg" style={{width:250, height:'auto'}}/></div></td>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/7.jpeg" style={{width:250, height:'auto'}}/></div></td>
    </tr>
  </table>
</div>

### 软件设置

拉取git仓库后，在VSCode中打开该文件夹。进入视图->命令面板->ESP-IDF: 添加vscode配置文件夹。
在底部面板中选择正确的COM端口、芯片（ESP-S3），然后进行构建、闪存和监控。

### 摄像头组件

- 摄像头配置：
  - 定义了用于各种摄像头功能的GPIO引脚（PWDN、RESET、XCLK、SIOD、SIOC、Y9-Y2、VSYNC、HREF、PCLK、LED）。
  - 设置摄像头参数的默认值（例如，时钟频率、帧缓冲区位置、像素格式、帧大小、JPEG质量、帧缓冲区计数、抓取模式）。

```c
#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

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

#endif //CAMERA_CONFIG_H
```

- 摄像头接口：
  声明了函数initialize_camera()和createCameraTask()。

- 摄像头实现：

  - 使用定义的配置初始化摄像头。

  ```c
  void initialize_camera(void)
  {
    camera_config_t camera_config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
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

        .xclk_freq_hz = 20000000,          // 图像传感器的时钟频率
        .fb_location = CAMERA_FB_IN_PSRAM, // 设置帧缓冲区存储位置
        .pixel_format = PIXFORMAT_JPEG,    // 图像的像素格式：PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
        .frame_size = FRAMESIZE_UXGA,      // 图像的分辨率大小：FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        .jpeg_quality = 15,                // JPEG图像的质量，范围从0到63。
        .fb_count = 2,                     // 使用的帧缓冲区数量。
        .grab_mode = CAMERA_GRAB_LATEST    // 图像捕获模式。
    };

    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI(cameraTag, "摄像头配置成功");
    }
    else
    {
        ESP_LOGI(cameraTag, "摄像头配置失败");
        return;
    }
  }
  ```

  - 设置摄像头参数（亮度、对比度、饱和度、特效、白平衡、曝光控制、AEC、AE级别、AEC值、增益控制、AGC增益、增益上限、BPC、WPC、原始GMA、LENC、水平镜像、垂直翻转、DCW、色条）。

  ```c
  sensor_t *s = esp_camera_sensor_get();

    s->set_brightness(s, 0);                 // -2 到 2
    s->set_contrast(s, 0);                   // -2 到 2
    s->set_saturation(s, 0);                 // -2 到 2
    s->set_special_effect(s, 0);             // 0 到 6 (0 - 无效果, 1 - 负片, 2 - 灰度, 3 - 红色调, 4 - 绿色调, 5 - 蓝色调, 6 - 怀旧)
    s->set_whitebal(s, 1);                   // 0 = 禁用 , 1 = 启用
    s->set_awb_gain(s, 1);                   // 0 = 禁用 , 1 = 启用
    s->set_wb_mode(s, 0);                    // 0 到 4 - 如果启用了 awb_gain (0 - 自动, 1 - 阳光, 2 - 阴天, 3 - 办公室, 4 - 家庭)
    s->set_exposure_ctrl(s, 1);              // 0 = 禁用 , 1 = 启用
    s->set_aec2(s, 0);                       // 0 = 禁用 , 1 = 启用
    s->set_ae_level(s, 0);                   // -2 到 2
    s->set_aec_value(s, 300);                // 0 到 1200
    s->set_gain_ctrl(s, 1);                  // 0 = 禁用 , 1 = 启用
    s->set_agc_gain(s, 0);                   // 0 到 30
    s->set_gainceiling(s, (gainceiling_t)0); // 0 到 6
    s->set_bpc(s, 0);                        // 0 = 禁用 , 1 = 启用
    s->set_wpc(s, 1);                        // 0 = 禁用 , 1 = 启用
    s->set_raw_gma(s, 1);                    // 0 = 禁用 , 1 = 启用
    s->set_lenc(s, 1);                       // 0 = 禁用 , 1 = 启用
    s->set_hmirror(s, 0);                    // 0 = 禁用 , 1 = 启用
    s->set_vflip(s, 0);                      // 0 = 禁用 , 1 = 启用
    s->set_dcw(s, 1);                        // 0 = 禁用 , 1 = 启用
    s->set_colorbar(s, 0);                   // 0 = 禁用 , 1 = 启用
  ```

  - 定义一个函数 takePicture（） 来捕获图像并将其保存到 SD 卡。

  ```c
  void takePicture()
  {
    ESP_LOGI(cameraTag, "Taking picture...");
    camera_fb_t *pic = esp_camera_fb_get();

    if (pic)
    {
        saveJpegToSdcard(pic);
    }

    ESP_LOGI(cameraTag, "Picture taken! Its size was: %zu bytes", pic->len);

    esp_camera_fb_return(pic);
  }
  ```

  - 创建一个任务 cameraTakePicture_5_sec（） 以每 5 秒连续拍摄一次照片。

  ```c
  void cameraTakePicture_5_sec(void *pvParameters)
  {
      for (;;)
      {
          takePicture();
          vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
  }

  void createCameraTask()
  {
      TaskHandle_t task;
      xTaskCreate(
          cameraTakePicture_5_sec,      /* 实现任务的函数。 */
          "cameraTakePicture_5_sec",    /* 任务的文本名称。 */
          configMINIMAL_STACK_SIZE * 4, /* 堆栈大小（以字为单位，或字节）。 */
          NULL,                         /* 传递给任务的参数。 */
          tskIDLE_PRIORITY,             /* 创建任务时的优先级。 */
          &task                         /* 用于传出创建的任务句柄。 */
      );
  }
  ```

代码结构：

- 头文件（camera_config.h, camera_interface.h）和实现文件（camera_interface.c）。
- camera_config.h 文件定义了相机配置参数。
- camera_interface.h 文件声明了相机初始化和任务创建的函数。
- camera_interface.c 文件实现了相机初始化、拍照和任务创建的逻辑。

### SD卡组件

- SD卡配置：  
  定义了用于SD卡接口的GPIO引脚（MISO、MOSI、CLK、CS）。

```c
#ifndef SDCARD_CONFIG_H
#define SDCARD_CONFIG_H

#define PIN_NUM_MISO  GPIO_NUM_8
#define PIN_NUM_MOSI  GPIO_NUM_9
#define PIN_NUM_CLK   GPIO_NUM_7
#define PIN_NUM_CS    GPIO_NUM_21

#endif //SDCARD_CONFIG_H
```

- SD 卡接口： 
  声明函数 initialize_sdcard（）、deinitialize_sdcard（） 和 saveJpegToSdcard（）。

```c
#ifndef SDCARD_INTERFACE_H
#define SDCARD_INTERFACE_H

#include "esp_camera.h"

void initialize_sdcard(void);
void deinitialize_sdcard();
void saveJpegToSdcard(camera_fb_t *);

#endif //SDCARD_INTERFACE_H
```

- SD卡实现：

  - 使用定义的配置初始化SD卡，并将SD卡挂载为FAT文件系统。

```c
sdmmc_card_t *card;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
const char mount_point[] = "/sd";

void initialize_sdcard()
{
    esp_err_t ret;

    // 如果format_if_mount_failed设置为true，则在挂载失败时会对SD卡进行分区和格式化。
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 32 * 1024};

    ESP_LOGI(sdcardTag, "正在初始化SD卡");

    // 使用上述设置初始化SD卡并挂载FAT文件系统。
    // 注意：esp_vfs_fat_sdmmc/sdspi_mount是一个一体化的便利函数。
    // 在开发生产应用时，请查看其源代码并实现错误恢复。
    ESP_LOGI(sdcardTag, "使用SPI外设");

    // 默认情况下，SD卡频率初始化为SDMMC_FREQ_DEFAULT（20MHz）
    // 如果需要设置特定频率，请使用host.max_freq_khz（SDSPI范围为400kHz - 20MHz）
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = host.max_freq_khz,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(sdcardTag, "初始化总线失败。");
        return;
    }

    // 这会初始化没有卡检测（CD）和写保护（WP）信号的插槽。
    // 如果你的板子有这些信号，请修改slot_config.gpio_cd和slot_config.gpio_wp。
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(sdcardTag, "挂载文件系统");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(sdcardTag, "挂载文件系统失败。"
                                "如果你希望卡片被格式化，请在sdcard_config.h中设置FORMAT_IF_MOUNT_FAILED");
        }
        else
        {
            ESP_LOGE(sdcardTag, "初始化卡片失败 (%s)。"
                                "确保SD卡线路上有上拉电阻。",
                      esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(sdcardTag, "文件系统已挂载");

    // 卡片已初始化，打印其属性
    sdmmc_card_print_info(stdout, card);

    // 格式化FATFS
#ifdef FORMAT_SD_CARD
    ret = esp_vfs_fat_sdcard_format(mount_point, card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(sdcardTag, "格式化FATFS失败 (%s)", esp_err_to_name(ret));
        return;
    }

    if (stat(file_foo, &st) == 0)
    {
        ESP_LOGI(sdcardTag, "文件仍然存在");
        return;
    }
    else
    {
        ESP_LOGI(sdcardTag, "文件不存在，格式化完成");
    }
#endif // CONFIG_EXAMPLE_FORMAT_SD_CARD
}
  ```

  - 提供将JPEG图像保存到SD卡的功能。

```c
uint16_t lastKnownFile = 0;

void saveJpegToSdcard(camera_fb_t *captureImage)
{
    // 找到下一个可用的文件名
    char filename[32];

    sprintf(filename, "%s/%u_img.jpg", mount_point, lastKnownFile++);

    // 创建文件并写入JPEG数据
    FILE *fp = fopen(filename, "wb");
    if (fp != NULL)
    {
        fwrite(captureImage->buf, 1, captureImage->len, fp);
        fclose(fp);
        ESP_LOGI(sdcardTag, "JPEG已保存为 %s", filename);
    }
    else
    {
        ESP_LOGE(sdcardTag, "创建文件失败: %s", filename);
    }
}
  ```

组件结构：

- 头文件 (sdcard_config.h, sdcard_interface.h) 和实现文件 (sdcard_interface.c)。
- sdcard_config.h 文件定义了SD卡的配置参数。
- sdcard_interface.h 文件声明了SD卡初始化、反初始化和图像保存的函数。
- sdcard_interface.c 文件实现了SD卡的初始化、反初始化和图像保存逻辑。

### 主函数

```c
// main.c
#include <stdio.h>
#include "camera_interface.h"
#include "sdcard_interface.h"

void initialize_drivers()
{
    initialize_sdcard();
    initialize_camera();
}

void start_tasks()
{
    createCameraTask();
}

void app_main(void)
{
    initialize_drivers();
    start_tasks();
}
```

- 包含摄像头和SD卡接口所需的头文件。
- 使用提供的函数初始化SD卡和摄像头。
- 启动摄像头任务，持续拍照。

### 输出

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/8.gif" alt="pir" width={600} height="auto" /></p>

#### UART输出

```shell
I (1119) main_task: Calling app_main()
I (1123) sdcard: Initializing SD card
I (1127) sdcard: Using SPI peripheral
I (1132) sdcard: Mounting filesystem
I (1137) gpio: GPIO[21]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1146) sdspi_transaction: cmd=52, R1 response: command not supported
I (1195) sdspi_transaction: cmd=5, R1 response: command not supported
I (1219) sdcard: Filesystem mounted
Name: SD32G
Type: SDHC/SDXC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 30448MB
CSD: ver=2, sector_size=512, capacity=62357504 read_bl_len=9
SSR: bus_width=1
I (1226) s3 ll_cam: DMA Channel=1
I (1230) cam_hal: cam init ok
I (1234) sccb: pin_sda 40 pin_scl 39
I (1238) sccb: sccb_i2c_port=1
I (1252) camera: Detected camera at address=0x30
I (1255) camera: Detected OV2640 camera
I (1255) camera: Camera PID=0x26 VER=0x42 MIDL=0x7f MIDH=0xa2
I (1344) cam_hal: buffer_size: 16384, half_buffer_size: 1024, node_buffer_size: 1024, node_cnt: 16, total_cnt: 375
I (1344) cam_hal: Allocating 384000 Byte frame buffer in PSRAM
I (1351) cam_hal: Allocating 384000 Byte frame buffer in PSRAM
I (1357) cam_hal: cam config ok
I (1361) ov2640: Set PLL: clk_2x: 0, clk_div: 0, pclk_auto: 0, pclk_div: 12
I (1453) camera: Camera configured successful
I (1487) main_task: Returned from app_main()
I (1487) camera: Taking picture...
I (1997) sdcard: JPEG saved as /sd/0_img.jpg
I (1997) camera: Picture taken! Its size was: 45764 bytes
I (6997) camera: Taking picture...
I (7348) sdcard: JPEG saved as /sd/1_img.jpg
I (7349) camera: Picture taken! Its size was: 51710 bytes
I (12349) camera: Taking picture...
I (12704) sdcard: JPEG saved as /sd/2_img.jpg
I (12705) camera: Picture taken! Its size was: 51853 bytes
I (17706) camera: Taking picture...
I (18054) sdcard: JPEG saved as /sd/3_img.jpg
I (18055) camera: Picture taken! Its size was: 51919 bytes
I (23055) camera: Taking picture...
I (23414) sdcard: JPEG saved as /sd/4_img.jpg
I (23414) camera: Picture taken! Its size was: 51809 bytes
I (28415) camera: Taking picture...
I (28768) sdcard: JPEG saved as /sd/5_img.jpg
I (28768) camera: Picture taken! Its size was: 51747 bytes
I (33771) camera: Taking picture...
I (34117) sdcard: JPEG saved as /sd/6_img.jpg
I (34117) camera: Picture taken! Its size was: 51968 bytes
```

#### 输出图像

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/9.jpg" alt="pir" width={600} height="auto" /></p>

## Arduino IDE中的FreeRTOS

FreeRTOS可以用于基于Arduino IDE的XIAO-S3构建。它与ESP-IDF类似，但只能在一个核心上运行，且没有针对ESP-IDF进行优化。

### 硬件设置

将Xiao-S3连接到[ Grove - 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html)（OLED显示屏和RTC），并将[Grove - 温湿度、压力和气体传感器（BME680）](https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-for-Arduino-BME680.html)连接到I2C总线。

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/4.jpg" alt="pir" width={600} height="auto" /></p>

### 软件设置

安装[pcf8563](https://github.com/Bill2462/PCF8563-Arduino-Library)、[U8x8lib](https://github.com/olikraus/U8g2_Arduino)和[bme680](https://github.com/Seeed-Studio/Seeed_Arduino_BME68x)库。参考[如何安装库](https://wiki.seeedstudio.com/How_to_install_Arduino_Library/)来安装Arduino的库。

```cpp
#include "time.h"
#include <WiFi.h>
#include <PCF8563.h>
#include <U8x8lib.h>
#include <Wire.h>
#include "seeed_bme680.h"

#define IIC_ADDR uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR); /* IIC 协议 */

// 用于 PCF8563 实时时钟的 I2C 通信库
PCF8563 pcf;

// OLED 显示库
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* 时钟=*/D4, /* 数据=*/D5, /* 重置=*/U8X8_PIN_NONE);  // 无重置的 OLED 显示屏

// WiFi 网络凭证
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// 用于时间同步的 NTP 服务器
const char* ntpServer = "pool.ntp.org";

// 时区偏移量（根据您所在的地区调整）
const long gmtOffset_sec = 5.5 * 60 * 60;  // 小时 * 分钟 * 秒（这里是 GMT+5:30）
const int daylightOffset_sec = 0;          // 假设没有夏令时

// 用于存储当前时间信息的全局变量
static Time nowTime;

// 任务的函数原型
void printDateAndTime(void* pvParameters);
void updateTime(void* pvParameters);
void ledBlink2Hz(void* pvParameters);
void oledDisplayUpdate(void* pvParameters);
void taskBME680(void* pvParameters);

// 设置函数（在启动时运行一次）
void setup() {

  Serial.begin(115200);  // 初始化串行通信，用于调试

  // 将内建 LED 引脚设置为输出，用于闪烁
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("正在连接到 ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  // 连接到 WiFi 网络
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  while (!bme680.init()) {
    Serial.println("bme680 初始化失败！无法找到设备！");
    delay(10000);
  }

  pcf.init();  // 初始化 PCF8563 实时时钟

  // 在设置时间之前停止时钟
  pcf.stopClock();

  // 配置通过 NTP 服务器进行时间同步
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  static struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("没有接收到时间信息... 正在等待...");
  }

  // 根据获取的时间设置 PCF8563 时钟
  pcf.setYear(timeinfo.tm_year);
  pcf.setMonth(timeinfo.tm_mon);
  pcf.setDay(timeinfo.tm_mday);
  pcf.setHour(timeinfo.tm_hour);
  pcf.setMinut(timeinfo.tm_min);
  pcf.setSecond(timeinfo.tm_sec);

  pcf.startClock();  // 设置时间后启动时钟

  Serial.println("WiFi 已连接，IP 地址为 " + WiFi.localIP());

  u8x8.begin();         // 初始化 OLED 显示屏
  u8x8.setFlipMode(1);  // 可选：旋转 OLED 显示内容

  // 创建不同功能的任务
  xTaskCreate(
    updateTime,
    "获取本地时间",
    configMINIMAL_STACK_SIZE * 2,
    (void*)1,
    tskIDLE_PRIORITY + 1,
    NULL);

  xTaskCreate(
    ledBlink2Hz,
    "任务 2",
    configMINIMAL_STACK_SIZE,
    (void*)1,
    tskIDLE_PRIORITY + 1,
    NULL);

  xTaskCreate(
    oledDisplayUpdate,
    "OLED 显示任务",
    configMINIMAL_STACK_SIZE * 2,
    (void*)1,
    tskIDLE_PRIORITY,
    NULL);

  xTaskCreate(
    printDateAndTime,
    "打印 UART",
    configMINIMAL_STACK_SIZE * 2,
    (void*)1,
    tskIDLE_PRIORITY,
    NULL);

  xTaskCreate(
    taskBME680,
    "BME680 传感器轮询",
    configMINIMAL_STACK_SIZE * 2,
    (void*)1,
    tskIDLE_PRIORITY + 1,
    NULL);
}

// 循环函数（在此案例中不执行任何操作，所有任务由任务管理处理）
void loop() {
  // 这里什么也不做，所有工作都由任务完成
}

// 作为任务运行的函数：打印当前日期和时间到串口
void printDateAndTime(void* pvParameters) {
  for (;;) {
    // 以格式化字符串（DD/MM/YY\tHH:MM:SS）打印当前时间到串口
    Serial.printf("%02d/%02d/%02d\t%02d:%02d:%02d\n",
                  nowTime.day, nowTime.month + 1, nowTime.year % 100,
                  nowTime.hour, nowTime.minute, nowTime.second);
    // 延迟 1 秒后再次读取时间
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// 作为任务运行的函数：从 PCF8563 时钟读取当前时间
void updateTime(void* pvParameters) {
  for (;;) {
    // 从 PCF8563 时钟更新全局 `nowTime` 变量的当前时间
    nowTime = pcf.getTime();
    // 延迟 0.5 秒后再次读取时间（可以根据所需的更新频率调整）
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// 作为任务运行的函数：以 2Hz 的频率闪烁内建 LED
void ledBlink2Hz(void* pvParameters) {
  bool state = true;  // LED 的初始状态（开或关）
  for (;;) {
    // 设置 LED 状态（HIGH 为开，LOW 为关）
    digitalWrite(LED_BUILTIN, (state ? HIGH : LOW));
    // 延迟 0.5 秒，以创建 2Hz 的闪烁频率（一个周期的开/关）
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 切换 LED 状态以进行下一个周期
    state = !state;
  }
}

// 作为任务运行的函数：更新 OLED 显示屏上的日期和时间
void oledDisplayUpdate(void* pvParameters) {
  for (;;) {

    // 设置第一行的字体（日期）
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    // 设置第一行的光标位置（居中）
    u8x8.setCursor(0, 0);

    char buffer1[12];  // 用于存储格式化日期字符串的缓冲区
    std::snprintf(buffer1, sizeof(buffer1), "%02d/%02d/%02d",
                  nowTime.day, nowTime.month + 1, nowTime.year % 100);
    u8x8.print(buffer1);

    // 格式化时间字符串（HH:MM:SS）并存入 buffer2
    std::snprintf(buffer1, sizeof(buffer1), "%02d:%02d:%02d",
                  nowTime.hour, nowTime.minute, nowTime.second);
    // 将格式化的时间字符串打印到 OLED 显示屏
    u8x8.print(buffer1);

    // 调整光标位置到第二行（第一行下方）
    u8x8.setCursor(0, 10);

    char buffer2[20];  // 用于存储格式化传感器数据的缓冲区

    std::snprintf(buffer2, sizeof(buffer2), "T: %.1f°C", bme680.sensor_result_value.temperature);
    u8x8.print(buffer2);
    u8x8.setCursor(0, 20);

    std::snprintf(buffer2, sizeof(buffer2), "P: %.1fkPa", bme680.sensor_result_value.pressure / 1000.0);
    u8x8.print(buffer2);

    u8x8.setCursor(0, 30);

    std::snprintf(buffer2, sizeof(buffer2), "H: %.1f%%", bme680.sensor_result_value.humidity);
    u8x8.print(buffer2);

    // std::snprintf(buffer2, sizeof(buffer2), "G: %.1f Kohms", bme680.sensor_result_value.gas / 1000.0);
    // u8x8.print(buffer2);

    vTaskDelay(100 / portTICK_PERIOD_MS);  // 每 0.1 秒更新一次（根据需要调整）
  }
}

void taskBME680(void* pvParameters) {
  for (;;) {
    if (bme680.read_sensor_data()) {
      Serial.println("读取失败 :(");
    } else {
      Serial.print("T: ");
      Serial.print(bme680.sensor_result_value.temperature, 2);
      Serial.print(" C  P: ");
      Serial.print(bme680.sensor_result_value.pressure / 1000.0, 2);
      Serial.print(" KPa  H: ");
      Serial.print(bme680.sensor_result_value.humidity, 2);
      Serial.print(" %  G: ");
      Serial.print(bme680.sensor_result_value.gas / 1000.0, 2);
      Serial.println(" Kohms");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
```

### 输出

<p style={{textAlign: 'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp32s3_freertos/5.gif" alt="pir" width={600} height="auto" /></p>

### 串行监视器输出

```shell
09/09/24	03:17:20
T: 29.01 C  P: 90.86 KPa  H: 63.41 %  G: 47.41 Kohms
09/09/24	03:17:21
T: 29.03 C  P: 90.86 KPa  H: 63.34 %  G: 47.85 Kohms
```

## Arduino FreeRTOS 与 ESP-IDF FreeRTOS 比较

| 特性                     | Arduino FreeRTOS                                                 | ESP-IDF FreeRTOS                                                                                                    |
| ------------------------ | --------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| 抽象层                   | 更高层次的抽象，适合初学者                                       | 更低层次的抽象，提供更多对经验丰富用户的控制                                                                       |
| 开发环境                 | Arduino IDE                                                     | ESP-IDF 命令行工具                                                                                                 |
| 兼容性                   | 主要兼容基于 Arduino 的开发板                                  | 兼容更广泛的 ESP32 和 ESP32-S2 开发板                                                                              |
| 特性                     | 基本的 RTOS 特性，任务创建、调度、同步                           | 完备的 RTOS 特性，任务创建、调度、同步、事件组、队列、互斥锁、信号量                                           |
| 性能                     | 由于抽象层的存在，通常性能较低                                  | 由于可以直接访问硬件和 RTOS API，性能较高                                                                          |
| 可定制性                 | 可定制性选项有限                                                | 通过配置文件和 API 提供广泛的定制选项                                                                              |
| 学习曲线                 | 对初学者来说更容易学习                                          | 对于不熟悉命令行工具和 C/C++ 的用户，学习曲线较陡                                                                  |
| 使用场景                 | 简单的物联网项目、原型开发                                      | 复杂的物联网应用、实时系统、定制硬件                                                                                |

## 故障排除

在硬件连接、软件调试或上传过程中可能会遇到一些问题。

## 技术支持与产品讨论

感谢您选择我们的产品！我们提供不同的支持渠道，以确保您在使用我们产品的过程中获得尽可能顺利的体验。我们提供多种沟通渠道，满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
