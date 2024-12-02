---
description: 在本教程中，我们将踏上探索使用 XIAO ESP32C6 开发板进行 Zigbee 应用程序开发的旅程。XIAO ESP32C6 是一款紧凑而强大的开发板，采用 ESP32-C6 芯片，提供集成的 Wi-Fi 和低功耗蓝牙 （BLE） 连接。通过利用 ESP Zigbee SDK，我们可以充分利用 XIAO ESP32C6 的潜力，并扩展其功能以包括 Zigbee 功能。
title: 在XIAO ESP32C6上快速开始Zigbee开发（使用IDF）
image: https://files.seeedstudio.com/wiki/xiaoc6_zigbee/3.jpg
slug: /cn/xiao_esp32c6_zigbee
last_update:
  date: 11/18/2024
  author: Agnes
---

# 在XIAO ESP32C6上快速开始Zigbee开发（使用IDF）

Zigbee是一种广泛采用的无线通信协议，广泛应用于家庭自动化、智能能源管理和物联网（IoT）应用。Zigbee以低功耗、可靠的数据传输和网状网络能力而闻名，是构建可扩展且高效的无线网络的绝佳选择。

在本教程中，我们将开始探索使用XIAO ESP32C6开发板进行Zigbee应用开发。XIAO ESP32C6是一款紧凑而强大的开发板，搭载了ESP32-C6芯片，具有集成的Wi-Fi和低功耗蓝牙（BLE）连接能力。通过利用ESP Zigbee SDK，我们可以充分发挥XIAO ESP32C6的潜力，并将其功能扩展到Zigbee通信。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6_zigbee/0.png" style={{width:800, height:'auto'}}/></div>

为了深入了解Zigbee开发，我们将重点关注ESP Zigbee SDK提供的两个示例程序：HA_on_off_light和HA_on_off_switch。这些示例分别展示了Zigbee灯设备和Zigbee开关设备的实现。通过详细分析这些示例的代码结构、数据模型和工作原理，我们将全面了解Zigbee设备的开发。

在本教程中，我们将覆盖以下关键内容：

1. 为XIAO ESP32C6和ESP Zigbee SDK配置开发环境。
2. 分析HA_on_off_light和HA_on_off_switch示例的代码结构和组织。
3. 理解Zigbee设备的数据模型以及它们在代码中的定义。
4. 探索Zigbee设备的初始化过程和事件处理机制。
5. 审视Zigbee设备之间的通信模式和消息交换。

通过本教程的学习，您将掌握使用XIAO ESP32C6和ESP Zigbee SDK进行Zigbee开发的基础知识和实际技能。掌握了这些知识后，您将能够轻松创建自己的Zigbee项目，并为不断壮大的Zigbee设备生态系统做出贡献。

那么，让我们开始这段充满挑战和机遇的Zigbee开发之旅，解锁这个强大无线通信协议的全部潜力吧！

## 硬件准备

在本教程中，我们将使用**两块**XIAO ESP32C6作为示例来讲解Zigbee。您可以通过以下链接购买它们。一块作为Zigbee终端设备，另一块作为Zigbee协调器。

<div class="table-center">
	<table>
		<tr>
			<th>Seeed Studio XIAO ESP32C6</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html
        ">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

## 环境准备与演示

在本节中，我们将引导您配置开发环境并上传示例中的两个程序。

### 第1步：准备ESP-IDF环境

要使用Zigbee SDK，建议使用Espressif的ESP-IDF开发框架。Espressif官方提供了详细的安装教程，您可以通过下面的按钮跳转并阅读适用于不同系统的安装流程。

<div class="get_one_now_container" style={{textAlign: 'center'}}>
    <a class="get_one_now_item" href="https://docs.espressif.com/projects/esp-idf/en/v5.1.3/esp32h2/get-started/index.html#installation">
            <strong><span><font color={'FFFFFF'} size={"4"}> 前往页面 🖱️</font></span></strong>
    </a>
</div>

如果您恰好使用的是Ubuntu系统，您需要在终端中执行以下命令：

```
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.1.3
git submodule update --init --recursive
./install.sh
source ./export.sh
cd ..
```

:::tip
Espressif建议使用**ESP-IDF v5.1.3**进行Zigbee开发，这是本教程验证过的版本。
:::

### 第2步：下载Zigbee SDK

克隆esp-zigbee-sdk：

```
git clone https://github.com/espressif/esp-zigbee-sdk.git
cd esp-zigbee-sdk/examples/esp_zigbee_HA_sample
```

### 第3步：编写HA_on_off_light程序

现在，我们准备好第一个XIAO ESP32C6。我们将为它编写并刷写Zigbee终端设备程序。

```
cd HA_on_off_light/main
```

由于示例程序中使用GPIO8作为LED，但XIAO上的LED是GPIO15，因此我们需要对示例程序进行一些简单的修改，以显示效果。

需要修改的程序位于`esp_zb_light.c`的主文件中。修改后的完整代码如下：

```cpp
#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"

#if !defined ZB_ED_ROLE
#error 在idf.py menuconfig中定义ZB_ED_ROLE以编译灯光（终端设备）源代码。
#endif

static const char *TAG = "ESP_ZB_ON_OFF_LIGHT";
#define BLINK_GPIO 15
/********************* 定义函数 **************************/
static esp_err_t deferred_driver_init(void)
{
    light_driver_init(LIGHT_DEFAULT_OFF);
    return ESP_OK;
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "示例配置为闪烁GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* 设置GPIO为推挽输出模式 */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "启动Zigbee调试失败");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "初始化Zigbee栈");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "延迟驱动程序初始化 %s", deferred_driver_init() ? "失败" : "成功");
            ESP_LOGI(TAG, "设备已在%s工厂重置模式启动", esp_zb_bdb_is_factory_new() ? "" : "非");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "开始网络引导");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "设备已重启");
            }
        } else {
            /* 调试失败 */
            ESP_LOGW(TAG, "初始化Zigbee栈失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "成功加入网络 (扩展PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 通道:%d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "网络引导失败 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "空消息");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "接收到的消息: 错误状态(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "接收到的消息: 端点(%d), 聚合群组(0x%x), 属性(0x%x), 数据大小(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "灯光设置为 %s", light_state ? "开" : "关");
                gpio_set_level(BLINK_GPIO, light_state);
                // light_driver_set_power(light_state);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "接收到Zigbee动作(0x%x)回调", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* 初始化Zigbee栈 */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    configure_led();
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
```

请保存此代码。

### 第4步：烧录 HA_on_off_light 程序

现在将您的 XIAO ESP32C6 开发板连接到计算机，并检查该开发板在哪个串口下可见。

串口的命名模式通常是：`/dev/tty`。通常，您的计算机可能有多个以 `tty` 开头的端口。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/11.png" style={{width:800, height:'auto'}}/></div>

确定端口也很简单，您可以使用查询命令查看默认情况下未连接 XIAO 时系统有哪些串口。

```
ls /dev/tty*
```

然后，在将 XIAO 连接到计算机后再次查询，新增的串口名称就是 XIAO 的端口号。

设置目标设备。

```
idf.py set-target esp32c6
```

通过运行以下命令来构建项目：

```
idf.py build
```

要烧录之前步骤中为 ESP32 构建的二进制文件，您需要运行以下命令：

```
idf.py -p PORT flash
```

将 `PORT` 替换为您的 XIAO ESP32C6 的 USB 端口名称。如果 **PORT** 未定义，`idf.py` 会尝试自动连接可用的 USB 端口。根据我们在第一步查询到的设备端口号，对于我来说，我应该输入以下命令来烧录程序。


```
idf.py -p /dev/ttyACM0 flash
```

如果烧录过程中没有问题，XIAO ESP32C6 会重启并启动 Zigbee 灯光应用程序。

### 第5步：烧录 HA_on_off_switch 程序

类似地，我们取出另一个 XIAO ESP32C6，并为其上传开关程序。步骤类似。

```
cd ../HA_on_off_switch
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash
```

如果一切顺利，接下来您可以使用 SWITCH 程序的 XIAO 上的 **BOOT** 按钮来控制 LIGHT 程序的 XIAO 上的 LED 灯开关。

<div class="table-center">
<iframe width="800" height="350" src="https://files.seeedstudio.com/wiki/xiaoc6_zigbee/2.mp4?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

## HA_on_off_light 和 HA_on_off_switch 程序结构

该文件夹包含演示 Zigbee HA 标准设备的示例

* `HA_on_off_light` 是一个标准的 HA 开关灯泡示例，演示了 Zigbee 终端设备。

* `HA_on_off_switch` 是一个标准的 HA 开关开关示例，演示了 Zigbee 协调器角色。它提供一个开关，用于控制 Zigbee HA 开关灯。

在本教程中，我们将深入分析 ESP Zigbee SDK 提供的两个示例程序：`HA_on_off_light` 和 `HA_on_off_switch`。通过分析这些示例的代码结构和组织方式，我们将全面理解如何开发 Zigbee 设备应用。


```
- esp_zigbee_HA_sample/
    - HA_on_off_light/
        - main/
          - CMakeLists.txt
          - esp_zb_light.c
          - esp_zb_light.h
          - idf_component.yml
        - CMakeLists.txt
        - partitions.csv
        - sdkconfig.defaults
        ...
    - HA_on_off_switch/
        - main/
          - CMakeLists.txt
          - esp_zb_switch.c
          - esp_zb_switch.h
          - idf_component.yml
        - CMakeLists.txt
        - partitions.csv
        - sdkconfig.defaults
        ...
```

1. esp_zigbee_HA_sample/: 该目录包含 ESP Zigbee SDK 提供的家庭自动化 (HA) 示例项目。

2. HA_on_off_light/: 该子目录表示“开/关灯”示例项目。
   - main/: 该目录包含“开/关灯”示例的主要源代码文件。
     - CMakeLists.txt: 该文件由 CMake 构建系统使用，用于指定“开/关灯”示例的源代码文件和依赖项。
     - esp_zb_light.c: 该文件包含 Zigbee 灯设备的主要实现代码，包括初始化、事件处理和与其他 Zigbee 设备的通信。
     - esp_zb_light.h: 该头文件包含 Zigbee 灯设备所需的函数声明、常量和数据结构。
     - idf_component.yml: 该文件是 ESP-IDF 组件配置文件，指定了“开/关灯”示例的组件依赖和构建设置。
   - CMakeLists.txt: 该文件是“开/关灯”示例项目的顶级 CMakeLists 文件，包含了必要的配置和构建目标。
   - partitions.csv: 该文件定义了“开/关灯”示例的分区表，指定了各个分区（如引导加载程序、应用程序和存储）的内存布局和大小。
   - sdkconfig.defaults: 该文件包含“开/关灯”示例项目的默认配置设置，用户可以覆盖这些设置。

3. HA_on_off_switch/: 该子目录表示“开/关开关”示例项目。
   - main/: 该目录包含“开/关开关”示例的主要源代码文件。
     - CMakeLists.txt: 类似于“开/关灯”示例，该文件由 CMake 构建系统使用，用于指定“开/关开关”示例的源代码文件和依赖项。
     - esp_zb_switch.c: 该文件包含 Zigbee 开关设备的主要实现代码，包括初始化、事件处理和与其他 Zigbee 设备的通信。
     - esp_zb_switch.h: 该头文件包含 Zigbee 开关设备所需的函数声明、常量和数据结构。
     - idf_component.yml: 该文件是“开/关开关”示例的 ESP-IDF 组件配置文件。
   - CMakeLists.txt: 这是“开/关开关”示例项目的顶级 CMakeLists 文件。
   - partitions.csv: 该文件定义了“开/关开关”示例的分区表。
   - sdkconfig.defaults: 该文件包含“开/关开关”示例项目的默认配置设置。

这些文件协同工作，提供了一个完整的 Zigbee 设备示例实现，使用 ESP Zigbee SDK。`.c` 和 `.h` 文件包含实际的代码实现，而 CMakeLists.txt、partitions.csv 和 sdkconfig.defaults 文件则用于构建配置和内存分区。

## Zigbee 终端设备和 Zigbee 数据模型

在本教程中，我们将探讨 Zigbee HA 开/关灯示例代码如何基于 Zigbee 数据模型进行结构化。通过理解代码和数据模型之间的关系，您将深入了解如何根据特定要求解释和修改代码。

在深入分析代码之前，首先要掌握 Zigbee 数据模型的关键概念：

- **节点 (Node)**：一个节点表示一个单独的基于 ESP32-H2 的产品和 Zigbee 网络中的一个网络节点。一个节点可以有多个端点。
  
- **端点 (Endpoint)**：一个端点由 1 到 240 之间的一个数字标识，定义了一个在 Zigbee 节点上运行的应用程序。一个节点可以有多个端点，每个端点服务于不同的目的或代表一个独立的设备。

- **集群 (Cluster)**：一个集群由一个 16 位数字标识，是一个应用对象，定义了与端点相关的功能和数据。集群包含属性和命令。

- **属性 (Attribute)**：一个属性由一个 16 位数字标识，表示集群内的当前状态或物理量。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6_zigbee/datamodel.png" style={{width:800, height:'auto'}}/></div>

现在，让我们来看一下 HA 开/关灯示例代码，看看它是如何映射到 Zigbee 数据模型的。

1. 创建端点

在示例代码中，`esp_zb_on_off_light_ep_create()` 函数用于创建一个 HA 开/关灯端点。该函数定义了端点 ID、设备 ID 和相关的集群。

```cpp
static void esp_zb_task(void *pvParameters)
{
    /* 初始化 Zigbee 堆栈 */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}
```

2. 注册设备

在创建端点后，调用 `esp_zb_device_register()` 函数将 Zigbee 设备与创建的端点注册。

```cpp
esp_zb_device_register(esp_zb_on_off_light_ep);
```

3. 属性回调

示例代码使用 `esp_zb_core_action_handler_register()` 注册一个属性变化回调。该回调在某些属性被修改时触发，允许您根据应用程序逻辑处理属性变化。

```cpp
esp_zb_core_action_handler_register(zb_action_handler);
```

在 `zb_action_handler` 函数中，您可以实现所需的行为，例如，当开/关属性变化时控制 LED 灯。

4. Zigbee 堆栈配置与启动

示例代码使用 `ESP_ZB_ZED_CONFIG()` 配置 Zigbee 终端设备，并使用 `esp_zb_init()` 初始化 Zigbee 堆栈。然后，堆栈通过 `esp_zb_start()` 启动，并由 `esp_zb_main_loop_iteration()` 处理主循环。


```cpp
esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
esp_zb_init(&zb_nwk_cfg);
...
ESP_ERROR_CHECK(esp_zb_start(false));
esp_zb_main_loop_iteration();
```

`esp_zb_app_signal_handler` 函数负责处理来自 Zigbee 应用层的各种信号。

```cpp
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "初始化 Zigbee 堆栈");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "延迟驱动初始化 %s", deferred_driver_init() ? "失败" : "成功");
            ESP_LOGI(TAG, "设备以 %s 出厂重置模式启动", esp_zb_bdb_is_factory_new() ? "" : "非");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "启动网络引导");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "设备已重启");
            }
        } else {
            /* 配置失败 */
            ESP_LOGW(TAG, "Zigbee 堆栈初始化失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "成功加入网络 (扩展 PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 通道: %d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "网络引导失败 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO 信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
```

1. 首先，函数从传入的 `esp_zb_app_signal_t` 结构体中获取信号类型 `sig_type` 和错误状态 `err_status`。

2. 然后，它使用 `switch` 语句根据信号类型执行不同的操作：

   - `ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP`：此信号表示跳过Zigbee栈的启动。在这种情况下，我们初始化Zigbee栈，然后调用 `esp_zb_bdb_start_top_level_commissioning` 函数以 `ESP_ZB_BDB_MODE_INITIALIZATION` 模式启动顶层调试过程。

   - `ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START` 和 `ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT`：这些信号表示设备的首次启动或重启。如果错误状态为 `ESP_OK`，我们执行一些初始化任务，如延迟驱动程序初始化。然后，我们检查设备是否处于全新状态。如果是，我们开始网络引导过程；否则，我们输出设备已重启的消息。如果错误状态不是 `ESP_OK`，我们输出Zigbee栈初始化失败的消息。

   - `ESP_ZB_BDB_SIGNAL_STEERING`：此信号表示网络引导过程的结果。如果错误状态为 `ESP_OK`，则表示设备成功加入网络。在这种情况下，我们输出一些网络信息，如PAN ID、频道号和短地址。如果错误状态不是 `ESP_OK`，则表示网络引导失败，我们输出错误消息。然后，我们使用 `esp_zb_scheduler_alarm` 函数设置定时器，在1秒后重新启动网络引导过程。

   - 其他信号：我们仅输出信号名称、类型和错误状态。

该函数的目的是根据不同的Zigbee应用层信号执行相应的操作。它是Zigbee应用程序的核心部分之一，处理设备启动、初始化和网络加入等关键过程。

## Zigbee协调器

对于Zigbee协调器设备，其初始化和RTOS任务与终端设备类似，唯一不同的是在RTOS任务中，注册回调函数的步骤较少。

因此，对于Zigbee协调器，最关键的部分是搜索并匹配相应的设备，并向设备发出控制命令。


```cpp
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "初始化Zigbee栈");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "延迟驱动初始化 %s", deferred_driver_init() ? "失败" : "成功");
            ESP_LOGI(TAG, "设备以%s出厂重置模式启动", esp_zb_bdb_is_factory_new() ? "" : "非");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "开始网络组建");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "设备已重启");
            }
        } else {
            ESP_LOGE(TAG, "Zigbee栈初始化失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "网络成功组建 (扩展PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 频道: %d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "重新启动网络组建 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "网络引导已启动");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "新设备已委托或重新加入 (短地址: 0x%04hx)", dev_annce_params->device_short_addr);
        esp_zb_zdo_match_desc_req_param_t  cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        /* 在设备加入网络后寻找可调光的彩色灯 */
        esp_zb_zdo_find_color_dimmable_light(&cmd_req, user_find_cb, NULL);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "网络(0x%04hx)开放 %d 秒", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "网络(0x%04hx)已关闭，不允许设备加入", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
```

让我们逐一分析不同的信号类型及其功能：

1. `ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP`:
   - 该信号表示应跳过Zigbee栈的初始化。
   - 它记录一条消息，表示正在初始化Zigbee栈。
   - 它以 `ESP_ZB_BDB_MODE_INITIALIZATION` 模式启动顶层调试过程。

2. `ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START` 和 `ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT`:
   - 这些信号表示设备首次启动或已重启。
   - 如果错误状态为 `ESP_OK`，它会记录关于延迟驱动初始化状态的消息，并指示设备是否以出厂重置模式启动。
   - 如果设备处于出厂新模式，它会通过调用 `esp_zb_bdb_start_top_level_commissioning` 以 `ESP_ZB_BDB_MODE_NETWORK_FORMATION` 模式开始网络组建过程。
   - 如果设备不在出厂新模式下，它会记录一条消息，表示设备已重启。
   - 如果错误状态不是 `ESP_OK`，它会记录一条错误消息。

3. `ESP_ZB_BDB_SIGNAL_FORMATION`:
   - 该信号表示网络组建过程的状态。
   - 如果错误状态为 `ESP_OK`，它会获取扩展的PAN ID，记录已组建网络的信息（PAN ID、频道、短地址），并通过调用 `esp_zb_bdb_start_top_level_commissioning` 以 `ESP_ZB_BDB_MODE_NETWORK_STEERING` 模式开始网络引导过程。
   - 如果错误状态不是 `ESP_OK`，它会记录重新启动网络组建的消息，并安排一个定时器，在1000毫秒后调用 `bdb_start_top_level_commissioning_cb`。

4. `ESP_ZB_BDB_SIGNAL_STEERING`:
   - 该信号表示网络引导过程的状态。
   - 如果错误状态为 `ESP_OK`，它会记录一条消息，表示网络引导已开始。

5. `ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE`:
   - 该信号在新设备被委托或重新加入网络时触发。
   - 它获取设备公告参数，并记录新设备的短地址。
   - 它准备一个匹配描述符请求（`esp_zb_zdo_match_desc_req_param_t`），目标地址和感兴趣的地址都设置为新设备的短地址。
   - 它调用 `esp_zb_zdo_find_color_dimmable_light` 来查找可调光的彩色灯设备，并指定 `user_find_cb` 作为回调函数。

6. `ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS`:
   - 该信号表示网络的允许加入状态。
   - 如果错误状态为 `ESP_OK`，它会记录一条消息，表示网络是否开放加入以及开放的持续时间。如果网络关闭，它会记录一条警告消息。

7. 默认情况:
   - 对于任何其他信号类型，它会记录一条包含信号类型和错误状态的通用消息。

总体来说，这段代码处理了各种Zigbee相关的事件，并执行了诸如初始化Zigbee栈、组建网络、引导网络、处理设备公告以及查找可调光的彩色灯设备等操作。

剩下的示例代码处理了按键稳定化和按键中断的逻辑。如果您感兴趣，可以自行阅读并理解它。

## 故障排除

### Q1: 可以在Arduino上编程Zigbee吗？

理论上是可以的，因为Espressif在Zigbee方面提供了很好的库支持，并提供了类似的[Zigbee示例程序](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Zigbee/Zigbee_Light_Bulb)。

然而，当我们实际使用这些示例时，我们发现XIAO无法正常工作，并且会遇到不断重启的问题。我们已经[向Espressif提交了问题](https://github.com/espressif/arduino-esp32/issues/9716#issuecomment-2138491439)，等待官方处理和解决方案。

### Q2: 在使用ESP_ZB_ON_OFF_LIGHT时仍然遇到问题：网络引导未能成功匹配另一个XIAO。

首先，请排查您正在使用的ESP-IDF版本，确保您使用的是**ESP-IDF v5.1.3**来编译Zigbee示例应用程序，如果不是，请更改IDF版本。

接下来，尝试重新插拔设备。您可以先启动上传了**HA_on_off_switch**程序的设备，然后再启动上传了**HA_on_off_light**程序的设备。

如果仍然无法解决，请擦除所有闪存并重新上传程序。

```
idf.py erase_flash flash monitor
```

如果上述方法都无效，请向Espressif提交问题。

### Q3: 如果我想在成功配对后匹配新设备，该怎么办？

直接使用闪存命令上传程序不会擦除闪存中保存的历史配对记录。请使用以下命令重新上传程序以匹配新设备。

```
idf.py erase_flash flash monitor
```

## 资源

- **[Espressif 官方文档]** [使用 ESP Zigbee SDK 开发](https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/esp32/developing.html)
- **[GITHUB]** [Zigbee SDK 仓库](https://github.com/espressif/esp-zigbee-sdk)

## 技术支持与产品讨论

感谢您选择我们的产品！我们在这里为您提供各种支持，确保您使用我们的产品时体验顺畅。我们提供了多个沟通渠道，以满足不同的偏好和需求。

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













