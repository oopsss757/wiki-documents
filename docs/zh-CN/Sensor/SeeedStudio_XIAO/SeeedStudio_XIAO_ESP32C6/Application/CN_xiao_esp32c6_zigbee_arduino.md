---
description: 在本教程中，我们将踏上探索使用 XIAO ESP32C6 开发板进行 Zigbee 应用程序开发的旅程。XIAO ESP32C6 是一款紧凑而强大的开发板，采用 ESP32-C6 芯片，提供集成的 Wi-Fi 和低功耗蓝牙 （BLE） 连接。通过利用 ESP Zigbee SDK，我们可以充分利用 XIAO ESP32C6 的潜力，并扩展其功能以包括 Zigbee 功能。
title: 在 XIAO ESP32C6 上使用 Zigbee 快速入门（通过 Arduino）
image: https://files.seeedstudio.com/wiki/xiaoc6_zigbee/3.jpg
slug: /cn/xiao_esp32c6_zigbee_arduino
last_update:
  date: 07/30/2024
  author: Allen
---

# 在 XIAO ESP32C6 上使用 Zigbee 快速入门（通过 Arduino）

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6_zigbee/0.png" style={{width:800, height:'auto'}}/></div>

Zigbee 是一种广泛采用的无线通信协议，广泛应用于家庭自动化、智能能源管理和物联网（IoT）应用。Zigbee 以其低功耗、可靠的数据传输和网状网络功能而闻名，是构建可扩展和高效无线网络的理想选择。

在本教程中，我们将涵盖以下关键内容：

1. **开发环境**：通过 Arduino 设置 XIAO ESP32C6 和 ESP Zigbee SDK 的开发环境。
2. **代码结构**：分析 Zigbee_Light_Buld 和 Zigbee_Light_Switch 示例的代码结构和组织。
3. **Zigbee 设备数据模型**：理解 Zigbee 设备数据模型及其在代码中的定义。
4. **Zigbee 处理机制**：探索 Zigbee 设备的初始化过程和事件处理机制。
5. **通信模式**：检查 Zigbee 设备之间的通信模式和消息交换。

让我们开始这段充满挑战的 Zigbee 开发旅程，充分发挥 XIAO ESP32C6 的强大无线通信协议的潜力吧！

## 第1部分：硬件准备

在本教程中，我们将使用 **两台** XIAO ESP32C6 作为示例来讲解 Zigbee。您可以通过下面的链接进行购买。一个作为 **Zigbee 终端设备**，另一个作为 **Zigbee 协调器**。

<div class="table-center">
	<table>
		<tr>
			<th>Seeed Studio XIAO ESP32C6</th>
			<th>Grove Shield For XIAO</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/x/i/xiao_-preview-25.png" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Shield-for-Seeeduino-XIAO-p-4621.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
		<tr>
			<th>Seeed Studio Grove 红色 LED</th>
			<th>Seeed Studio Grove 按钮</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/h/t/httpsstatics3.seeedstudio.comseeedfile2018-09bazaar939479_1040300054.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/p/e/perspectiive.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Red-LED.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Button.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> 立即购买 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>

## 第2部分：环境准备

**步骤 1.** 启动 Arduino 应用程序。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/seeed_logo/arduino.jpg" style={{width:800, height:'auto'}}/></div>

<div class="download_arduino_container" style={{textAlign: 'center'}}>
    <a class="download_arduino_item" href="https://www.arduino.cc/en/software"><strong><span><font color={'FFFFFF'} size={"4"}>下载 Arduino IDE</font></span></strong>
    </a>
</div>

**步骤 2.** 选择您的开发板型号并将其添加到 Arduino IDE 中。

- 如果您想使用 **Seeed Studio XIAO ESP32C6** 进行后续操作，请参考 **[本教程](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)** 完成添加。

## 第3部分：程序结构

### Zigbee 智能灯泡

在本节中，我们将探讨 Zigbee HA 开关灯示例代码是如何基于 Zigbee 数据模型进行结构化的。通过理解代码与数据模型之间的关系，您将了解如何根据特定需求解读和修改代码。

在深入了解代码之前，了解 Zigbee 数据模型的关键概念至关重要：

- **节点（Node）**：一个节点代表基于 ESP32-H2 的单个产品和 Zigbee 网络中的一个网络节点。一个节点可以有多个端点。
  
- **端点（Endpoint）**：端点由 1 到 240 之间的数字表示，定义了运行在 Zigbee 节点上的应用程序。一个节点可以有多个端点，每个端点可能有不同的用途或代表不同的设备。
  
- **集群（Cluster）**：集群由一个 16 位数字标识，是定义与端点相关的功能和数据的应用对象。集群包含属性和命令。
  
- **属性（Attribute）**：属性由一个 16 位数字标识，表示集群中的当前状态或物理量。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6_zigbee/datamodel.png" style={{width:800, height:'auto'}}/></div>

现在，让我们查看 HA 开关灯的示例代码，看看它是如何映射到 Zigbee 数据模型的。


1. 创建端点

在示例代码中，`esp_zb_on_off_light_ep_create()` 函数用于创建一个 HA 开关灯端点。此函数定义了端点 ID、设备 ID 和关联的集群。

```cpp
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    // 在创建连接到新协调器之前擦除 NVRAM
    //esp_zb_nvram_erase_at_start(true); // 如果您要连接到新的协调器，请取消注释此行以擦除 NVRAM 数据

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}
```

2. 注册设备

在创建端点之后，调用 `esp_zb_device_register()` 函数将 Zigbee 设备与创建的端点注册。

```cpp
esp_zb_device_register(esp_zb_on_off_light_ep);
```

3. 属性回调

示例代码使用 `esp_zb_core_action_handler_register()` 注册了一个属性变化的回调。当某些属性被修改时，此回调将被调用，允许您根据应用程序逻辑处理属性变化。

```cpp
esp_zb_core_action_handler_register(zb_action_handler);
```

在 `zb_action_handler` 函数中，您可以实现当开关属性变化时的期望行为，例如控制 LED 灯。

4. Zigbee 栈配置和启动

示例代码使用 `ESP_ZB_ZED_CONFIG()` 配置 Zigbee 终端设备，并通过 `esp_zb_init()` 初始化 Zigbee 栈。然后，使用 `esp_zb_start()` 启动栈，并通过 `esp_zb_main_loop_iteration()` 处理主循环。


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
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        log_i("Zigbee 栈初始化完成");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            log_i("开始网络引导");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* 委托失败 */
            log_w("Zigbee 栈初始化失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            log_i("成功加入网络 (扩展 PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 信道: %d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            log_i("网络引导未成功 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        log_i("ZDO 信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
```

1. 首先，函数从传入的 `esp_zb_app_signal_t` 结构体中获取信号类型 `sig_type` 和错误状态 `err_status`。

2. 然后，使用 `switch` 语句根据信号类型执行不同的操作：

   - `ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP`：该信号表示跳过 Zigbee 栈的启动。在这种情况下，我们初始化 Zigbee 栈，然后调用 `esp_zb_bdb_start_top_level_commissioning` 函数，使用 `ESP_ZB_BDB_MODE_INITIALIZATION` 模式开始顶层初始化过程。

   - `ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START` 和 `ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT`：这些信号表示设备的首次启动或重启。如果错误状态为 `ESP_OK`，我们执行一些初始化任务，例如延迟驱动程序初始化。然后，我们检查设备是否处于出厂状态。如果是，则开始网络引导过程；否则，我们输出设备重启的消息。如果错误状态不是 `ESP_OK`，则输出 Zigbee 栈初始化失败的消息。

   - `ESP_ZB_BDB_SIGNAL_STEERING`：该信号表示网络引导过程的结果。如果错误状态为 `ESP_OK`，则表示设备成功加入网络。在这种情况下，我们输出一些网络信息，如 PAN ID、信道号和短地址。如果错误状态不是 `ESP_OK`，则表示网络引导失败，并输出错误消息。然后，我们使用 `esp_zb_scheduler_alarm` 函数设置一个定时器，在 1 秒后重新启动网络引导过程。

   - 其他信号：我们简单地输出信号名称、类型和错误状态。

该函数的目的是根据不同的 Zigbee 应用层信号执行适当的操作。它是 Zigbee 应用程序的核心部分之一，处理设备启动、初始化和网络加入等关键过程。

### Zigbee 开关

对于 Zigbee 协调器设备（开关），其初始化和 RTOS 任务与终端设备相似，唯一不同的是，在 RTOS 任务中，少了一步注册回调函数的过程。

因此，对于 Zigbee 协调器，最关键的部分是搜索和匹配相应的设备，并向设备发出控制命令。

```cpp
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        log_i("Zigbee 栈已初始化");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            log_i("开始网络组建");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            log_e("Zigbee 栈初始化失败（状态: %s）", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            log_i("网络组建成功（扩展 PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 信道: %d, 短地址: 0x%04hx）",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            log_i("重新启动网络组建（状态: %s）", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            log_i("网络引导已开始");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        log_i("新设备已加入或重新加入（短地址: 0x%04hx）", dev_annce_params->device_short_addr);
        esp_zb_zdo_zdo_match_desc_req_param_t  cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        esp_zb_zdo_find_on_off_light(&cmd_req, user_find_cb, NULL);
        break;
    default:
        log_i("ZDO 信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
```

让我们逐一讲解不同的信号类型及其功能：

1. `ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP`：
   - 此信号表示应跳过 Zigbee 栈的初始化。
   - 它记录一条消息，表示 Zigbee 栈已初始化。
   - 它启动顶级委托过程，模式设置为 `ESP_ZB_BDB_MODE_INITIALIZATION`。

2. `ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START` 和 `ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT`：
   - 这些信号表示设备已首次启动或已重新启动。
   - 如果错误状态为 `ESP_OK`，则记录关于延迟驱动程序初始化状态的消息，并检查设备是否处于出厂重置模式。
   - 如果设备处于出厂新模式，则通过调用 `esp_zb_bdb_start_top_level_commissioning`，并将模式设置为 `ESP_ZB_BDB_MODE_NETWORK_FORMATION` 来启动网络组建过程。
   - 如果设备不是出厂新模式，则记录一条消息，表示设备已重新启动。
   - 如果错误状态不是 `ESP_OK`，则记录错误消息。

3. `ESP_ZB_BDB_SIGNAL_FORMATION`：
   - 此信号表示网络组建过程的状态。
   - 如果错误状态为 `ESP_OK`，则获取扩展的 PAN ID，记录关于已组建网络的信息（包括 PAN ID、信道和短地址），并通过调用 `esp_zb_bdb_start_top_level_commissioning` 来启动网络引导过程，模式设置为 `ESP_ZB_BDB_MODE_NETWORK_STEERING`。
   - 如果错误状态不是 `ESP_OK`，则记录一条消息，表示重新启动网络组建，并调度一个警报，在延迟 1000 毫秒后调用 `bdb_start_top_level_commissioning_cb`。

4. `ESP_ZB_BDB_SIGNAL_STEERING`：
   - 此信号表示网络引导过程的状态。
   - 如果错误状态为 `ESP_OK`，则记录一条消息，表示网络引导已开始。

5. `ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE`：
   - 当新设备被委托或重新加入网络时，会触发此信号。
   - 它获取设备公告参数，并记录包含新设备短地址的消息。
   - 它准备一个匹配描述符请求（`esp_zb_zdo_match_desc_req_param_t`），目标地址和感兴趣地址设置为新设备的短地址。
   - 它调用 `esp_zb_zdo_find_color_dimmable_light` 来查找一个可调色温的灯设备，并指定 `user_find_cb` 作为回调函数。

6. `ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS`：
   - 此信号表示网络的允许加入状态。
   - 如果错误状态为 `ESP_OK`，则记录一条消息，表示网络是否开放加入，以及开放的时长。如果网络关闭，则记录一条警告消息。

7. 默认情况：
   - 对于任何其他信号类型，它会记录一个包含信号类型和错误状态的一般消息。

总的来说，这段代码处理了各种 Zigbee 相关的事件，并执行了初始化 Zigbee 栈、组建网络、引导网络、处理设备公告以及查找可调色温灯设备等操作。

其余的示例代码涉及按键稳定和按键中断的逻辑。如果你感兴趣，可以自行阅读并理解。

## 第四部分：Zigbee 灯效

本部分我们将连接设备并上传代码到两块 XIAO ESP32C6，看看效果。

### 步骤 1：硬件连接

<div class="table-center">
	<table>
		<tr>
			<th>XIAO ESP32C6 通过 D9 连接到灯泡</th>
			<th>XIAO ESP32C6 通过 D0 连接到开关</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/light.jpg" style={{width:500, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/switch.jpg" style={{width:500, height:'auto'}}/></div></td>
		</tr>
	</table>
</div>

### 步骤 2：设置 Arduino 串口为调试级别

我们需要将串口设置为调试级别，以便稍后查看 Zigbee 灯泡和开关的串口消息。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/1.png" style={{width:800, height:'auto'}}/></div>

### 步骤 3：上传灯泡代码

```cpp

#ifndef ZIGBEE_MODE_ED
#error "没有在 Tools->Zigbee 模式中选择 Zigbee 终端设备模式"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define LED_PIN D10

/* 默认终端设备配置 */
#define ESP_ZB_ZED_CONFIG()                                     \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg = {                                            \
            .zed_cfg = {                                        \
                .ed_timeout = ED_AGING_TIMEOUT,                 \
                .keep_alive = ED_KEEP_ALIVE,                    \
            },                                                  \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,  \
    }

/* Zigbee 配置 */
#define INSTALLCODE_POLICY_ENABLE       false    /* 启用安装码策略以增强安全性 */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 毫秒 */
#define HA_ESP_LIGHT_ENDPOINT           10    /* esp 灯泡设备的端点，用于处理灯光控制命令 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee 主通道掩码，示例中使用的通道 */

/********************* Zigbee 函数 **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        log_i("Zigbee 栈已初始化");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            log_i("开始网络引导");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* 委托失败 */
            log_w("Zigbee 栈初始化失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            log_i("成功加入网络 (扩展 PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 通道:%d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            log_i("网络引导失败 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        log_i("ZDO 信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        log_w("接收到 Zigbee 动作回调 (0x%x)", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    // 在连接到新的协调器之前擦除 NVRAM
    //esp_zb_nvram_erase_at_start(true); // 如果你要连接到新协调器，请取消注释此行以擦除 NVRAM 数据

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

/* 处理灯光属性 */

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    if(!message){
      log_e("空消息");
    }
    if(message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS){
      log_e("接收到的消息: 错误状态(%d)", message->info.status);
    }

    log_i("接收到的消息: 端点(%d), 集群(0x%x), 属性(0x%x), 数据大小(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                log_i("灯光设置为 %s", light_state ? "开启" : "关闭");
                // 在此处控制 LED 开关
                if(light_state == 1){
                  digitalWrite(LED_PIN, HIGH);
                }else{
                  digitalWrite(LED_PIN, LOW);
                }

            }
        }
    }
    return ret;
}

/********************* Arduino 函数 **************************/
void setup() {
    // 初始化 Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // 初始化 LED 引脚
    pinMode(LED_PIN, OUTPUT);
    
    // 关闭 LED
    digitalWrite(LED_PIN, LOW);

    // 启动 Zigbee 任务
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
    // 空的，Zigbee 在任务中运行
}
```

### 步骤 4：上传 Light Switch 代码

```cpp

#ifndef ZIGBEE_MODE_ZCZR
#error "没有在工具->Zigbee模式中选择Zigbee协调器模式"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

/* 开关配置 */
#define GPIO_INPUT_IO_TOGGLE_SWITCH  GPIO_NUM_9
#define PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

typedef enum {
    SWITCH_ON_CONTROL,
    SWITCH_OFF_CONTROL,
    SWITCH_ONOFF_TOGGLE_CONTROL,
    SWITCH_LEVEL_UP_CONTROL,
    SWITCH_LEVEL_DOWN_CONTROL,
    SWITCH_LEVEL_CYCLE_CONTROL,
    SWITCH_COLOR_CONTROL,
} switch_func_t;

typedef struct {
    uint8_t pin;
    switch_func_t func;
} switch_func_pair_t;

typedef enum {
    SWITCH_IDLE,
    SWITCH_PRESS_ARMED,
    SWITCH_PRESS_DETECTED,
    SWITCH_PRESSED,
    SWITCH_RELEASE_DETECTED,
} switch_state_t;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

/* 默认协调器配置 */
#define ESP_ZB_ZC_CONFIG()                                      \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,          \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg = {                                            \
            .zczr_cfg = {                                       \
                .max_children = MAX_CHILDREN,                   \
            },                                                  \
        }                                                       \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }

typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

/* Zigbee 配置 */
#define MAX_CHILDREN                    10          /* 最大连接设备数 */
#define INSTALLCODE_POLICY_ENABLE       false    /* 启用安装代码策略以增加安全性 */
#define HA_ONOFF_SWITCH_ENDPOINT        1          /* esp 灯开关设备端点 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* 使用的Zigbee主频道掩码 */

/********************* 定义函数 **************************/
static void esp_zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* 实现灯开关的切换功能 */
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
        log_i("发送'开关切换'命令");
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        log_i("绑定成功！");
        if (user_ctx) {
            light_bulb_device_params_t *light = (light_bulb_device_params_t *)user_ctx;
            log_i("来自地址(0x%x)的灯，端点(%d)", light->short_addr, light->endpoint);
            free(light);
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        log_i("找到灯具");
        esp_zb_zdo_bind_req_param_t bind_req;
        light_bulb_device_params_t *light = (light_bulb_device_params_t *)malloc(sizeof(light_bulb_device_params_t));
        light->endpoint = endpoint;
        light->short_addr = addr;
        esp_zb_ieee_address_by_short(light->short_addr, light->ieee_addr);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp = HA_ONOFF_SWITCH_ENDPOINT;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, light->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp = endpoint;
        bind_req.req_dst_addr = esp_zb_get_short_address();
        log_i("尝试绑定开/关");
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)light);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        log_i("Zigbee 栈初始化完成");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            log_i("开始网络组建");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            log_e("初始化 Zigbee 栈失败 (状态: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            log_i("网络组建成功 (扩展 PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, 通道:%d, 短地址: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            log_i("重新启动网络组建 (状态: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            log_i("网络引导已启动");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        log_i("新设备已加入或重新加入网络 (短地址: 0x%04hx)", dev_annce_params->device_short_addr);
        esp_zb_zdo_zdp_match_desc_req_param_t  cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        esp_zb_zdo_find_on_off_light(&cmd_req, user_find_cb, NULL);
        break;
    default:
        log_i("ZDO 信号: %s (0x%x), 状态: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_switch_ep = esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);
    esp_zb_device_register(esp_zb_on_off_switch_ep);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

/********************* GPIO函数 **************************/
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t *)arg, NULL);
}

static void switch_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < PAIR_SIZE(button_func_pair); ++i) {
        if (enabled) {
            enableInterrupt((button_func_pair[i]).pin);
        } else {
            disableInterrupt((button_func_pair[i]).pin);
        }
    }
}

/********************* Arduino函数 **************************/
void setup() {
    // 初始化Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // 初始化按钮开关 
    for (int i = 0; i < PAIR_SIZE(button_func_pair); i++) {
        pinMode(button_func_pair[i].pin, INPUT_PULLUP);
        /* 创建一个队列来处理来自ISR的GPIO事件 */
        gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
        if ( gpio_evt_queue == 0) {
            log_e("队列未创建，不能使用");
            while(1);
        }
        attachInterruptArg(button_func_pair[i].pin, gpio_isr_handler, (void *) (button_func_pair + i), FALLING);
    }

    // 启动Zigbee任务
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
    // 在loop()中处理按钮开关
    uint8_t pin = 0;
    switch_func_pair_t button_func_pair;
    static switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;

    /* 检查是否接收到队列，如果收到则读取按钮功能对 */
    if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
        pin =  button_func_pair.pin;
        switch_gpios_intr_enabled(false);
        evt_flag = true;
    }
    while (evt_flag) {
        bool value = digitalRead(pin);
        switch (switch_state) {
        case SWITCH_IDLE:
            switch_state = (value == LOW) ? SWITCH_PRESS_DETECTED : SWITCH_IDLE;
            break;
        case SWITCH_PRESS_DETECTED:
            switch_state = (value == LOW) ? SWITCH_PRESS_DETECTED : SWITCH_RELEASE_DETECTED;
            break;
        case SWITCH_RELEASE_DETECTED:
            switch_state = SWITCH_IDLE;
            /* 调用按钮处理函数 */
            (*esp_zb_buttons_handler)(&button_func_pair);
            break;
        default:
            break;
        }
        if (switch_state == SWITCH_IDLE) {
            switch_gpios_intr_enabled(true);
            evt_flag = false;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
```

### 第5步：最终效果

将代码上传到两块XIAO ESP32C6（分别为灯泡和灯开关）后，您可以在串口看到类似下图的消息。当您点击开关时，灯泡会开或关。

:::tip
如果串口的消息与下图不一样，您可以按一下XIAO ESP32C6的重置按钮或重新插拔设备以重启。
:::

<div class="table-center">
	<table>
		<tr>
			<th>Zigbee灯泡</th>
			<th>Zigbee开关</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/2.png" style={{width:500, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/3.png" style={{width:500, height:'auto'}}/></div></td>
		</tr>
	</table>
</div>

这里是最终效果。您可以通过开关控制灯泡，操作是通过Zigbee完成的。

<div class="table-center">
<iframe width="800" height="500" src="https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32C6/img/xiaoc6_zigbee_arduino/video.mp4?autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

恭喜您成功完成了Zigbee控制灯光的项目！还有许多令人兴奋的Zigbee应用等待您去探索。继续保持出色的工作！

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供各种支持，确保您能够顺利使用我们的产品。我们提供了多个沟通渠道，以满足不同的需求和偏好。

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