---
description: 以光的代码为例，介绍 Matter 的发展框架。
title: Matter 开发框架概述
keywords:
- matter
- XIAO
- light
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/matter_development_framework
last_update:
  date: 11/27/2024
  author: Agnes
---

# Matter 开发框架概述

:::tip
本文是 Seeed Studio XIAO ESP32 开发 Matter 系列的第四篇教程。如果您还没有阅读之前的教程，建议您先阅读它们，以确保您的设备已经按要求配置好。

- **[在 XIAO 上使用 Espressif ESP-IDF 开发](https://wiki.seeedstudio.com/xiao_idf)**
- **[快速开始使用 XIAO ESP32 系列开发 Matter](https://wiki.seeedstudio.com/getting_started_with_matter)**
- **[XIAO ESP32 系列的 Matter 开发](https://wiki.seeedstudio.com/xiao_esp32_matter_env/)**
:::

通过我们的全面教程，开始一段揭示 Matter 开发世界的旅程。通过经典的灯光示例，我们将揭示构成 Matter 开发基础的基本概念和知识。从集群和属性到命令及更多内容，本教程将为您提供掌握 Matter 框架的工具，帮助您信心十足地探索 Matter 开发的各个方面。准备好提升您的 Matter 开发技能吧，让我们深入探索这个充满活力的物联网世界！

## 教程大纲

1. **[理解灯光示例](#understanding-the-light-example)**
2. **[设备初始化](#device-initialisation)**
3. **[创建 Matter 节点](#create-matter-node)**
4. **[设置端点的属性](#sets-the-attribute-of-the-endpoint)**
5. **[创建端点并自动匹配集群](#create-endpoint--auto-match-cluster)**
6. **[使用默认值首次设置 Matter 设备](#setting-up-the-matter-device-for-the-first-time-with-default-values)**
7. **[数据更新和延迟持久化](#data-updates-and-deferred-persistence)**

在本节中，我们将重点介绍 ESP-Matter 提供的 [灯光](https://github.com/espressif/esp-matter/tree/main/examples/light) 示例，该示例详细描述了 Matter 开发框架中集群、属性和命令的基本概念。通过阅读本文，您将更好地理解 Matter 开发框架的结构和工作原理。

## 理解灯光示例

首先，让我们看一下 ESP-Matter 环境中的文件目录及其功能。

```
- esp-matter/
   - components/
      - esp_matter
      - esp_matter_bridge
      - esp_matter_console
      - esp_matter_controller
      - esp_matter_rainmaker
      - esp_matter_thread_br
   - connectedhomeip/
   - device_hal/
      - button_driver
      - device
      - led_driver
   - docs/
   - examples/
   - tools/
      - mfg_tool
   CMakeLists.txt
   RELEASE_NOTES.md
   export.sh
   install.sh
   requirements.txt
   ...
```

**esp-matter**：这是整个 Matter 开发框架的根目录。

**components**：该目录包含各种组件，是 Matter 框架的核心部分。
   - esp_matter：这是在 ESP32 上实现 Matter 协议栈的部分，包括数据模型、应用层逻辑等。
   - esp_matter_bridge：该组件负责将 ESP 设备与其他非 ESP 设备桥接，实现互操作性。
   - esp_matter_console：这是基于 REPL 的交互式控制台，用于调试和控制 Matter 设备。
   - esp_matter_controller：该组件实现了 Matter 控制器的功能，能够控制其他 Matter 设备。
   - esp_matter_rainmaker：该组件与 Espressif 的 RainMaker 云平台集成，实现云端控制。
   - esp_matter_thread_br：该组件实现了 Thread Border Router 的功能，用于创建 Thread 网络。

**connectedhomeip**：这是 Matter 协议栈的上游开源项目，ESP Matter 从该项目同步代码。

**device_hal**：该目录包含硬件抽象层驱动。
   - button_driver：按钮驱动程序。
   - device：设备抽象，定义通用的设备接口。
   - led_driver：LED 驱动程序。

**docs**：该目录存储 ESP-Matter 开发文档和 API 参考手册。

**examples**：各种示例代码，演示如何使用 Matter 框架进行开发。

**tools**：包含各种开发工具脚本。
   - mfg_tool：用于生成制造商证书的工具。

**CMakeLists.txt**：CMake 构建脚本，定义项目的编译规则。

**RELEASE_NOTES.md**：版本发布说明，记录每个版本的更改内容。

**export.sh**：导出脚本，用于导出与 Matter 相关的环境变量。

**install.sh**：安装脚本，用于安装 Matter 开发所需的依赖项和工具链。

**requirements.txt**：Python 依赖列表，指定了运行 Matter 开发框架所需的 Python 库。

这个目录结构体现了 Matter 开发框架的模块化设计理念。核心协议栈、硬件抽象、应用组件、辅助工具等各个部分各司其职，同时有机地结合在一起，为开发者提供了一个完整的 Matter 开发环境。

以 **examples/light** 为例，ESP-Matter 提供的示例结构如下：

```
- light/
   - main/
      - CMakeLists.txt
      - app_driver.cpp
      - app_main.cpp
      - app_priv.h
      - idf_components.yml
   CMakeLists.txt
   README.md
   partitions.csv
   sdkconfig.defaults
   ...
```

- **main**: 该子目录包含主要的应用程序代码和配置文件。
   - CMakeLists.txt: 主要应用程序的 CMake 构建脚本。
   - app_driver.cpp: 灯光应用程序的驱动代码。
   - app_main.cpp: 灯光应用程序的主入口文件。
   - app_priv.h: 包含灯光应用程序私有声明的头文件。
   - idf_components.yml: 用于灯光应用程序中的 ESP-IDF 组件的配置文件。

- **CMakeLists.txt**: 灯光示例的顶级 CMake 构建脚本。

- **README.md**: 提供灯光示例信息和说明的 README 文件。

- **partitions.csv**: 定义灯光示例的分区表文件。

- **sdkconfig.defaults**: 灯光示例的默认配置设置。

## 设备初始化

接下来，我们将深入分析灯光的代码，通过代码分析和理论结合来加深对 Matter 开发流程的理解。以下代码位于 [main/app_main.cpp](https://github.com/espressif/esp-matter/blob/main/examples/light/main/app_main.cpp)。

```cpp
app_driver_handle_t light_handle = app_driver_light_init();
app_driver_handle_t button_handle = app_driver_button_init();
app_reset_button_register(button_handle);
```

`app_driver_handle_t light_handle = app_driver_light_init();`：这一行初始化了灯光驱动并返回一个指向灯光驱动实例的句柄。

`app_driver_handle_t button_handle = app_driver_button_init();`：与灯光初始化类似，这一行初始化了按钮驱动。

`app_reset_button_register(button_handle);`：这一行为按钮注册了特定的功能，用于处理重置操作。

以 `app_driver_light_init()` 函数为例，以下程序初始化了所有的灯泡，但只使用了第一个（设置为默认的颜色和亮度值）。这也是示例程序只能使用一个灯泡的原因。

```cpp
// app_driver.cpp
app_driver_handle_t app_driver_light_init()
{
#if CONFIG_BSP_LEDS_NUM > 0
    /* 初始化 LED */
    led_indicator_handle_t leds[CONFIG_BSP_LEDS_NUM];
    ESP_ERROR_CHECK(bsp_led_indicator_create(leds, NULL, CONFIG_BSP_LEDS_NUM));
    led_indicator_set_hsv(leds[0], SET_HSV(DEFAULT_HUE, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS));
    
    return (app_driver_handle_t)leds[0];
#else
    return NULL;
#endif
}
```

## 创建 Matter 节点

Matter 设备配置的下一步是创建一个 Matter 节点，代码如下：

```cpp
node::config_t node_config;

// node 句柄可以用来添加/修改其他端点。
node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "创建 Matter 节点失败"));
```

Matter 数据模型是一个标准化的方式，用于表示和组织 Matter 生态系统中的数据。它定义了设备、属性和交互的通用语言和结构，从而实现 Matter 兼容设备之间的互操作性和无缝通信。

以下图示展示了 Matter 数据模型的简化视图：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/34.png" style={{width:600, height:'auto'}}/></div>

**Matter 节点**：
Matter 节点代表 Matter 生态系统中的一个物理设备或逻辑实体。它是 Matter 数据模型的顶层组件。每个 Matter 节点都有一个唯一的标识符，并且可以包含一个或多个端点。
   - Matter 节点代表 Matter 生态系统中的 **物理设备**。
   - 它就像一个房子，可以包含多个端点（房间）。
   - 每个 Matter 节点都有自己唯一的标识符，用于在网络中识别和寻址。

## 设置端点的属性

一旦 Matter 节点创建完成，就需要为端点的属性设置默认值。

```cpp
extended_color_light::config_t light_config;
light_config.on_off.on_off = DEFAULT_POWER;
light_config.on_off.lighting.start_up_on_off = nullptr;
light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;
```

1. `light_config.on_off.on_off = DEFAULT_POWER;`
   - 设置端点的初始开/关状态为 `DEFAULT_POWER`。
   - `DEFAULT_POWER` 是一个预定义常量，表示默认的电源状态（例如，`true` 表示开启，`false` 表示关闭）。

2. `light_config.on_off.lighting.start_up_on_off = nullptr;`
   - 设置端点的启动时开/关状态为 `nullptr`。
   - 当设备重启或断电重启时，如果该值为 `nullptr`，则表示使用上次的开/关状态。
   - 如果设置为非 `nullptr` 的值，则表示使用指定的开/关状态。

3. `light_config.level_control.current_level = DEFAULT_BRIGHTNESS;`
   - 设置端点的初始亮度级别为 `DEFAULT_BRIGHTNESS`（64）。
   - `DEFAULT_BRIGHTNESS` 是一个预定义常量，表示默认亮度级别（例如，0 到 254 之间的值）。

4. `light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;`
   - 设置端点的启动亮度级别为 `DEFAULT_BRIGHTNESS`（64）。
   - 当设备重启或断电重启时，如果该值非 `nullptr`，则表示使用指定的亮度级别。
   - 如果设置为 `nullptr`，则表示使用上次的亮度级别。

5. `light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;`
   - 设置端点的颜色模式为 `ColorControl::ColorMode::kColorTemperature`。
   - 这意味着端点使用色温模式，通过调整色温来控制灯光的颜色。
   - `(uint8_t)` 是类型转换，将枚举值转换为无符号 8 位整数。

6. `light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;`
   - 设置端点的增强色彩模式为 `ColorControl::ColorMode::kColorTemperature`。
   - 增强色彩模式提供了更多的颜色控制选项，但这里也设置为色温模式。

7. `light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;`
   - 设置端点的启动色温为 `nullptr`。
   - 当设备重启或断电重启时，如果该值为 `nullptr`，则表示使用上次的色温设置。
   - 如果设置为非 `nullptr` 的值，则表示使用指定的色温值。

Matter 中的属性类似于设备的特性或特征。它们存储有关设备状态的信息，例如设备是否打开、亮度级别或色温。这些属性按功能分组，称为簇（Clusters），每个簇与设备的特定功能相关。

属性使不同的设备和应用程序能够更容易地进行通信并无缝协作。通过采用标准化的方式表示和访问设备属性，Matter 简化了智能家居系统的开发，并确保来自不同品牌的设备可以高效地互操作。

`esp_matter_endpoint.h` 是 ESP Matter SDK 中一个重要的头文件，定义了与端点相关的常量、数据类型和函数。在 Matter 中，端点表示设备的逻辑接口，每个端点包含一组属性和命令，用于描述和控制设备的特定功能。

```cpp
namespace extended_color_light {
typedef struct config {
    cluster::descriptor::config_t descriptor;
    cluster::identify::config_t identify;
    cluster::groups::config_t groups;
    cluster::scenes_management::config_t scenes_management;
    cluster::on_off::config_t on_off;
    cluster::level_control::config_t level_control;
    cluster::color_control::config_t color_control;
} config_t;

uint32_t get_device_type_id();
uint8_t get_device_type_version();
endpoint_t *create(node_t *node, config_t *config, uint8_t flags, void *priv_data);
esp_err_t add(endpoint_t *endpoint, config_t *config);
} /* extended_color_light */
```

## 创建端点 & 自动匹配簇

在上面的代码中，我们首先提到了 Matter 中的两个重要术语：端点（Endpoints）和簇（Clusters）。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/xiaoc6-matter/35.png" style={{width:500, height:'auto'}}/></div>

**端点（Endpoint(s)）[设备类型（Device-Type(s)）]**：
端点是 Matter 节点中一个特定功能或服务的逻辑表示。它封装了与特定设备类型相关的一组功能和行为。一个 Matter 节点可以有多个端点，每个端点代表一个不同的设备类型。设备类型定义了端点的具体特性和功能。Matter 定义了一组标准的设备类型，例如灯泡、恒温器、门锁等。每种设备类型都有一个唯一的标识符和一组与之相关的预定义簇、属性和命令。

- 端点是 Matter 节点中的一个逻辑组件，表示设备的一个特定功能或服务。
- 就像房子中的不同房间一样，每个端点都有自己专门的用途，例如卧室、厨房或客厅。
- 每个端点与特定的设备类型相关联，例如灯泡、恒温器或门锁。
- 一个 Matter 节点可以有多个端点，每个端点代表不同的设备类型和功能。

**簇（Cluster(s)）**：
簇是端点中相关属性和命令的逻辑分组。它们表示设备的特定功能或特性。簇提供了一种组织和分类端点功能的方式。例如，“开/关簇”包含与开启或关闭设备相关的属性和命令，而“亮度控制簇”则涉及控制设备的亮度或级别。

- 簇是端点中的一个逻辑分组，包含相关的属性和命令。
- 它就像房间中的家具或设备，例如灯、电视或空调，每个设备都有自己的属性和操作。
- 每个簇表示设备的一个特定功能或特性。
- 例如，“开/关簇”包含与设备开/关状态相关的属性和命令，而“亮度控制簇”包含用于调整设备亮度或级别的属性和命令。
- 一个端点可以有多个簇，每个簇负责不同的功能。

总而言之，Matter 节点就像一栋房子，包含多个端点（房间）。每个端点就像一个房间，表示设备的特定功能或服务。簇就像房间中的家具或设备，包含相关的属性和命令，用于控制和交互。

这种层次化的组织结构使得设备能够清晰地描述其功能和特性，从而使应用程序和其他设备更容易与它们进行交互和控制。通过标准化设备类型、簇、属性和命令，Matter 实现了不同厂商设备之间的互操作性和兼容性。

在代码中，设置完属性后，最终通过以下代码段创建一个端点，并自动匹配设置属性的簇。

```cpp
endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "创建扩展色光端点失败"));
```

## 如何实现自动匹配簇？

让我们通过一个设置属性的示例来说明如何实现自动匹配簇。

```cpp
light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
```

`light_config.level_control` 是在端点（`esp_matter_endpoint.h`）中定义的一个属性，而 `light_config.level_control.lighting` 是在簇（`esp_matter_cluster`）中定义的属性。通过这种设置，系统能够自动匹配与该属性相对应的簇，而无需开发者手动设置。

## 初次设置 Matter 设备的默认值

在配置了上述属性、簇和端点后，我们可以开始启动 Matter 设备。启动的步骤和方法如下所示。

```cpp
light_endpoint_id = endpoint::get_id(endpoint);
ESP_LOGI(TAG, "创建的灯光端点 ID 为 %d", light_endpoint_id);

/* 启动 Matter */
err = esp_matter::start(app_event_cb);
ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "启动 Matter 失败，错误代码: %d", err));

/* 使用默认值启动驱动程序 */
app_driver_light_set_defaults(light_endpoint_id);
```

如您所见，设置默认值的函数是 `app_driver_light_set_defaults()`，我们需要传入一个端点 ID 作为参数。我们还需要关注如何获取某个簇的值，某个属性的值，以及如何设置默认的簇和属性值。这些细节可以在 `app_driver.cpp` 中找到。

```cpp
esp_err_t err = ESP_OK;
void *priv_data = endpoint::get_priv_data(endpoint_id);
led_indicator_handle_t handle = (led_indicator_handle_t)priv_data;
node_t *node = node::get();
endpoint_t *endpoint = endpoint::get(node, endpoint_id);
cluster_t *cluster = NULL;
attribute_t *attribute = NULL;
esp_matter_attr_val_t val = esp_matter_invalid(NULL);

/* 设置亮度 */
cluster = cluster::get(endpoint, LevelControl::Id);
attribute = attribute::get(cluster, LevelControl::Attributes::CurrentLevel::Id);
attribute::get_val(attribute, &val);
err |= app_driver_light_set_brightness(handle, &val);
```

1. **获取簇（Cluster）**：
   - 要获取簇，首先需要使用 `endpoint::get(node, endpoint_id)` 函数获取端点的指针，其中 `node` 是指向节点的指针，`endpoint_id` 是端点的 ID。
   - 一旦获取了端点的指针，就可以使用 `cluster::get(endpoint, LevelControl::Id)` 函数获取所需的簇的指针，指定端点和簇的 ID（此处为 `LevelControl::Id`）。

2. **获取特定属性（Attribute）**：
   - 获取簇的指针后，可以使用 `attribute::get(cluster, LevelControl::Attributes::CurrentLevel::Id)` 函数获取簇中的特定属性。
   - 在此示例中，我们从 `LevelControl` 簇中检索 `CurrentLevel` 属性。

3. **获取属性值**：
   - 要获取属性的当前值，首先需要声明一个类型为 `esp_matter_attr_val_t` 的变量来存储属性值。
   - 在代码片段中，变量 `val` 被初始化为 `esp_matter_invalid(NULL)`。
   - 然后，使用 `attribute::get_val(attribute, &val)` 函数获取属性的当前值，并将其存储在 `val` 变量中。

4. **设置灯光亮度**：
   - 要设置灯光的亮度，首先需要获取与端点关联的 LED 指示灯的句柄。
   - 在代码片段中，通过将私有数据指针（`priv_data`）强制转换为适当的类型（`led_indicator_handle_t`）来获取 LED 指示灯句柄。
   - 最后，使用 `app_driver_light_set_brightness(handle, &val)` 函数设置 LED 指示灯的亮度。
   - 参数 `handle` 是 LED 指示灯的句柄，`&val` 是指向包含所需亮度值的 `esp_matter_attr_val_t` 变量的指针。

### 总结步骤：
1. 使用 `endpoint::get(node, endpoint_id)` 获取端点指针。
2. 使用 `cluster::get(endpoint, LevelControl::Id)` 获取簇指针。
3. 使用 `attribute::get(cluster, LevelControl::Attributes::CurrentLevel::Id)` 获取属性指针。
4. 使用 `attribute::get_val(attribute, &val)` 获取属性的当前值。
5. 使用 `app_driver_light_set_brightness(handle, &val)` 设置灯光的亮度，其中 `handle` 是与端点关联的 LED 指示灯的句柄。

通过遵循这些步骤，您可以获取簇和属性的必要指针，检索属性的当前值，并相应地设置灯光的亮度。

## 数据更新与延迟持久化

在 `app_driver.cpp` 代码中，使用 `app_driver_attribute_update()` 函数更新属性的值。

```cpp
if (endpoint_id == light_endpoint_id) {
   led_indicator_handle_t handle = (led_indicator_handle_t)driver_handle;
   if (cluster_id == OnOff::Id) {
      if (attribute_id == OnOff::Attributes::OnOff::Id) {
            err = app_driver_light_set_power(handle, val);
      }
   } else if (cluster_id == LevelControl::Id) {
      if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
            err = app_driver_light_set_brightness(handle, val);
      }
   } else if (cluster_id == ColorControl::Id) {
      if (attribute_id == ColorControl::Attributes::CurrentHue::Id) {
            err = app_driver_light_set_hue(handle, val);
      } else if (attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
            err = app_driver_light_set_saturation(handle, val);
      } else if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
            err = app_driver_light_set_temperature(handle, val);
      }
   }
}
```

这段代码定义了一个名为 `app_driver_attribute_update` 的函数，该函数接受多个参数，包括驱动句柄（`driver_handle`）、端点 ID（`endpoint_id`）、簇 ID（`cluster_id`）、属性 ID（`attribute_id`）以及指向属性值的指针（`val`）。

该函数的目的是根据接收到的数据更新灯光端点的属性值。它遵循特定的逻辑来确定需要更新的灯光端点的属性。

以下是数据更新逻辑的逐步分解：

1. 函数首先检查 `endpoint_id` 是否与 `light_endpoint_id` 匹配。这确保了更新是针对灯光端点进行的。

2. 如果 `endpoint_id` 匹配，函数将 `driver_handle` 转换为适当的类型（`led_indicator_handle_t`），以获取与灯光端点关联的 LED 指示灯句柄。

3. 接着，函数检查 `cluster_id`，以确定属性属于哪个簇。支持的簇包括 `OnOff`、`LevelControl` 和 `ColorControl`。

4. 根据 `cluster_id`，函数进一步检查 `attribute_id`，以识别该簇中的特定属性。

5. 根据 `cluster_id` 和 `attribute_id`，函数调用相应的设置函数来更新属性值：
   - 如果 `cluster_id` 是 `OnOff::Id` 且 `attribute_id` 是 `OnOff::Attributes::OnOff::Id`，则调用 `app_driver_light_set_power(handle, val)` 来设置灯光的电源状态。
   - 如果 `cluster_id` 是 `LevelControl::Id` 且 `attribute_id` 是 `LevelControl::Attributes::CurrentLevel::Id`，则调用 `app_driver_light_set_brightness(handle, val)` 来设置灯光的亮度。
   - 如果 `cluster_id` 是 `ColorControl::Id`，则进一步检查 `attribute_id`：
     - 如果 `attribute_id` 是 `ColorControl::Attributes::CurrentHue::Id`，则调用 `app_driver_light_set_hue(handle, val)` 来设置灯光的色调。
     - 如果 `attribute_id` 是 `ColorControl::Attributes::CurrentSaturation::Id`，则调用 `app_driver_light_set_saturation(handle, val)` 来设置灯光的饱和度。
     - 如果 `attribute_id` 是 `ColorControl::Attributes::ColorTemperatureMireds::Id`，则调用 `app_driver_light_set_temperature(handle, val)` 来设置灯光的色温。

总的来说，这个函数作为更新灯光端点属性值的中心点。它接收必要的信息（端点 ID、簇 ID、属性 ID 和属性值），并根据簇和属性 ID 将更新分发到相应的设置函数。

通过以这种方式组织逻辑，代码变得更加模块化且易于维护。它允许通过一个函数更新灯光端点的不同属性（电源状态、亮度、色调、饱和度、色温），简化了根据接收到的数据更新灯光特征的过程。

但并不是所有属性都会实时更新。标记为延迟持久化的属性可以提高性能，减少对非易失性存储的写入次数，并延长设备的使用寿命。

```cpp
/* 标记可能会频繁变化的某些属性为延迟持久化 */
cluster_t *level_control_cluster = cluster::get(endpoint, LevelControl::Id);
attribute_t *current_level_attribute = attribute::get(level_control_cluster, LevelControl::Attributes::CurrentLevel::Id);
attribute::set_deferred_persistence(current_level_attribute);

cluster_t *color_control_cluster = cluster::get(endpoint, ColorControl::Id);
attribute_t *current_x_attribute = attribute::get(color_control_cluster, ColorControl::Attributes::CurrentX::Id);
attribute::set_deferred_persistence(current_x_attribute);
```

## 定义自定义数据模型

本节演示了如何创建在 Matter 规范中定义的标准端点、簇、属性和命令。

#### 端点

可以通过编辑示例中的 *app_main.cpp* 文件中的端点/设备类型来定制设备。以下是一些示例：

- **开关灯（on_off_light）**：

```cpp
   on_off_light::config_t light_config;
   endpoint_t *endpoint = on_off_light::create(node, &light_config, ENDPOINT_FLAG_NONE);
```

- **温度传感器（temperature_sensor）**：

```cpp
    esp_matter::endpoint::temperature_sensor::config_t temperature_sensor_config;
    endpoint_t *endpoint = temperature_sensor::create(node, &temperature_sensor_config, ENDPOINT_FLAG_NONE, NULL);
```

- **风扇（fan）**：

```cpp
   fan::config_t fan_config;
   endpoint_t *endpoint = fan::create(node, &fan_config, ENDPOINT_FLAG_NONE);
```

- **门锁（door_lock）**：

```cpp
   door_lock::config_t door_lock_config;
   endpoint_t *endpoint = door_lock::create(node, &door_lock_config, ENDPOINT_FLAG_NONE);
```

- **窗帘设备（window_covering_device）**：

```cpp
   window_covering_device::config_t window_covering_device_config(static_cast<uint8_t>(chip::app::Clusters::WindowCovering::EndProductType::kTiltOnlyInteriorBlind));
   endpoint_t *endpoint = window_covering_device::create(node, &window_covering_config, ENDPOINT_FLAG_NONE);
```

`window_covering_device` 的 `config_t` 结构包括一个构造函数，允许指定不同于默认值的终端产品类型，默认值为“卷帘”。
一旦实例化了 `config_t`，其终端产品类型将不能再修改。

- **泵（pump）**：

```cpp
   pump::config_t pump_config(1, 10, 20);
   endpoint_t *endpoint = pump::create(node, &pump_config, ENDPOINT_FLAG_NONE);
```

`pump` 的 `config_t` 结构包括一个构造函数，允许指定最大压力、最大速度和最大流量值。如果未设置，这些值将默认为 null。
一旦实例化了 `config_t`，这三个值将无法修改。


### 簇（Clusters）

也可以向端点添加其他簇。示例：

- **开关（on_off）**：

```cpp
   on_off::config_t on_off_config;
   cluster_t *cluster = on_off::create(endpoint, &on_off_config, CLUSTER_FLAG_SERVER, on_off::feature::lighting::get_id());
```

- **温度测量（temperature_measurement）**：

```cpp
   temperature_measurement::config_t temperature_measurement_config;
   cluster_t *cluster = temperature_measurement::create(endpoint, &temperature_measurement_config, CLUSTER_FLAG_SERVER);
```

- **窗帘控制（window_covering）**：

```cpp
   window_covering::config_t window_covering_config(static_cast<uint8_t>(chip::app::Clusters::WindowCovering::EndProductType::kTiltOnlyInteriorBlind));
   cluster_t *cluster = window_covering::create(endpoint, &window_covering_config, CLUSTER_FLAG_SERVER);
```

`window_covering` 的 `config_t` 结构包括一个构造函数，允许指定与默认值不同的终端产品类型，默认类型为“卷帘”。
一旦实例化了 `config_t`，其终端产品类型将无法修改。

- **泵配置与控制（pump_configuration_and_control）**：

```cpp
   pump_configuration_and_control::config_t pump_configuration_and_control_config(1, 10, 20);
   cluster_t *cluster = pump_configuration_and_control::create(endpoint, &pump_configuration_and_control_config, CLUSTER_FLAG_SERVER);
```

`pump_configuration_and_control` 的 `config_t` 结构包括一个构造函数，允许指定最大压力、最大速度和最大流量值。如果未设置，这些值将默认为 null。
一旦实例化了 `config_t`，这三个值将无法修改。

### 属性和命令（Attributes and Commands）

也可以向簇添加其他属性和命令。示例：

- **属性：开关（on_off）**：

```cpp
   bool default_on_off = true;
   attribute_t *attribute = on_off::attribute::create_on_off(cluster, default_on_off);
```

- **属性：簇修订版（cluster_revision）**：

```cpp
   uint16_t default_cluster_revision = 1;
   attribute_t *attribute = global::attribute::create_cluster_revision(cluster, default_cluster_revision);
```

- **命令：切换（toggle）**：

```cpp
   command_t *command = on_off::command::create_toggle(cluster);
```

- **命令：移动到指定亮度（move_to_level）**：

```cpp
   command_t *command = level_control::command::create_move_to_level(cluster);
```

### 特性（Features）

可以向簇添加适用的可选特性。

- **特性：标签列表（taglist）：描述符簇（Descriptor cluster）**：

```cpp
   cluster_t* cluster = cluster::get(endpoint, Descriptor::Id);
   descriptor::feature::taglist::add(cluster);
```

### 添加自定义数据模型字段

本节演示了如何创建不在 Matter 规范中定义的自定义端点、簇、属性和命令，这些可以特定于厂商。

#### 端点（Endpoints）

可以创建非标准端点，不包含任何簇。

- **创建端点**：

```cpp
   endpoint_t *endpoint = endpoint::create(node, ENDPOINT_FLAG_NONE);
```

#### 簇（Clusters）

也可以创建非标准/自定义簇：

- **创建簇**：

```cpp
   uint32_t custom_cluster_id = 0x131bfc00;
   cluster_t *cluster = cluster::create(endpoint, custom_cluster_id, CLUSTER_FLAG_SERVER);
```

#### 属性和命令（Attributes and Commands）

可以在任何簇上创建非标准/自定义属性：

- **创建属性**：

```cpp
   uint32_t custom_attribute_id = 0x0;
   uint16_t default_value = 100;
   attribute_t *attribute = attribute::create(cluster, custom_attribute_id, ATTRIBUTE_FLAG_NONE, esp_matter_uint16(default_value));
```

- **创建命令**：

```cpp
   static esp_err_t command_callback(const ConcreteCommandPath &command_path, TLVReader &tlv_data, void *opaque_ptr)
   {
      ESP_LOGI(TAG, "自定义命令回调");
      return ESP_OK;
   }

   uint32_t custom_command_id = 0x0;
   command_t *command = command::create(cluster, custom_command_id, COMMAND_FLAG_ACCEPTED, command_callback);
```


Matter 数据模型以层次化的方式组织这些组件。一个 Matter 节点包含一个或多个端点（Endpoints），每个端点代表一种特定的设备类型。每个端点由多个簇（Clusters）组成，簇将相关的属性和命令分组在一起。属性存储簇的状态和配置，而命令用于与设备进行交互和控制。

通过这种方式构建数据模型，Matter 实现了不同厂商设备之间的互操作性和标准化。开发人员可以使用定义好的设备类型、簇、属性和命令来创建应用程序，从而无缝地控制和与 Matter 兼容的设备进行通信。

Matter 数据模型为设备提供了一种共同的语言和框架，用于描述其功能并与其他设备进行交互，从而为智能家居提供更加统一和一致的体验。

## 技术支持与产品讨论

感谢您选择我们的产品！我们将为您提供不同的支持方式，确保您在使用我们产品的过程中体验顺畅。我们提供了多种沟通渠道，以满足不同的偏好和需求。


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
