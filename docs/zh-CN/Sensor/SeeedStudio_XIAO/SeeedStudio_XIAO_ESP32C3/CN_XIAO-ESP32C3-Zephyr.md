---
description: ESP32C3与Zephyr - RTOS
title:  Zephyr(RTOS) 编程
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/XIAO-ESP32C3-Zephyr
last_update:
  date: 11/11/2024
  author: Agnes
---

# XIAO ESP32C3 with Zephyr(RTOS)

<div align="center"><img width ="{600}" src="https://files.seeedstudio.com/wiki/xiao_topicpage/zephyr-esp32c3.png"/></div>

## 什么是 RTOS

当今嵌入式系统最重要的组件之一是 **RTOS**，也称为 **实时操作系统**，它负责从任务调度到执行应用程序的所有工作。

**RTOS（实时操作系统）** 旨在提供可预测的执行模式。当处理必须满足系统的时间限制时，RTOS被使用。因此，相比于GPOS（通用操作系统），RTOS通常重量轻、体积小，通常只提供运行特定类型应用程序所需的功能，并且仅适用于特定硬件。在某些情况下，开发人员可以修改现有的RTOS，将其缩小到仅提供特定应用程序所需的功能，和/或自定义其功能或性能特征。

## 什么是 [Zephyr](https://www.zephyrproject.org/)

<div align="center"><img width ="{200}" src="https://files.seeedstudio.com/wiki/XIAO/Zephyr_logo.png"/></div>

The [**Zephyr**](https://www.zephyrproject.org/) 操作系统基于一个小型内核，专为资源受限的嵌入式系统设计：从简单的嵌入式环境传感器和LED可穿戴设备，到复杂的嵌入式控制器、智能手表和物联网无线应用。

## 特性
Zephyr 提供了大量不断增长的功能，包括：

### 广泛的内核服务套件

Zephyr 提供了多个开发所需的常见服务：

- *多线程服务* 支持协作式、基于优先级、非抢占式和抢占式线程，并可选择进行轮询时间切片。包括支持POSIX pthreads兼容的API。
- *中断服务* 用于在编译时注册中断处理程序。
- *内存分配服务* 用于动态分配和释放固定大小或可变大小的内存块。
- *线程间同步服务* 提供二进制信号量、计数信号量和互斥信号量。
- *线程间数据传递服务* 支持基本消息队列、增强消息队列和字节流。
- *电源管理服务* 包括应用程序或策略定义的系统电源管理和驱动程序定义的细粒度设备电源管理。

### 多种调度算法

  Zephyr 提供了全面的线程调度选择：
  - 协作式和抢占式调度
  - 最早截止时间优先（EDF）
  - 实现“中断底半部”或“任务处理”行为的元中断调度
  - 时间切片：在优先级相同的可抢占线程之间启用时间切片
  - 多种队列策略：
    - 简单链表就绪队列
    - 红黑树就绪队列
    - 传统的多队列就绪队列

### 支持蓝牙低功耗 5.0
支持蓝牙5.0（ESR10）和蓝牙低功耗控制器（LE 链接层）。包括蓝牙Mesh和支持蓝牙认证的蓝牙控制器。

- 泛用访问配置文件（GAP），支持所有可能的LE角色
- 泛用属性配置文件（GATT）
- 配对支持，包括蓝牙4.2中的安全连接特性
- 清晰的HCI驱动程序抽象
- 原始HCI接口，使Zephyr能够作为控制器运行，而不是完整的主机堆栈
- 与多个流行的控制器兼容
- 高度可配置

Mesh支持：

- 中继节点、朋友节点、低功耗节点（LPN）和GATT代理特性
- 支持两种配置承载方式（PB-ADV和PB-GATT）
- 高度可配置，适用于至少16KB RAM的设备

*参考： [**Zephyr 项目**](https://docs.zephyrproject.org/latest/introduction/index.html#)*

## 开始

与Zephyr一起工作的第一步是为本地开发设置SDK和工具链。应参考 [Zephyr入门指南](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) 了解适合您环境的设置程序。

一旦设置好Zephyr工具链并下载了相关SDK，您就可以开始应用程序开发了。

对于Xiao ESP32C3，可以参考 [板描述文件](https://docs.zephyrproject.org/latest/boards/seeed/xiao_esp32c3/doc/index.html) 以获取更多设置信息。

要获取使用ESP32C3所需的二进制文件，请运行以下命令：

```
west blobs fetch hal_espressif
```

完成后，可以开始构建样本并将其闪存到开发板。

最简单的示例是运行开发板上的“Hello World”示例。在切换到Zephyr安装目录后，运行以下命令：

```
west build -p always -b xiao_esp32c3 samples/hello_world
west flash
west espressif monitor
```

在执行最后一个命令后，您应该能看到显示“Hello World！”的响应。

```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
Hello World! xiao_esp32c3/esp32c3
```

为了帮助使用Zephyr与Xiao及其扩展板的过程，已经构建了一个包含多个叠加和配置的仓库。在此Wiki文章中包含的命令假设该仓库位于Zephyr根目录下的`../applications/xiao-zephyr-examples`路径。可以通过更新路径来提供其他路径。

```
git clone https://github.com/Cosmic-Bee/xiao-zephyr-examples
```

## 硬件准备

<table align="center">
  <tbody><tr>
      <th>Seeed Studio XIAO ESP32C3</th>
      <th>Seeed Studio 扩展板</th>
      <th>Grove 红色 LED</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/board-pic.png" style={{width:300, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:210, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Raspi_wiki/img/red_led.jpg" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/seeed-xiao-esp32c3-p-5431.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Grove-Red-LED-p-1142.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

### 开发者知识

#### XIAO 扩展板

  为了使用 Grove 模块与 Seeed Studio 的 XIAO ESP32C3，我们将使用[Seeed Studio XIAO 扩展底座](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) 并将 XIAO ESP32C3 连接到它上面。

  之后，可以使用底座上的 Grove 连接器来连接 Grove 模块。

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/29.png"style={{width:700, height:'auto'}}/></div>

#### 引脚定义

  连接 Grove 模块到 Seeed Studio XIAO 的 Grove 扩展板上的 Grove 连接器时，需要根据下面的图示使用适当的内部引脚编号。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO_WiFi/pin_map-2.png"style={{width:900, height:'auto'}}/></div>

### 主要功能

- GPIO / LED 启用
- 蓝牙
- Wi-Fi
- TFLite

#### GPIO / LED 启用

为了测试此设置，我们可以使用 Zephyr 的 blinky 示例，但我们将使用额外的 Grove LED 和一个叠加文件，将 LED 覆盖到 D0 引脚上进行演示（Xiao ESP32C3 没有可编程的板载 LED）：

```
west build -p always -b xiao_esp32c3 samples/basic/blinky -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/d0_led.overlay
west flash
west espressif monitor
```

双击 RESET 或将 RST 引脚短接到 GND。

```
west flash
```

您将看到连接的红色 LED 开关闪烁，形成闪烁效果。

我们来深入了解一下这个示例，看看它为什么能工作。

相关的示例代码引用了 led0：
```
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
```

LED 叠加文件（`xiao-zephyr-examples/d0_led.overlay`）为我们定义了这个 LED：
```
/ {
	aliases {
		led0 = &led0;
	};

	leds {
		compatible = "gpio-leds";
		led0: led_0 {
			gpios = <&xiao_d 0 GPIO_ACTIVE_HIGH>;
			label = "Demo LED";
		};
	};
};
```

这个叠加文件将 D0（通过 `&xiao_d 0` 显示）设置为 LED，并将其与别名 `led0` 关联。`led0` 是一个别名，允许许多示例使用它作为要控制的 LED，因为它专门查找这个名称。 

如果通过 `west espressif monitor` 监控串行输出，您将看到类似的输出：
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
LED state: OFF
LED state: ON
LED state: OFF
LED state: ON
```

#### 蓝牙

为了测试此设置，我们可以使用 Zephyr 的现有示例：

```
west build -p always -b xiao_esp32c3 samples/bluetooth/observer
west flash
west espressif monitor
```

您将看到一个可用于向开发板发送命令的控制台：
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
Starting Observer Demo
Started scanning...
Device found: E5:44:60:88:DB:99 (random) (RSSI -92), type 0, AD data len 27
Device found: F3:38:F3:AD:FC:C6 (random) (RSSI -63), type 3, AD data len 8
Device found: 49:E6:31:0F:A6:25 (random) (RSSI -55), type 2, AD data len 28
Device found: EC:11:27:22:AF:D2 (public) (RSSI -80), type 0, AD data len 31
Device found: FB:3C:4A:AC:64:33 (random) (RSSI -78), type 0, AD data len 30
Device found: 79:05:36:B8:1E:1B (random) (RSSI -34), type 2, AD data len 4
Device found: 77:4D:FC:E2:12:D4 (random) (RSSI -86), type 2, AD data len 4
Device found: E0:9D:13:29:DA:15 (public) (RSSI -99), type 0, AD data len 31
Device found: 53:17:1B:22:70:23 (random) (RSSI -97), type 0, AD data len 18
```

```
CONFIG_BT=y
CONFIG_BT_OBSERVER=y
```

这里的 [conf 文件](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/bluetooth/observer/prj.conf) 启用了与蓝牙相关的功能。

#### Wi-Fi

为了测试此设置，我们可以使用 Zephyr 的现有示例：

```
west build -p always -b xiao_esp32c3 samples/net/wifi
west flash
west espressif monitor
```

您将看到一个可用于向开发板发送命令的控制台：
```
*** Booting Zephyr OS build v3.6.0-1155-g1a55caf8263e ***
uart:~$
```

有几个命令可以让您查看并连接本地网络，更多信息请查看[示例的 README](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/net/wifi/README.rst) for more information.
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
uart:~$ wifi scan
Scan requested
Num  | SSID                             (len) | Chan (Band)   | RSSI | Security        | BSSID             | MFP
1    | Maredonia                        9     | 6    (2.4GHz) | -41  | WPA2-PSK        |                   | Disable
2    | Maredonia                        9     | 6    (2.4GHz) | -41  | WPA2-PSK        |                   | Disable
3    | Aernazonea                       10    | 6    (2.4GHz) | -41  | WPA2-PSK        |                   | Disable
4    | Aernazonea                       10    | 6    (2.4GHz) |
```

让我们深入了解这个示例，看看它为什么能工作：
```
&wifi {
	status = "okay";
};
```

应用的 [叠加文件](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/net/wifi/boards/xiao_esp32c3.overlay)  用于设置各种板载组件。使用这个文件，示例可以启用 Wi-Fi 功能。

```
CONFIG_WIFI=y

CONFIG_NETWORKING=y
CONFIG_NET_L2_ETHERNET=y

CONFIG_NET_IPV6=n
CONFIG_NET_IPV4=y
CONFIG_NET_DHCPV4=y
CONFIG_ESP32_WIFI_STA_AUTO_DHCPV4=y

CONFIG_NET_LOG=y
```

这里的 [conf 文件](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/net/wifi/boards/xiao_esp32c3.conf) 启用了与网络相关的多个功能。

#### TFLite - Hello World

启用 Zephyr 中的 TFLite，并更新配置：
```
west config manifest.project-filter -- +tflite-micro
west update
```

构建示例并将其闪存到您的开发板上：
```
west build -p always -b xiao_esp32c3 samples/modules/tflite-micro/hello_world
west flash
west espressif monitor
```

您将看到来自控制台的返回结果：
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
x_value: 1.0*2^-127, y_value: 1.0*2^-127

x_value: 1.2566366*2^-2, y_value: 1.4910772*2^-2

x_value: 1.2566366*2^-1, y_value: 1.1183078*2^-1

x_value: 1.8849551*2^-1, y_value: 1.677462*2^-1

x_value: 1.2566366*2^0, y_value: 1.9316229*2^-1

x_value: 1.5707957*2^0, y_value: 1.0420598*2^0

x_value: 1.8849551*2^0, y_value: 1.9146791*2^-1

x_value: 1.0995567*2^1, y_value: 1.6435742*2^-1

x_value: 1.2566366*2^1, y_value: 1.0674761*2^-1

x_value: 1.4137159*2^1, y_value: 1.8977352*2^-3
```

有关 TFLite 的更多信息超出了本指南的范围，但此示例作为设备能力和运行 TFLite 设置所需组件的指南。

### 附加组件

- [Grove - 扩展板 - I2C 显示屏](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - I2C Display
- [Grove - 扩展板 - 按钮](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - Button
- [Grove - 温湿度传感器 (SHT31)](https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-SHT31.html)
- [1.69英寸 LCD 显示模块，240×280 分辨率，SPI 接口](https://www.seeedstudio.com/1-69inch-240-280-Resolution-IPS-LCD-Display-Module-p-5755.html)
- [适用于 Xiao 的圆形显示屏](https://www.seeedstudio.com/Seeed-Studio-Round-Display-for-XIAO-p-5638.html)

#### Grove - 扩展板 - I2C 显示屏

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/esp32c3/xiao_expansion_oled.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp23c3_zephyr/xiao_expansion_oled.jpg" style={{width:600, height:'auto'}}/></div>

要测试此设置，可以使用 Zephyr 中的现有示例：

```
west build -p always -b xiao_esp32c3 samples/drivers/display --  -DSHIELD=seeed_xiao_expansion_board
west flash
```

显示屏将显示多个黑色框和一个闪烁的框，因该显示屏仅支持两种颜色。

让我们深入了解一下这个示例，看看它为什么能工作：
```
/ {
    chosen {
      zephyr,display = &ssd1306;
    };
};

&xiao_i2c {
  status = "okay";

  ssd1306: ssd1306@3c {
    compatible = "solomon,ssd1306fb";
    reg = <0x3c>;
    width = <128>;
    height = <64>;
    segment-offset = <0>;
    page-offset = <0>;
    display-offset = <0>;
    multiplex-ratio = <63>;
    segment-remap;
    com-invdir;
    prechargep = <0x22>;
  };
};

```

此扩展板设置了一个 SSD1306 OLED 屏幕，在 0x3C 寄存器上。它被选为 Zephyr 显示屏，并在 chosen 部分进行配置。


#### Grove - 扩展板 - 按钮

要测试此设置，我们可以使用 Zephyr 中的现有示例，但我们将使用额外的 Grove LED，并通过覆盖文件将 LED 重定向到 D0 引脚，以演示此功能（因为 Xiao ESP32C3 没有可编程的板载 LED）：

```
west build -p always -b xiao_esp32c3 samples/basic/button -- -DSHIELD=seeed_xiao_expansion_board -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/d0_led.overlay
west flash
west espressif monitor
```

按下按钮时，D0 引脚上关联的 LED 会亮起。

您将看到来自控制台的返回结果：

```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
Set up button at gpio@60004000 pin 3
Set up LED at gpio@60004000 pin 2
Press the button
Button pressed at 39818120
Button pressed at 63872629
Button pressed at 168304681
Button pressed at 241105558
Button pressed at 346324767
Button pressed at 382181856
Button pressed at 419342954
```

让我们深入了解一下这个示例，看看它为什么能工作：
```
/ {
    aliases {
      sw0 = &xiao_button0;
    };

    buttons {
      compatible = "gpio-keys";
      xiao_button0: button_0 {
        gpios = <&xiao_d 1 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
        label = "SW0";
        zephyr,code = <INPUT_KEY_0>;
      };
    };
};
```

扩展板/覆盖文件用于设置各种板载组件。通过使用此文件，可以使用按钮示例，因为覆盖文件使 Zephyr 配置按钮并使其可用于关联的代码。

在这个例子中，D1 引脚（Xiao ESP32C3）被设置为按钮，并通过 sw0 别名来使用它，这样可以用于该示例中期望使用该名字的代码。

LED 配置与上述 LED 示例类似，也是通过覆盖文件进行配置的。现在按下按钮时，LED 会亮起。

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/esp32c3/led-button.gif?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp23c3_zephyr/led-button.gif" style={{width:600, height:'auto'}}/></div>


#### Grove - 扩展板 - 蜂鸣器

我们将使用闪烁 PWM 示例来激活蜂鸣器，通过 PWM 信号控制其激活。为此，我们将使用一个自定义覆盖文件，启用 A3 引脚的 PWM 功能。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_esp32c3 samples/basic/blinky_pwm -- -DDTC_OVERLAY_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/xiao-esp32c3/xiao_expansion_buzzer.overlay"
west flash
```

闪存后，您应该能听到一系列的蜂鸣声，声音会随着示例程序的运行而变化。

让我们看一下为什么这个示例能够正常工作：

```
#include <zephyr/dt-bindings/pwm/pwm.h>

/ {
	aliases {
		pwm-0 = &ledc0;
		pwm-led0 = &pwm_buzzer;
	};

	pwmleds {
		compatible = "pwm-leds";
		pwm_buzzer: pwm_led_gpio0_5 {
			label = "PWM LED0";
			pwms = <&ledc0 0 1000 PWM_POLARITY_NORMAL>;
		};
	};
};

&pinctrl {
	ledc0_default: ledc0_default {
		group1 {
			pinmux = <LEDC_CH0_GPIO5>;
			output-enable;
		};
	};
};

&ledc0 {
	pinctrl-0 = <&ledc0_default>;
	pinctrl-names = "default";
	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>;
	channel0@0 {
		reg = <0x0>;
		timer = <0>;
	};
};
```

该覆盖文件配置了引脚 5 的 PWM 功能，这对应于 ESP32C3 引脚图中的 A3 引脚。

#### Grove - 扩展板 - SD 卡

我们将使用文件系统示例，结合 Xiao 扩展板扩展盾牌，尝试通过 SPI 接口与 SD 卡读取器进行通信。扩展板已经将 CS 引脚配置为与 `&xiao_d 2` 引脚关联，因此您只需添加扩展盾牌即可，无需做额外配置。为了进一步准备，我们使用一个自定义配置，启用 SD 卡功能。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_esp32c3 samples/subsys/fs/fs_sample -- -DEXTRA_CONF_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/xiao_expansion_sd.conf" -DSHIELD=seeed_xiao_expansion_board
```

然后进行闪存并监控：
```
west flash
west espressif monitor
```

您应该会看到类似以下的响应：
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
[00:00:00.032,000] <inf> sd: Maximum SD clock is under 25MHz, using clock of 24000000Hz
[00:00:00.033,000] <inf> main: Block count 15519744
Sector size 512
Memory Size(MB) 7578
Disk mounted.
Listing dir /SD: ...
[FILE] IMAGE1.JPG (size = 58422)
[FILE] IMAGE2.JPG (size = 97963)
```

在这个例子中，我的 SD 卡中有两个文件，它们的文件名和大小被输出到控制台。

让我们看看这里的关键配置：
```
CONFIG_SPI=y
CONFIG_DISK_DRIVER_SDMMC=y
CONFIG_GPIO=y
```

在相关配置中，我们启用了 SPI、SDMMC 磁盘驱动程序和 GPIO。如果没有这个配置，覆盖文件将导致错误，因为示例无法找到 SD 卡。

以下是 Xiao 扩展板扩展盾牌的相关部分：

```
&xiao_spi {
	status = "okay";
	cs-gpios = <&xiao_d 2 GPIO_ACTIVE_LOW>;

	sdhc0: sdhc@0 {
		compatible = "zephyr,sdhc-spi-slot";
		reg = <0>;
		status = "okay";
		mmc {
			compatible = "zephyr,sdmmc-disk";
			status = "okay";
		};
		spi-max-frequency = <24000000>;
	};
};
```

正如前面所提到的，`&xiao_d 2` 引脚映射用于选择 D2 引脚，这样就可以在支持 `&xiao_d` 引脚配置的任何开发板上使用该功能，而不必担心具体使用的板子类型。

#### Grove - 温湿度传感器 (SHT31)

首先，焊接引脚并将 Xiao ESP32C3 连接到扩展板。然后，通过 Grove 连接线将 Grove SHT31 传感器与扩展板上的一个 I2C 接口连接。

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/esp32c3/xiao_sht31.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp23c3_zephyr/xiao_sht31.jpg" style={{width:600, height:'auto'}}/></div>


为了测试这个设置，我们可以使用 Zephyr 提供的现有示例：

```
west build -p always -b xiao_esp32c3 samples/sensor/sht3xd -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/sht31.overlay
west flash
west espressif monitor
```

您将看到来自控制台的返回结果：
```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
SHT3XD: 25.92 Cel ; 53.37 %RH
SHT3XD: 25.97 Cel ; 54.37 %RH
SHT3XD: 26.00 Cel ; 54.43 %RH
SHT3XD: 26.02 Cel ; 54.11 %RH
SHT3XD: 26.03 Cel ; 53.33 %RH
SHT3XD: 26.02 Cel ; 52.88 %RH
SHT3XD: 26.04 Cel ; 52.12 %RH
SHT3XD: 26.07 Cel ; 51.87 %RH
SHT3XD: 26.13 Cel ; 52.81 %RH
```

让我们深入了解一下为什么这个示例能够正常工作：
```
 &xiao_i2c {
	sht3xd@44 {
			compatible = "sensirion,sht3xd";
			reg = <0x44>;
		};
	};
```

应用程序覆盖文件用于设置各种板卡组件。使用这个文件，SHT31 示例可以被利用，因为覆盖文件 [告知示例逻辑](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/sensor/sht3xd/src/main.c) 如何配置传感器以适配我们的开发板。

#### 1.69 英寸 LCD 显示模块，240×280 分辨率，SPI 接口

在这个示例中，我们将通过 SPI 接口连接一个 1.69 英寸 LCD 屏幕，分辨率为 240x280。

首先，使用以下图片作为参考，将您的开发板与 LCD 屏幕连接（在本示例中，我们使用 Xiao ESP32C3，但连接的引脚布局相同）。

| 1.69英寸LCD SPI显示屏| XIAO ESP32C3 |
| ------------- | ------------------------- |
| VCC | 3V3 |
| GND | GND |
| DIN | D10 |
| CLK | D8 |
| CS | D1 |
| DC | D3 |
| RST | D0 |
| BL | D6 |

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/lcd_spi_display/10.png" style={{width:700, height:'auto'}}/></div>

现在我们可以构建并闪存固件：
```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_esp32c3 samples/drivers/display -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/240x280_st7789v2.overlay -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/240x280_st7789v2.conf
west flash
```

闪存完成后，设备现在会显示之前在扩展板上看到的相同的演示屏幕，只不过这次是更新为通过 SPI 连接的彩色 LCD 屏幕。

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/esp32c3/spi_lcd.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_esp23c3_zephyr/spi_lcd.jpg" style={{width:600, height:'auto'}}/></div>


#### Xiao 圆形显示屏

要测试这个设置，我们可以使用 Zephyr 提供的现有示例：

```
west build -p always -b xiao_esp32c3 samples/drivers/display --  -DSHIELD=seeed_xiao_round_display
```

进入引导加载模式并闪存设备：
```
west flash
```

您将看到一个显示多个彩色角落和一个黑色角落闪烁的屏幕。

另一个示例展示了触摸屏的使用：

```
west build -p always -b xiao_esp32c3 samples/modules/lvgl/demos --  -DSHIELD=seeed_xiao_round_display -DCONFIG_LV_Z_DEMO_MUSIC=y
```

此处展示的音乐演示只显示了实际屏幕的一部分，但仍然展示了触摸屏的工作原理。如您所见，触摸播放按钮会启动音乐动画。

您可以从 [扩展板文件](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/shields/seeed_xiao_round_display/seeed_xiao_round_display.overlay) 中看到，这一切是通过 SPI 与 GC9A01 圆形显示驱动程序以及通过 I2C 与 CHSC6X 触摸模块进行接口的。

让我们深入了解一下这个示例，看看它是如何工作的：
```
/ {
    chosen {
      zephyr,display = &gc9a01_xiao_round_display;
    };

	lvgl_pointer {
		compatible = "zephyr,lvgl-pointer-input";
		input = <&chsc6x_xiao_round_display>;
	};
};

/*
 * xiao_serial 使用了 Xiao 的 D6 和 D7 引脚，分别用于控制屏幕背光和触摸控制器中断。
 */
&xiao_serial {
	status = "disabled";
};

&xiao_i2c {
	clock-frequency = < I2C_BITRATE_FAST >;

	chsc6x_xiao_round_display: chsc6x@2e {
		status = "okay";
		compatible = "chipsemi,chsc6x";
		reg = <0x2e>;
		irq-gpios = <&xiao_d 7 GPIO_ACTIVE_LOW>;
	};
};

&xiao_spi {
	status = "okay";
	cs-gpios = <&xiao_d 1 GPIO_ACTIVE_LOW>, <&xiao_d 2 GPIO_ACTIVE_LOW>;

	gc9a01_xiao_round_display: gc9a01@0 {
		status = "okay";
		compatible = "galaxycore,gc9x01x";
		reg = <0>;
		spi-max-frequency = <DT_FREQ_M(100)>;
		cmd-data-gpios = <&xiao_d 3 GPIO_ACTIVE_HIGH>;
		pixel-format = <PANEL_PIXEL_FORMAT_RGB_565>;
		width = <240>;
		height = <240>;
		display-inversion;
	};
};
```

该扩展板实现了以下功能：
- 选择 GC9A01 显示屏作为 Zephyr 使用的显示屏
- 设置 LVGL 指针逻辑使用 CHSC6X 模块
- 禁用串口，因为 D6 和 D7 引脚用于背光和触摸中断（如上所示： `irq-gpios = <&xiao_d 7 GPIO_ACTIVE_LOW>;`）
- 配置圆形显示屏使用 SPI，通过 D1、D2 和 D3 引脚

[示例逻辑](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/modules/lvgl/demos/src/main.c) 依赖于 [LVGL 演示代码](https://github.com/lvgl/lvgl/tree/master/demos/music)，您可以进一步研究。


## ✨ 贡献者项目

- 本项目由 Seeed Studio [贡献者项目](https://github.com/orgs/Seeed-Studio/projects/6?pane=issue&itemId=57293521)支持。
- T感谢 **Tim的努力** ，您的工作将被 [展示](https://wiki.seeedstudio.com/Honorary-Contributors/)。

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供多种支持渠道，以确保您在使用我们的产品时获得顺畅的体验。我们提供多种通信渠道，以满足不同的需求和偏好。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
