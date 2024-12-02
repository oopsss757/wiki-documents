---
description: XIAO RP2040 与 Zephyr（RTOS）
title:  XIAO RP2040 与 Zephyr（RTOS）
keywords:
- Sorftware
image: https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png
slug: /cn/XIAO-RP2040-Zephyr-RTOS
last_update:
  date: 11/13/2024
  author: Agnes
---

# XIAO RP2040 与 Zephyr（RTOS）

<div align="center"><img width ="{600}" src="https://files.seeedstudio.com/wiki/xiao_topicpage/zephyr-rp2040.png"/></div>

本维基介绍了 [Zephyr](https://www.zephyrproject.org/) 在 [Seeed Studio XIAO RP2040](https://wiki.seeedstudio.com/xiao_rp2040_getting_started/)上的支持。通过本指南，您将能够利用该开发板提供的功能。

## 什么是 [Zephyr](https://www.zephyrproject.org/)

<div align="center"><img width ="{200}" src="https://files.seeedstudio.com/wiki/XIAO/Zephyr_logo.png"/></div>

[**Zephyr**](https://www.zephyrproject.org/) 操作系统基于一个小型内核，专为资源受限的嵌入式系统设计，从简单的嵌入式环境传感器、LED 可穿戴设备到复杂的嵌入式控制器、智能手表和物联网无线应用均可使用。

对于每个支持的设备，Zephyr 有一个 [设备树](https://docs.zephyrproject.org/latest/build/dts/index.html) 文件，描述了开发板及其特性。 [Xiao RP2040 Zephyr 开发板页面](https://docs.zephyrproject.org/latest/boards/seeed/xiao_rp2040/doc/index.html#supported-features) 描述了当前可用的支持特性，这些特性由 [开发板的 dts 文件](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/seeed/xiao_rp2040/xiao_rp2040.yaml#L7)定义。

*参考资料: [**Zephyr 项目**](https://docs.zephyrproject.org/latest/introduction/index.html#)*

## 入门指南

开始使用 Zephyr 的第一步是设置 SDK 和工具链，以便进行本地开发。可以参考 [Zephyr 入门指南](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) 来完成环境的配置。

一旦配置好了 Zephyr 工具链并下载了相关的 SDK，您就可以开始应用程序的开发了。

对于 Xiao RP2040，您可以参考 [开发板描述文件](https://docs.zephyrproject.org/latest/boards/seeed/xiao_rp2040/doc/index.html) 获取更多的配置信息。

要编程 Xiao RP2040，可以按照以下步骤操作：
1. 构建示例或您的应用程序
2. 插入 Xiao RP2040
3. 按住指定的 `B`（引导）按钮并按下 `R`（重置）按钮，这会将设备挂载为大容量存储设备
4. 运行 flash 命令来刷新设备 `west flash -r uf2`

最简单的示例是运行板上的 "Hello World" 示例。在 Zephyr 安装目录中执行以下命令。

```
west build -p always -b xiao_rp2040 samples/subsys/usb/console
```

进入引导加载程序模式并按照前述步骤刷新设备：

```
west flash -r uf2
```

设备在接收到文件后会重启，您的机器应该通过 USB 连接以进行串口通信。

查找设备的端口，例如在 Ubuntu 上输入 `ls /dev/tty*`，确认当您插入 USB 时，哪个设备出现。

在我的示例中，我看到 `/dev/ttyACM0` 是新添加的设备。

接下来，您可以使用 `screen` 连接并监视串口响应：
```
screen /dev/ttyACM0 115200
```

您应该看到类似以下的响应：
```
*** Booting Zephyr OS build v3.6.0-2212-gc38ea288eee9 ***
Hello World! arm
Hello World! arm
Hello World! arm
```

为了帮助在 Xiao 和其扩展板上使用 Zephyr，已经建立了一个包含多个覆盖层和配置的仓库，您可以在此使用。文中提供的命令假设该仓库位于 Zephyr 根目录的 `../applications/xiao-zephyr-examples`。您可以通过更新路径提供其他路径。

```
git clone https://github.com/Cosmic-Bee/xiao-zephyr-examples
```

## 硬件准备

<table align="center">
  <tbody><tr>
      <th>Seeed Studio XIAO RP2040</th>
      <th>Seeed Studio Expansion 扩展板</th>
    </tr>
    <tr>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/102010428_Preview-07.jpg" style={{width:300, height:'auto'}}/></div></td>
      <td><div align="center"><img src="https://files.seeedstudio.com/wiki/Seeeduino-XIAO-Expansion-Board/Update_pic/zheng1.jpg" style={{width:210, height:'auto'}}/></div></td>
    </tr>
    <tr>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取 🖱️</font></span></strong>
            </a>
        </div></td>
        <td align="center"><div class="get_one_now_container" style={{textAlign: 'center'}}>
            <a class="get_one_now_item" href="https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html">
            <strong><span><font color={'FFFFFF'} size={"4"}> 立即获取🖱️</font></span></strong>
            </a>
        </div></td>
    </tr>
  </tbody></table>

### 开发者知识

#### XIAO 扩展板

  为了使用 Grove 模块与 Seeed Studio XIAO RP2040，我们将使用 [Seeed Studio XIAO扩展基板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) 并将 XIAO RP2040 连接到该板上。

  之后，您可以使用板上的 Grove 接口来连接 Grove 模块。

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/C3-ESPHome-full_function/29.png"style={{width:700, height:'auto'}}/></div>

#### 引脚定义

  在连接 Grove 模块到 Seeed Studio XIAO 的 Grove 扩展板时，您需要按照下图选择适当的内部引脚编号。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2040/img/xinpin.jpg"style={{width:900, height:'auto'}}/></div>

### 主要功能

- WS2812 LED
- LED PWM
- 时钟
- TFLite

#### WS2812 LED

在这个示例中，Xiao RP2040 使用其板载 LED，持续不断地切换红色、绿色和蓝色。

为了测试这个设置，我们可以使用 Zephyr 中现有的示例：

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/drivers/led_strip
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

您将看到板载的 WS2812 LED 持续闪烁，依次显示红色、蓝色和绿色。

让我们深入了解这个示例，看看为什么它能正常工作：
```

 / {
     aliases {
         led-strip = &ws2812;
     };
 }
 &gpio0 {
     status = "okay";
     neopixel-power-enable {
		gpio-hog;
		gpios = <11 GPIO_ACTIVE_HIGH>;
		output-high;
	};
 };
 &pio0 {
     status = "okay";

     pio-ws2812 {
         compatible = "worldsemi,ws2812-rpi_pico-pio";
         status = "okay";
         pinctrl-0 = <&ws2812_pio0_default>;
         pinctrl-names = "default";
         bit-waveform = <3>, <3>, <4>;

         ws2812: ws2812 {
             status = "okay";
             gpios = <&gpio0 12 GPIO_ACTIVE_HIGH>;
             chain-length = <1>;
             color-mapping = <LED_COLOR_ID_GREEN
                      LED_COLOR_ID_RED
                      LED_COLOR_ID_BLUE>;
             reset-delay = <280>;
             frequency = <800000>;
         };
     };
 };
```

这些设备树的元素展示了板载 WS2812 的使用。由于 WS2812 的 VCC 引脚连接到 RP2040 的引脚 11，因此设备树使用 gpio-hog 功能，使得可以通过环境变量启用 LED。在这种情况下，引脚 12 用于 WS2812 数据线，所以通过启用 CONFIG_GPIO_HOGS 环境变量，LED 条带可以用于该示例。

之所以能正常工作，部分原因在于该示例有一个 xiao_rp2040.conf 文件，作为其开发板目录的一部分，合并了该配置并启用了它。

```
CONFIG_GPIO=y
CONFIG_GPIO_HOGS=y
```

如果您希望使用板载 WS2812，建议启用此变量，以允许其获取电力。

<!-- <div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Cosmic-Bee/xiao-zephyr-examples/main/images/rp2040/ws2812.gif" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_rp2040_zephyr/ws2812.gif" style={{width:300, height:'auto'}}/></div>

#### LED PWM

在这个示例中，我们将演示 Xiao RP2040 的 PWM 功能。我们将使用板载的蓝色 LED，通过 PWM 实现持续渐变效果。

为了测试这个设置，我们可以使用 Zephyr 中现有的示例：

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/basic/fade_led
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

您将看到 RGB 板载 LED 的蓝色灯光逐渐变暗并重复该过程。

让我们深入了解这个示例，看看为什么它能正常工作：
```
&pwm {
	status = "okay";
	divider-int-4 = <255>;
};
```

这段逻辑位于示例的 `boards/xiao_rp2040.overlay` 中，它启用了通常被禁用的 PWM 功能。Xiao RP2040 设置中，蓝色板载 RGB LED 被设置为默认 PWM。

如 Zephyr 开发板文件中的 `xiao_rp2040-pinctrl.dtsi` 所示，以下配置存在：
```
	pwm_ch4b_default: pwm_ch4b_default {
		group1 {
			pinmux = <PWM_4B_P25>;
		};
	};
```

在此案例中，PWM 使用配置的设备树 PWM LED，它与引脚 25（蓝色 LED）关联。PWM 引脚可以参考 [RP2040 文档](https://docs.zephyrproject.org/apidoc/latest/rpi-pico-rp2040-pinctrl_8h.html).

<!-- <div style={{textAlign:'center'}}><img src="https://raw.githubusercontent.com/Cosmic-Bee/xiao-zephyr-examples/main/images/rp2040/led_fade.gif" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_rp2040_zephyr/led_fade.gif" style={{width:300, height:'auto'}}/></div>

#### 时钟

我们将使用一个现有的示例和我们的控制台覆盖：
```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/drivers/counter/alarm -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.overlay -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.conf
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

连接并监视（快速重置开发板以确保其重启）：
```
screen /dev/ttyACM0 115200
```

您将看到一系列定时器在设置的延迟后相继触发：
```
*** Booting Zephyr OS build v3.6.0-2212-gc38ea288eee9 ***
Counter alarm sample

Set alarm in 2 sec (2000000 ticks)
!!! Alarm !!!
Now: 2
Set alarm in 4 sec (4000000 ticks)
!!! Alarm !!!
Now: 6
Set alarm in 8 sec (8000000 ticks)
!!! Alarm !!!
Now: 14
Set alarm in 16 sec (16000000 ticks)
!!! Alarm !!!
Now: 30
Set alarm in 32 sec (32000000 ticks)
```

#### TFLite - Hello World

启用 Zephyr 上的 TFLite，并更新：
```
west config manifest.project-filter -- +tflite-micro
west update
```

在这个示例中，我们将使用 tflite "Hello World" 示例，并配合控制台覆盖和配置文件，通过 USB 串行接口读取响应。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/modules/tflite-micro/hello_world -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.overlay -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.conf
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

连接到监视器：
```
screen /dev/ttyACM0 115200
```

您将看到从控制台返回的结果：
```
*** Booting Zephyr OS build v3.6.0-1155-g1a55caf8263e ***
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

### 附加组件

- [Grove - Expansion 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - I2C 显示器
- [Grove - Expansion 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - 俺就
- [Grove - Expansion 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - 蜂鸣器
- [Grove - Expansion 扩展板](https://www.seeedstudio.com/Seeeduino-XIAO-Expansion-board-p-4746.html) - SD 卡
- [Grove - 温湿度传感器 (SHT31)](https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-SHT31.html)
- [1.69 英寸 LCD 显示模块，240×280 分辨率，SPI 接口](https://www.seeedstudio.com/1-69inch-240-280-Resolution-IPS-LCD-Display-Module-p-5755.html)

#### Grove - 扩展板 - I2C 显示器

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/rp2040/xiao_expansion_oled.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_rp2040_zephyr/xiao_expansion_oled.jpg" style={{width:300, height:'auto'}}/></div>

为了测试此设置，我们可以使用 Zephyr 中现有的示例：

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/drivers/display -- -DSHIELD=seeed_xiao_expansion_board
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

您将看到显示屏显示多个黑色框和一个闪烁的框，因为此显示器仅支持两种颜色。

让我们深入了解这个示例，看看为什么它能正常工作：
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

此示例中的扩展板覆盖文件设置了一个 SSD1306 OLED 屏幕，位于 0x3C 寄存器。它在 chosen 部分中被选择为 Zephyr 显示器。

#### Grove - 扩展板 - 按钮

为了测试此设置，我们可以使用 Zephyr 中现有的示例，并配合 USB 控制台覆盖和配置文件。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/basic/button -- -DDTC_OVERLAY_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/console.overlay" -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.conf -DSHIELD=seeed_xiao_expansion_board
```

进入引导加载程序模式并刷新设备：
```
west flash -r uf2
```

连接到监视器：
```
screen /dev/ttyACM0 115200
```

按下按钮时，示例将触发板载 LED 点亮。

您将看到从控制台返回的结果：

```
*** Booting Zephyr OS build v3.6.0-2212-gc38ea288eee9 ***
Set up button at gpio@40014000 pin 27
Set up LED at gpio@40014000 pin 25
Press the button
Button pressed at 1934761489
Button pressed at 2178879257
Button pressed at 3084766465
Button pressed at 3388674993
```

让我们深入了解这个示例，看看为什么它能正常工作：
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

应用程序覆盖文件用于设置各种板载组件。通过这个文件，按钮示例可以被利用，覆盖文件使 Zephyr 配置按钮并使其可用于关联的代码。

在此案例中，GPIO 27 对应于 Xiao RP2040 的引脚 A1/D1。它在此覆盖中被设置为按钮，并被别名为 sw0 名称，以便它可以被用于示例，该示例的代码会期望它。

#### Grove - 扩展板 - 蜂鸣器

我们将通过PWM信号控制蜂鸣器的激活，使用闪烁PWM示例来实现。为此，我们将使用自定义的overlay来启用A3引脚的PWM。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/basic/blinky_pwm -- -DDTC_OVERLAY_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/xiao-rp2040/xiao_expansion_buzzer.overlay"
```

进入引导加载模式并刷新设备：
```
west flash -r uf2
```

刷新uf2文件后，您应该开始听到一系列的蜂鸣声，随着示例的运行，声音会有所变化。

让我们看看为什么这样设置有效：
```
/delete-node/ &pwm_led0;

/ {
	aliases {
		pwm-led = &pwm_led0;
	};
};

&{/pwm_leds} {
	status = "okay";
	compatible = "pwm-leds";

	pwm_led0: pwm_led0 {
		status = "okay";
		pwms = <&pwm 13 PWM_HZ(880) PWM_POLARITY_NORMAL>;
	};
};

&pinctrl {
	pwm_ch6b_default: pwm_ch6b_default {
		group1 {
			pinmux = <PWM_6B_P29>;
		};
	};
};

&pwm {
	status = "okay";
	pinctrl-0 = <&pwm_ch6b_default>;
	divider-frac-6 = <15>;
	divider-int-6 = <255>;
};
```

此overlay首先删除现有的 `pwm_led0` 节点，因为该板已通过板载LED设置支持PWM。接着，它配置A3引脚作为PWM。

我们在这里使用的是通道6B的PWM，因为A3引脚在Xiao RP2040板上对应的是引脚29。有关更多信息和其他引脚的引脚映射，请参见 [RP2040引脚控制文档](https://docs.zephyrproject.org/apidoc/latest/rpi-pico-rp2040-pinctrl_8h.html#a8c0c1058a626d83ba5f7e18238aba150) 。

#### Grove - 扩展板 - SD卡

在此，我们将使用文件系统示例，并结合Xiao扩展板的shield尝试通过SPI接口与SD卡读取器进行交互。扩展板shield已经将CS引脚配置为关联的 `&xiao_d 2` 引脚，因此您无需额外配置此功能，只需要添加shield即可。为了进一步准备，我们使用了启用SD卡功能的自定义配置。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/subsys/fs/fs_sample -- -DDTC_OVERLAY_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/console.overlay" -DEXTRA_CONF_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/console.conf $(dirname $(pwd))/applications/xiao-zephyr-examples/xiao_expansion_sd.conf" -DSHIELD=seeed_xiao_expansion_board
```

进入引导加载模式并刷新设备：
```
west flash -r uf2
```

连接到监视器：
```
screen /dev/ttyACM0 115200
```

```
*** Booting Zephyr OS build v3.6.0-2566-gc9b45bf4672a ***
[00:00:00.201,000] <inf> sd: Maximum SD clock is under 25MHz, using clock of 24000000Hz
[00:00:00.202,000] <inf> main: Block count 15519744
Sector size 512
Memory Size(MB) 7578
Disk mounted.

Listing dir /SD: ...
[FILE] IMAGE1.JPG (size = 58422)
[FILE] IMAGE2.JPG (size = 97963)
```

在这种情况下，我的SD卡中有两个文件。它们的名称和大小被输出到控制台。

让我们来看一下这里使用的相关元素：
```
CONFIG_SPI=y
CONFIG_DISK_DRIVER_SDMMC=y
CONFIG_GPIO=y
```

在相关的配置中，我们启用了SPI、SDMMC磁盘驱动程序和GPIO。如果没有此配置，overlay会导致错误，因为示例无法找到SD卡。

Xiao扩展板shield的相关部分如下所示：

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

如前所述， `&xiao_d 2` 引脚映射用于让D2引脚能够为此功能提供支持，无论使用何种板，只要它支持 `&xiao_d` 引脚设置。

#### Grove - 温湿度传感器 (SHT31)

首先焊接引脚并将Xiao RP2040连接到扩展板。然后，使用Grove连接线将Grove SHT31连接到扩展板的一个I2C端口。

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/rp2040/xiao_sht31.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_rp2040_zephyr/xiao_sht31.jpg" style={{width:500, height:'auto'}}/></div>

为了测试这个设置，我们可以使用Zephyr的现有示例，并通过overlay和conf启用USB控制台支持。

```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/sensor/sht3xd -- -DDTC_OVERLAY_FILE="$(dirname $(pwd))/applications/xiao-zephyr-examples/sht31.overlay $(dirname $(pwd))/applications/xiao-zephyr-examples/console.overlay" -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/console.conf
```

进入引导加载模式并刷新设备：
```
west flash -r uf2
```

连接到监视器：
```
screen /dev/ttyACM0 115200
```

您将看到从控制台返回的结果：
```
*** Booting Zephyr OS build v3.6.0-2212-gc38ea288eee9 ***
SHT3XD: 26.20 Cel ; 52.49 %RH
SHT3XD: 26.19 Cel ; 52.69 %RH
SHT3XD: 26.20 Cel ; 52.75 %RH
SHT3XD: 26.24 Cel ; 52.88 %RH
SHT3XD: 26.24 Cel ; 52.67 %RH
SHT3XD: 26.23 Cel ; 52.49 %RH
SHT3XD: 26.23 Cel ; 52.48 %RH
SHT3XD: 26.24 Cel ; 52.30 %RH
```

让我们深入了解一下这个示例为什么有效：
```
 &xiao_i2c {
	sht3xd@44 {
			compatible = "sensirion,sht3xd";
			reg = <0x44>;
		};
	};
```

应用overlay文件用于设置各种板组件。使用该文件，SHT31示例可以根据overlay提供的信息配置传感器，从而使 [示例逻辑](https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/sensor/sht3xd/src/main.c) 知道如何配置我们板上的传感器。

#### 1.69英寸LCD显示模块，240×280分辨率，SPI接口

在这个示例中，我们将使用SPI连接一个1.69英寸的LCD显示屏，分辨率为240×280。

首先，使用以下图片连接您的板子与LCD屏幕（在此示例中，我们使用的是Xiao RP2040，但同样的引脚布局也适用于其他连接）。

| 1.69英寸LCD SPI显示屏| XIAO RP2040 |
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

接下来，硬件准备好后，我们可以构建uf2文件并进行烧录：
```
cd ~/zephyrproject/zephyr
west build -p always -b xiao_rp2040 samples/drivers/display -- -DDTC_OVERLAY_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/240x280_st7789v2.overlay -DEXTRA_CONF_FILE=$(dirname $(pwd))/applications/xiao-zephyr-examples/240x280_st7789v2.conf
```

进入引导加载模式并刷新设备：
```
west flash -r uf2
```

刷新完成后，设备现在会显示与扩展板上相同的演示屏幕，只不过这是通过SPI接口更新为彩色LCD。

<!-- <div style={{textAlign:'center'}}><img src="https://github.com/Cosmic-Bee/xiao-zephyr-examples/blob/main/images/rp2040/spi_lcd.jpg?raw=true" style={{width:300, height:'auto'}}/></div> -->
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/wiki-ranger/Contributions/xiao_rp2040_zephyr/spi_lcd.jpg" style={{width:500, height:'auto'}}/></div>

## ✨ 贡献者项目

- 该项目由Seeed Studio [贡献者项目](https://github.com/orgs/Seeed-Studio/projects/6/views/1?pane=issue&itemId=57293558)支持。
- 感谢 **Tim的努力** ，您的工作将会被 [展示](https://wiki.seeedstudio.com/Honorary-Contributors/)。


## 技术支持与产品讨论

感谢您选择我们的产品！我们提供不同的支持方式，确保您使用我们产品的体验尽可能顺利。我们提供多个沟通渠道，以适应不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
