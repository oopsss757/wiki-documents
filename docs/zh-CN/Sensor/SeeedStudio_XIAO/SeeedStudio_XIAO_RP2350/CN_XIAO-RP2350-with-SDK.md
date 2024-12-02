---
description: “在 XIAO RP2350 上使用 Pico SDK”
title: Seeed Studio XIAO RP2350 与 C/C++ SDK
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /cn/xiao-rp2350-c-cpp-sdk
sidebar_position: 2
last_update:
    date: 11/18/2024
    author: Agnes
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## 介绍

Seeed Studio XIAO RP2350 配备 RP2350 微控制器，提供强大的性能，且体积小巧。本指南将提供设置和使用 C/C++ SDK 的基本步骤，帮助您在 XIAO RP2350 上开始开发。

## 前提条件

在开始之前，请确保您具备以下条件：

- 一台运行 Windows、macOS 或 Linux 的计算机。
- 一根用于将 XIAO RP2350 连接到计算机的 USB 电缆。
- 基本的 C/C++ 编程知识。

## 通过 Vscode 安装指南

:::info
对于那些喜欢本地开发体验的用户，可以参考 [Raspberry Pi Pico C/C++ SDK 文档](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) 或 [Raspberry Pi Pico SDK | GitHub](https://github.com/raspberrypi/pico-sdk)。
:::

为了获得更简单和更流畅的 SDK 编程体验，尤其适合新手，您可以为 Visual Studio Code (VSCode) 安装 [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) 扩展。

此扩展简化了设置过程，通过引导您完成必要的工具链安装，避免了手动单独安装每个工具。不过，您仍然需要确保您的系统符合平台要求：Windows x64、macOS（Sonoma 及更新版本）、Linux x64 或 arm64。

有关适合您操作系统的详细安装说明，请参考 [Raspberry Pi Pico Extension for VSCode](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) 页面。

#### 步骤 1：安装扩展

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/0-install-pico-extension.png" style={{width:500, height:'auto'}}/>
<div style={{ marginTop: '-8px' }}><em>在 VSCode 中安装扩展</em></div>
<br></br>
</div>

#### 步骤 2：创建新项目

页面加载后，您将看到所需的内容。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/1-new-example-project.png" style={{width:500, height:'auto'}}/>
<div style={{ marginTop: '-8px' }}><em>从示例中创建新项目</em></div>
<br></br>
</div>

尝试通过 `从示例创建新项目` 创建一个项目。

#### 步骤 3：配置您的项目

- **名称：** 通常这是示例项目的名称；在此，我们选择 `blink` 项目。
- **板类型：** `Pico 2`
- **位置：** 选择您希望存储 XIAO RP2350 项目的位置。
- **SDK 版本：** 必须是 `v2.0.0` 或更高版本。
- **调试器：** 如果您计划使用 SWD 调试接口，请勾选 SWD 调试器选项以便以后启用调试。

<Tabs>
<TabItem value="c1" label="配置项目">

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/2-create-blink-project.png" style={{width:500, height:'auto'}}/></div>

</TabItem>

<TabItem value="c2" label="高级选项">

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/3-advanced-options.png" style={{width:500, height:'auto'}}/></div>

如果您想对工具链设置进行微调并避免下载冗余资源，可以勾选 **高级选项**。在这里，您可以为 Ninja 和 CMake 等工具指定路径。如果您以前没有安装 CMake 或 Python 环境，或者不想担心这些问题，可以跳过此步骤。

在此示例中，我将使用已经在 Windows 计算机上安装并添加到系统 PATH 的系统版本。因此，我选择 **使用系统版本**。

</TabItem>
</Tabs>

如果这是您第一次运行设置，当您点击 `创建` 时，扩展会为您下载并管理 SDK。在 Windows 上，SDK 通常会放置在 `%userprofile%\.pico-sdk` 目录下。安装所需时间取决于您的互联网速度。完成后，将会打开一个新窗口，显示您的项目。

#### 步骤 4：构建项目

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/4-blink-example-created.png" style={{width:500, height:'auto'}}/></div>

:::caution 注意

第一次设置项目时，您需要手动修改 CMake 项目中的板类型，因为扩展默认不包含 XIAO RP2350 板。将板类型设置为 `seeed_xiao_rp2350`，如下所示：

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/set-xiao-rp2350-board.png" style={{width:500, height:'auto'}}/></div>

**修改板类型后，请清理 `build` 文件夹**，以确保使用 `%userprofile%/.pico-sdk/sdk/2.0.0/src/boards/include/boards/seeed_xiao_rp2350.h` 中的正确板配置。然后输入以下命令，在构建文件夹中生成 CMake 缓存：
```shell
cmake .. # 在 build 文件夹中执行
```
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/get-cmake-cache.png" style={{width:500, height:'auto'}}/></div>

这将使扩展的编译任务正常工作。

:::

<Tabs>
<TabItem value="compile" label="编译项目">

现在您可以按 **编译** 按钮来构建项目。此操作将在 `build` 文件夹中生成 `blink.uf2` 文件，您可以将其拖放到计算机识别的 RP2350 驱动器中。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/5-compile-project.png" style={{width:500, height:'auto'}}/></div>

</TabItem>

<TabItem value="run" label="运行项目">

如果设备处于 BOOT 模式，您可以按 **运行** 按钮来编译并自动将 `.uf2` 文件复制到 RP2350，省去手动拖放文件的步骤。

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/6-run-project.png" style={{width:500, height:'auto'}}/></div>

</TabItem>
</Tabs>

我们刚刚设置了开发环境，并成功使用 Raspberry Pi Pico 扩展创建了一个新项目。项目已准备就绪，工具也已配置好，您可以轻松地编译并在 XIAO RP2350 上运行您的代码，从而简化开发流程。


## 示例 1：LED 闪烁

为了演示基本的 SDK 使用方法，以下示例展示了如何编程控制板载 LED 闪烁：

```c title="blink.c"
#include "pico/stdlib.h"

const int sleep_time = 250;

int main() {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN; // GPIO25
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(sleep_time);
        gpio_put(LED_PIN, 0);
        sleep_ms(sleep_time);
    }
}
```
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/rp2350-blink.gif" style={{width:400, height:'auto', "border-radius": '12.8px'}}/></div>

## 示例 2: RGB 闪烁

<Tabs>
<TabItem value="ws2812.c" label="ws2812.c">

```c
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

#define IS_RGBW true
#define NUM_PIXELS 1

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// 如果板子没有默认的 WS2812 引脚定义，则默认为引脚 2
#define WS2812_PIN 22
#endif

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void pattern_snakes(uint len, uint t) {
    for (uint i = 0; i < len; ++i) {
        uint x = (i + (t >> 1)) % 64;
        if (x < 10)
            put_pixel(urgb_u32(0xff, 0, 0));
        else if (x >= 15 && x < 25)
            put_pixel(urgb_u32(0, 0xff, 0));
        else if (x >= 30 && x < 40)
            put_pixel(urgb_u32(0, 0, 0xff));
        else
            put_pixel(0);
    }
}

void pattern_random(uint len, uint t) {
    if (t % 8)
        return;
    for (int i = 0; i < len; ++i)
        put_pixel(rand());
}

void pattern_sparkle(uint len, uint t) {
    if (t % 8)
        return;
    for (int i = 0; i < len; ++i)
        put_pixel(rand() % 16 ? 0 : 0xffffffff);
}

void pattern_greys(uint len, uint t) {
    int max = 100; // 为了不消耗过多电流！
    t %= max;
    for (int i = 0; i < len; ++i) {
        put_pixel(t * 0x10101);
        if (++t >= max) t = 0;
    }
}

typedef void (*pattern)(uint len, uint t);
const struct {
    pattern pat;
    const char *name;
} pattern_table[] = {
        {pattern_snakes,  "蛇形!"},
        {pattern_random,  "随机数据"},
        {pattern_sparkle, "闪光"},
        {pattern_greys,   "灰度"},
};

int main() {
    //set_sys_clock_48();
    stdio_init_all();

    const int RGB_POWER = 23;
    gpio_init(RGB_POWER);
    gpio_set_dir(RGB_POWER, GPIO_OUT);
    gpio_put(RGB_POWER, 1);

    printf("WS2812 烟雾测试，使用引脚 %d", WS2812_PIN);

    // todo 获取空闲的状态机
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    int t = 0;
    while (1) {
        int pat = rand() % count_of(pattern_table);
        int dir = (rand() >> 30) & 1 ? 1 : -1;
        puts(pattern_table[pat].name);
        puts(dir == 1 ? "(正向)" : "(反向)");
        for (int i = 0; i < 1000; ++i) {
            pattern_table[pat].pat(NUM_PIXELS, t);
            sleep_ms(10);
            t += dir;
        }
    }
}
```
</TabItem>
<TabItem value="ws2812.pio" label="ws2812.pio">

```assembly
;
; Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
;
; SPDX-License-Identifier: BSD-3-Clause
;

.program ws2812
.side_set 1

.define public T1 2
.define public T2 5
.define public T3 3

.lang_opt python sideset_init = pico.PIO.OUT_HIGH
.lang_opt python out_init     = pico.PIO.OUT_HIGH
.lang_opt python out_shiftdir = 1

.wrap_target
bitloop:
    out x, 1       side 0 [T3 - 1] ; 即使指令停顿时，Side-set仍然会发生
    jmp !x do_zero side 1 [T1 - 1] ; 根据我们移出的位进行跳转。正脉冲
do_one:
    jmp  bitloop   side 1 [T2 - 1] ; 继续驱动高电平，产生长脉冲
do_zero:
    nop            side 0 [T2 - 1] ; 或者驱动低电平，产生短脉冲
.wrap

% c-sdk {
#include "hardware/clocks.h"

static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq, bool rgbw) {

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
%}

.program ws2812_parallel

.define public T1 2
.define public T2 5
.define public T3 3

.wrap_target
    out x, 32
    mov pins, !null [T1-1]
    mov pins, x     [T2-1]
    mov pins, null  [T3-2]
.wrap

% c-sdk {
#include "hardware/clocks.h"

static inline void ws2812_parallel_program_init(PIO pio, uint sm, uint offset, uint pin_base, uint pin_count, float freq) {
    for(uint i=pin_base; i<pin_base+pin_count; i++) {
        pio_gpio_init(pio, i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, true);

    pio_sm_config c = ws2812_parallel_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, pin_base, pin_count);
    sm_config_set_set_pins(&c, pin_base, pin_count);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_parallel_T1 + ws2812_parallel_T2 + ws2812_parallel_T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
%}
```
</TabItem>

<TabItem value="ws2812-cmake" label="CMakeLists.txt">

复制以下代码段并将其附加到 `CMakeLists.txt` 文件中。

```cmake title="CMakeLists.txt"
project(pio_ws2812 C CXX ASM)

# 初始化 Raspberry Pi Pico SDK
pico_sdk_init()

# 为 pio_ws2812 添加可执行目标
add_executable(pio_ws2812)

# 为生成的文件创建一个目录
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# 从 ws2812.pio 生成 PIO 头文件
pico_generate_pio_header(pio_ws2812 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# 将源文件添加到 pio_ws2812 目标中
target_sources(pio_ws2812 PRIVATE ws2812.c)

# 将必要的库链接到 pio_ws2812 目标
target_link_libraries(pio_ws2812 PRIVATE pico_stdlib hardware_pio)

# 生成额外的输出格式（例如，UF2，BIN）
pico_add_extra_outputs(pio_ws2812)

# 可选地，从 PIO 汇编生成 Python 文件，以供进一步分析或文档使用
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio
    COMMAND pioasm -o python ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py
    VERBATIM)
add_custom_target(pio_ws2812_datasheet DEPENDS ${CMAKE_CURRENT_LIST_DIR}/generated/ws2812.py)
add_dependencies(pio_ws2812 pio_ws2812_datasheet)
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/11-ws2812-rgb-cmake.png" style={{width:500, height:'auto'}}/></div>

</TabItem>
</Tabs>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/rp2350-rgb.gif" style={{width:240, height:'auto', "border-radius": '12.8px' }}/></div>

## 示例 3: UART 打印

:::tip USB 串口
如果你想通过 USB 启用 `printf` 输出到计算机，你需要在项目的 `CMakeLists.txt` 文件中添加以下行：

```cmake
pico_enable_stdio_usb(your_project_name 1)
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/sdk/10-cmake-usb-enabled.png" alt="CMake USB Enabled" style={{width:400, height:'auto'}}/></div>

另外，确保在你的代码中初始化标准 I/O，通过在主函数中添加`stdio_init_all();` 。

:::

```c title="hello_uart.c"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <pico/stdio.h>
#include <pico/time.h>
#include <stdio.h>

#define UART_ID uart0
#define BAUD_RATE 115200

// 我们使用的是引脚 0 和 1，更多可用引脚请参见数据手册中的 GPIO 功能选择表
#define UART_TX_PIN 0
#define UART_RX_PIN 1

int main() {
   stdio_init_all();
  // 设置我们所需的波特率来初始化 UART
  uart_init(UART_ID, BAUD_RATE);

  // 通过使用 GPIO 的功能选择设置 TX 和 RX 引脚
  // 更多信息请参见数据手册中的功能选择
  gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
  gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

  // 使用一些 UART 函数来发送数据
  // 在默认系统中，printf 也会通过默认 UART 输出

  // 发送一个字符，不做任何转换
  uart_putc_raw(UART_ID, 'A');

  // 发送一个字符，进行 CR/LF 转换
  uart_putc(UART_ID, 'B');

  // 发送一个字符串，进行 CR/LF 转换
  uart_puts(UART_ID, " Hello, UART!\n");

  // 打印测试
  int i = 0;
  for (;;) {
    sleep_ms(500);
    printf("Hello %d", i++);
  }
}
```

## 示例 4: 读取电池电压

```c title="hello_adc.c"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

void init_gpio() {
    const int gpio = 19;

    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
}

int main() {
    stdio_init_all();
    printf("ADC 电池示例 - GPIO29 A3\n");

    init_gpio();
    adc_init();

    // 确保GPIO为高阻抗，没有上拉等
    adc_gpio_init(29);
    // 选择ADC输入 0（GPIO26）
    adc_select_input(3);

    while (1) {
        // 12位转换，假设最大值 == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("原始值: 0x%03x, 电压: %f V\n", result, result * conversion_factor * 2);
        sleep_ms(500);
    }
}
```

## 常见问题解答

#### TinyUSB 子模块未初始化；USB 支持不可用

**问题：**
在构建项目时，您可能会看到以下警告：

```plaintext
TinyUSB 子模块尚未初始化；USB 支持将不可用
```

**解决方案：**

1. 在 Linux/macOS 上打开终端，或在 Windows 上打开命令提示符/PowerShell/Git Bash。
2. **进入 Pico SDK 目录：**

   ```bash
   cd /path/to/your/pico-sdk
   ```

3. **初始化子模块：**

   ```bash
   git submodule update --init
   ```

这将为您的项目启用 USB 支持。

## 资源

- 🔗 **[链接]** [Raspberry Pi Pico SDK | GitHub](https://github.com/raspberrypi/pico-sdk)
- 📄 **[PDF]** [Raspberry Pi Pico系列 C/C++SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) — 该书记录了 SDK 的 API
- 📄 **[PDF]** [Raspberry Pi Pico系列入门指南](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) — 官方 Raspberry Pi 文档。
- 📽️ **[视频]** [Raspberry Pi Pico 和 RP2040 简介](https://www.youtube.com/watch?v=B5rQSoOmR5w) — 一段视频教程。

## 技术支持与产品讨论

感谢您选择我们的产品！我们为您提供多种支持方式，确保您在使用我们产品时的体验尽可能顺畅。我们提供多种沟通渠道，以满足不同的偏好和需求。

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>  