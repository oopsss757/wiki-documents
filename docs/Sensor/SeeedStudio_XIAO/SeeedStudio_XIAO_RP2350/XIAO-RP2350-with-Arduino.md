---
description: Using Arduino on your XIAO RP2350 board
title: Getting Started with Seeed Studio XIAO RP2350(Arduino)
image: https://files.seeedstudio.com/wiki/XIAO-RP2350/img/2-102010550_XIAO_RP2350-45font_1.webp
slug: /xiao_rp2350_arduino
sidebar_position: 1
last_update:
  date: 2024-10-30T01:39:16.136Z
  author: Spencer
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Seeed Studio XIAO RP2350 with Arduino

The Seeed Studio XIAO RP2350 board now supports programming via Arduino, thanks to the [arduino-pico core](https://github.com/earlephilhower/arduino-pico). This guide will help you set up and begin using Arduino on your RP2350 board.

## Prerequisites

To get started, ensure you have:

- An RP2350 board
- The Arduino IDE
- A USB cable

## Setting Up the Software

### 1. Install the Arduino IDE

Download and install the latest Arduino IDE from the official site: [Arduino Software](https://www.arduino.cc/en/software).

### 2. Add RP2350 Board Support

1. Open the Arduino IDE and navigate to **File** > **Preferences**.
2. In the **Additional Boards Manager URLs** field, add this URL:

    ```shell
    https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    ```

    <div style={{ textAlign: 'center' }}>
    <img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-url.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
    </div>

3. Click **OK** to save your settings.
4. Go to **Tools** > **Board** > **Boards Manager**.
5. In the Boards Manager, search for **pico** and click **Install**.
6. After installation, go to **Tools** > **Board** and select the board shown below as your board.

:::note
Ensure you install version 4.2.0 or later for full support of the XIAO RP2350 board.
:::

<div style={{ textAlign: 'center' }}>
<img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-board-option.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
</div>

### 3. Uploading a Sketch

Before uploading a sketch, place your XIAO RP2350 into BOOT mode. Use one of the methods below:

<Tabs>
<TabItem value="method1" label="Method 1: Before Connecting to Computer" default>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-no-charge.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/><div style={{ marginTop: '-8px' }}><em>Hold Boot-> Plug in Cable-> Release Boot</em></div></div>

</TabItem>

<TabItem value="method2" label="Method 2: While Connected to Computer">

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/enter-boot-charged.gif" style={{width:500, height:'auto', "border-radius": '12.8px' }}/><div style={{ marginTop: '-8px' }}><em>Hold Boot-> Click Reset-> Release Boot</em></div></div>

</TabItem>
</Tabs>

1. Open the Arduino IDE and create a new sketch.
2. Write your code. For example, use the `Blink` example code.
3. Go to **Tools** > **Port** and select the port where your RP2350 is connected.

<div style={{ textAlign: 'center' }}>
<img src="https://files.seeedstudio.com/wiki/XIAO-RP2350/img/arduino-firmware-upload.png" style={{ width: 680, height: 'auto', "border-radius": '12.8px' }} />
</div>

## Additional Resources

- [arduino-pico GitHub](https://github.com/earlephilhower/arduino-pico)
- [Arduino-Pico Core Documentation](https://arduino-pico.readthedocs.io/en/latest/install.html)

## Support & Discussion

Thank you for using Seeed products! We offer multiple channels for support and community discussion:

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a>
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/kpY74apCWj" class="button_discord"></a>
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
