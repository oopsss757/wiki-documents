---
title: MR60BHA2 Breath-Heartbeat Sensor with Home Assistant
description: | 
  MR60BHA2 Heartbeat mmWave Sensor with Home Assistant
image: https://files.seeedstudio.com/wiki/seeed_logo/logo_2023.png
slug: /ha_with_mr60bha2
keywords:
  - ESPHome
sidebar_position: 1
last_update:
  date: 09/23/2024
  author: Spencer
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Introduction {#introduction}

The MR60BHA2 is a 60GHz mmWave Breathing and Heartbeat Detection Sensor module designed for integration with the XIAO ESP32C6 microcontroller. This advanced sensor utilizes millimeter-wave technology to provide non-invasive monitoring of vital signs and presence detection.

This guide aims to provide a clear and comprehensive walkthrough for integrating the MR60BHA2 mmWave Sensor with Home Assistant using the XIAO ESP32C6 microcontroller. By following this guide, users will learn how to set up the sensor for heartbeat detection, connect it to their Home Assistant environment, and utilize ESPHome to manage and monitor the device effectively.

This integration empowers users to enhance their smart home systems with advanced sensing capabilities, enabling automated responses and real-time monitoring for various applications.

<div><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-sensor-light-on.jpg" style={{"border-radius": '6px'}}/></div>

## Product Overview

<div class="table-center">
   <table align="center">
      <tr>
         <th>MR60BHA2 mmWave Sensor</th>
      </tr>
      <tr>
         <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/2-mmWave-45font.jpg" style={{width:360, height:'auto', "border-radius": '6px'}}/></div></td>
      </tr>
      <tr>
         <td>
            <div class="get_one_now_container" style={{textAlign: 'center'}}>
               <a class="get_one_now_item" href="https://www.seeedstudio.com/MR60BHA2-60GHz-mmWave-Sensor-Breathing-and-Heartbeat-Module-p-5945.html?utm_source=wiki"><strong><span><font color={'FFFFFF'} size={"4"}> Get One Now 🖱️</font></span></strong></a>
            </div>
         </td>
      </tr>
   </table>
</div>

### MR60BHA2 mmWave Sensor with XIAO ESP32C6

To effectively integrate the MR60BHA2 mmWave Sensor with Home Assistant using the XIAO ESP32C6, follow these essential steps:

1. **[Set Up Home Assistant](#setting-up-home-assistant)**: Begin by installing and configuring Home Assistant to manage your smart home devices, ensuring a seamless connection with the sensor.
2. **[Connect the MR60BHA2 Sensor](#discovering-and-adding-the-device-in-home-assistant)**: Learn how to discover and add the MR60BHA2 Sensor to your Home Assistant setup, enabling real-time monitoring of vital signs.
3. **[Monitor Sensor Data](#sensor-data-monitoring)**: Once integrated, you can monitor sensor data effectively, allowing for insights into heart rate and breathing patterns.
4. **[Implement Automation](#implementing-automation-in-home-assistant)**: Explore the powerful automation features of Home Assistant to create responsive actions based on the sensor data, enhancing your smart home environment.
5. **[Modify Firmware with ESPHome](#modifying-the-firmware-with-esphome)**: Utilize ESPHome to customize the sensor's functionality, tailoring it to meet your specific needs for greater flexibility and control.

These steps will guide you through the integration process, helping you make the most of your MR60BHA2 mmWave Sensor and XIAO ESP32C6 setup.

## Getting Started {#getting-started}

:::note Attention
Please note that when we refer to firmware updates or modifications, we are specifically addressing the firmware on the XIAO ESP32C6.
:::

To successfully integrate the MR60BHA2 mmWave Sensor with Home Assistant, you will need the following components:

- **Home Assistant**: A smart home platform that will manage the sensor data.
- **ESPHome Add-on**: Firmware that enables easy configuration and management of ESP32 devices.

### Step 1: Setting Up Home Assistant {#setting-up-home-assistant}

1. **Installation**: For optimal performance, it's recommended to install [Home Assistant OS](https://www.home-assistant.io/installation/) on a Virtual Machine or Raspberry Pi. Follow the official installation guide on the Home Assistant website.
2. **Enabling ESPHome Add-on**:
   - Access the Home Assistant dashboard.
   - Navigate to the "Add-ons" section and search for the ESPHome add-on.
   - Click "Install" and then "Start" to enable it.
   - Once installed, configure the add-on to ensure proper communication with the XIAO ESP32C6.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave_kit/2.png" style={{width:1000, height:'auto'}}/></div>

By gathering the necessary components and setting up Home Assistant with the ESPHome add-on, you'll be ready to proceed with the integration of the MR60BHA2 mmWave Sensor.

:::tip install Home Assistant
We have also written how to install Home Assistant for some of Seeed Studio products, please refer to them.

- [Getting Started with Home Assistant on ODYSSEY-X86](/ODYSSEY-X86-Home-Assistant)
- [Getting Started with Home Assistant on reTerminal](/reTerminal_Home_Assistant)
- [Getting Started with Home Assistant on LinkStar H68K/reRouter CM4](/h68k-ha-esphome)
:::

### Step 2: Preparing the Kit

By default, your device (XIAO ESP32C6) comes pre-flashed with firmware for breathing and heart rate detection. However, there are two scenarios where you may need to update the firmware:

1. **Re-flashing the Firmware**: If the existing firmware is corrupted or you need to start fresh.
2. **Upgrading the Firmware**: If there is a newer version of the firmware with improved functionality.

There are two simple methods for flashing the firmware:

<Tabs>
<TabItem value='Web Tool'>

You can use this [Web Tool](https://limengdu.github.io/MR60BHA2_ESPHome_external_components/) for an easy and direct method to flash your firmware. Simply follow the on-screen instructions.

- Click the `CONNECT` button to initiate the connection. The tool will automatically update the firmware.

If something goes wrong, follow the on-screen troubleshooting steps or switch to the `ESPHome Web` method to complete the process.

</TabItem>
<TabItem value='ESPHome Web'>

For this method, you'll need to download the `bin` firmware file from [here](https://github.com/limengdu/MR60BHA2_ESPHome_external_components/releases)(you'll need to unzip the downloaded file).

1. Connect the sensor kit to your PC.
2. Visit the [ESPHome Web](https://web.esphome.io/) page.
3. Select the firmware file with the `*.factory.bin` suffix.

Watch the following video for a detailed walkthrough of flashing the firmware via ESPHome Web:

<iframe class="youtube-video-r" src="https://www.youtube.com/embed/J3AVeZCoLK8?si=1AeNTsdmbTvMl0Nq" title="Install firmware via ESPHome Web" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

</TabItem>
</Tabs>

With either method, you'll have your firmware updated and ready for integration with Home Assistant.

#### Connect to the kit's hotspot

With the firmware, you could power on the sensor kit, and a Wi-Fi access point will appear: `seeedstudio-mr60bha2`.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/hotspot-name.png" style={{width:360, height:'auto', "border-radius": '15px'}}/></div>

Navigate to `192.168.4.1` to configure your Home Assistant server's local network settings.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/opt-for-wifi-ap.jpg" style={{width:360, height:'auto', "border-radius": '15px'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-enter-psw.JPEG" style={{width:360, height:'auto', "border-radius": '15px'}}/></div>

### Step 3: Discovering and Adding the Device in Home Assistant {#discovering-and-adding-the-device-in-home-assistant}

In this section, we'll go through the process using the Home Assistant app, where the logic is the same as on the web.

1. **Open the App**: Once you launch the app, select your Home Assistant server. The app will automatically find your server.

  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-server-option.JPG" style={{width:360, height:'auto', "border-radius": '15px'}}/></div>
2. **Create an Account**: If you haven't created an account, you'll need to do so. After that, log in with your credentials.
  <div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-login.JPG" style={{width:360, height:'auto', "border-radius": '15px'}}/></div>
3. **Navigate to the Integration Page**: Once logged in, go to the "Integrations" page in Home Assistant. If you have installed the ESPHome add-on and both the XIAO ESP32C6 and your Home Assistant server are on the same network, you should see the device `Seeed Studio MR60BHA2 Kit {device-mac-address}` appear under discovered devices.
4. **Add the Device**: Click to add the device to your Home Assistant setup.
  <div class="img-container" align="center">
    <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-device-find.JPG" alt="find device"/>
    <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-device-submit.JPG" alt="submit a device"/>
    <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-device-add.JPG" alt="area"/>
    <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-addon-device.JPG" alt="addon"/>
  </div>
  
  Click the `CONFIGURE` button, confirm by pressing the `SUBMIT` button, and assign the device to your preferred area (e.g., Bedroom). After this, the device will be managed through your ESPHome integration, enabling full control and monitoring in Home Assistant.

## Monitoring and Control {#monitoring-and-control}

By following the steps above, you'll have successfully discovered and added the MR60BHA2 Sensor to your Home Assistant, enabling real-time monitoring and control.

### Sensor data Monitoring

Now that the sensor is added to the "Bedroom," navigate to the "Overview" tab. You will see the mmWave card displayed in the Bedroom section.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-sensor-data.jpg" style={{width:680, height:'auto', "border-radius": '15px'}}/></div>

### RGB light Controlling

In this section, we will explore how to control an RGB light.

<div class="img-container">
   <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-sensor-light-on.jpg" style={{"border-radius": '6px'}}/>
   <img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/light-on.jpg" style={{"border-radius": '6px'}}/>
</div>

Click on the corresponding box to directly control the RGB light:

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-light-panel.jpg" style={{width:680, height:'auto', "border-radius": '15px'}}/></div>

<iframe class="video-mp4" src="https://files.seeedstudio.com/wiki/mmwave-for-xiao/mr60/mr60bha2/ha-control-light.mp4" title="Home Assistant Control RGB Light" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture;" allowfullscreen></iframe>

### Next Steps {#next-steps}

Now that you have successfully integrated the MR60BHA2 mmWave Sensor with the XIAO ESP32C6 and Home Assistant, you can explore further enhancements to make the most out of your setup. Here are two essential avenues to consider:

#### Implementing Automation in Home Assistant

Home Assistant's powerful automation features allow you to create a more responsive smart home environment based on the data collected from your MR60BHA2 Sensor. This means you can set up actions that occur automatically based on specific triggers related to motion or heartbeat detection.

Here's how to implement automations step-by-step:

1. **Navigate to the Automations Section**: In your Home Assistant dashboard, find and click on the "Automations" tab. This area is dedicated to creating and managing automated actions within your home.
2. **Create a New Automation**: Click on the "Add Automation" button. Home Assistant offers a user-friendly wizard that guides you through the process of setting up an automation.
3. **Define the Trigger**: Choose a trigger based on the MR60BHA2 Sensor's data. For example, you can set the automation to trigger when the sensor detects motion or a specific heartbeat pattern. This means your automation can respond immediately to the sensor's readings.
4. **Set Conditions (Optional)**: Conditions allow you to refine when the automation should execute. For instance, you might want the lights to turn on only when it's dark outside or if a specific user is home.
5. **Determine the Action**: Specify what action should occur when the trigger conditions are met. Actions can include turning on lights, sending notifications, or even adjusting thermostat settings. For example, you could create an action that turns on an RGB light when motion is detected, enhancing both security and comfort.
6. **Save and Test**: After configuring your automation, save it and test its functionality. Walk past the sensor to see if the lights turn on as expected. If any issues arise, you can return to the automation settings to troubleshoot and adjust.

By leveraging Home Assistant's automation capabilities, you can create a truly smart environment that responds to your movements and health metrics, ensuring that your living space adapts to your lifestyle seamlessly.

## Modifying the Firmware with ESPHome

One of the significant advantages of using the XIAO ESP32C6 is its compatibility with ESPHome, a powerful tool for managing and customizing your microcontroller firmware. With ESPHome, you can directly modify the firmware to tailor the functionality of the MR60BHA2 mmWave Sensor to your specific needs.

To get started with ESPHome, follow these steps:

1. **Access the ESPHome Dashboard**: In Home Assistant, navigate to the ESPHome add-on. You should see your XIAO ESP32C6 listed among the devices.

2. **Create a New Configuration**: Click on the device to open its configuration. Here, you can adjust settings such as the sensor's sensitivity, reporting intervals, and output formats. ESPHome uses a YAML configuration format, which is user-friendly and allows you to define various parameters. You can use the following template YAML file as a starting point for your configuration, which is designed specifically for the MR60BHA2 Sensor:

   ```yaml showLineNumbers title=example/mr60bha2.yaml
   # template from https://github.com/limengdu/MR60BHA2_ESPHome_external_components

   substitutions:
     name: "seeedstudio-mr60bha2-kit"
     friendly_name: "Seeed Studio MR60BHA2 Kit"

   esphome:
     name: "${name}"
     friendly_name: "${friendly_name}"
     name_add_mac_suffix: true
     project:
       name: "seeedstudio.mr60bha2_kit"
       version: "1.0"
     platformio_options:
       board_upload.maximum_size: 4194304
     min_version: "2024.3.2" # Fix logger compile error on ESP32-C6

   esp32:
     board: esp32-c6-devkitc-1
     variant: esp32c6
     flash_size: 4MB # upload.flash_size
     framework:
       type: esp-idf
       platform_version: 6.5.0 # Minimum version required for ESP32-C6

   external_components:
     - source:
         type: git
         url: https://github.com/ssieb/esphome
         ref: adc
       components: [ adc ]
       refresh: 1min
     - source:
         type: git
         url: https://github.com/limengdu/MR60BHA2_ESPHome_external_components
         ref: main
       components: [ seeed_mr60bha2 ]
       refresh: 0s

   logger:
     hardware_uart: USB_SERIAL_JTAG
     level: DEBUG

   api:

   ota:
   - platform: esphome

   wifi:
     ap:
       ssid: "seeedstudio-mr60bha2"

   light:
     - platform: esp32_rmt_led_strip
       id: led_ring
       name: "Seeed MR60BHA2 RGB Light"
       pin: GPIO1
       num_leds: 1
       rmt_channel: 0
       rgb_order: GRB
       chipset: ws2812

   i2c:
     sda: GPIO22
     scl: GPIO23
     scan: true
     id: bus_a

   uart:
     id: uart_bus
     baud_rate: 115200
     rx_pin: 17
     tx_pin: 16
     parity: NONE
     stop_bits: 1

   seeed_mr60bha2:
     id: my_seeed_mr60bha2

   sensor:
     - platform: bh1750
       name: "Seeed MR60BHA2 Illuminance"
       address: 0x23
       update_interval: 1s
     - platform: seeed_mr60bha2
       breath_rate:
         name: "Real-time respiratory rate"
       heart_rate:
         name: "Real-time heart rate"
       distance:
         name: "Distance to detection object"
   ```

3. **Customize Functionality**: You can enhance the sensor's capabilities by exploring various features available in ESPHome, allowing for flexible adjustments to suit your specific needs.

4. **Upload the Updated Firmware**: After making your modifications, save the configuration. The ESPHome dashboard allows you to upload the firmware directly over the air. Simply click the `Upload` button, and follow the prompts to complete the process. This streamlined method makes it easy to keep your firmware up to date.

5. **Test and Iterate**: Once the upload is complete, test your changes in real-time. Monitor the sensor's performance to ensure it operates as expected. If you encounter any issues, revisit the ESPHome dashboard to refine your settings. This iterative approach enables you to continually improve your firmware, ensuring it meets your requirements effectively.

By utilizing ESPHome, you empower yourself to make ongoing improvements to your sensor setup, adapting it to suit your evolving needs. This capability not only enhances your project's functionality but also provides a platform for learning and experimentation with IoT development.

Through these steps, you can maximize the functionality of your MR60BHA2 mmWave Sensor and XIAO ESP32C6 setup, transforming it into a highly customized and responsive smart home system tailored to your preferences and needs.

## Resources

- [ESPHome — ESPHome](https://esphome.io/)
- [Installation - Home Assistant](https://www.home-assistant.io/installation/)
- [limengdu/MR60BHA2_ESPHome_external_components](https://github.com/limengdu/MR60BHA2_ESPHome_external_components)

## Tech Support & Product Discussion

Thank you for choosing our products! We are here to provide you with different support to ensure that your experience with our products is as smooth as possible. We offer several communication channels to cater to different preferences and needs.

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
