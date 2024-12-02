---
description: Hardware Interface
title: Hardware Interface
keywords:
  - Edge
  - reCamera
image: https://files.seeedstudio.com/wiki/reCamera/image.webp
slug: /reCamera_hardware_interface
last_update:
  date: 11/11/2024
  author: Parker Hu
---
# Hardware Interface of reCamera

- [Camera](#jump1)
- [Light](#jump2)
- [Mic && Speaker](#jump3)
- [WIFI](#jump4)
- [SD Card](#jump5)
- [Uart (Debug)](#jump6)

## <span id="jump1"> Camera </span>
Regarding the camera call, we have adapted node for recamera, you can get the camera's image data in node-red.
<br />
Connect the recamera to your computer using **type-c** and access node-red by visiting `192.168.42.1:1880`. You can also access node-red from your recamera network on port `LocalIP:1880` after the recamera is connected.
<br />
After the image configuration, you can get the camera's base64 image data and the RTSP's H.264 image push data.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image.png" /></div>

By accessing `rtsp://192.168.42.1:554/live` to see the h.264 video stream

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-1.png" /></div>

### Flow Export
```json
[{"id":"86fe9b81.31c418","type":"http request","z":"fafa85fb.98fd68","name":"","method":"GET","ret":"bin","paytoqs":false,"url":"http://192.168.178.21/tmpfs/snap.jpg?usr=admin&pwd=instar","tls":"","proxy":"","authType":"","x":265,"y":343,"wires":[["4068ec45.05b034"]]},{"id":"54d0ea94.c1b9a4","type":"ui_template","z":"fafa85fb.98fd68","group":"73b331f4.a8bda","name":"Display image","order":1,"width":"6","height":"6","format":"\n<img alt=\"HTTP Snap\" src=\"data:image/jpg;base64,{{msg.payload}}\" />\n","storeOutMessages":true,"fwdInMessages":true,"templateScope":"local","x":575.3645515441895,"y":343.6041603088379,"wires":[[]]},{"id":"4068ec45.05b034","type":"base64","z":"fafa85fb.98fd68","name":"","action":"","property":"payload","x":415,"y":343,"wires":[["54d0ea94.c1b9a4"]]},{"id":"3fa57e6a.6819f2","type":"inject","z":"fafa85fb.98fd68","name":"","topic":"","payload":"","payloadType":"date","repeat":"","crontab":"","once":false,"onceDelay":0.1,"x":115,"y":343,"wires":[["86fe9b81.31c418"]]},{"id":"73b331f4.a8bda","type":"ui_group","z":"","name":"HTTP Snap","tab":"d75e440d.a22678","disp":true,"width":"6","collapse":false},{"id":"d75e440d.a22678","type":"ui_tab","z":"fafa85fb.98fd68","name":"Cameras","icon":"camera_alt","order":2}]
```
## <span id="jump2">Light</span>

There are four indicators on the recamera, three of which are controllable indicators and the **green** indicator is the power indicator. 
Among them, there are four **white** lights, which are the fill lights of the camera.
**Red** is the status indicator of the CPU and **blue** is the reading status indicator of the system emmc.
<br />
The switch of the fill light and the red and blue indicator lights can be controlled by the following instructions.

``` bash
echo 1 > /sys/devices/platform/leds/leds/white/brightness //light on
echo 0 > /sys/devices/platform/leds/leds/white/brightness //light off

echo 1 > /sys/devices/platform/leds/leds/red/brightness //red indicator
echo 0 > /sys/devices/platform/leds/leds/red/brightness 

echo 1 > /sys/devices/platform/leds/leds/blue/brightness //blue indicator
echo 0 > /sys/devices/platform/leds/leds/blue/brightness 
```

## <span id="jump3">Mic && Speaker</span>

The recamera has a microphone and a speaker. You can invoke the microphone and speaker by following the command. The recamera can play audio files in **wav** format.

The default format of the player is: 16bit bitrate; The sampling rate is 16,000

```bash
sudo arecord -D hw:0,0 -r 16000 -f S16_LE -c 1 -d 5 /home/recamera/test.wav //Record five seconds of audio

sudo aplay -D hw:1,0 /home/recamera/test.wav //Playing audio
```

## <span id="jump4">WIFI</span>

The wifi of recamera is AP+STA dual mode, which can be used to configure the device 
network or configure the device in AP mode.

The **SSID** of wifi in AP mode is: `reCamera_+ the last six bits of MAC`.
<br />
The **Passwd** of wifi in AP mode is: `12345678`.
<br />
The ssid and passwd of the WiFi_AP can be configured in the
 /etc/hostapd_2g4.conf file on the recamera system.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-2.png" /></div>

The reCamera STA configuration file is in **/etc/wpa_supplicant.conf**, where you configure the WiFi account and password to connect to.
In STA mode, please connect to WiFi in **5G** band.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-3.png" /></div>

In the terminal, you can scan and connect to WiFi with the following command:

```bash
wpa_cli -i wlan0 scan                           #Start a scan

wpa_cli -i wlan0 scan_results                   #Returns wifi scan results

wpa_cli add_network                             # Adding a new network,Returns a network ID
wpa_cli set_network ID ssid "your_wifi_name"    # set network SSID
wpa_cli set_network ID psk "your_wifi_password" # set network passwd
wpa_cli enable_network ID                       # enable network and connect
wpa_cli status                                  # Checking connection status
```

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-4.png" /></div>

## <span id="jump5">SD Card</span>

The SD card slot can expand the storage space of the recamera. After inserting the SD card, you can find the existence of your SD by using the `lsblk` command.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-5.png" /></div>

Once you have found a storage device like mmcblk1p1, you can mount it to your system using the following command.
<br/>

Note that recamera only recognizes **ext4** filesystems.

```bash
sudo blkid /dev/mmcblk1p1 #If TYPE is not ext4, the filesystem needs to be formatted
/dev/mmcblk1p1: UUID="F080-C970" BLOCK_SIZE="512" TYPE="exfat" PARTUUID="04ec0085-01"

sudo mkfs.ext4 /dev/mmcblk1p1 #Format the filesystem to ext4
mkdir /userdata/mmcblk1 #Create a mount file
sudo mount -t ext4 /dev/mmcblk1p1 /userdata/mmcblk1 #Mount the SD card to the /userdata/mmcblk1 path
```
Check the mount result by `lsblk`

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-6.png" /></div>

## <span id="jump6">Uart (Debug)</span>

The base circuit board of the recamera has three contacts tx,rx, and gnd, which serve as the debug serial port of the recamera.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-7.png" /></div>

The device has a serial port baud rate of 115200 and can log into the recamera system.

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-8.png" /></div>

<div align="center"><img width={600} src="https://files.seeedstudio.com/wiki/reCamera/image-9.png" /></div>


## Tech Support & Product Discussion

Thank you for choosing our products! We are here to provide you with different support to ensure that your experience with our products is as smooth as possible. We offer several communication channels to cater to different preferences and needs.

<div class="button_tech_support_container">
<a href="https://forum.seeedstudio.com/" class="button_forum"></a> 
<a href="https://www.seeedstudio.com/contacts" class="button_email"></a>
</div>

<div class="button_tech_support_container">
<a href="https://discord.gg/eWkprNDMU7" class="button_discord"></a> 
<a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" class="button_discussion"></a>
</div>
