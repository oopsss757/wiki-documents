---
description: Install and Upgrade OS
title: System update & upgrade
keywords:
  - Edge
  - reCamera
image: https://files.seeedstudio.com/wiki/reCamera/001.webp
slug: /recamera_flash_os
last_update:
  date: 11/6/2024
  author: Parker Hu
---

# reCamera System update & upgrade

You can check out the latest version of the [reCamera OS here.](https://github.com/Seeed-Studio/reCamera-OS)

Manually update the latest OTA firmware:
```bash
/mnt/system/upgrade.sh latest https://github.com/Seeed-Studio/reCamera-OS/releases/latest 

/mnt/system/upgrade.sh start
```

Manually update the specified version of OTA firmware. You can also update the system version in your branch.

```bash
/mnt/system/upgrade.sh latest https://github.com/Seeed-Studio/reCamera-OS/releases/download/0.1.3/sg2002_reccamera_emmc_md5sum.txt #Replace it with your branch link

/mnt/system/upgrade.sh start
```

Update the firmware manually using the local ota package.

```bash
/mnt/system/upgrade.sh start sg2002_reCamera_0.1.3_emmc_ota.zip
```
You can change system files to be readable or writable by typing `rootfs_rw on/off`.

Manually restore factory Settings.

```bash
/mnt/system/upgrade.sh recovery
```

## Resources

[reCamera OS](https://github.com/Seeed-Studio/reCamera-OS)

[reCamera Series](https://github.com/Seeed-Studio/OSHW-reCamera-Series)

[reCamera SSCMA](https://github.com/Seeed-Studio/sscma-example-sg200x)

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
