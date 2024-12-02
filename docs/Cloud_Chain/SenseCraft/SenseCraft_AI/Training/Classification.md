---
description: How to use Training(Classification)
title: Classification
image: https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/main_page.png
slug: /sensecraft_ai_Training_Classification
sidebar_position: 1
last_update:
  date: 11/27/2024
  author: qiuyu wei
---

# Type of training - Classification
**Classification** training is a machine learning method that learns the relationship between data and categories by giving the model sample data labelled with categories, ultimately enabling the model to classify new data into predefined categories.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/main_page.png" style={{width:700, height:'auto'}}/></div>

## Getting Started
Next, we will show you how to train a classification model of your own. The model will recognise whether people are wearing masks or not.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/11.png" style={{width:800, height:'auto'}}/></div>

## Training to recognise mask models
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/main.png" style={{width:800, height:'auto'}}/></div>

**step 1.** Connect the device to the computer via USB, the device selected in this demo is XIAO ESP32S3 Sense. Select the corresponding device and click **Connect**, then select the **correct serial port** for connection.
:::tip
If the device is successfully connected, a live preview of the camera will appear in the right-hand box.
:::
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/main3.png" style={{width:800, height:'auto'}}/></div>

**step 2.** This demo is to identify whether people are wearing masks or not, so we can see that we need to create two categories, after creating the categories you need to rename the different categories.
**Category 1:** Wear masks;
**Category 2:** No mask worn.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/name4.png" style={{width:800, height:'auto'}}/></div>

**step 3.** Select the appropriate category and capture the corresponding content with the camera. 
:::tip
Press and hold **‘Hold to Record’** to take a picture. The higher the number of relevant photos, the higher the recognition accuracy of the model.
:::
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/mask5.png" style={{width:800, height:'auto'}}/></div>
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/mask6.png" style={{width:800, height:'auto'}}/></div>

**step 4.** Once you have collected a sufficient number of images by category, you can click **‘Start Training’** to train the model.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/training7.png" style={{width:800, height:'auto'}}/></div>

:::tip
Model training time is about 1-3 minutes, please be patient!
:::

**step 5.** After the model training is completed, we can deploy the operation, in this demo we use the **XIAO ESP32S3 Sense**, so we need to select the appropriate device, and then click **‘Deploy to device’**.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/deploy8.png" style={{width:800, height:'auto'}}/></div>

Then click **‘Confirm’**, and finally select the correct **serial port** for device connection, to complete the above operation model will officially start deployment. The process will also last 1-3 minutes, please be patient!
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/deploy9.png" style={{width:800, height:'auto'}}/></div>
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/deploy10.png" style={{width:800, height:'auto'}}/></div>

## Demonstration of results
After completing the above steps, the mask recognition model has been successfully trained and deployed, next you can point the camera at yourself to test the actual effect.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/12.png" style={{width:800, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/SenseCraft_AI/img3/classification/13.png" style={{width:800, height:'auto'}}/></div>

You can try to train the model you want according to the above method, you can also replace Grove Vision AI (V2) for testing, the method and steps are the same.

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

