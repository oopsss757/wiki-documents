---
title: Microphone and 6-Axis IMU Usage for Sense Version
description: This article describes how to use the Microphone on the XIAO MG24 Sense.
image: https://files.seeedstudio.com/wiki/mg24_mic/mg24.jpg
slug: /xiao_mg24_mic_and_6-axis_sense
keywords:
  - XIAO
  - MG24
last_update:
  date: 11/20/2024 
  author: qiu yu wei
sidebar_position: 2
---

# Usage of Seeed Studio XIAO MG24 Sense Microphone and 6-Axis IMU

:::caution
All contents of this tutorial are applicable to XIAO MG24 Sense only.
:::

## Getting Started

This tutorial describes how to use the **microphone** and **6-axis IMU** on the **XIAO MG24 Sense** based on the **Arduino IDE**.Before starting this tutorial, it is necessary to configure the environment to ensure that the programme will run smoothly.

### Preparing the Development Environment

**step 1.** Click on **File**, then **Preference**, and add the following URL.

```
https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json 
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/1.png" style={{width:850, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/2.png" style={{width:850, height:'auto'}}/></div>

**step 2.** Select **Silicon Labs** for installation.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/3.png" style={{width:850, height:'auto'}}/></div>

:::caution
At this moment, XIAO MG24 cannot be searched in the Board, it is necessary to update the relevant firmware.
:::

**step 3.** Complex the file into the target path and delete the **original 2.1.0 folder**.

<https://files.seeedstudio.com/wiki/mg24_mic/silabs_arduino_core-20240912-xiao-mg24.zip>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/local1.0.png" style={{width:800, height:'auto'}}/></div>

Unzip the file to the target path and change the filename to 2.1.0, reopen the arduino ide and you can then search for the MG24 in the motherboard.

**step 4.** Click on the box indicated by the arrow, then click on **‘Select other board and port’** , then search and select **XIAO MG24**, and finally select the correct port.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/5.png" style={{width:850, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/6.png" style={{width:850, height:'auto'}}/></div>

With everything in place, we can now start writing the code!

## Microphone
The Microphone sensor in the XIAO MG24 is **MSM381ACT001**，this tutorial will show you how to use the Microphone on the XIAO MG24.
**MSM381ACT001 Schematic**
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/MSM.png" style={{width:850, height:'auto'}}/></div>

This is the schematic diagram of **MSM381ACT001**, the master pin **PC08/MIC_EN** outputs 3V3 voltage through **RS3236-3.3YUTDN4** to make MSM381ACT001 work. The data generated by the microphone will be output through pin **PC09/MIC_ADC**.
### Microphone Programming Routine
The next step will give you an idea of how the Microphone on the XIAO MG24 sense should be used with the example below, where you will see the **brightness of the LED change with the sound level**.
#### Header File Configuration
These header files are typically used for embedded development based on Silicon Labs platforms. Below is a description of the functionality of each header file:
- `#include "em_device.h"` Defines registers, memory maps, and peripheral addresses for the selected chip. The content varies based on the target chip.
- `#include "em_chip.h"` Includes functions for basic chip setup, such as power configuration and system adjustments.
- `#include "em_core.h"` Provides functions to control interrupts, manage critical sections, and ensure thread safety.
- `#include "em_cmu.h"` Configures and manages the chip’s clock sources and peripheral clock gating.
- `#include "em_emu.h"` Provides functions for entering and configuring low-power modes.
- `#include "em_gpio.h"` Configures and manipulates GPIO pins.
- `#include "em_iadc.h"` Used for sampling analog signals and converting them into digital values.
- `#include "em_ldma.h"` Configures and manages DMA transfers for efficient data movement.
- `#include "em_letimer.h"` Provides low-power timer functionality, often used in energy-efficient applications.
- `#include "em_prs.h"` Routes events between peripherals without CPU intervention, enabling low-latency communication.

#### function initialisation
Here is an explanation of the **initialisation functions** for each module and function.
- `initPRS()` Configures the Peripheral Reflex System (PRS) to enable event routing between peripherals without CPU intervention.
- `initIADC()` Initializes the Integrated Analog-to-Digital Converter (IADC) for analog signal sampling.
- `initLETIMER()` Initializes the Low-Energy Timer (LETIMER) to generate periodic signals.
- `initLDMA()` Initializes the Low-Energy Direct Memory Access (LDMA) module for efficient data transfer.

#### Determine the function
The following function code will implement the function of changing the brightness of the LED with the sound level, and I will also explain to you the specific function of each section of the function.
- `float findMaxAverage(uint32_t arr[], int size, int n)` This function calculates the average of the largest n elements in an array.
- `float mapVoiceLevel(int voice_level)` This function maps an input voice_level value from one range (960 to 1500) to another (0 to 20).
- `void LDMA_IRQHandler(void)` This is an interrupt handler for LDMA (Linked Direct Memory Access). It processes data when an LDMA transfer is complete.
- `void setup(void)` Initializes the system and peripherals before the main loop begins execution.
- `void loop(void)` Implements the main logic to control an LED's brightness based on the voice_level.
```cpp
void loop() {
    int j = 0;
    while (1)
    {
        low_time = voice_level;
        high_time = pwm_t - low_time;
        j = 0;
        while (j < 10)
        {
            digitalWrite(LED_BUILTIN, HIGH);  
            delay(high_time);                      
            digitalWrite(LED_BUILTIN, LOW);   
            delay(low_time);  /* code */
            j++;
        }
       
    }
 
}
```

#### Full Code Show
:::tip
**Click Tool --> Protocol stack --> BLE(Arduino)** 
The code will run successfully only if this option is selected correctly.
:::
- Selecting **Matter** includes the Matter stack - which is quite resource heavy. Selecting this is only recommended when developing Matter applications. BLE is included, but cannot be used by users as the Matter SDK takes ownership of it. Matter examples will only work with this option.
- Selecting **BLE (Arduino)** provides compatibility with the ArduinoBLE library and is moderately resource heavy. This variant is compatible with all ArduinoBLE examples and applications based on the library.
- Selecting **BLE (Silabs)** includes Silicon Labs' BLE stack and API (BGAPI) - which is moderately resource heavy. Select this if you're developing BLE applications based on BGAPI. The included Silicon Labs BLE examples will only work with this option.
- Selecting **None** does not include a radio protocol stack in your sketch - and saves a considerable amount of Flash/RAM. You can use your board as a regular Arduino without wireless hardware.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/6.5.png" style={{width:850, height:'auto'}}/></div>


```cpp
#include "em_device.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "em_ldma.h"
#include "em_letimer.h"
#include "em_prs.h"

#define CLK_SRC_ADC_FREQ        20000000
#define CLK_ADC_FREQ            10000000
#define IADC_INPUT_0_PORT_PIN   iadcPosInputPortCPin9
#define IADC_INPUT_0_BUS        CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC   GPIO_CDBUSALLOC_CDODD1_ADC0
#define ADC_FREQ                16000//16000/8500
#define IADC_LDMA_CH            0
#define PRS_CHANNEL             0
#define NUM_SAMPLES             200
#define LETIMER_FREQ            9000

const int duration = 3;

int voice_level = 0;

LDMA_Descriptor_t descriptor;
uint32_t singleBuffer[NUM_SAMPLES];

void initPRS(void) {
    CMU_ClockEnable(cmuClock_PRS, true);
    PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0, PRS_LETIMER0_CH0);
    PRS_ConnectConsumer(PRS_CHANNEL, prsTypeAsync, prsConsumerIADC0_SINGLETRIGGER);
}

void initIADC(void) {
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;

    CMU_ClockEnable(cmuClock_IADC0, true);
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);
    init.warmup = iadcWarmupKeepInStandby;
    init.iadcClkSuspend1 = true;
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].vRef = 1210;
    initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0, CLK_ADC_FREQ, 0, iadcCfgModeNormal, init.srcClkPrescale);
    initSingle.triggerSelect = iadcTriggerSelPrs0PosEdge;
    initSingle.dataValidLevel = iadcFifoCfgDvl2;
    initSingle.fifoDmaWakeup = true;
    initSingle.start = true;
    singleInput.posInput = IADC_INPUT_0_PORT_PIN;
    singleInput.negInput = iadcNegInputGnd;
    IADC_init(IADC0, &init, &initAllConfigs);
    IADC_initSingle(IADC0, &initSingle, &singleInput);
    GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
}

void initLETIMER(void) {
    CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;
    LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

    CMU_LFXOInit(&lfxoInit);
    CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_LETIMER0, true);
    uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / LETIMER_FREQ;
    letimerInit.comp0Top = true;
    letimerInit.topValue = topValue;
    letimerInit.ufoa0 = letimerUFOAPulse;
    letimerInit.repMode = letimerRepeatFree;
    LETIMER_Init(LETIMER0, &letimerInit);
}

void initLDMA(uint32_t *buffer, uint32_t size) {
    LDMA_Init_t init = LDMA_INIT_DEFAULT;
    LDMA_TransferCfg_t transferCfg = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SINGLE);
    descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SINGLEFIFODATA, buffer, size, 0);
    LDMA_Init(&init);
    LDMA_StartTransfer(IADC_LDMA_CH, &transferCfg, &descriptor);
}

#include <stdio.h>

#define SIZE 200
#define N 5

#include <stdint.h> 

float findMaxAverage(uint32_t arr[], int size, int n) {
    uint32_t maxElements[N];

    for (int i = 0; i < n; i++) {
        maxElements[i] = arr[i]; 
    }

    for (int i = n; i < size; i++) {
        for (int j = 0; j < n; j++) {
            if (arr[i] > maxElements[j]) {
                maxElements[j] = arr[i];
                break; 
            }
        }
    }

    uint32_t sum = 0; 
    for (int i = 0; i < n; i++) {
        sum += maxElements[i];

    }
    return (float)sum / n; 
}

float mapVoiceLevel(int voice_level) {

    int original_min = 960;
    int original_max = 1500;
    int target_min = 0;
    int target_max = 20;

    if (voice_level < original_min) {
        voice_level = original_min;
    } else if (voice_level > original_max) {
        voice_level = original_max;
    }

    float mapped_value = (float)(voice_level - original_min) / (original_max - original_min) * (target_max - target_min) + target_min;

    return mapped_value;
}

int datatimes = 0;
int pwmdata = 0;
int pwmdata2 = 0;

void LDMA_IRQHandler(void) {
    
    voice_level = findMaxAverage(singleBuffer, SIZE, N);
    Serial.println(voice_level);
    voice_level = mapVoiceLevel(voice_level);
    LDMA_IntClear(1 << IADC_LDMA_CH);
}

void setup(void) {
    Serial.begin(115200);
    CHIP_Init();
    initPRS();
    initIADC();
    initLDMA(singleBuffer, NUM_SAMPLES);
    initLETIMER();
    pinMode(LED_BUILTIN, OUTPUT);
}

int high_time = 0;
int pwm_t = 20;
int low_time = 0;

void loop() {
    int j = 0;
    while (1)
    {
        low_time = voice_level;
        high_time = pwm_t - low_time;
        j = 0;
        while (j < 10)
        {
            digitalWrite(LED_BUILTIN, HIGH);  
            delay(high_time);                      
            digitalWrite(LED_BUILTIN, LOW);   
            delay(low_time);  /* code */
            j++;
        }
       
    }
 
}
```

If all goes well, you will see the following effect, the brightness of the LEDs will change according to the volume of the sound.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/test.gif" style={{width:720, height:'auto'}}/></div>
## 6-Axis IMU
6-Axis IMU (Inertial Measurement Unit) Sensors like the **LSM6DS3TR-C** integrate accelerometers and gyroscopes to measure the motion and orientation of an object in three-dimensional space. Specifically, the LSM6DS3TR-C has the following features:
**Accelerometer function:**
- Measures the acceleration of an object along the X, Y, and Z axes. It is able to sense object motion (e.g., rest, acceleration, deceleration) and tilt changes (e.g., angle of the object).
- It can be used to detect gait, position changes, vibrations, etc.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/xyz1.5.jpg" style={{width:320, height:'auto'}}/></div>

**Gyroscope function (Gyroscope):**
- Measures the angular velocity of an object around the X, Y, and Z axes, i.e., the rotation of the object.
- Can be used to detect rotation, rate of rotation, and change in direction.
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/xyz2.0.jpg" style={{width:320, height:'auto'}}/></div>

- The **X-axis angle ( Roll )** is the angle in the direction of rotation around the X-axis.
- The **Y-axis angle ( Pitch )** is the angle in the direction of rotation around the Y-axis.
- The **Z-axis angle ( Yaw )** is the angle in the direction of rotation around the Z-axis.

**LSM6DS3TR-C Schematic**
<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/imu%20Schematic.png" style={{width:850, height:'auto'}}/></div>

### 6-Axis IMU Programming Routine
We will show you how to use the **6-Axis IMU (LSM6DS3TR-C)**, and with this program routine you will observe the acceleration data in the serial monitor in your Arduino!
#### IIC initialisation
These functions provide the fundamental operations needed for I2C protocol communication, including starting and stopping communication, sending and receiving data, and handling acknowledgment signals. They enable two-way data transfer between a Microcontroller and an I2C slave device.

- `I2C_Start(void)` Sends the I2C Start Condition.
- `I2C_Stop(void)` Sends the I2C Stop Condition.
- `I2C_ACK(void)` Sends an Acknowledge (ACK) signal.
- `I2C_NACK(void)` Sends a Not Acknowledge (NACK) signal.
- `I2C_GetACK(void)` Waits for an ACK signal from the slave device.
- `I2C_SendBYTE(uint8_t data)` Sends one byte of data.
- `I2C_ReadBYTE(uint8_t ack)` Receives one byte of data.

#### EEPROM initialisation
These functions implement basic I2C operations for reading and writing single and multiple bytes of data to an EEPROM memory. They handle start and stop conditions, device address sending, data transfer, and the necessary address formatting (8-bit or 16-bit). Additionally, they manage EEPROM write delays to ensure proper operation and avoid errors during writing.
- `EEPROM_WriteByte(uint16_t addr, uint8_t data)` Writes a single byte of data to the EEPROM.
- `EEPROM_ReadByte(uint16_t addr, uint8_t *pdata)` Reads a single byte of data from the EEPROM.
- `EEPROM_Write_NByte(uint16_t addr, uint8_t *pdata, uint16_t size)` Writes multiple bytes of data to the EEPROM.
- `EEPROM_Read_NByte(uint16_t addr, uint8_t *pdata, uint16_t size)` Reads multiple bytes of data from the EEPROM.

#### Loop function
- **Read accelerometer X-axis data**
```cpp
EEPROM_Read_NByte(0x22, tmp_data, 2);
int16_t accelX = (tmp_data[0] | tmp_data[1] << 8);
float accelX_g = accelX * 0.061 / 1000;
```
The accelX is converted to an acceleration value in g (the unit of gravitational acceleration) and calculated by **accelX_g = accelX * 0.061 / 1000**. The multiplication factor 0.061 is due to the sensitivity of the sensor, and the division by 1000 is to convert to more reasonable units.
- **Read accelerometer Y-axis data**
```cpp
EEPROM_Read_NByte(0x24, tmp_data, 2);
int16_t accelY = (tmp_data[0] | tmp_data[1] << 8);
float accelY_g = accelY * 0.061 / 1000;
```
Reads 2 bytes of data from EEPROM address 0x24, processed in the same way as the X-axis. The read Y-axis data is converted to the acceleration value accelY_g.
- **Read accelerometer Z-axis data**
```cpp
EEPROM_Read_NByte(0x26, tmp_data, 2);
int16_t accelZ = (tmp_data[0] | tmp_data[1] << 8);
float accelZ_g = accelZ * 0.061 / 1000;
```
Reads 2 bytes of data from EEPROM address 0x26, processed in the same way as the X-axis. The read Z-axis data is converted to the acceleration value accelZ_g.
:::tip
To reduce noise and make the measurement results smoother and more accurate. The output also includes low-pass filtered data. For example LaccelX_g, LaccelY_g, LaccelZ_g.
:::

#### Full Code Show
```cpp
// Using I2C2 to connect M24C02, OLED, LM75AD, HT1382  
#define SDA_PIN   PB2
#define SCL_PIN   PB3

#define IMU_PWR   PD5

#define I2C_SDA_IN()    pinMode(SDA_PIN, INPUT)
#define I2C_SDA_OUT()   pinMode(SDA_PIN, OUTPUT)
 
// IO operation macros  
#define SCL_H()   digitalWrite(SCL_PIN, 1)
#define SCL_L()   digitalWrite(SCL_PIN, 0)
#define SDA_H()   digitalWrite(SDA_PIN, 1)
#define SDA_L()   digitalWrite(SDA_PIN, 0)
#define SDA_INPUT()    digitalRead(SDA_PIN)  // Read SDA input
#define I2C_Delay()   delay(2)

#define ACK (0)
#define NACK (1)

#define EEPROM_DEV_ADDR     (0xD4)
#define EEPROM_WR           (0x00)
#define EEPROM_RD           (0x01)

#define EEPROM_WORD_ADDR_SIZE   (0x08)

// Start condition
void I2C_Start(void) {
    I2C_SDA_OUT(); // Set SDA as output
    
    SCL_H(); // Set clock line high
    I2C_Delay(); // Delay for timing
    
    SDA_H(); // Set data line high
    I2C_Delay();
    
    SDA_L(); // Pull data line low to indicate start condition
    I2C_Delay();
}

// Stop condition
void I2C_Stop(void) {
    I2C_SDA_OUT(); // Set SDA as output
    
    SDA_L(); // Pull data line low
    I2C_Delay();
    
    SCL_H(); // Set clock line high
    I2C_Delay();
    
    SDA_H(); // Pull data line high to indicate stop condition
    I2C_Delay();
}

// Send an ACK signal
void I2C_ACK(void) {
    I2C_SDA_OUT(); // Set SDA as output
    
    SCL_L(); // Pull clock line low
    I2C_Delay();
    
    SDA_L(); // Pull data line low for ACK
    I2C_Delay();
    
    SCL_H(); // Set clock line high
    I2C_Delay();
    
    SCL_L(); // Pull clock line low
    I2C_Delay();
}

// Send a NACK signal
void I2C_NACK(void) {
    I2C_SDA_OUT(); // Set SDA as output
    
    SCL_L(); // Pull clock line low
    I2C_Delay();
    
    SDA_H(); // Pull data line high for NACK
    I2C_Delay();
    
    SCL_H(); // Set clock line high
    I2C_Delay();
    
    SCL_L(); // Pull clock line low
    I2C_Delay();
}

// Wait for an ACK signal from the slave
uint8_t I2C_GetACK(void) {
    uint8_t time = 0;
    
    I2C_SDA_IN(); // Set SDA as input
    
    SCL_L(); // Pull clock line low
    I2C_Delay();
    
    SDA_H(); // Release SDA
    I2C_Delay();
    
    SCL_H(); // Set clock line high
    I2C_Delay();
    
    while (SDA_INPUT()) { // Wait for SDA to go low (ACK)
        time++;
        if (time > 250) {
            SCL_L(); // Pull clock line low
            return 1; // No ACK received
        }
    }
    SCL_L(); // Pull clock line low
    return 0; // ACK received
}

// Send one byte of data
void I2C_SendBYTE(uint8_t data) {
    uint8_t cnt = 0;
    I2C_SDA_OUT();
    for (cnt = 0; cnt < 8; cnt++) {
        SCL_L();
        I2C_Delay();
        if (data & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        data <<= 1;
        SCL_H();
        I2C_Delay();
    }
    SCL_L(); // Finish sending data
    I2C_Delay();
    I2C_GetACK();
}

// Read one byte of data
uint8_t I2C_ReadBYTE(uint8_t ack) {
    uint8_t cnt = 0;
    uint8_t data = 0xFF; // Initialize data
    
    SCL_L();
    I2C_Delay();
    
    for (cnt = 0; cnt < 8; cnt++) {
        SCL_H(); // Set SCL high to read data
        I2C_Delay();
        data <<= 1;
        if (SDA_INPUT()) {
            data |= 0x01;
        }
        SCL_L();
        I2C_Delay();
    }
    // Send ACK or NACK signal
    if (ack == 0) {
        I2C_ACK();
    } else {
        I2C_NACK();
    }
    return data;
}

// Write a byte to EEPROM
void EEPROM_WriteByte(uint16_t addr, uint8_t data) {
    I2C_Start(); // Start condition
    I2C_SendBYTE(EEPROM_DEV_ADDR | EEPROM_WR); // Send device address
    if (EEPROM_WORD_ADDR_SIZE == 0x08) {
        I2C_SendBYTE((uint8_t)(addr & 0x00FF)); // Send address
    } else {
        I2C_SendBYTE((uint8_t)(addr >> 8));
        I2C_SendBYTE((uint8_t)(addr & 0x00FF));
    }
    I2C_SendBYTE(data); // Send data
    I2C_Stop(); // Stop condition
}

// Read a byte from EEPROM
void EEPROM_ReadByte(uint16_t addr, uint8_t *pdata) {
    I2C_Start(); // Start condition
    I2C_SendBYTE(EEPROM_DEV_ADDR | EEPROM_WR); // Send device address
    if (EEPROM_WORD_ADDR_SIZE == 0x08) {
        I2C_SendBYTE((uint8_t)(addr & 0x00FF)); // Send address
    } else {
        I2C_SendBYTE((uint8_t)(addr >> 8));
        I2C_SendBYTE((uint8_t)(addr & 0x00FF));
    }
    I2C_Start(); // Restart condition
    I2C_SendBYTE(EEPROM_DEV_ADDR | EEPROM_RD); // Send device address with read flag
    *pdata = I2C_ReadBYTE(NACK); // Read data
    I2C_Stop(); // Stop condition
}

// Write multiple bytes to EEPROM
void EEPROM_Write_NByte(uint16_t addr, uint8_t *pdata, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        EEPROM_WriteByte(addr, pdata[i]);
        addr++;
        delay(10); // Delay to prevent write errors
    }
}

// Read multiple bytes from EEPROM
void EEPROM_Read_NByte(uint16_t addr, uint8_t *pdata, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        EEPROM_ReadByte(addr, &pdata[i]);
        addr++;
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(IMU_PWR, OUTPUT);
    digitalWrite(IMU_PWR, 1); // Power on IMU

    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
  
    Serial.begin(9600);
    delay(1000);

    uint8_t tmp1 = 0xb0;
    uint8_t tmp2 = 0x10;
    EEPROM_WriteByte(0x10, tmp1); // Write to EEPROM
    EEPROM_WriteByte(0x11, tmp2);
    delay(500);
}

void loop() {
    uint8_t tmp_data[2] = {0};

    // Read accelerometer data from EEPROM
    EEPROM_Read_NByte(0x22, tmp_data, 2);
    int16_t accelX = (tmp_data[0] | tmp_data[1] << 8);
    float accelX_g = accelX * 0.061 / 1000;

    EEPROM_Read_NByte(0x24, tmp_data, 2);
    int16_t accelY = (tmp_data[0] | tmp_data[1] << 8);
    float accelY_g = accelY * 0.061 / 1000;

    EEPROM_Read_NByte(0x26, tmp_data, 2);
    int16_t accelZ = (tmp_data[0] | tmp_data[1] << 8);
    float accelZ_g = accelZ * 0.061 / 1000;

    EEPROM_Read_NByte(0x28, tmp_data, 2);
    int16_t LaccelX = (tmp_data[0] | tmp_data[1] << 8);
    float LaccelX_g = LaccelX * 0.061 / 1000;

    EEPROM_Read_NByte(0x2a, tmp_data, 2);
    int16_t LaccelY = (tmp_data[0] | tmp_data[1] << 8);
    float LaccelY_g = LaccelY * 0.061 / 1000;

    EEPROM_Read_NByte(0x2c, tmp_data, 2);
    int16_t LaccelZ = (tmp_data[0] | tmp_data[1] << 8);
    float LaccelZ_g = LaccelZ * 0.061 / 1000;

    // Print results to the serial monitor
    Serial.printf("accelX_g = %f    LaccelX_g = %f\r\n", accelX_g, LaccelX_g);
    Serial.printf("accelY_g = %f    LaccelY_g = %f\r\n", accelY_g, LaccelY_g);
    Serial.printf("accelZ_g = %f    LaccelZ_g = %f\r\n\n", accelZ_g, LaccelZ_g);

    delay(500);
}

```

If all goes well, you will observe the acceleration along the X, Y, and Z axes in the Serial Monitor.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/six.png" style={{width:850, height:'auto'}}/></div>

## Resources
### For Seeed Studio XIAO MG24 Sense

- 📄 **[PDF]** [Seeed Studio 6-Axis IMU(LSM6DS3TR-C) datasheet](https://statics3.seeedstudio.com/fusion/opl/sheets/314040211.pdf)
- 📄 **[PDF]** [Seeed Studio Analog Microphone(MSM381ACT001) datasheet](https://files.seeedstudio.com/wiki/mg24_mic/312030602_MEMSensing_MSM381ACT001_Datasheet.pdf)

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