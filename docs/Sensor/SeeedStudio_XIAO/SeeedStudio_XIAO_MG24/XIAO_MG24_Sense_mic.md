---
title: Microphone and 6-Axis IMU Usage for Sense Version
description: This article describes how to use the microphone on the XIAO MG24 Sense.
image: https://files.seeedstudio.com/wiki/mg24_mic/mg24.jpg
slug: /xiao_mg24_mic_and_6-Axis_sense
keywords:
  - XIAO
  - MG24
last_update:
  date: 11/20/2024 
  author: qiu yu wei
sidebar_position: 2
---

# Usage of Seeed Studio XIAO MG24 microphone and 6-Axis IMU

This tutorial describes how the microphone and 6-Axis IMU on the XIAO MG24 will be used.

:::caution
All contents of this tutorial are applicable to XIAO MG24 Sense only.
:::

## Getting Started

Before starting this tutorial, it is necessary to configure the environment to ensure that the programme can run smoothly.

### Preparing the development environment

**step 1.** Click on **File**, then **Preference**, and add the following URL.

```
https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json 
```

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/1.png" style={{width:850, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/2.png" style={{width:850, height:'auto'}}/></div>

**step 2.** Select **Silicon Labs** for installation.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/3.png" style={{width:850, height:'auto'}}/></div>

:::caution
At this moment, Xiao MG24 cannot be searched in the Board, it is necessary to update the relevant firmware.
:::

**step 3.** Complex the file into the target path and delete the **original 2.1.0 folder**.

<https://files.seeedstudio.com/wiki/mg24_mic/silabs_arduino_core-20240912-xiao-mg24.zip>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/4.png" style={{width:850, height:'auto'}}/></div>

Unzip the file to the target path and change the filename to 2.1.0, reopen the arduino ide and you can then search for the MG24 in the motherboard.

**step 4.** Click on the box indicated by the arrow, then click on **‘Select other board and port’** , then search and select **xiao mg24**, and finally select the correct port.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/5.png" style={{width:850, height:'auto'}}/></div>

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/6.png" style={{width:850, height:'auto'}}/></div>

**step 5.** Install the instructions in the picture to select the **BLE (Arduino)**.

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/mg24_mic/6.5.png" style={{width:850, height:'auto'}}/></div>

With everything in place, we can now start writing the code!

## Microphone (software)

### Header files and macro definitions

These header files are libraries for the Silicon Labs EFR32 microcontroller and are used to manipulate the abstract interface to the device hardware.

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
```

### function initialisation.

PRS (Peripheral Reflex System) is a routing system for peripheral signals.

```cpp
void initPRS(void) {
    CMU_ClockEnable(cmuClock_PRS, true);
    PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0, PRS_LETIMER0_CH0);
    PRS_ConnectConsumer(PRS_CHANNEL, prsTypeAsync, prsConsumerIADC0_SINGLETRIGGER);
}

```

**IADC_calcSrcClkPrescale** and **IADC_calcAdcClkPrescale** calculate the divider value of the sampling frequency.

```cpp
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
```

Initialise LETIMER as a trigger signal for the sampling clock.

```cpp
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
```

LDMA (Linked Direct Memory Access) is responsible for automatically transferring the sample results from the IADC to the buffer.

```cpp
void initLDMA(uint32_t *buffer, uint32_t size) {
    LDMA_Init_t init = LDMA_INIT_DEFAULT;
    LDMA_TransferCfg_t transferCfg = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SINGLE);
    descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SINGLEFIFODATA, buffer, size, 0);
    LDMA_Init(&init);
    LDMA_StartTransfer(IADC_LDMA_CH, &transferCfg, &descriptor);
}
```

### Volume Handling Functions

Find the N largest values in the buffer arr and find the average value.

```cpp
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
```

Maps the volume value from the original range (960-1500) to the LED's duty cycle range (0-20).

```cpp
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
```

### Full Code Show

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

## 6-Axis IMU(Software)

### Macro definitions about IIC

The code implements communication with peripherals (e.g. EEPROM, OLED, sensors, etc.) by emulating the I²C protocol. It defines the pins of the data/clock lines, high and low level control, input/output mode switching, delay, etc., and contains the device address and read/write mode configuration of the EEPROM, which provides a basic operating framework for I²C communication.

```cpp
//Using IIC2 to mount M24C02,OLED,LM75AD,HT1382  
#define SDA_PIN   PB2
#define SCL_PIN   PB3

#define IMU_PWR   PD5

#define I2C_SDA_IN()    pinMode(SDA_PIN, INPUT)
#define I2C_SDA_OUT()   pinMode(SDA_PIN, OUTPUT)
 
// IO operation functions	 
#define SCL_H()   digitalWrite(SCL_PIN, 1)
#define SCL_L()   digitalWrite(SCL_PIN, 0)
#define SDA_H()   digitalWrite(SDA_PIN, 1)
#define SDA_L()   digitalWrite(SDA_PIN, 0)
#define SDA_INPUT()    digitalRead(SDA_PIN) 
#define I2C_Delay()   delay(2)

#define ACK (0)
#define NACK (1)

#define EEPROM_DEV_ADDR     (0xD4)
#define EEPROM_WR           (0x00)
#define EEPROM_RD           (0x01)

#define EEPROM_WORD_ADDR_SIZE   (0x08)
```

### IIC

This code provides basic functions for I²C communication, including:

1. **Start and Stop Conditions**: Generate start and stop signals for initiating and terminating I²C communication.

2. **ACK/NACK Signals**: Send acknowledgment (ACK) or non-acknowledgment (NACK) signals during data transfer.

3. **ACK Detection**: Check for acknowledgment from the slave device after data is sent.

   These functions are essential building blocks for implementing I²C communication protocols.

   ```cpp
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
   
   ```

### Full Code Show

This code demonstrates how to implement a software-based I²C protocol to communicate with devices like an EEPROM (M24C02). It includes I²C bit-banging functions for start, stop, ACK/NACK signaling, sending, and receiving data. It writes and reads accelerometer data to/from the EEPROM, processes the raw data into *g*-force values, and prints the results via the serial monitor. The setup initializes the I²C pins and an IMU power pin, while the loop performs continuous data acquisition and output.

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
