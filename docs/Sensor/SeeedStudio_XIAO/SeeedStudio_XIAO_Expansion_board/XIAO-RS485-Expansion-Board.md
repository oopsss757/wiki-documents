---
description: XIAO Communcate with RS485 Expansion Board
title: RS485 Expansion Board for XIAO
keywords:
- xiao
image: https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/top.webp
slug: /XIAO-RS485-Expansion-Board
last_update:
  date: 11/20/2024
  author: Jason
---


# Getting Started with Seeed Studio XIAO-RS485-Expansion-Board

<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/top.jpg" style={{width:600, height:'auto'}}/></div>


## Hardware Overview

### prepared material

<div class="table-center">
	<table align="center">
		<tr>
			<th>Seeed Studio XIAO RS485-Expansion-Board</th>
			<th>Seeed Studio XIAO ESP32-C3</th>
		</tr>
		<tr>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/hadware.jpg" style={{width:250, height:'auto'}}/></div></td>
			<td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/esp32.jpg" style={{width:250, height:'auto'}}/></div></td>
		</tr>
		<tr>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/RS485-Breakout-Board-for-XIAO-p-6306.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> Get One Now 🖱️</font></span></strong>
				</a>
			</div></td>
			<td><div class="get_one_now_container" style={{textAlign: 'center'}}>
				<a class="get_one_now_item" href="https://www.seeedstudio.com/seeed-xiao-esp32c3-p-5431.html">
				<strong><span><font color={'FFFFFF'} size={"4"}> Get One Now 🖱️</font></span></strong>
				</a>
			</div></td>
		</tr>
	</table>
</div>


### Pinout Diagram

<div class="table-center">
  <table align="center">
    <tr>
        <th>XIAO RS485 Expansion indication diagram</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/pinlist.png" style={{width:700, height:'auto'}}/></div></td>
    </tr>
  </table>
</div>

- 5V OUT/IN SWITCH : One is the input and output of the 5V port, when the slave set the IN file, as the input, when the host set the OUT file, this port is discharged externally, can be connected to the sensor to the sensor power supply.

- 120R SWITCH : The 120R switch is used to determine whether the 120 ohm resistor is accessible or not. 485 needs to add a 120 ohm resistor at the beginning and end of long wiring environments to do the matching and ensure communication.

- INT : Reserved interrupt port.

:::tip
When it is used as an input mode, you need to turn the switch to IN, and if it is used as an output mode, you need to turn the switch to OUT to prevent burning out.
:::

### Connection Schematic
<div class="table-center">
  <table align="center">
    <tr>
        <th>Two RS485 expansion board connections</th>
    </tr>
    <tr>
        <td><div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/connect1.png" style={{width:700, height:'auto'}}/></div></td>
    </tr>
  </table>
</div>

## Software Overview

### Sender Code

```cpp
#include <SoftwareSerial.h> // Include the SoftwareSerial library

// Create a SoftwareSerial object using D5 and D4 pins
SoftwareSerial mySerial(D5, D4);
#define enable_pin D2 // Define the enable pin as D2

void setup() {
  Serial.begin(115200); // Initialize the hardware serial with a baud rate of 115200
  mySerial.begin(115200); // Initialize the software serial with a baud rate of 115200
  
  // Wait for the software serial to be ready
  while(!mySerial);
  // Wait for the hardware serial to be ready
  while(!Serial);

  pinMode(enable_pin, OUTPUT); // Set the enable pin as an output
  digitalWrite(enable_pin, HIGH); // Set the enable pin to high
}

void loop() {
  // Check if there is data available from the hardware serial
  if (Serial.available()) {
    char data = Serial.read(); // Read the data from the hardware serial
    
    // If the received data is not a newline character
    if (data != '\n' && data != '\r') {
      Serial.println("send successfully"); // Print a success message
      mySerial.print("Master send information is: "); // Send a prompt message to the software serial
      mySerial.println(data); // Send the received data to the software serial
    }
  }
  delay(1000); // Delay for 1 second
}

```
- **SoftwareSerial Library:** Allows the creation of additional serial ports on ESP32, typically used for communication with devices (such as sensors or modules).
- `SoftwareSerial mySerial(D5, D4)`: Defines a SoftwareSerial object named mySerial, using D5 and D4 as the receive and transmit pins.
- `#define enable_pin D2`: Defines an enable pin used to control the sending and receiving state of the RS485 module.

- `setup()`：
  - `while(!mySerial)`: Waits until the software serial is ready for communication.
  - `while(!Serial)`: Waits until the hardware serial is ready for communication.
  - `pinMode(enable_pin, OUTPUT)`: Configures the enable_pin as an output pin to control the RS485 module.
  - `digitalWrite(enable_pin, HIGH)`: Sets the enable_pin to HIGH, configuring the RS485 module for sending mode.

- `loop():`
  - `if (mySerial.available())`: Checks if there is any data available to read from the software serial port.
  - `char data = mySerial.read()`: Reads a single character from the software serial port and stores it in the variable data.
  - `if(data != '\n' && data != '\r')`: Checks if the received data is not a newline (\n) or carriage return (\r) character.
  - `Serial.println("send successfully")`: Print a success message.
  - `mySerial.print("Master send information is: ")`: Send a prompt message to the software serial.
  - `mySerial.println(data)` :Send the data you need to an RS485 expansion board.

### Receiver Code

```cpp
#include <SoftwareSerial.h> // Include the SoftwareSerial library

SoftwareSerial mySerial(D5, D4); // Create a SoftwareSerial object; D5 is RX and D4 is TX
#define enable_pin D2 // Define the enable pin as D2

void setup() {
  Serial.begin(115200); // Initialize the hardware serial with a baud rate of 115200
  mySerial.begin(115200); // Initialize the software serial with a baud rate of 115200
  
  // Wait for the hardware serial to be ready
  while(!Serial);
  // Wait for the software serial to be ready
  while(!mySerial);
  
  pinMode(enable_pin, OUTPUT); // Set the enable pin as an output
  digitalWrite(enable_pin, LOW); // Set the enable pin to low
}

void loop() {
  // Check if there is data available from the software serial
  if (mySerial.available()) {
    char data = mySerial.read(); // Read the data from the software serial
    
    // If the received data is not a newline character
    if(data != '\n' && data != '\r'){
      Serial.println("receiver information"); // Print a message indicating that data is received
      mySerial.println(data); // Echo the received data back to the software serial
    }
  }
}

```

- **SoftwareSerial Library:** Allows the creation of additional serial ports on ESP32, typically used for communication with devices (such as sensors or modules).
- `SoftwareSerial mySerial(D5, D4)`: Defines a SoftwareSerial object named mySerial, using D5 as RX and D4 as TX.
- `define enable_pin D2`: Defines an enable pin used to control the sending and receiving state of the RS485 module.

- `setup()`:
  - `Serial.begin(115200`: Initializes the hardware serial port with a baud rate of 115200.
  - `mySerial.begin(115200)`: Initializes the software serial port with a baud rate of 115200.
  - `while(!Serial)`: Waits until the hardware serial port is ready for communication.
  - `while(!mySerial)`: Waits until the software serial port is ready for communication.
  - `pinMode(enable_pin, OUTPUT)`: Configures the enable_pin as an output pin to control the RS485 module.
  - `digitalWrite(enable_pin, LOW)`: Sets the enable_pin to low, configuring the RS485 module for receiving mode.

- `loop()`:
  - `if (mySerial.available())`: Checks if there is any data available to read from the software serial port.
  - `char data = mySerial.read()`: Reads a single character from the software serial port and stores it in the variable data.
  - `if(data != '\n' && data != '\r')`: Checks if the received data is not a newline (\n) or carriage return (\r) character.
  - `Serial.println("receiver information")`: Prints a message to the hardware serial indicating that data has been received.
  - `mySerial.println(data)`: Print the data sent to the receiver RS485.


## RS485 Transmission Result


<div style={{textAlign:'center'}}><img src="https://files.seeedstudio.com/wiki/rs485_ExpansionBoard/result.png" style={{width:900, height:'auto'}}/></div>


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