# CyberBee Driver API for Raspberry Pi 4B

## Table of Contents

- [Introduction](#introduction)
- [Client-Side Setup](#client-side-setup)
  - [Hardware Connection](#hardware-connection)
  - [Verifying Connection](#verifying-connection)
- [Serial Communication API Documentation](#serial-communication-api-documentation)
  - [API Overview](#api-overview)
  - [Command Structure](#command-structure)
  - [Available Commands](#available-commands)
  - [Incoming Message Structure](#incoming-message-structure)
- [Configuration and Startup Routine](#configuration-and-startup-routine)
  - [Default Setup](#default-setup)
  - [Start Routine](#start-routine)
  - [Set Baud Rate Routine](#set-baud-rate-routine)
- [System Requirements](#system-requirements)
- [Installation Instructions](#installation-instructions)
- [Troubleshooting and FAQs](#troubleshooting-and-faqs)
- [Advanced Configuration and Customization](#advanced-configuration-and-customization)
- [Security and Privacy Considerations](#security-and-privacy-considerations)
- [Performance and Optimization](#performance-and-optimization)
- [Integration with Other Systems](#integration-with-other-systems)
- [Firmware and Software Updates](#firmware-and-software-updates)
- [Technical Support and Customer Service](#technical-support-and-customer-service)
- [Warranty and Service Information](#warranty-and-service-information)
- [Regulatory Compliance and Certifications](#regulatory-compliance-and-certifications)
- [Glossary of Terms](#glossary-of-terms)
- [Appendices](#appendices)
  - [Sample Code and Scripts](#sample-code-and-scripts)
  - [Compatibility Lists](#compatibility-lists)
  - [Change Log and Version History](#change-log-and-version-history)

## Introduction

The CyberBee Driver API facilitates the communication between computing devices, such as the Raspberry Pi 4B, and the BeeMX8 module,This documentation guides clients through connecting and reading pose data from devices over UART/USB interfaces, focusing on robotics and similar applications. It includes comprehensive instructions for hardware connections, serial communication via Python Or CPP scripts, and detailed API documentation for command execution.

**!Important Note!:**
Currently Use The CPP Version it's more stable, outgoing commands are not supported and default baud rate is 115200. 
BeeMX8 support USB-C connector - next release will also support the UART on the BeeMX8.

## Client-Side Setup

### Hardware Connection

- **Identify USB/UART Port:** Connect your Raspberry Pi 4B or other computing module to the CyberBee BeeMX8 using the appropriate USB-C or UART port connection. The port should be recognized as COMx on Windows or /dev/ttyUSBx or /dev/ttyACMx on Linux.

![IMG_0337](https://github.com/CyberBeeOfficial/Driver/assets/80558934/453b5dac-6b71-4490-9e99-1665d25974fa)

![Screenshot from 2024-03-07 13-52-46](https://github.com/CyberBeeOfficial/Driver/assets/80558934/014325c5-bc38-4f00-a101-039500a5a72a)

- **Port Names:** 
    -   For **UART** GPIO on Pi 4B: Use **/dev/ttyAMA0.** 
     -  For **BeeMX8:**
           -  **Type-C Port**: **/dev/ttyUSB0** or **/dev/ttyUSB1**
            
         -  UART Connector: **/dev/ttymxc0**

### Verifying Connection

Use PuTTY on Windows or screen/minicom on Linux, configured with the correct port number and the default baud rate of 115200, to verify the connection. Send Messages between devices to cehk data passes.

## Serial Communication API Documentation

### API Overview

This section outlines the serial commands for communication with the hardware driver.

### Outgoing Command Structure

Commands follow the format `<ST>;command;data;<EN>` for both sending commands and receiving messages.

### Available Commands Your Device Can Send To BeeMX8:

- **Test Command:** `<ST>;0x20;0;<EN>`
- **Change Baud Rate:** `<ST>;0x30;2;<EN>` for 38400 baud rate
- **Set Division Rate:** `<ST>;0x22;3;<EN>` for division rate 3
- **Confirm Command:** `<ST>;0x23;0;<EN>`
- **GPS Command:** `<ST>;0x25;0;<EN>`

### Incoming Pose Message Structure

Messages follow the format 

Messages are formatted to include message type, timestamp, position data, quaternion data, velocity data, and a checksum, ensuring comprehensive data transmission and integrity.


`<ST>;message_type;timestamp;position_data;quaternion_data;velocity_data;checksum;<EN>`, 

";" is  breakdown for each component.

**Components:**
1) **message_type**: A byte indicating the data type, e.g., 01 for ODOMETRY, 02 for POSITION.

2) **timestamp**: A Unix timestamp in seconds with nanosec resolution.
3
3) **position_data**: Three floating-point numbers for X, Y, Z coordinates.

4) **quaternion_data**: Four floating-point numbers representing the quaternion (W, X, Y, Z).

5) **velocity_data**: Three floating-point numbers for Vx, Vy, Vz) velocity.

6. **checksum**: A byte representing the checksum of the message. It is calculated by summing up all the bytes in the message before the checksum.

7. <ST> message Prefix
8. <EN> messge Suffix 

Example Message:

```
<ST>;01;1696499957.774450967;-9.80,10.75,0.24;0.4,-0.3,-0.5,0.6;0.2,0.3,0.5;123;<EN>
```
Message type: is **01** - meaning it’s an odometry message.
Time Stamp is -  **1696499957.774450967**  in nanoseconds.
Position Data shows  -  **x = -9.8, y =10.75, z = 0.24**.   
Quaternion Data - **x =0.4, y = -0.3, z = -0.5, w = 0.6.**
Checksum equals - **123**

***Checksum is calculated by summing all of the numeric ASCII values of the string message.***

### Incoming GPS Message Structure

Messages follow the format 

`<ST>;command;seconds.nanoseconds;latitude,longitude,altitude;yaw,pitch,roll;gps_status;checksum;<EN>`, 

";" is  breakdown for each component. command = 0x25.

***Checksum is calculated by summing all of the numeric ASCII values of the string message.***


## Configuration and Startup Routine


### Default Setup

- **Baud Rate:** Default is 115200. Adjust as necessary following the [Set Baud Rate Routine](#set-baud-rate-routine).

### Start Routine

1. Power up the BeeMX8 and connect it to your device.
2. Navigate to the directory containing the driver code and execute the appropriate script for your setup (Python or C++).

>     <Your_Path> - The Location where you have cloned the Cyberbee Driver
>     To get the path write pwd

   Use  Bash FOr Linux  or other Terminal 

for the python script 
```
cd <Your_Path>/Driver/example
```
for the cpp script
```
cd <Your_Path>/Driver/example/src 
```
3. **Run The Driver Script**

    **For Python**

        python3 python_driver.py

     **For CPP Driver First Compile**

   Make sure to run all 5 compilation lines to create the exe file
    ```
   /usr/bin/g++ -std=c++17 -g <Your_Path>/Driver/example/src/driver_main.cpp <Your_Path>/Driver/example/src/serial_comm.cpp <Your_Path>/Driver/example/src/data_sender.cpp <Your_Path>/Driver/example/src/data_receiver.cpp -o <Your_Path>/Driver/example/src/<Executble_Name>
    
    ```

* ***Make Sure you change the <name_of_executable> with appropriate name.***


    **Run CPP Driver:**    

    ```
       ./Driver/example/bin/<Executble_Name>
    
    ```

4. Execute the Test Command, then the SetDivisionRate command to start receiving position data.



## System Requirements

* For the C++ driver, a GCC compiler is required. [https://gcc.gnu.org/install/](url)
* For the Python driver, Python 3.x and the pyserial package are needed.
* Pyserial Package For python Script 


List of hardware and software prerequisites for using the API.

## Installation Instructions


1) **Install Dependencies:** For Python, ensure pyserial is installed using pip install pyserial.

2) **Clone or Download the Driver**: Obtain the driver code for your device.

3) **Compile (C++) or Run (Python) the Driver:** Follow the startup routine instructions for compiling and running the driver.
Step-by-step guide for setting up the API on the Raspberry Pi 4B.

## Troubleshooting and FAQs

Common issues and their resolutions, along with frequently asked questions.

## Advanced Configuration and Customization

Guidance on tailoring the API to specific needs or integrating with other systems.

## Firmware and Software Updates

Information on updating the driver and firmware for optimal performance and security.

## Technical Support and Customer Service

Contact details and support channels for assistance.

## Appendix

Additional resources, including sample code, compatibility lists, and a comprehensive change log.

/usr/bin/g++ -std=c++17 -g /root/Driver/example/src/driver_main.cpp /root/Driver/example/src/serial_comm.cpp /root/Driver/example/src/data_sender.cpp /root/Driver/example/src/data_receiver.cpp -o /root/Driver/example/src/CppDrivr -pthread

