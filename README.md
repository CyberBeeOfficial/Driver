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

This documentation guides clients through connecting and reading pose data from devices over UART/USB interfaces, focusing on robotics and similar applications. It includes comprehensive instructions for hardware connections, serial communication via Python scripts, and detailed API documentation for command execution.

## Client-Side Setup

### Hardware Connection

- **Identify USB/UART Port:** Connect your Raspberry Pi 4B to your computer using the appropriate micro USB connection. The port should be recognized as COMx on Windows or /dev/ttyUSBx or /dev/ttyACMx on Linux.

### Verifying Connection

- **Tools Required:** Use PuTTY for Windows or screen for Linux, configured with the correct port number and baud rate (default 115200).

## Serial Communication API Documentation

### API Overview

This section outlines the serial commands for communication with the hardware driver.

### Command Structure

Commands follow the format `<ST>;command;data;<EN>` for both sending commands and receiving messages.

### Available Commands

- **Test Command:** `<ST>;0x20;0;<EN>`
- **Change Baud Rate:** `<ST>;0x30;2;<EN>` for 38400 baud rate
- **Set Division Rate:** `<ST>;0x22;3;<EN>` for division rate 3
- **Confirm Command:** `<ST>;0x23;0;<EN>`

### Incoming Message Structure

Messages follow the format `<ST>;message_type;timestamp;position_data;quaternion_data;velocity_data;checksum;<EN>`, with detailed breakdowns of each component.

## Configuration and Startup Routine

### Default Setup

- **Baud Rate:** Default is 115200. Adjust as necessary following the [Set Baud Rate Routine](#set-baud-rate-routine).

### Start Routine

1. Power up and connect to the module.
2. Connect serially at the default baud rate.
3. Execute the Test Command, then the SetDivisionRate command to start receiving position data.

## System Requirements

List of hardware and software prerequisites for using the API.

## Installation Instructions

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

## Installation Instructions

Before running the provided Python script, ensure you have Python installed on your system. The script requires Python 3.x.

### Dependencies

The script uses the `pyserial` package for serial communication. Install this package using pip:

```
pip install pyserial
```

### Running the Script

1. Clone the repository or download the script to your local machine.
2. Open a terminal or command prompt.
3. Navigate to the directory containing the script.
4. Run the script using Python:

```
python <script_name>.py
```

Replace `<script_name>` with the name of the Python script.

Ensure your Raspberry Pi 4B or other compatible hardware is connected and configured as per the instructions in the "Client-Side Setup" section of this README.

## Script Update for Checksum Validation

The Python example code includes a function, `parse_and_print_message`, which has been updated to validate the checksum of received messages. This ensures the integrity of the data transmitted between the device and the client application. The updated function calculates a checksum for the received message and compares it with the transmitted checksum to verify the message's correctness.

### Updating Your Application

To incorporate this checksum validation into your application:

1. Ensure the `calculate_checksum` function is defined and correctly calculates the checksum of a given command string.
2. Update the `parse_and_print_message` function in your application code to include checksum parsing and validation as shown in the provided script update.
3. Test the updated application with your hardware to ensure correct message processing and error handling for checksum mismatches.

This update enhances the reliability and integrity of data communication in your application.
