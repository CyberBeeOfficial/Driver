/*
 * Copyright (c) 2024 CyberBee Ltd.
 * All rights reserved.
 *
 * This file is part of the CyberBee IMX8 C++ Driver.
 *
 * Description:
 * This code is part of the CyberBee  IMX8 C++ Driver layer, responsible for
 * managing communication between imx8 and other devices sucha s PI-4B.
 * This code meant to be compiled on the client device i.e. PI-4B,or other
 * ubuntu system.
 *
 */

#include "../../include/cpp/data_receiver.hpp"

#include <iostream>
#include <sstream>

#include <iomanip>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numeric>

#include <stdexcept> 
#include <string>
#include <tuple>
#include <vector>

// Define message types
enum MessageType 
{
    ODOMETRY = 0x01,
    POSITION = 0x02,
    IMU = 0x03,
    STATUS = 0x04,
    ERROR = 0x05
    // Add more message types as needed
};

DataReceiver::DataReceiver(SerialPort* serialPort) : serialPort_(serialPort)
{
    // data receiver init shall  be here
}

void DataReceiver::readData()
{
    if (serialPort_ != nullptr)
    {
        while (!stopThreads)
        {
            serialPort_->ReadFromBuffer();
        }
    }
}

void DataReceiver::processData()
{
    std::vector<std::uint8_t> message;
    while (!stopThreads)
    {
        // This part needs to check the message and only than parse it in a different function
        std::vector<std::vector<uint8_t>> messages = serialPort_->ExtractMessageBinary();  // Assuming this method now just
                                            // extracts based on `temp_storage`
        std::cout << "got message from extract" << std::endl;
        for(auto msg = messages.begin(); msg != messages.end(); msg++)
        {
            message.assign((*msg).begin() + 1, (*msg).end() - 3); // without start byte and end byte

            uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];

            if (!message.empty())
            {
                switch(messageType)
                {
                    case ODOMETRY:
                        std::cout << "Odometry message" << std::endl;
                        ParseOdometryMsg(message);
                        break;
                    case POSITION:
                        std::cout << "PoseStamped message" << std::endl;
                        ParsePoseStampedMsg(message);
                        break;
                    case IMU:
                        std::cout << "Imu message" << std::endl;
                        ParseImuMsg(message);
                        break;
                    case STATUS:
                        std::cout << "Status message" << std::endl;
                        ParseStatusMsg(message);
                        break;
                    case ERROR:
                        std::cout << "Error message" << std::endl;
                        ParseErrorMsg(message);
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

void DataReceiver::stop()
{
    stopThreads = true;
    serialPort_->NotifyDataAvailable();  // You might need to implement this
}

// ------------------------------- Parse functions ------------------------------------

void DataReceiver::ParseOdometryMsg(const std::vector<uint8_t>& message)
{
    if (message.size() < 50) // Adjusted for the new message structure
    { 
        throw std::runtime_error("Incomplete message received.");
    }

    // Extracting and printing the 2-byte message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Extracting and printing the timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2); // Starts from the 3rd byte
    std::cout << "Timestamp: " << timestamp << std::endl;

    size_t offset = 10; // Adjust offset for position data start

    // Extracting and printing the position data
    std::cout << "Position Data: ";
    for (size_t i = offset; i < offset + 12; i += 4) 
    {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    offset += 12; // Adjust for quaternion data start

    // Extracting and printing the quaternion data
    std::cout << "Quaternion Data: ";
    for (size_t i = offset; i < offset + 16; i += 4) 
    {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    offset += 16; // Adjust for velocity data start

    // Extracting and printing the velocity data
    std::cout << "Velocity Data: ";
    for (size_t i = offset; i < offset + 12; i += 4) 
    {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;
}

void DataReceiver::ParsePoseStampedMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 38) // Adjusted for the new message structure
    { 
        throw std::runtime_error("Incomplete message received.");
    }
    
    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Extracting timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2);
    std::cout << "Timestamp: " << timestamp << std::endl;

    // Position starts at byte 10
    std::cout << "Position Data: ";
    for (size_t i = 10; i < 22; i += 4) 
    { // 12 bytes for position (x, y, z)
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    // Orientation starts immediately after position
    std::cout << "Quaternion Data: ";
    for (size_t i = 22; i < 38; i += 4) 
    { // 16 bytes for quaternion (x, y, z, w)
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;
}

void DataReceiver::ParseImuMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 50) 
    { 
        throw std::runtime_error("Incomplete message received.");
    }
 
    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Extracting timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2);
    std::cout << "Timestamp: " << timestamp << std::endl;

    // Orientation starts at byte 10
    std::cout << "Orientation Data: ";
    for (size_t i = 10; i < 26; i += 4) { // 16 bytes for quaternion (x, y, z, w)
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    // Angular velocity starts immediately after orientation
    std::cout << "Angular Velocity: ";
    for (size_t i = 26; i < 38; i += 4) { // 12 bytes for angular velocity (x, y, z)
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    // Linear acceleration starts after angular velocity
    std::cout << "Linear Acceleration: ";
    for (size_t i = 38; i < 50; i += 4) // 12 bytes for linear acceleration (x, y, z)
    { 
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;
}

void DataReceiver::ParseStatusMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 18) // Adjusted for the new message structure
    { 
        throw std::runtime_error("Incomplete message received.");
    }
    
    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Status information
    std::cout << "Status Information: ";
    for (size_t i = 2; i < 18; i++) 
    { // Iterate through the 16 bytes of data
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(message[i]) << " ";
    }
    std::cout << std::dec << std::endl; // Switch back to decimal output
}

void DataReceiver::ParseErrorMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 18) // Adjusted for the new message structure
    { 
        throw std::runtime_error("Incomplete message received.");
    }
    
    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Error information
    std::cout << "Error Information: ";
    for (size_t i = 2; i < 18; i++) // Iterate through the 16 bytes of data
    { 
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(message[i]) << " ";
    }
    std::cout << std::dec << std::endl; // Switch back to decimal output
}

// ------------------------------- Helper functions ------------------------------------

// Helper function to convert from big endian to host endianess for floating-point numbers
float DataReceiver::bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset) 
{
    uint32_t temp = 0;
    std::memcpy(&temp, &data[offset], sizeof(temp));
    temp = ntohl(temp); // Assumes your system has ntohl. Otherwise, implement a similar function
    float result;
    std::memcpy(&result, &temp, sizeof(result));
    return result;
}

// Helper function to convert from big endian to host endianess for 64-bit integers
uint64_t DataReceiver::bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset) 
{
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i) 
    {
        result = (result << 8) | data[offset + i];
    }
    return result;
}

