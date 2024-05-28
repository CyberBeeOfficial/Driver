/*
 * Copyright (c) 2024 CyberBee Ltd.
 * All rights reserved.
 *
 * This file is part of the CyberBee IMX8 C++ Driver.
 * 
 * Description:
 * This code is part of the CyberBee  IMX8 C++ Driver layer, responsible for
 * managing communication between imx8 and other devices sucha s PI-4B.
 * This code meant to be compiled on the client device i.e. PI-4B,or other ubuntu system.
 *
 */

#include "../include/data_sender.hpp"
#include <iostream>
#include <sstream>

enum UserCommand
{
    TestBaudRate = 0x20,
    SetDivisionRate = 0x22,
    Confirm = 0x23,
    ChangeBaudRate = 0x30,
    GpsMsg = 0x25,
    EstReset = 0x27
    // Add more commands as needed
};

DataSender::DataSender(SerialPort* serialPort) : serialPort(serialPort)
{
    // data sender constructor
}

uint16_t DataSender::calculateCheckSum(const std::string& message) 
{  
    uint16_t checksum = 0;
    for (char c : message) 
    {
        checksum += c;
    }
    return checksum;
}

void DataSender::SendCommand(uint8_t command, const std::string& data, bool loop)
{
    std::stringstream temp_ss;
    std::stringstream full_ss;
    uint16_t chksum;

    if ((serialPort) && (serialPort->IsSerialConnectionOpen()))
    {
        std::cout << "Serial connection is open." << std::endl;
        temp_ss << static_cast<int>(command) << ";" << data;  // Use int to print numeric value
        chksum = DataSender::calculateCheckSum(temp_ss.str());
        std::string chksumStr = std::to_string(chksum);

        full_ss << "<ST>;" << static_cast<int>(command) << ";" << data << ";" << chksumStr << ";<EN>";
        std::cout << full_ss.str() << std::endl;
        if (loop == 1){
            while (serialPort->IsSerialConnectionOpen())
            {
                auto write_success = serialPort->WriteToBuffer(full_ss.str());
                if (write_success)
                {
                    std::cout << "Command sent successfully: " << full_ss.str() << std::endl;
                }
                else
                {
                    std::cout << "Failed to send the command due to a communication error." << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } else{
            if (serialPort->IsSerialConnectionOpen())
            {
                auto write_success = serialPort->WriteToBuffer(full_ss.str());
                if (write_success)
                {
                    std::cout << "Command sent successfully: " << full_ss.str() << std::endl;
                }
                else
                {
                    std::cout << "Failed to send the command due to a communication error." << std::endl;
                }
            }
        }
    }
    else
    {
        std::cout << "Serial connection is closed, Command can't be sent" << std::endl;
    }
}

std::stringstream DataSender::createGpsMsg()
{
    std::stringstream ss;
    
    // Obtaining the current time in seconds and nanoseconds since epoch
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;

    // Start building the GPS message with current timestamp
    ss << seconds << "." << nanoseconds << ";";
    ss << "31.2006,35.0538,11077.7;";  // Latitude, Longitude, and Altitude set to zero
    ss << "0.0263676,0.0715068,-0.10377;1"; // Yaw, Pitch, Roll set to zero, and Status set to one
    return ss;
}