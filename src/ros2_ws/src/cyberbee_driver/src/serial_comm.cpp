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

#include "../include/cyberbee_driver/serial_comm.hpp"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
// serial_comm.cpp
#include <iostream>
// #include "serial_comm.hpp"

enum MessageType {
    ODOMETRY = 0x01,
    POSITION = 0x02,
    IMU = 0x03,
    STATUS = 0x04,
    ERROR = 0x05,
    // Add more message types as needed
};

SerialPort::SerialPort(const std::string& port, unsigned int baud_rate)
    : io_service(), serial(io_service), port(port), baud_rate(baud_rate)
{
    // Constructor doesn't open the port immediately anymore
}

SerialPort::~SerialPort()
{
    CloseSerialConnection();  // Ensure the connection is closed on destruction
}

void SerialPort::CloseSerialConnection()
{
    boost::system::error_code error_code;
    if (serial.is_open())
    {
        serial.close(error_code);
        if (error_code)
        {
            std::cerr << "Error closing serial port: " << error_code.message()
                      << std::endl;
        }
    }
}

void SerialPort::OpenSerialConnection()
{
    boost::system::error_code error_code;
    serial.open(port, error_code);
    if (error_code)
    {
        std::cerr << "Error opening serial port: " << error_code.message()
                  << std::endl;
        return;
    }
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),
                      error_code);
    if (error_code)
    {
        std::cerr << "Error setting baud rate: " << error_code.message()
                  << std::endl;
        return;
    }
}

std::vector<uint8_t> SerialPort::ExtractMessageBinary()
{
    std::unique_lock<std::mutex> extract_lock(mtx);
    cv.wait(extract_lock, [this] { return data_available; });

    if (temp_storage_binary.size() < TARGET_MESSAGE_SIZE) { // Minimum size of the message without markers
        data_available = false;
        return {}; // Return empty vector if no valid message is found or message is incomplete
    }
    
    // Extracting and printing the 2-byte message type
    uint16_t messageType = (static_cast<uint16_t>(temp_storage_binary[0]) << 8) | temp_storage_binary[1];

    std::cout << "Message Type: " << messageType << std::endl;

    switch(messageType)
    {
        case ODOMETRY:
            std::cout << "Got Odometry message"<< std::endl;
            messageSize_ = 52;
            break;
        case POSITION:
            std::cout << "Got Pose message"<< std::endl;
            messageSize_ = 40;
            break;
        case IMU:
            std::cout << "Got Pose message"<< std::endl;
            messageSize_ = 52;
            break;
        case STATUS:
            std::cout << "Got Pose message"<< std::endl;
            messageSize_ = 20;
            break;
        case ERROR:
            std::cout << "Got Pose message"<< std::endl;
            messageSize_ = 20;
            break;
        default:
            std::cout << "Got unknown message"<< std::endl;
            messageSize_ = 20;
            break;
    }

    std::vector<uint8_t> message(temp_storage_binary.begin(), temp_storage_binary.begin() + messageSize_);

    // After extracting the message, erase it from the buffer
    temp_storage_binary.erase(temp_storage_binary.begin(), temp_storage_binary.end());

    if (temp_storage_binary.empty()) {
        data_available = false;
    } else {
        // If there's more data, leave data_available as true
    }

    return message;
}

void SerialPort::ReadFromBuffer() 
{
    boost::asio::streambuf
        read_buf;  // A buffer for incoming data, part of Boost.Asio
    boost::system::error_code error_code;  // info about any error that occurs

    // Read data into the buffer. Since we don't have a specific delimiter that
    // works with read_until, we read available data and then process it to find
    // our message.
    std::size_t n = boost::asio::read(serial, read_buf.prepare(TARGET_MESSAGE_SIZE), error_code);
    read_buf.commit(n);  // Make read data available in the input sequence.

    // TODO: n is number of bytes read - find a usage or remove

    std::cout << "number of bytes been read: " << n << std::endl;

    // Check if an error occurred during the read operation, excluding
    // end-of-file (EOF) which is normal for last read
    if (error_code && error_code != boost::asio::error::eof)
    {
        std::cerr << "Error during read: " << error_code.message() << std::endl;
        return;  
    }
    
    std::vector<uint8_t> dataPtr(boost::asio::buffers_begin(read_buf.data()), 
                                 boost::asio::buffers_begin(read_buf.data()) + n);

    {
        std::lock_guard<std::mutex> read_lock(mtx);
        temp_storage_binary.insert(temp_storage_binary.end(), dataPtr.begin(), dataPtr.end());
        std::cout << "stored the message " << n << std::endl;
        data_available = true;
    }
    cv.notify_one();
}

void SerialPort::NotifyDataAvailable()
{
    std::lock_guard<std::mutex> lock(mtx);  // Lock the mutex
    data_available = true;  // Set the flag indicating data is available
    cv.notify_one();        // Wake up one waiting thread
}

bool SerialPort::IsSerialConnectionOpen() const { return serial.is_open(); }

bool SerialPort::WriteToBuffer(const std::string &data)
{
    try {
            // Write data synchronously
            boost::asio::write(serial, boost::asio::buffer(data));
            return true; // Data was successfully written
        } catch (const boost::system::system_error& e) {
            std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
            return false; // An error occurred
        }
}
