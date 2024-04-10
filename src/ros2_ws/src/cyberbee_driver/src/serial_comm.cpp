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

enum MessageType 
{
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

std::vector<std::vector<uint8_t>> SerialPort::ExtractMessageBinary()
{
    std::unique_lock<std::mutex> extract_lock(mtx);
    cv.wait(extract_lock, [this] { return data_available; });

    static std::vector<uint8_t> partialMessage; // Static to persist across function calls
    std::vector<uint8_t> message;
    std::vector<uint8_t> processedMessage;
    std::vector<std::vector<uint8_t>> allMessages;
    bool startByteFound = false;
    bool escapeNextByte = false;
    bool endByteFound = false;
    bool uncompleteMsg = false;
    int i;

    // Prepend any existing partial message to the start of the temp_storage_binary
    if (!partialMessage.empty()) 
    {
        temp_storage_binary.insert(temp_storage_binary.begin(), partialMessage.begin(), partialMessage.end());
        partialMessage.clear(); // Clear partial message after merging
        std::cout << "got partial message in ExtractMessageBinary" << std::endl;
    }
    
    
    while(!temp_storage_binary.empty() && !uncompleteMsg)
    {
        i = 0;
        for (auto it = temp_storage_binary.begin(); it != temp_storage_binary.end() && !endByteFound; ++it) 
        {
            uint8_t byte = *it;
            if (!startByteFound && byte == START_BYTE) // [start wasn't found yet and current byte is equal to start]
            { 
                startByteFound = true;                   
                message.clear();                       
                message.push_back(START_BYTE);           
            } 
            else if(startByteFound) // won't enter until we found the start byte
            {
                if (byte == END_BYTE && !escapeNextByte) // if the byte found is exit byte, and no escape byte is in front 
                {                                        // of it
                    message.push_back(byte); 
                    endByteFound = true; 
                } 
                else if(byte == START_BYTE && !escapeNextByte) //if I found another start byte but without escape byte 
                {                                              // beforehand
                    endByteFound = true;
                    temp_storage_binary.insert(temp_storage_binary.begin(), byte);
                }
                else if (byte == ESCAPE_BYTE && !escapeNextByte) // a escape byte was found but first time
                {                                                // more relevent to the next byte
                    escapeNextByte = true; 
                }
                else 
                {
                    if (escapeNextByte) // if there is an escape byte we check if it is for esc byte or data
                    {
                        if(byte == START_BYTE || byte == END_BYTE) 
                        {
                            escapeNextByte = false;
                            message.push_back(byte);
                        }
                        else if(byte == ESCAPE_BYTE) 
                        {
                            message.push_back(ESCAPE_BYTE);
                        }
                        else 
                        {
                            message.push_back(ESCAPE_BYTE);
                            message.push_back(byte);
                            escapeNextByte = false;
                        }
                    }
                    else 
                    {
                        message.push_back(byte); // Add byte to current message
                    }
                }
            }
            i++;
        }
        
        if(endByteFound)
        {
            allMessages.push_back(message);
        }
        message.clear();

        std::vector<uint8_t> temp = allMessages.back();

        if(endByteFound && startByteFound)
        {
            temp_storage_binary.erase(temp_storage_binary.begin(), temp_storage_binary.begin() + i);
        }
        else 
        {
            uncompleteMsg = true;
        }
        endByteFound = false;
        startByteFound = false;
    }

    if (!temp_storage_binary.empty()) 
    {
        // If a message is started but not completed, save the remainder as a partial message
        if(!temp_storage_binary.empty()) 
        {
            partialMessage.insert(partialMessage.begin(), temp_storage_binary.begin(), temp_storage_binary.end());
            temp_storage_binary.clear();
        }
        std::cout << "\nleft partial message in ExtractMessageBinary" << std::endl;
    }

    // Process the complete message, if any
    for(auto msg = allMessages.begin(); msg != allMessages.end(); msg++)
    {
        std::vector<uint8_t> currMsg = *msg;
        if (!currMsg.empty()) 
        {
            // Assuming checksum is the last two bytes of the message
            if (currMsg.size() > 2) 
            {
                processedMessage.assign(currMsg.begin() + 1, currMsg.end() - 3); // Exclude start, end and checksum bytes 

                uint16_t receivedChecksum = (static_cast<uint16_t>(currMsg[currMsg.size() - 3]) << 8) + currMsg[currMsg.size() - 2];
                uint16_t calculatedChecksum = calculateChecksumBinary(processedMessage);

                if (calculatedChecksum != receivedChecksum) 
                {
                    std::cerr << "Checksum mismatch." << std::endl;
                    // Handle checksum mismatch, perhaps clear processedMessage to indicate an error
                    processedMessage.clear();
                    allMessages.erase(msg);
                } else 
                {
                    //std::cerr << "Checksum successfully verified." << std::endl;
                    // Checksum valid, processedMessage contains the verified message data
                }
            } else 
            {
                std::cerr << "Incomplete message received." << std::endl;
                processedMessage.clear(); // Clear processedMessage to indicate an error or incomplete state
                allMessages.erase(msg);
            }
        }
    }
    
    // Clear temp_storage_binary since it's been processed or moved to partialMessage
    temp_storage_binary.clear();
    data_available = false; // Reflect whether there's pending data to process

    return allMessages;
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

uint16_t SerialPort::calculateChecksumBinary(const std::vector<uint8_t>& data) 
{
    uint16_t checksum = 0;
    for (auto byte : data) 
    {
        checksum += byte;
    }
    
    return checksum;
}
