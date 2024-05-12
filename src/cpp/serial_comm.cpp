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

#include "../../include/cpp/serial_comm.hpp"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>

enum MessageType 
{
    ODOMETRY = 0x01,
    POSITION = 0x02,
    IMU = 0x03,
    STATUS = 0x04,
    ERROR = 0x05
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
        std::cerr << "Error opening serial port: " << error_code.message() << std::endl;
        return;
    }
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), error_code);
    if (error_code)
    {
        std::cerr << "Error setting baud rate: " << error_code.message()
                  << std::endl;
        return;
    }
}

/*
This is the version with the binary messages prints for the debug

std::vector<std::vector<uint8_t>> SerialPort::ExtractMessageBinary() {
    std::unique_lock<std::mutex> extract_lock(mtx);
    cv.wait(extract_lock, [this] { return data_available; });

    //std::cout << "got the message in ExtractMessageBinary" << std::endl;
    static std::vector<uint8_t> partialMessage; // Static to persist across function calls
    std::vector<uint8_t> message;
    std::vector<uint8_t> processedMessage;
    std::vector<std::vector<uint8_t>> allMessages;
    bool startByteFound = false;
    bool escapeNextByte = false;
    bool endByteFound = false;
    bool uncompleteMsg = false;
    
    int i;

    std::stringstream hexStream1;
    hexStream1 << "temp_storage_binary: ";
    for (auto &byte : temp_storage_binary) {
        // Ensure that std::hex and formatting applies to each byte correctly
        hexStream1 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
    }

    // Use the insertion operator (<<) to output the stringstream's contents to std::cout
    std::cout << hexStream1.str() << std::endl;
    std::cout << "\nbefore checking for partial\n" << std::endl;

    // Prepend any existing partial message to the start of the temp_storage_binary
    if (!partialMessage.empty()) {
        temp_storage_binary.insert(temp_storage_binary.begin(), partialMessage.begin(), partialMessage.end());
        partialMessage.clear(); // Clear partial message after merging
        std::cout << "got partial message in ExtractMessageBinary" << std::endl;
    }

    while(!temp_storage_binary.empty() && !uncompleteMsg)
    {
        std::stringstream hexStream101;
        hexStream101 << "temp_storage_binary before for loop: ";
        for (auto &byte : temp_storage_binary) {
            // Ensure that std::hex and formatting applies to each byte correctly
            hexStream101 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
        }

        std::cout << hexStream101.str() << std::endl;
        std::cout << "\n" << std::endl;
        
        //std::cout << "got inside the while" << std::endl;
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

        std::stringstream hexStream10;
        hexStream10 << "temp_storage_binary after for loop: ";
        for (auto &byte : temp_storage_binary) {
            // Ensure that std::hex and formatting applies to each byte correctly
            hexStream10 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
        }

        std::cout << hexStream10.str() << std::endl;
        std::cout << "\n" << std::endl;

        std::stringstream hexStream20;
        hexStream20 << "message: ";
        for (auto &byte : message) {
            // Ensure that std::hex and formatting applies to each byte correctly
            hexStream20 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
        }

        std::cout << hexStream20.str() << std::endl;
        std::cout << "\n" << std::endl;
        
        if(endByteFound)
        {
            allMessages.push_back(message);
        }
        message.clear();

        std::vector<uint8_t> temp = allMessages.back();

        std::stringstream hexStream100;
        hexStream100 << "check: ";
        for (auto &byte : temp) {
            // Ensure that std::hex and formatting applies to each byte correctly
            hexStream100 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
        }

        std::cout << hexStream100.str() << std::endl;

        std::cout << "\nsaved the new complete message" << std::endl;

        if(endByteFound && startByteFound)
        {
            temp_storage_binary.erase(temp_storage_binary.begin(), temp_storage_binary.begin() + i);
        }
        else {
            uncompleteMsg = true;
        }
        endByteFound = false;
        startByteFound = false;
        
        //std::cout << "deleted the message\n" << std::endl;

    }

    std::cout << "finished going throught temporary at ExtractMessageBinary" << std::endl;
    std::cout << "\n" << std::endl;
    // std::cout << endByteFound << std::endl;
    // std::cout << "\n" << std::endl;

    if (!temp_storage_binary.empty()) {
        // If a message is started but not completed, save the remainder as a partial message
        if(!temp_storage_binary.empty()) {
            partialMessage.insert(partialMessage.begin(), temp_storage_binary.begin(), temp_storage_binary.end());
            temp_storage_binary.clear();
        }
        std::cout << "\nleft partial message in ExtractMessageBinary" << std::endl;
    }

    // std::cout << "\n after if (!endByteFound)\n" << std::endl;

    std::stringstream hexStream2;
    hexStream2 << "message: ";
    for (auto &byte : message) {
        // Ensure that std::hex and formatting applies to each byte correctly
        hexStream2 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
    }

    // // Use the insertion operator (<<) to output the stringstream's contents to std::cout
    std::cout << hexStream2.str() << std::endl;
    std::cout << "\n" << std::endl;

    std::stringstream hexStream3;
    hexStream3 << "partialMessage: ";
    for (auto &byte : partialMessage) {
        // Ensure that std::hex and formatting applies to each byte correctly
        hexStream3 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
    }

    // // Use the insertion operator (<<) to output the stringstream's contents to std::cout
    std::cout << hexStream3.str() << std::endl;
    std::cout << "\n" << std::endl;

    std::stringstream hexStream4;
    hexStream4 << "temp_storage_binary: ";
    for (auto &byte : temp_storage_binary) {
        // Ensure that std::hex and formatting applies to each byte correctly
        hexStream4 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
    }

    // // Use the insertion operator (<<) to output the stringstream's contents to std::cout
    std::cout << hexStream4.str() << std::endl;
    std::cout << "\n" << std::endl;

    // Process the complete message, if any
    for(auto msg = allMessages.begin(); msg != allMessages.end(); msg++)
    {
        std::vector<uint8_t> currMsg = *msg;
        if (!currMsg.empty()) {
            // Assuming checksum is the last two bytes of the message
            if (currMsg.size() > 2) {
                processedMessage.assign(currMsg.begin() + 1, currMsg.end() - 3); // Exclude start, end and checksum bytes 
                
                // std::stringstream hexStream5;
                // hexStream5 << "Binary message: ";
                // for (auto &byte : processedMessage) {
                //     // Ensure that std::hex and formatting applies to each byte correctly
                //     hexStream5 << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
                // }

                // // Use the insertion operator (<<) to output the stringstream's contents to std::cout
                // std::cout << hexStream5.str() << std::endl;

                uint16_t receivedChecksum = (static_cast<uint16_t>(currMsg[currMsg.size() - 3]) << 8) + currMsg[currMsg.size() - 2];
                uint16_t calculatedChecksum = calculateChecksumBinary(processedMessage);

                //std::cout << "go to checksum check in ExtractMessageBinary" << std::endl;

                if (calculatedChecksum != receivedChecksum) {
                    std::cerr << "Checksum mismatch." << std::endl;
                    // Handle checksum mismatch, perhaps clear processedMessage to indicate an error
                    processedMessage.clear();
                    allMessages.erase(msg);
                } else {
                    //std::cerr << "Checksum successfully verified." << std::endl;
                    // Checksum valid, processedMessage contains the verified message data
                }
            } else {
                std::cerr << "Incomplete message received." << std::endl;
                processedMessage.clear(); // Clear processedMessage to indicate an error or incomplete state
                allMessages.erase(msg);
            }
        }
    }
    

    // Clear temp_storage_binary since it's been processed or moved to partialMessage
    temp_storage_binary.clear();

    //std::cout << "emptyed temp in ExtractMessageBinary" << std::endl;

    // if(!partialMessage.empty())
    // {
        // std::stringstream hexStream300;
        // hexStream300 << "partialMessage: ";
        // for (auto &byte : partialMessage) {
        //     // Ensure that std::hex and formatting applies to each byte correctly
        //     hexStream300<< std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte) << " ";
        // }

        // // Use the insertion operator (<<) to output the stringstream's contents to std::cout
        // std::cout << hexStream300.str() << std::endl;
        // std::cout << "the partial message to next time\n" << std::endl;
    // }

    data_available = false; // Reflect whether there's pending data to process

    return allMessages;
}
*/

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
    std::cout << "inside ReadFromBuffer" << std::endl;
    boost::asio::streambuf read_buf;       // A buffer for incoming data, part of Boost.Asio
    boost::system::error_code error_code;  // Info about any error that occurs

    // Attempt to read up to TARGET_MESSAGE_SIZE bytes
    std::size_t n = boost::asio::read(serial, read_buf.prepare(TARGET_MESSAGE_SIZE), error_code);
    read_buf.commit(n); // Make the read data available in the input sequence

    std::cout << "read data with asio" << std::endl;
    // Logging the number of bytes read for debugging purposes
    std::cout << "\nNumber of bytes been read: " << n << std::endl;

    // Handle possible read errors
    if (error_code && error_code != boost::asio::error::eof) // EOF is expected for the last read
    { 
        std::cerr << "Error during read: " << error_code.message() << std::endl;
        return;  // Early exit on error
    }

    // If data was read, convert it to a vector<uint8_t> for further processing
    std::vector<uint8_t> dataPtr(boost::asio::buffers_begin(read_buf.data()),
                                 boost::asio::buffers_begin(read_buf.data()) + n);

    // Ensure we actually read data before proceeding
    if (!dataPtr.empty()) 
    {
        std::lock_guard<std::mutex> read_lock(mtx); // Lock the mutex to safely access shared data
        temp_storage_binary.insert(temp_storage_binary.end(), dataPtr.begin(), dataPtr.end());

        // Notify that new data is available if this is a transition from no data to having data
        if (!data_available) 
        {
            data_available = true;
            cv.notify_one(); // Notify potentially waiting threads that new data is available
            std::cout << "Stored the message " << n << " bytes" << std::endl;
        }
    }
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
    try 
    {
        // Write data synchronously
        boost::asio::write(serial, boost::asio::buffer(data));
        return true; // Data was successfully written
    }
    catch(const boost::system::system_error& e) 
    {
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