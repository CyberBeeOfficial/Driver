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

#include "../include/data_receiver.hpp"

#include <iostream>
#include <sstream>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numeric>

#include <stdexcept>  // Includes standard exception types
#include <string>
#include <tuple>
#include <vector>

// Define message types
enum MessageType {
    ODOMETRY = 0x01,
    POSITION = 0x02,
    // Add more message types as needed
};

DataReceiver::DataReceiver(SerialPort* serialPort) : serialPort_(serialPort)
{
    // data receiver init shall  be here
}

std::vector<float> DataReceiver::parseCSVFloats(const std::string& csv)
{
    std::istringstream stream(csv);
    std::string token;
    std::vector<float> floats;

    while (std::getline(stream, token, ','))
    {
        try
        {
            floats.push_back(std::stof(token));
        }
        catch (const std::invalid_argument& e)
        {
            throw std::runtime_error("Invalid format encountered.");
        }
        catch (const std::out_of_range& e)
        {
            throw std::runtime_error("Float value out of range.");
        }
    }

    return floats;
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
    while (!stopThreads)
    {
        // This part needs to check the message and only than parse it in a different function
        std::vector<uint8_t> message = serialPort_->ExtractMessageBinary();  // Assuming this method now just
                                            // extracts based on `temp_storage`
        if (!message.empty())
        {
            ParseAndPrintMsgBinary(message);
        }
    }
}

void DataReceiver::stop()
{
    stopThreads = true;
    serialPort_->NotifyDataAvailable();  // You might need to implement this
}

uint16_t DataReceiver::calculateCheckSum(
    std::array<std::string, 8>& sliced_msg_parts)
{
    std::string concatenatedParts =
        sliced_msg_parts[1] + ";" + sliced_msg_parts[2] + ";" +
        sliced_msg_parts[3] + ";" + sliced_msg_parts[4] + ";" +
        sliced_msg_parts[5] + ";";

    uint16_t checksum = 0;
    for (char c : concatenatedParts)
    {
        checksum += c;
    }
    return checksum;
}

void DataReceiver::ParseAndPrintMsgBinary(const std::vector<uint8_t>& message)
{
    if (message.size() < 52) { // Adjusted for the new message structure
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
    for (size_t i = offset; i < offset + 12; i += 4) {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    offset += 12; // Adjust for quaternion data start

    // Extracting and printing the quaternion data
    std::cout << "Quaternion Data: ";
    for (size_t i = offset; i < offset + 16; i += 4) {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    offset += 16; // Adjust for velocity data start

    // Extracting and printing the velocity data
    std::cout << "Velocity Data: ";
    for (size_t i = offset; i < offset + 12; i += 4) {
        std::cout << bigEndianToFloat(message, i) << " ";
    }
    std::cout << std::endl;

    // Verifying and printing the 2-byte checksum
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[message.size() - 2]) << 8) | message[message.size() - 1];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum 
              << " Received Checksum: " << receivedChecksum << std::endl;

    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
        if (!isTimeMarked) 
            {
              startTime = std::chrono::steady_clock::now();
              isTimeMarked = true;
            }
            messageCountBinary++;
            std::cout << messageCountBinary << std::endl;
            if(messageCountBinary % 15 == 0)
            {
              auto endTime = std::chrono::steady_clock::now();
              std::chrono::duration<double> elapsed = endTime - startTime;
              double frequency = messageCountBinary / elapsed.count();
              std::cout << "\n\n\n Message send frequency: " << frequency << "messages per second \n\n\n" << std::endl; 
              messageCountBinary=0;
              isTimeMarked = false;
            }
    }

}

std::array<std::string, 8> DataReceiver::ParseAndPrintMsg(
    const std::string& message) //this is relevent to handeling strings, so we 
{                               // put aside for now
    /// veriables to hold sections of msg
    std::vector<std::string> messageParts = { "prefix_value",
                                              "type_value",
                                              "timestamp",
                                              "position_data_value",
                                              "quaternion_data_value",
                                              "velocity_data_value",
                                              "received_checksum_value",
                                              "suffix_value" };

    uint16_t computed_checksum;

    char delimiter = ';';
    std::string part;
    std::array<std::string, 8> sliced_msg_parts;
    std::istringstream PartsStream(message);
    size_t index = 0;

    std::cout << "Raw message :" << std::endl;
    std::cout << message << std::endl;

    // Splitting the message into its components
    try
    {
        int parts_counter = 0;
        while (std::getline(PartsStream, part, delimiter) &&
               index < sliced_msg_parts.size())
        {
            sliced_msg_parts[index++] = part;
            parts_counter += 1;
        }

        if (parts_counter != 8)
        {
            throw std::runtime_error("Expected 8 parts in the message, found " +
                                     std::to_string(parts_counter));
        }

        std::string msg_prefix = sliced_msg_parts[0];
        std::string msg_type = sliced_msg_parts[1];
        std::string timestamp = sliced_msg_parts[2];
        std::string position_data = sliced_msg_parts[3];
        std::string quaternion_data = sliced_msg_parts[4];
        std::string velocity_data = sliced_msg_parts[5];
        uint16_t received_checksum =
            static_cast<uint16_t>(std::stoul(sliced_msg_parts[6]));
        std::string msgSuffix = sliced_msg_parts[7];

        // function to parse a cooma seperated string to 3 float tuple
        auto position = parseCSVFloats(sliced_msg_parts[3]);
        auto quaternion = parseCSVFloats(sliced_msg_parts[4]);
        auto velocity = parseCSVFloats(sliced_msg_parts[5]);

        // checksum  called
        computed_checksum = this->calculateCheckSum(sliced_msg_parts);
        std::cout << "computed checksum: " << computed_checksum << "  VS "
                  << "Received Checksum : " << received_checksum << std::endl;
        if (computed_checksum != received_checksum)
        {
            throw std::runtime_error("Checksums Don't Match");
        }
        PoseMessage my_message(msg_type, timestamp, position, quaternion,
                               velocity);
        return sliced_msg_parts;
    }

    catch (const std::runtime_error& e)
    {
        // Handle specific exceptions
        std::cerr << "Run Time Error: " << e.what() << std::endl;
    }

    catch (...)
    {
        // Catch all handler for any other types of exceptions
        std::cerr << "Caught an unknown exception" << std::endl;
    }

    return sliced_msg_parts;
}

// Helper functions ---------------------------------------------------------

// Helper function to convert from big endian to host endianess for floating-point numbers
float DataReceiver::bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset) {
    uint32_t temp = 0;
    std::memcpy(&temp, &data[offset], sizeof(temp));
    temp = ntohl(temp); // Assumes your system has ntohl. Otherwise, implement a similar function
    float result;
    std::memcpy(&result, &temp, sizeof(result));
    return result;
}

// Helper function to convert from big endian to host endianess for 64-bit integers
uint64_t DataReceiver::bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset) {
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i) {
        result = (result << 8) | data[offset + i];
    }
    return result;
}

// Helper function for 16-bit checksum calculation
uint16_t DataReceiver::calculateChecksumBinary(const std::vector<uint8_t>& data) {
    uint32_t sum = std::accumulate(data.begin(), data.end() - 2, 0u); // Include all bytes for the sum except the last two checksum bytes
    // No need to convert sum to big endian, simply truncate to 16 bits
    uint16_t checksum = static_cast<uint16_t>(sum & 0xFFFF);
    return checksum; // Return the checksum without converting it to big endian
}
