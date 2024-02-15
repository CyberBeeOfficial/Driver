#include "../include/data_receiver.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>  // Includes standard exception types
#include <string>
#include <tuple>
#include <vector>

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
        std::string message =
            serialPort_->ExtractMessage();  // Assuming this method now just
                                            // extracts based on `temp_storage`
        if (!message.empty())
        {
            ParseAndPrintMsg(message);
        }

        // Optionally, add a short delay or use a condition variable to wait for
        // new data
    }
}

void DataReceiver::stop()
{
    stopThreads = true;
    // Ensure ExtractMessage exits if waiting
    serialPort_->NotifyDataAvailable();  // You might need to implement this
    // SerialPort to directly signal cv
}

uint16_t DataReceiver::calculateCheckSum(
    std::array<std::string, 8>& sliced_msg_parts)
{
    // Concatenate the relevant parts with semicolons (if needed)
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

std::array<std::string, 8> DataReceiver::ParseAndPrintMsg(
    const std::string& message)
{
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

    // function to parse a cooma seperated string to 3 float tuple
    //  Example function to parse a comma-separated string into a tuple of three
    //  floats

    try
    {
        int parts_counter = 0;
        while (std::getline(PartsStream, part, delimiter) &&
               index < sliced_msg_parts.size())
        {
            sliced_msg_parts[index++] = part;
            parts_counter += 1;
        }

        // for (const auto& p : sliced_msg_parts)
        // {
        //     std::cout << p << std::endl;
        // }
        // make sure message is intact
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
