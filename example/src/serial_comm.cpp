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

#include "../include/serial_comm.hpp"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
// serial_comm.cpp
#include <iostream>
// #include "serial_comm.hpp"

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

std::string SerialPort::ExtractMessage()
{
    std::unique_lock<std::mutex> extract_lock(mtx);
    cv.wait(extract_lock, [this] { return data_available; }); //blocks the thread from executing the function  function is still locked by mutex at that time 

    std::size_t start_pos = temp_storage.find("<ST>");
    std::size_t end_pos = temp_storage.find("<EN>", start_pos);

    if (start_pos != std::string::npos && end_pos != std::string::npos)
    {
        std::string message = temp_storage.substr(start_pos, end_pos - start_pos + 4);
        temp_storage.erase(0, end_pos + 4);
        if (temp_storage.empty())
        {
            data_available = false;
        }
        return message;
    }

    data_available = false;
    return "";
}

void SerialPort::ReadFromBuffer()
{
    boost::asio::streambuf read_buf;  // A buffer for incoming data, part of Boost.Asio
    boost::system::error_code error_code;  // info about any error that occurs

    // Read data into the buffer. Since we don't have a specific delimiter that
    // works with read_until, we read available data and then process it to find
    // our message.
    std::size_t n = boost::asio::read(serial, read_buf.prepare(1024), error_code);
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

    // Convert data in the streambuf to a std::string
    std::istream is(&read_buf);
    std::string data((std::istreambuf_iterator<char>(is)),
                     std::istreambuf_iterator<char>());

    std::lock_guard<std::mutex> read_lock(mtx);

   
    temp_storage += data;
    data_available = true;
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
    try 
    {
        // Write data synchronously
        boost::asio::write(serial, boost::asio::buffer(data));
        return true; // Data was successfully written
    } 
    catch (const boost::system::system_error& e) 
    {
        std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
        return false; // An error occurred
    }
}
