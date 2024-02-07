#include "../include/serial_comm.hpp"
// serial_comm.cpp
#include <iostream>
// #include "serial_comm.hpp"

SerialPort::SerialPort(const std::string& port, unsigned int baud_rate)
    : io_service(),
      serial(io_service),
      port(port),
      baud_rate(baud_rate),
      should_run(false)
{
    // Constructor doesn't open the port immediately anymore
}

SerialPort::~SerialPort()
{
    CloseSerialConnection();  // Ensure the connection is closed on destruction
}

void SerialPort::StartCommunication()
{
    should_run.store(true);
    communication_thread = std::thread(&SerialPort::Run, this);
}

void SerialPort::CloseSerialConnection()
{
    should_run.store(false);
    if (communication_thread.joinable())
    {
        communication_thread.join();  // Wait for the thread to finish
    }
    boost::system::error_code ec;
    if (serial.is_open())
    {
        serial.close(ec);
        if (ec)
        {
            std::cerr << "Error closing serial port: " << ec.message()
                      << std::endl;
        }
    }
}

void SerialPort::Run()
{
    boost::system::error_code ec;
    serial.open(port, ec);
    if (ec)
    {
        std::cerr << "Error opening serial port: " << ec.message() << std::endl;
        return;
    }
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), ec);
    if (ec)
    {
        std::cerr << "Error setting baud rate: " << ec.message() << std::endl;
        return;
    }

    while (should_run.load())
    {
        // Perform serial port communication here (e.g., read/write)
        // Make sure to handle errors and potentially break the loop if
        // necessary
    }
}
