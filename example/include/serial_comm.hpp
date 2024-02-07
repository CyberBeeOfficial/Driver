#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class SerialPort
{
   public:
    std::string port;
    unsigned int baud_rate;

    SerialPort(const std::string& port, unsigned int baud_rate);
    ~SerialPort();
    // void OpenSerialConnection(boost::system::error_code& error); //same as
    // initialize in python file
    void StartCommunication();
    void CloseSerialConnection();

   private:
    void Run();

    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
    // std::string port;
    // unsigned int baud_rate;
    std::thread communication_thread;
    std::atomic<bool> should_run;
};

#endif  // for SERIAL_COMM_H