#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

class SerialPort{
public:
    std::string port;
    unsigned int baud_rate;

    SerialPort(const std::string& port, unsigned int baud_rate);

    
    void OpenSerialConnection(boost::system::error_code& error); //same as initialize in python file 

private:
   
    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
    std::mutex mtx; // Mutex for thread-safe data access
    boost::system::error_code error;



};



#endif //for SERIAL_COMM_H