#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class SerialPort
{
   public:
    std::string port;
    unsigned int baud_rate;
    std::string writeBuffer;

    SerialPort(const std::string& port, unsigned int baud_rate);
    ~SerialPort();
    // void OpenSerialConnection(boost::system::error_code& error); //same as
    // initialize in python file
    void OpenSerialConnection();
    void CloseSerialConnection();
    bool WriteToBuffer(const std::string &data);
    void ReadFromBuffer();
    void NotifyDataAvailable();
    std::string
    ExtractMessage();  // Method to extract messages from temp_storage
    bool IsSerialConnectionOpen() const;

   private:
    void Run();

    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
    std::string temp_storage;     // Buffer for incoming data
    std::mutex mtx;               // Mutex for temp_storage access
    std::condition_variable cv;   // For signaling new data
    bool data_available = false;  // Flag to indicate new data is available
};

#endif  // for SERIAL_COMM_H