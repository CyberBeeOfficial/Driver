#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <sstream>
#include <iomanip>

/*









#include <stdexcept> 
#include <string>
#include <tuple>
#include <vector>
*/

class SerialPort
{
   public:
    std::string port;
    unsigned int baud_rate;
    std::string writeBuffer;

    SerialPort(const std::string& port, unsigned int baud_rate);
    ~SerialPort();
    void OpenSerialConnection();
    void CloseSerialConnection();
    bool WriteToBuffer(const std::string &data);
    void ReadFromBuffer();
    void NotifyDataAvailable();
    bool IsSerialConnectionOpen() const;
    std::vector<std::vector<uint8_t>> ExtractMessageBinary();

   private:

    uint16_t calculateChecksumBinary(const std::vector<uint8_t>& data);
    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
    std::string temp_storage;     // Buffer for incoming data
    std::vector<uint8_t> temp_storage_binary; // Buffer for incoming data binary
    std::mutex mtx;               // Mutex for temp_storage access
    std::condition_variable cv;   // For signaling new data
    bool data_available = false;  // Flag to indicate new data is available
    int messageSize_;             // Sets what size the sent message is to extract
    static constexpr size_t TARGET_MESSAGE_SIZE = 256;
    static constexpr uint8_t START_BYTE = 0xFF;
    static constexpr uint8_t END_BYTE = 0xEE;
    static constexpr uint8_t ESCAPE_BYTE = 0x10;
};

#endif  // for SERIAL_COMM_H