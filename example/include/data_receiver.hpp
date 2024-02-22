#ifndef DATA_RECEIVER_HPP
#define DATA_RECEIVER_HPP

#include "pose_message.hpp"
#include "serial_comm.hpp"
class DataReceiver
{
   public:
    explicit DataReceiver(SerialPort* serialPort);
    void stop();
    void readData();
    void processData();
    void updateBaudRate(unsigned int newBaudRate);
    uint16_t calculateCheckSum(std::array<std::string, 8> &sliced_msg_parts);
    std::array<std::string, 8> ParseAndPrintMsg(const std::string& message);

   private:
    SerialPort* serialPort_;
    std::mutex mtx;  // For synchronizing access to a shared resource
    std::condition_variable cv;  // For signaling between threads
    bool stopThreads = false;    // Flag to control the stopping of threads

    std::vector<float> parseCSVFloats(const std::string& csv);
};

#endif  // DATA_RECEIVER_HPP