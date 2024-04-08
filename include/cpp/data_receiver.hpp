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
    
    void ParseOdometryMsg(const std::vector<uint8_t>& message);
    void ParsePoseStampedMsg(const std::vector<uint8_t>& message);
    void ParseImuMsg(const std::vector<uint8_t>& message);
    void ParseStatusMsg(const std::vector<uint8_t>& message);
    void ParseErrorMsg(const std::vector<uint8_t>& message);

    float bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset);
    uint64_t bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset);
    uint16_t calculateChecksumBinary(const std::vector<uint8_t>& data);

   private:
    SerialPort* serialPort_;
    std::mutex mtx;  // For synchronizing access to a shared resource
    std::condition_variable cv;  // For signaling between threads
    bool stopThreads = false;    // Flag to control the stopping of threads
};

#endif  // DATA_RECEIVER_HPP