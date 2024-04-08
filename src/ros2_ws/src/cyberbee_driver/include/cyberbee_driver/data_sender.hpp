#ifndef DATA_SENDER_HPP
#define DATA_SENDER_HPP

#include "serial_comm.hpp"

class DataSender
{
   public:
    explicit DataSender(SerialPort* serialPort);

    void SendCommand(uint8_t command, const std::string& data);

   private:
    SerialPort* serialPort;
    uint16_t calculateCheckSum(const std::string& message);
};

#endif  // DATA_SENDER_HPP