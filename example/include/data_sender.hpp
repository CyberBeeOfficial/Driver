#ifndef DATA_SENDER_HPP
#define DATA_SENDER_HPP

#include "serial_comm.hpp"
enum UserCommand
{
    TestBaudRate = 0x20,
    SetDivisionRate = 0x22,
    Confirm = 0x23,
    ChangeBaudRate = 0x30,
    GpsMsg = 0x25
    // Add more commands as needed
};
class DataSender
{
   public:
    explicit DataSender(SerialPort* serialPort);
    std::stringstream createGpsMsg();
    void SendCommand(UserCommand command, const std::string& data);

   private:
    SerialPort* serialPort;
    uint16_t calculateCheckSum(const std::string& message);
};

#endif  // DATA_SENDER_HPP