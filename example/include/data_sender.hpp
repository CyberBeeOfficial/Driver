#ifndef DATA_SENDER_HPP
#define DATA_SENDER_HPP

#include "serial_comm.hpp"

class DataSender {

public:
    DataSender(SerialPort* serialPort) : serialPort_(serialPort) {}

    void WriteData(SerialPort& serialPort);
private:
    SerialPort* serialPort_;
};


#endif // DATA_SENDER_HPP