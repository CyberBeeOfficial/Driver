#ifndef DATA_RECEIVER_HPP
#define DATA_RECEIVER_HPP

#include "serial_comm.hpp"

class DataReceiver {

public:
    DataReceiver(SerialPort* serialPort) : serialPort_(serialPort) {}

    void readData();
    void ParseAndPrintMsg(const std::string& message);
    void updateBaudRate(unsigned int newBaudRate);

private:
    SerialPort* serialPort_;
};


#endif // RECEIVE_DATA_H