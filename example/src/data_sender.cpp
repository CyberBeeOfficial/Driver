#include "../include/data_sender.hpp"

#include <iostream>
#include <sstream>
DataSender::DataSender(SerialPort* serialPort) : serialPort(serialPort)
{
    // data sender constructor
}

uint16_t DataSender::calculateCheckSum(const std::string& message) { return 0; }
// the Sender Class cpp file

void DataSender::SendCommand(uint8_t command, const std::string& data)
{
    std::stringstream temp_ss;
    std::stringstream full_ss;
    uint16_t chksum;

    if ((serialPort) && (serialPort->IsSerialConnectionOpen()))
    {
        std::cout << "Serial connection is open." << std::endl;
        // build command
        temp_ss << command << ";" << data;
        chksum = DataSender::calculateCheckSum(temp_ss.str());
        std::string chksumStr = std::to_string(chksum);

        full_ss << "<ST>;" << command << ";" << data << ";" << chksumStr
                << ";<EN>";
        std::cout << full_ss.str() << std::endl;
        auto write_success = serialPort->WriteToBuffer(full_ss.str());
        if (write_success)
        {
            std::cout << "Command sent successfully: " << full_ss.str()
                      << std::endl;
        }
        else
        {
            std::cout
                << "Failed to send the command due to a communication error."
                << std::endl;
        }
    }
    else
    {
        std::cout << "Serial connection is closed, Commmand Can't be Sent"
                  << std::endl;
    }
}
