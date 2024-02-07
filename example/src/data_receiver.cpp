#include "data_receiver.hpp"
#include "calculate_checksum.hpp"
// Use the template function with Sender
int senderChecksum = calculateCheckSum<DataReceiver>(message, ReceiverTag);

void DataReceiver::readData() 
{
    if (serialPort_ != nullptr) {
            // Use serialPort_-> to call methods on SerialPort
            std::string data = serialPort_->read(); // Assuming SerialPort has a read method
            ParseAndPrintMsg(data);
        }
}