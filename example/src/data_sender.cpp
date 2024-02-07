#include "data_receiver.hpp"
#include "calculate_checksum.hpp"


//the Sender Class cpp file 



// Use the template function with Sender
int senderChecksum = calculateCheckSum<Sender>(message, SenderTag);